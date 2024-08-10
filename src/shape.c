// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"

#include "allocate.h"
#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

#include <stddef.h>

static b2Shape* b2GetShape( b2World* world, b2ShapeId shapeId )
{
	int id = shapeId.index1 - 1;
	b2CheckIdAndRevision( world->shapeArray, id, shapeId.revision );
	b2Shape* shape = world->shapeArray + id;
	return shape;
}

b2Transform b2GetOwnerTransform( b2World* world, b2Shape* shape )
{
	return b2GetBodyTransform( world, shape->bodyId );
}

static b2ChainShape* b2GetChainShape( b2World* world, b2ChainId chainId )
{
	int id = chainId.index1 - 1;
	b2CheckIdAndRevision( world->chainArray, id, chainId.revision );
	b2ChainShape* chain = world->chainArray + id;
	return chain;
}

static void b2UpdateShapeAABBs( b2Shape* shape, b2Transform transform, b2BodyType proxyType )
{
	// Compute a bounding box with a speculative margin
	const float speculativeDistance = b2_speculativeDistance;
	const float aabbMargin = b2_aabbMargin;

	b2AABB aabb = b2ComputeShapeAABB( shape, transform );
	aabb.lowerBound.x -= speculativeDistance;
	aabb.lowerBound.y -= speculativeDistance;
	aabb.upperBound.x += speculativeDistance;
	aabb.upperBound.y += speculativeDistance;
	shape->aabb = aabb;

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	float margin = proxyType == b2_staticBody ? speculativeDistance : aabbMargin;
	b2AABB fatAABB;
	fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
	fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
	fatAABB.upperBound.x = aabb.upperBound.x + margin;
	fatAABB.upperBound.y = aabb.upperBound.y + margin;
	shape->fatAABB = fatAABB;
}

static b2Shape* b2CreateShapeInternal( b2World* world, b2Body* body, b2Transform transform, const b2ShapeDef* def,
									   const void* geometry, b2ShapeType shapeType )
{
	B2_ASSERT( b2IsValid( def->density ) && def->density >= 0.0f );
	B2_ASSERT( b2IsValid( def->friction ) && def->friction >= 0.0f );
	B2_ASSERT( b2IsValid( def->restitution ) && def->restitution >= 0.0f );

	int shapeId = b2AllocId( &world->shapeIdPool );

	if ( shapeId == b2Array( world->shapeArray ).count )
	{
		b2Array_Push( world->shapeArray, ( b2Shape ){ 0 } );
	}
	else
	{
		B2_ASSERT( world->shapeArray[shapeId].id == B2_NULL_INDEX );
	}

	b2CheckIndex( world->shapeArray, shapeId );
	b2Shape* shape = world->shapeArray + shapeId;

	switch ( shapeType )
	{
		case b2_capsuleShape:
			shape->capsule = *(const b2Capsule*)geometry;
			break;

		case b2_circleShape:
			shape->circle = *(const b2Circle*)geometry;
			break;

		case b2_polygonShape:
			shape->polygon = *(const b2Polygon*)geometry;
			break;

		case b2_segmentShape:
			shape->segment = *(const b2Segment*)geometry;
			break;

		case b2_smoothSegmentShape:
			shape->smoothSegment = *(const b2SmoothSegment*)geometry;
			break;

		default:
			B2_ASSERT( false );
			break;
	}

	shape->id = shapeId;
	shape->bodyId = body->id;
	shape->type = shapeType;
	shape->density = def->density;
	shape->friction = def->friction;
	shape->restitution = def->restitution;
	shape->filter = def->filter;
	shape->userData = def->userData;
	shape->customColor = def->customColor;
	shape->isSensor = def->isSensor;
	shape->enlargedAABB = false;
	shape->enableSensorEvents = def->enableSensorEvents;
	shape->enableContactEvents = def->enableContactEvents;
	shape->enableHitEvents = def->enableHitEvents;
	shape->enablePreSolveEvents = def->enablePreSolveEvents;
	shape->isFast = false;
	shape->proxyKey = B2_NULL_INDEX;
	shape->localCentroid = b2GetShapeCentroid( shape );
	shape->aabb = ( b2AABB ){ b2Vec2_zero, b2Vec2_zero };
	shape->fatAABB = ( b2AABB ){ b2Vec2_zero, b2Vec2_zero };
	shape->revision += 1;

	if ( body->setIndex != b2_disabledSet )
	{
		b2BodyType proxyType = body->type;
		b2CreateShapeProxy( shape, &world->broadPhase, proxyType, transform, def->forceContactCreation );
	}

	// Add to shape doubly linked list
	if ( body->headShapeId != B2_NULL_INDEX )
	{
		b2CheckId( world->shapeArray, body->headShapeId );
		b2Shape* headShape = world->shapeArray + body->headShapeId;
		headShape->prevShapeId = shapeId;
	}

	shape->prevShapeId = B2_NULL_INDEX;
	shape->nextShapeId = body->headShapeId;
	body->headShapeId = shapeId;
	body->shapeCount += 1;

	b2ValidateSolverSets( world );

	return shape;
}

b2ShapeId b2CreateShape( b2BodyId bodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType )
{
	b2CheckDef( def );
	B2_ASSERT( b2IsValid( def->density ) && def->density >= 0.0f );
	B2_ASSERT( b2IsValid( def->friction ) && def->friction >= 0.0f );
	B2_ASSERT( b2IsValid( def->restitution ) && def->restitution >= 0.0f );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return ( b2ShapeId ){ 0 };
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2Shape* shape = b2CreateShapeInternal( world, body, transform, def, geometry, shapeType );

	if ( body->automaticMass == true )
	{
		b2UpdateBodyMassData( world, body );
	}

	b2ValidateSolverSets( world );

	b2ShapeId id = { shape->id + 1, bodyId.world0, shape->revision };
	return id;
}

b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle )
{
	return b2CreateShape( bodyId, def, circle, b2_circleShape );
}

b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule )
{
	float lengthSqr = b2DistanceSquared( capsule->center1, capsule->center2 );
	if ( lengthSqr <= b2_linearSlop * b2_linearSlop )
	{
		b2Circle circle = { b2Lerp( capsule->center1, capsule->center2, 0.5f ), capsule->radius };
		return b2CreateShape( bodyId, def, &circle, b2_circleShape );
	}

	return b2CreateShape( bodyId, def, capsule, b2_capsuleShape );
}

b2ShapeId b2CreatePolygonShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon )
{
	B2_ASSERT( b2IsValid( polygon->radius ) && polygon->radius >= 0.0f );
	return b2CreateShape( bodyId, def, polygon, b2_polygonShape );
}

b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment )
{
	float lengthSqr = b2DistanceSquared( segment->point1, segment->point2 );
	if ( lengthSqr <= b2_linearSlop * b2_linearSlop )
	{
		B2_ASSERT( false );
		return b2_nullShapeId;
	}

	return b2CreateShape( bodyId, def, segment, b2_segmentShape );
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
void b2DestroyShapeInternal( b2World* world, b2Shape* shape, b2Body* body, bool wakeBodies )
{
	int shapeId = shape->id;

	// Remove the shape from the body's doubly linked list.
	if ( shape->prevShapeId != B2_NULL_INDEX )
	{
		b2CheckId( world->shapeArray, shape->prevShapeId );
		world->shapeArray[shape->prevShapeId].nextShapeId = shape->nextShapeId;
	}

	if ( shape->nextShapeId != B2_NULL_INDEX )
	{
		b2CheckId( world->shapeArray, shape->nextShapeId );
		world->shapeArray[shape->nextShapeId].prevShapeId = shape->prevShapeId;
	}

	if ( shapeId == body->headShapeId )
	{
		body->headShapeId = shape->nextShapeId;
	}

	body->shapeCount -= 1;

	// Remove from broad-phase.
	b2DestroyShapeProxy( shape, &world->broadPhase );

	// Destroy any contacts associated with the shape.
	int contactKey = body->headContactKey;
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2CheckIndex( world->contactArray, contactId );
		b2Contact* contact = world->contactArray + contactId;
		contactKey = contact->edges[edgeIndex].nextKey;

		if ( contact->shapeIdA == shapeId || contact->shapeIdB == shapeId )
		{
			b2DestroyContact( world, contact, wakeBodies );
		}
	}

	// Return shape to free list.
	b2FreeId( &world->shapeIdPool, shapeId );
	shape->id = B2_NULL_INDEX;

	b2ValidateSolverSets( world );
}

void b2DestroyShape( b2ShapeId shapeId )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );

	int id = shapeId.index1 - 1;
	b2CheckIdAndRevision( world->shapeArray, id, shapeId.revision );

	b2Shape* shape = world->shapeArray + id;

	// need to wake bodies because this might be a static body
	bool wakeBodies = true;

	b2Body* body = b2GetBody( world, shape->bodyId );
	b2DestroyShapeInternal( world, shape, body, wakeBodies );

	if ( body->automaticMass == true )
	{
		b2UpdateBodyMassData( world, body );
	}
}

b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def )
{
	b2CheckDef( def );
	B2_ASSERT( b2IsValid( def->friction ) && def->friction >= 0.0f );
	B2_ASSERT( b2IsValid( def->restitution ) && def->restitution >= 0.0f );
	B2_ASSERT( def->count >= 4 );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return ( b2ChainId ){ 0 };
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	int chainId = b2AllocId( &world->chainIdPool );

	if ( chainId == b2Array( world->chainArray ).count )
	{
		b2Array_Push( world->chainArray, ( b2ChainShape ){ 0 } );
	}
	else
	{
		B2_ASSERT( world->chainArray[chainId].id == B2_NULL_INDEX );
	}

	b2CheckIndex( world->chainArray, chainId );
	b2ChainShape* chainShape = world->chainArray + chainId;

	chainShape->id = chainId;
	chainShape->bodyId = body->id;
	chainShape->nextChainId = body->headChainId;
	chainShape->revision += 1;
	body->headChainId = chainId;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.userData = def->userData;
	shapeDef.restitution = def->restitution;
	shapeDef.friction = def->friction;
	shapeDef.filter = def->filter;
	shapeDef.enableContactEvents = false;
	shapeDef.enableHitEvents = false;
	shapeDef.enableSensorEvents = false;

	int n = def->count;
	const b2Vec2* points = def->points;

	if ( def->isLoop )
	{
		chainShape->count = n;
		chainShape->shapeIndices = b2Alloc( n * sizeof( int ) );

		b2SmoothSegment smoothSegment;

		int prevIndex = n - 1;
		for ( int i = 0; i < n - 2; ++i )
		{
			smoothSegment.ghost1 = points[prevIndex];
			smoothSegment.segment.point1 = points[i];
			smoothSegment.segment.point2 = points[i + 1];
			smoothSegment.ghost2 = points[i + 2];
			smoothSegment.chainId = chainId;
			prevIndex = i;

			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &smoothSegment, b2_smoothSegmentShape );
			chainShape->shapeIndices[i] = shape->id;
		}

		{
			smoothSegment.ghost1 = points[n - 3];
			smoothSegment.segment.point1 = points[n - 2];
			smoothSegment.segment.point2 = points[n - 1];
			smoothSegment.ghost2 = points[0];
			smoothSegment.chainId = chainId;
			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &smoothSegment, b2_smoothSegmentShape );
			chainShape->shapeIndices[n - 2] = shape->id;
		}

		{
			smoothSegment.ghost1 = points[n - 2];
			smoothSegment.segment.point1 = points[n - 1];
			smoothSegment.segment.point2 = points[0];
			smoothSegment.ghost2 = points[1];
			smoothSegment.chainId = chainId;
			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &smoothSegment, b2_smoothSegmentShape );
			chainShape->shapeIndices[n - 1] = shape->id;
		}
	}
	else
	{
		chainShape->count = n - 3;
		chainShape->shapeIndices = b2Alloc( n * sizeof( int ) );

		b2SmoothSegment smoothSegment;

		for ( int i = 0; i < n - 3; ++i )
		{
			smoothSegment.ghost1 = points[i];
			smoothSegment.segment.point1 = points[i + 1];
			smoothSegment.segment.point2 = points[i + 2];
			smoothSegment.ghost2 = points[i + 3];
			smoothSegment.chainId = chainId;

			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &smoothSegment, b2_smoothSegmentShape );
			chainShape->shapeIndices[i] = shape->id;
		}
	}

	b2ChainId id = { chainId + 1, world->worldId, chainShape->revision };
	return id;
}

void b2DestroyChain( b2ChainId chainId )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );

	int id = chainId.index1 - 1;
	b2CheckIdAndRevision( world->chainArray, id, chainId.revision );

	b2ChainShape* chain = world->chainArray + id;
	bool wakeBodies = true;

	b2Body* body = b2GetBody( world, chain->bodyId );

	// Remove the chain from the body's singly linked list.
	int* chainIdPtr = &body->headChainId;
	bool found = false;
	while ( *chainIdPtr != B2_NULL_INDEX )
	{
		if ( *chainIdPtr == chain->id )
		{
			*chainIdPtr = chain->nextChainId;
			found = true;
			break;
		}

		chainIdPtr = &( world->chainArray[*chainIdPtr].nextChainId );
	}

	B2_ASSERT( found == true );
	if ( found == false )
	{
		return;
	}

	int count = chain->count;
	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chain->shapeIndices[i];
		b2CheckId( world->shapeArray, shapeId );
		b2Shape* shape = world->shapeArray + shapeId;
		b2DestroyShapeInternal( world, shape, body, wakeBodies );
	}

	b2Free( chain->shapeIndices, chain->count * sizeof( int ) );
	chain->shapeIndices = NULL;

	// Return chain to free list.
	b2FreeId( &world->chainIdPool, id );
	chain->id = B2_NULL_INDEX;

	b2ValidateSolverSets( world );
}

b2AABB b2ComputeShapeAABB( const b2Shape* shape, b2Transform xf )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleAABB( &shape->capsule, xf );
		case b2_circleShape:
			return b2ComputeCircleAABB( &shape->circle, xf );
		case b2_polygonShape:
			return b2ComputePolygonAABB( &shape->polygon, xf );
		case b2_segmentShape:
			return b2ComputeSegmentAABB( &shape->segment, xf );
		case b2_smoothSegmentShape:
			return b2ComputeSegmentAABB( &shape->smoothSegment.segment, xf );
		default:
		{
			B2_ASSERT( false );
			b2AABB empty = { xf.p, xf.p };
			return empty;
		}
	}
}

b2Vec2 b2GetShapeCentroid( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2Lerp( shape->capsule.center1, shape->capsule.center2, 0.5f );
		case b2_circleShape:
			return shape->circle.center;
		case b2_polygonShape:
			return shape->polygon.centroid;
		case b2_segmentShape:
			return b2Lerp( shape->segment.point1, shape->segment.point2, 0.5f );
		case b2_smoothSegmentShape:
			return b2Lerp( shape->smoothSegment.segment.point1, shape->smoothSegment.segment.point2, 0.5f );
		default:
			return b2Vec2_zero;
	}
}

// todo maybe compute this on shape creation
float b2GetShapePerimeter( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return 2.0f * b2Length( b2Sub( shape->capsule.center1, shape->capsule.center2 ) ) +
				   2.0f * b2_pi * shape->capsule.radius;
		case b2_circleShape:
			return 2.0f * b2_pi * shape->circle.radius;
		case b2_polygonShape:
		{
			const b2Vec2* points = shape->polygon.vertices;
			int count = shape->polygon.count;
			float perimeter = 2.0f * b2_pi * shape->polygon.radius;
			B2_ASSERT( count > 0 );
			b2Vec2 prev = points[count - 1];
			for ( int i = 0; i < count; ++i )
			{
				b2Vec2 next = points[i];
				perimeter += b2Length( b2Sub( next, prev ) );
				prev = next;
			}

			return perimeter;
		}
		case b2_segmentShape:
			return 2.0f * b2Length( b2Sub( shape->segment.point1, shape->segment.point2 ) );
		case b2_smoothSegmentShape:
			return 2.0f * b2Length( b2Sub( shape->smoothSegment.segment.point1, shape->smoothSegment.segment.point2 ) );
		default:
			return 0.0f;
	}
}

b2MassData b2ComputeShapeMass( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleMass( &shape->capsule, shape->density );
		case b2_circleShape:
			return b2ComputeCircleMass( &shape->circle, shape->density );
		case b2_polygonShape:
			return b2ComputePolygonMass( &shape->polygon, shape->density );
		default:
		{
			return ( b2MassData ){ 0 };
		}
	}
}

b2ShapeExtent b2ComputeShapeExtent( const b2Shape* shape, b2Vec2 localCenter )
{
	b2ShapeExtent extent = { 0 };

	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			float radius = shape->capsule.radius;
			extent.minExtent = radius;
			b2Vec2 c1 = b2Sub( shape->capsule.center1, localCenter );
			b2Vec2 c2 = b2Sub( shape->capsule.center2, localCenter );
			extent.maxExtent = sqrtf( b2MaxFloat( b2LengthSquared( c1 ), b2LengthSquared( c2 ) ) ) + radius;
		}
		break;

		case b2_circleShape:
		{
			float radius = shape->circle.radius;
			extent.minExtent = radius;
			extent.maxExtent = b2Length( b2Sub( shape->circle.center, localCenter ) ) + radius;
		}
		break;

		case b2_polygonShape:
		{
			const b2Polygon* poly = &shape->polygon;
			float minExtent = b2_huge;
			float maxExtentSqr = 0.0f;
			int count = poly->count;
			for ( int i = 0; i < count; ++i )
			{
				b2Vec2 v = poly->vertices[i];
				float planeOffset = b2Dot( poly->normals[i], b2Sub( v, poly->centroid ) );
				minExtent = b2MinFloat( minExtent, planeOffset );

				float distanceSqr = b2LengthSquared( b2Sub( v, localCenter ) );
				maxExtentSqr = b2MaxFloat( maxExtentSqr, distanceSqr );
			}

			extent.minExtent = minExtent + poly->radius;
			extent.maxExtent = sqrtf( maxExtentSqr ) + poly->radius;
		}
		break;

		case b2_segmentShape:
		{
			extent.minExtent = 0.0f;
			b2Vec2 c1 = b2Sub( shape->segment.point1, localCenter );
			b2Vec2 c2 = b2Sub( shape->segment.point2, localCenter );
			extent.maxExtent = sqrtf( b2MaxFloat( b2LengthSquared( c1 ), b2LengthSquared( c2 ) ) );
		}
		break;

		case b2_smoothSegmentShape:
		{
			extent.minExtent = 0.0f;
			b2Vec2 c1 = b2Sub( shape->smoothSegment.segment.point1, localCenter );
			b2Vec2 c2 = b2Sub( shape->smoothSegment.segment.point2, localCenter );
			extent.maxExtent = sqrtf( b2MaxFloat( b2LengthSquared( c1 ), b2LengthSquared( c2 ) ) );
		}
		break;

		default:
			break;
	}

	return extent;
}

b2CastOutput b2RayCastShape( const b2RayCastInput* input, const b2Shape* shape, b2Transform transform )
{
	b2RayCastInput localInput = *input;
	localInput.origin = b2InvTransformPoint( transform, input->origin );
	localInput.translation = b2InvRotateVector( transform.q, input->translation );

	b2CastOutput output = { 0 };
	switch ( shape->type )
	{
		case b2_capsuleShape:
			output = b2RayCastCapsule( &localInput, &shape->capsule );
			break;
		case b2_circleShape:
			output = b2RayCastCircle( &localInput, &shape->circle );
			break;
		case b2_polygonShape:
			output = b2RayCastPolygon( &localInput, &shape->polygon );
			break;
		case b2_segmentShape:
			output = b2RayCastSegment( &localInput, &shape->segment, false );
			break;
		case b2_smoothSegmentShape:
			output = b2RayCastSegment( &localInput, &shape->smoothSegment.segment, true );
			break;
		default:
			return output;
	}

	output.point = b2TransformPoint( transform, output.point );
	output.normal = b2RotateVector( transform.q, output.normal );
	return output;
}

b2CastOutput b2ShapeCastShape( const b2ShapeCastInput* input, const b2Shape* shape, b2Transform transform )
{
	b2ShapeCastInput localInput = *input;

	for ( int i = 0; i < localInput.count; ++i )
	{
		localInput.points[i] = b2InvTransformPoint( transform, input->points[i] );
	}

	localInput.translation = b2InvRotateVector( transform.q, input->translation );

	b2CastOutput output = { 0 };
	switch ( shape->type )
	{
		case b2_capsuleShape:
			output = b2ShapeCastCapsule( &localInput, &shape->capsule );
			break;
		case b2_circleShape:
			output = b2ShapeCastCircle( &localInput, &shape->circle );
			break;
		case b2_polygonShape:
			output = b2ShapeCastPolygon( &localInput, &shape->polygon );
			break;
		case b2_segmentShape:
			output = b2ShapeCastSegment( &localInput, &shape->segment );
			break;
		case b2_smoothSegmentShape:
			output = b2ShapeCastSegment( &localInput, &shape->smoothSegment.segment );
			break;
		default:
			return output;
	}

	output.point = b2TransformPoint( transform, output.point );
	output.normal = b2RotateVector( transform.q, output.normal );
	return output;
}

void b2CreateShapeProxy( b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform transform, bool forcePairCreation )
{
	B2_ASSERT( shape->proxyKey == B2_NULL_INDEX );

	b2UpdateShapeAABBs( shape, transform, type );

	// Create proxies in the broad-phase.
	shape->proxyKey =
		b2BroadPhase_CreateProxy( bp, type, shape->fatAABB, shape->filter.categoryBits, shape->id, forcePairCreation );
	B2_ASSERT( B2_PROXY_TYPE( shape->proxyKey ) < b2_bodyTypeCount );
}

void b2DestroyShapeProxy( b2Shape* shape, b2BroadPhase* bp )
{
	if ( shape->proxyKey != B2_NULL_INDEX )
	{
		b2BroadPhase_DestroyProxy( bp, shape->proxyKey );
		shape->proxyKey = B2_NULL_INDEX;
	}
}

b2DistanceProxy b2MakeShapeDistanceProxy( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2MakeProxy( &shape->capsule.center1, 2, shape->capsule.radius );
		case b2_circleShape:
			return b2MakeProxy( &shape->circle.center, 1, shape->circle.radius );
		case b2_polygonShape:
			return b2MakeProxy( shape->polygon.vertices, shape->polygon.count, shape->polygon.radius );
		case b2_segmentShape:
			return b2MakeProxy( &shape->segment.point1, 2, 0.0f );
		case b2_smoothSegmentShape:
			return b2MakeProxy( &shape->smoothSegment.segment.point1, 2, 0.0f );
		default:
		{
			B2_ASSERT( false );
			b2DistanceProxy empty = { 0 };
			return empty;
		}
	}
}

b2BodyId b2Shape_GetBody( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return b2MakeBodyId( world, shape->bodyId );
}

void b2Shape_SetUserData( b2ShapeId shapeId, void* userData )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	shape->userData = userData;
}

void* b2Shape_GetUserData( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->userData;
}

bool b2Shape_IsSensor( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->isSensor;
}

bool b2Shape_TestPoint( b2ShapeId shapeId, b2Vec2 point )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );

	b2Transform transform = b2GetOwnerTransform( world, shape );
	b2Vec2 localPoint = b2InvTransformPoint( transform, point );

	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2PointInCapsule( localPoint, &shape->capsule );

		case b2_circleShape:
			return b2PointInCircle( localPoint, &shape->circle );

		case b2_polygonShape:
			return b2PointInPolygon( localPoint, &shape->polygon );

		default:
			return false;
	}
}

b2CastOutput b2Shape_RayCast( b2ShapeId shapeId, b2Vec2 origin, b2Vec2 translation )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );

	b2Transform transform = b2GetOwnerTransform( world, shape );

	// input in local coordinates
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.origin = b2InvTransformPoint( transform, origin );
	input.translation = b2InvRotateVector( transform.q, translation );

	b2CastOutput output = { 0 };
	switch ( shape->type )
	{
		case b2_capsuleShape:
			output = b2RayCastCapsule( &input, &shape->capsule );
			break;

		case b2_circleShape:
			output = b2RayCastCircle( &input, &shape->circle );
			break;

		case b2_segmentShape:
			output = b2RayCastSegment( &input, &shape->segment, false );
			break;

		case b2_polygonShape:
			output = b2RayCastPolygon( &input, &shape->polygon );
			break;

		case b2_smoothSegmentShape:
			output = b2RayCastSegment( &input, &shape->smoothSegment.segment, true );
			break;

		default:
			B2_ASSERT( false );
			return output;
	}

	if ( output.hit )
	{
		// convert to world coordinates
		output.normal = b2RotateVector( transform.q, output.normal );
		output.point = b2TransformPoint( transform, output.point );
	}
	return output;
}

void b2Shape_SetDensity( b2ShapeId shapeId, float density )
{
	B2_ASSERT( b2IsValid( density ) && density >= 0.0f );

	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( density == shape->density )
	{
		// early return to avoid expensive function
		return;
	}

	shape->density = density;
}

float b2Shape_GetDensity( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->density;
}

void b2Shape_SetFriction( b2ShapeId shapeId, float friction )
{
	B2_ASSERT( b2IsValid( friction ) && friction >= 0.0f );

	b2World* world = b2GetWorld( shapeId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->friction = friction;
}

float b2Shape_GetFriction( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->friction;
}

void b2Shape_SetRestitution( b2ShapeId shapeId, float restitution )
{
	B2_ASSERT( b2IsValid( restitution ) && restitution >= 0.0f );

	b2World* world = b2GetWorld( shapeId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->restitution = restitution;
}

float b2Shape_GetRestitution( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->restitution;
}

b2Filter b2Shape_GetFilter( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->filter;
}

static void b2ResetProxy( b2World* world, b2Shape* shape, bool wakeBodies, bool destroyProxy )
{
	b2Body* body = b2GetBody( world, shape->bodyId );

	int shapeId = shape->id;

	// destroy all contacts associated with this shape
	int contactKey = body->headContactKey;
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2CheckIndex( world->contactArray, contactId );
		b2Contact* contact = world->contactArray + contactId;
		contactKey = contact->edges[edgeIndex].nextKey;

		if ( contact->shapeIdA == shapeId || contact->shapeIdB == shapeId )
		{
			b2DestroyContact( world, contact, wakeBodies );
		}
	}

	b2Transform transform = b2GetBodyTransformQuick( world, body );
	if ( shape->proxyKey != B2_NULL_INDEX )
	{
		b2BodyType proxyType = B2_PROXY_TYPE( shape->proxyKey );
		b2UpdateShapeAABBs( shape, transform, proxyType );

		if ( destroyProxy )
		{
			b2BroadPhase_DestroyProxy( &world->broadPhase, shape->proxyKey );

			bool forcePairCreation = true;
			shape->proxyKey = b2BroadPhase_CreateProxy( &world->broadPhase, proxyType, shape->fatAABB, shape->filter.categoryBits,
														shapeId, forcePairCreation );
		}
		else
		{
			b2BroadPhase_MoveProxy( &world->broadPhase, shape->proxyKey, shape->fatAABB );
		}
	}
	else
	{
		b2BodyType proxyType = body->type;
		b2UpdateShapeAABBs( shape, transform, proxyType );
	}

	b2ValidateSolverSets( world );
}

void b2Shape_SetFilter( b2ShapeId shapeId, b2Filter filter )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( filter.maskBits == shape->filter.maskBits && filter.categoryBits == shape->filter.categoryBits &&
		 filter.groupIndex == shape->filter.groupIndex )
	{
		return;
	}

	// If the category bits change, I need to destroy the proxy because it affects the tree sorting.
	bool destroyProxy = filter.categoryBits == shape->filter.categoryBits;

	shape->filter = filter;

	// need to wake bodies because a filter change may destroy contacts
	bool wakeBodies = true;
	b2ResetProxy( world, shape, wakeBodies, destroyProxy );
}

void b2Shape_EnableSensorEvents( b2ShapeId shapeId, bool flag )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->enableSensorEvents = flag;
}

bool b2Shape_AreSensorEventsEnabled( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->enableSensorEvents;
}

void b2Shape_EnableContactEvents( b2ShapeId shapeId, bool flag )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->enableContactEvents = flag;
}

bool b2Shape_AreContactEventsEnabled( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->enableContactEvents;
}

void b2Shape_EnablePreSolveEvents( b2ShapeId shapeId, bool flag )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->enablePreSolveEvents = flag;
}

bool b2Shape_ArePreSolveEventsEnabled( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->enablePreSolveEvents;
}

void b2Shape_EnableHitEvents( b2ShapeId shapeId, bool flag )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->enableHitEvents = flag;
}

bool b2Shape_AreHitEventsEnabled( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->enableHitEvents;
}

b2ShapeType b2Shape_GetType( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->type;
}

b2Circle b2Shape_GetCircle( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	B2_ASSERT( shape->type == b2_circleShape );
	return shape->circle;
}

b2Segment b2Shape_GetSegment( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	B2_ASSERT( shape->type == b2_segmentShape );
	return shape->segment;
}

b2SmoothSegment b2Shape_GetSmoothSegment( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	B2_ASSERT( shape->type == b2_smoothSegmentShape );
	return shape->smoothSegment;
}

b2Capsule b2Shape_GetCapsule( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	B2_ASSERT( shape->type == b2_capsuleShape );
	return shape->capsule;
}

b2Polygon b2Shape_GetPolygon( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	B2_ASSERT( shape->type == b2_polygonShape );
	return shape->polygon;
}

void b2Shape_SetCircle( b2ShapeId shapeId, const b2Circle* circle )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->circle = *circle;
	shape->type = b2_circleShape;

	// need to wake bodies so they can react to the shape change
	bool wakeBodies = true;
	bool destroyProxy = true;
	b2ResetProxy( world, shape, wakeBodies, destroyProxy );
}

void b2Shape_SetCapsule( b2ShapeId shapeId, const b2Capsule* capsule )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->capsule = *capsule;
	shape->type = b2_capsuleShape;

	// need to wake bodies so they can react to the shape change
	bool wakeBodies = true;
	bool destroyProxy = true;
	b2ResetProxy( world, shape, wakeBodies, destroyProxy );
}

void b2Shape_SetSegment( b2ShapeId shapeId, const b2Segment* segment )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->segment = *segment;
	shape->type = b2_segmentShape;

	// need to wake bodies so they can react to the shape change
	bool wakeBodies = true;
	bool destroyProxy = true;
	b2ResetProxy( world, shape, wakeBodies, destroyProxy );
}

void b2Shape_SetPolygon( b2ShapeId shapeId, const b2Polygon* polygon )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->polygon = *polygon;
	shape->type = b2_polygonShape;

	// need to wake bodies so they can react to the shape change
	bool wakeBodies = true;
	bool destroyProxy = true;
	b2ResetProxy( world, shape, wakeBodies, destroyProxy );
}

b2ChainId b2Shape_GetParentChain( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->type == b2_smoothSegmentShape )
	{
		int chainId = shape->smoothSegment.chainId;
		if ( chainId != B2_NULL_INDEX )
		{
			b2CheckId( world->chainArray, chainId );
			b2ChainShape* chain = world->chainArray + chainId;
			b2ChainId id = { chainId + 1, shapeId.world0, chain->revision };
			return id;
		}
	}

	return ( b2ChainId ){ 0 };
}

void b2Chain_SetFriction( b2ChainId chainId, float friction )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape( world, chainId );

	int count = chainShape->count;

	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chainShape->shapeIndices[i];
		b2CheckId( world->shapeArray, shapeId );
		b2Shape* shape = world->shapeArray + shapeId;
		shape->friction = friction;
	}
}

void b2Chain_SetRestitution( b2ChainId chainId, float restitution )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape( world, chainId );

	int count = chainShape->count;

	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chainShape->shapeIndices[i];
		b2CheckId( world->shapeArray, shapeId );
		b2Shape* shape = world->shapeArray + shapeId;
		shape->restitution = restitution;
	}
}

int b2Shape_GetContactCapacity( b2ShapeId shapeId )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->isSensor )
	{
		return 0;
	}

	b2Body* body = b2GetBody( world, shape->bodyId );

	// Conservative and fast
	return body->contactCount;
}

// todo sample needed
int b2Shape_GetContactData( b2ShapeId shapeId, b2ContactData* contactData, int capacity )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->isSensor )
	{
		return 0;
	}

	b2Body* body = b2GetBody( world, shape->bodyId );
	int contactKey = body->headContactKey;
	int index = 0;
	while ( contactKey != B2_NULL_INDEX && index < capacity )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2CheckIndex( world->contactArray, contactId );
		b2Contact* contact = world->contactArray + contactId;

		// Does contact involve this shape and is it touching?
		if ( ( contact->shapeIdA == shapeId.index1 - 1 || contact->shapeIdB == shapeId.index1 - 1 ) &&
			 ( contact->flags & b2_contactTouchingFlag ) != 0 )
		{
			b2Shape* shapeA = world->shapeArray + contact->shapeIdA;
			b2Shape* shapeB = world->shapeArray + contact->shapeIdB;

			contactData[index].shapeIdA = ( b2ShapeId ){ shapeA->id + 1, shapeId.world0, shapeA->revision };
			contactData[index].shapeIdB = ( b2ShapeId ){ shapeB->id + 1, shapeId.world0, shapeB->revision };

			b2ContactSim* contactSim = b2GetContactSim( world, contact );
			contactData[index].manifold = contactSim->manifold;
			index += 1;
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}

	B2_ASSERT( index <= capacity );

	return index;
}

b2AABB b2Shape_GetAABB( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return ( b2AABB ){ 0 };
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->aabb;
}

b2Vec2 b2Shape_GetClosestPoint( b2ShapeId shapeId, b2Vec2 target )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return ( b2Vec2 ){ 0 };
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	b2Body* body = b2GetBody( world, shape->bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &target, 1, 0.0f );
	input.transformA = transform;
	input.transformB = b2Transform_identity;
	input.useRadii = true;

	b2DistanceCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	return output.pointA;
}
