// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"

#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "sensor.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

#include <stddef.h>

B2_ARRAY_SOURCE( b2ChainShape, b2ChainShape )
B2_ARRAY_SOURCE( b2Shape, b2Shape )

static b2Shape* b2GetShape( b2World* world, b2ShapeId shapeId )
{
	int id = shapeId.index1 - 1;
	b2Shape* shape = b2ShapeArray_Get( &world->shapes, id );
	B2_ASSERT( shape->id == id && shape->generation == shapeId.generation );
	return shape;
}

static b2ChainShape* b2GetChainShape( b2World* world, b2ChainId chainId )
{
	int id = chainId.index1 - 1;
	b2ChainShape* chain = b2ChainShapeArray_Get( &world->chainShapes, id );
	B2_ASSERT( chain->id == id && chain->generation == chainId.generation );
	return chain;
}

static void b2UpdateShapeAABBs( b2Shape* shape, b2Transform transform, b2BodyType proxyType )
{
	// Compute a bounding box with a speculative margin
	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;
	const float aabbMargin = B2_AABB_MARGIN;

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
	int shapeId = b2AllocId( &world->shapeIdPool );

	if ( shapeId == world->shapes.count )
	{
		b2ShapeArray_Push( &world->shapes, (b2Shape){ 0 } );
	}
	else
	{
		B2_ASSERT( world->shapes.data[shapeId].id == B2_NULL_INDEX );
	}

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

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

		case b2_chainSegmentShape:
			shape->chainSegment = *(const b2ChainSegment*)geometry;
			break;

		default:
			B2_ASSERT( false );
			break;
	}

	shape->id = shapeId;
	shape->bodyId = body->id;
	shape->type = shapeType;
	shape->density = def->density;
	shape->friction = def->material.friction;
	shape->restitution = def->material.restitution;
	shape->rollingResistance = def->material.rollingResistance;
	shape->tangentSpeed = def->material.tangentSpeed;
	shape->userMaterialId = def->material.userMaterialId;
	shape->filter = def->filter;
	shape->userData = def->userData;
	shape->customColor = def->material.customColor;
	shape->enlargedAABB = false;
	shape->enableSensorEvents = def->enableSensorEvents;
	shape->enableContactEvents = def->enableContactEvents;
	shape->enableHitEvents = def->enableHitEvents;
	shape->enablePreSolveEvents = def->enablePreSolveEvents;
	shape->proxyKey = B2_NULL_INDEX;
	shape->localCentroid = b2GetShapeCentroid( shape );
	shape->aabb = (b2AABB){ b2Vec2_zero, b2Vec2_zero };
	shape->fatAABB = (b2AABB){ b2Vec2_zero, b2Vec2_zero };
	shape->generation += 1;

	if ( body->setIndex != b2_disabledSet )
	{
		b2BodyType proxyType = body->type;
		b2CreateShapeProxy( shape, &world->broadPhase, proxyType, transform, def->invokeContactCreation || def->isSensor );
	}

	// Add to shape doubly linked list
	if ( body->headShapeId != B2_NULL_INDEX )
	{
		b2Shape* headShape = b2ShapeArray_Get( &world->shapes, body->headShapeId );
		headShape->prevShapeId = shapeId;
	}

	shape->prevShapeId = B2_NULL_INDEX;
	shape->nextShapeId = body->headShapeId;
	body->headShapeId = shapeId;
	body->shapeCount += 1;

	if ( def->isSensor )
	{
		shape->sensorIndex = world->sensors.count;
		b2Sensor sensor = {
			.overlaps1 = b2ShapeRefArray_Create( 16 ),
			.overlaps2 = b2ShapeRefArray_Create( 16 ),
			.shapeId = shapeId,
		};
		b2SensorArray_Push( &world->sensors, sensor );
	}
	else
	{
		shape->sensorIndex = B2_NULL_INDEX;
	}

	b2ValidateSolverSets( world );

	return shape;
}

static b2ShapeId b2CreateShape( b2BodyId bodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType )
{
	B2_CHECK_DEF( def );
	B2_ASSERT( b2IsValidFloat( def->density ) && def->density >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->material.friction ) && def->material.friction >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->material.restitution ) && def->material.restitution >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->material.rollingResistance ) && def->material.rollingResistance >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->material.tangentSpeed ) );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return (b2ShapeId){ 0 };
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2Shape* shape = b2CreateShapeInternal( world, body, transform, def, geometry, shapeType );

	if ( def->updateBodyMass == true )
	{
		b2UpdateBodyMassData( world, body );
	}

	b2ValidateSolverSets( world );

	b2ShapeId id = { shape->id + 1, bodyId.world0, shape->generation };
	return id;
}

b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle )
{
	return b2CreateShape( bodyId, def, circle, b2_circleShape );
}

b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule )
{
	float lengthSqr = b2DistanceSquared( capsule->center1, capsule->center2 );
	if ( lengthSqr <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
	{
		b2Circle circle = { b2Lerp( capsule->center1, capsule->center2, 0.5f ), capsule->radius };
		return b2CreateShape( bodyId, def, &circle, b2_circleShape );
	}

	return b2CreateShape( bodyId, def, capsule, b2_capsuleShape );
}

b2ShapeId b2CreatePolygonShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon )
{
	B2_ASSERT( b2IsValidFloat( polygon->radius ) && polygon->radius >= 0.0f );
	return b2CreateShape( bodyId, def, polygon, b2_polygonShape );
}

b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment )
{
	float lengthSqr = b2DistanceSquared( segment->point1, segment->point2 );
	if ( lengthSqr <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
	{
		B2_ASSERT( false );
		return b2_nullShapeId;
	}

	return b2CreateShape( bodyId, def, segment, b2_segmentShape );
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
static void b2DestroyShapeInternal( b2World* world, b2Shape* shape, b2Body* body, bool wakeBodies )
{
	int shapeId = shape->id;

	// Remove the shape from the body's doubly linked list.
	if ( shape->prevShapeId != B2_NULL_INDEX )
	{
		b2Shape* prevShape = b2ShapeArray_Get( &world->shapes, shape->prevShapeId );
		prevShape->nextShapeId = shape->nextShapeId;
	}

	if ( shape->nextShapeId != B2_NULL_INDEX )
	{
		b2Shape* nextShape = b2ShapeArray_Get( &world->shapes, shape->nextShapeId );
		nextShape->prevShapeId = shape->prevShapeId;
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

		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
		contactKey = contact->edges[edgeIndex].nextKey;

		if ( contact->shapeIdA == shapeId || contact->shapeIdB == shapeId )
		{
			b2DestroyContact( world, contact, wakeBodies );
		}
	}

	if ( shape->sensorIndex != B2_NULL_INDEX )
	{
		b2Sensor* sensor = b2SensorArray_Get( &world->sensors, shape->sensorIndex );
		for ( int i = 0; i < sensor->overlaps2.count; ++i )
		{
			b2ShapeRef* ref = sensor->overlaps2.data + i;
			b2SensorEndTouchEvent event = {
				.sensorShapeId =
					{
						.index1 = shapeId + 1,
						.world0 = world->worldId,
						.generation = shape->generation,
					},
				.visitorShapeId =
					{
						.index1 = ref->shapeId + 1,
						.world0 = world->worldId,
						.generation = ref->generation,
					},
			};

			b2SensorEndTouchEventArray_Push( world->sensorEndEvents + world->endEventArrayIndex, event );
		}

		// Destroy sensor
		b2ShapeRefArray_Destroy( &sensor->overlaps1 );
		b2ShapeRefArray_Destroy( &sensor->overlaps2 );

		int movedIndex = b2SensorArray_RemoveSwap( &world->sensors, shape->sensorIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// Fixup moved sensor
			b2Sensor* movedSensor = b2SensorArray_Get( &world->sensors, shape->sensorIndex );
			b2Shape* otherSensorShape = b2ShapeArray_Get( &world->shapes, movedSensor->shapeId );
			otherSensorShape->sensorIndex = shape->sensorIndex;
		}
	}

	// Return shape to free list.
	b2FreeId( &world->shapeIdPool, shapeId );
	shape->id = B2_NULL_INDEX;

	b2ValidateSolverSets( world );
}

void b2DestroyShape( b2ShapeId shapeId, bool updateBodyMass )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );

	// need to wake bodies because this might be a static body
	bool wakeBodies = true;
	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	b2DestroyShapeInternal( world, shape, body, wakeBodies );

	if ( updateBodyMass == true )
	{
		b2UpdateBodyMassData( world, body );
	}
}

b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def )
{
	B2_CHECK_DEF( def );
	B2_ASSERT( def->count >= 4 );
	B2_ASSERT( def->materialCount == 1 || def->materialCount == def->count );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return (b2ChainId){ 0 };
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	int chainId = b2AllocId( &world->chainIdPool );

	if ( chainId == world->chainShapes.count )
	{
		b2ChainShapeArray_Push( &world->chainShapes, (b2ChainShape){ 0 } );
	}
	else
	{
		B2_ASSERT( world->chainShapes.data[chainId].id == B2_NULL_INDEX );
	}

	b2ChainShape* chainShape = b2ChainShapeArray_Get( &world->chainShapes, chainId );

	chainShape->id = chainId;
	chainShape->bodyId = body->id;
	chainShape->nextChainId = body->headChainId;
	chainShape->generation += 1;

	int materialCount = def->materialCount;
	chainShape->materialCount = materialCount;
	chainShape->materials = b2Alloc( materialCount * sizeof( b2SurfaceMaterial ) );

	for ( int i = 0; i < materialCount; ++i )
	{
		const b2SurfaceMaterial* material = def->materials + i;
		B2_ASSERT( b2IsValidFloat( material->friction ) && material->friction >= 0.0f );
		B2_ASSERT( b2IsValidFloat( material->restitution ) && material->restitution >= 0.0f );
		B2_ASSERT( b2IsValidFloat( material->rollingResistance ) && material->rollingResistance >= 0.0f );
		B2_ASSERT( b2IsValidFloat( material->tangentSpeed ) );

		chainShape->materials[i] = *material;
	}

	body->headChainId = chainId;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.userData = def->userData;
	shapeDef.filter = def->filter;
	shapeDef.enableSensorEvents = def->enableSensorEvents;
	shapeDef.enableContactEvents = false;
	shapeDef.enableHitEvents = false;

	const b2Vec2* points = def->points;
	int n = def->count;

	if ( def->isLoop )
	{
		chainShape->count = n;
		chainShape->shapeIndices = b2Alloc( chainShape->count * sizeof( int ) );

		b2ChainSegment chainSegment;

		int prevIndex = n - 1;
		for ( int i = 0; i < n - 2; ++i )
		{
			chainSegment.ghost1 = points[prevIndex];
			chainSegment.segment.point1 = points[i];
			chainSegment.segment.point2 = points[i + 1];
			chainSegment.ghost2 = points[i + 2];
			chainSegment.chainId = chainId;
			prevIndex = i;

			int materialIndex = materialCount == 1 ? 0 : i;
			shapeDef.material = def->materials[materialIndex];

			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &chainSegment, b2_chainSegmentShape );
			chainShape->shapeIndices[i] = shape->id;
		}

		{
			chainSegment.ghost1 = points[n - 3];
			chainSegment.segment.point1 = points[n - 2];
			chainSegment.segment.point2 = points[n - 1];
			chainSegment.ghost2 = points[0];
			chainSegment.chainId = chainId;

			int materialIndex = materialCount == 1 ? 0 : n - 2;
			shapeDef.material = def->materials[materialIndex];

			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &chainSegment, b2_chainSegmentShape );
			chainShape->shapeIndices[n - 2] = shape->id;
		}

		{
			chainSegment.ghost1 = points[n - 2];
			chainSegment.segment.point1 = points[n - 1];
			chainSegment.segment.point2 = points[0];
			chainSegment.ghost2 = points[1];
			chainSegment.chainId = chainId;

			int materialIndex = materialCount == 1 ? 0 : n - 1;
			shapeDef.material = def->materials[materialIndex];

			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &chainSegment, b2_chainSegmentShape );
			chainShape->shapeIndices[n - 1] = shape->id;
		}
	}
	else
	{
		chainShape->count = n - 3;
		chainShape->shapeIndices = b2Alloc( chainShape->count * sizeof( int ) );

		b2ChainSegment chainSegment;

		for ( int i = 0; i < n - 3; ++i )
		{
			chainSegment.ghost1 = points[i];
			chainSegment.segment.point1 = points[i + 1];
			chainSegment.segment.point2 = points[i + 2];
			chainSegment.ghost2 = points[i + 3];
			chainSegment.chainId = chainId;

			// Material is associated with leading point of solid segment
			int materialIndex = materialCount == 1 ? 0 : i + 1;
			shapeDef.material = def->materials[materialIndex];

			b2Shape* shape = b2CreateShapeInternal( world, body, transform, &shapeDef, &chainSegment, b2_chainSegmentShape );
			chainShape->shapeIndices[i] = shape->id;
		}
	}

	b2ChainId id = { chainId + 1, world->worldId, chainShape->generation };
	return id;
}

void b2FreeChainData( b2ChainShape* chain )
{
	b2Free( chain->shapeIndices, chain->count * sizeof( int ) );
	chain->shapeIndices = NULL;

	b2Free( chain->materials, chain->materialCount * sizeof( b2SurfaceMaterial ) );
	chain->materials = NULL;
}

void b2DestroyChain( b2ChainId chainId )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2ChainShape* chain = b2GetChainShape( world, chainId );

	b2Body* body = b2BodyArray_Get( &world->bodies, chain->bodyId );

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

		chainIdPtr = &( world->chainShapes.data[*chainIdPtr].nextChainId );
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
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		bool wakeBodies = true;
		b2DestroyShapeInternal( world, shape, body, wakeBodies );
	}

	b2FreeChainData( chain );

	// Return chain to free list.
	b2FreeId( &world->chainIdPool, chain->id );
	chain->id = B2_NULL_INDEX;

	b2ValidateSolverSets( world );
}

b2WorldId b2Chain_GetWorld( b2ChainId chainId )
{
	b2World* world = b2GetWorld( chainId.world0 );
	return (b2WorldId){ chainId.world0 + 1, world->generation };
}

int b2Chain_GetSegmentCount( b2ChainId chainId )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2ChainShape* chain = b2GetChainShape( world, chainId );
	return chain->count;
}

int b2Chain_GetSegments( b2ChainId chainId, b2ShapeId* segmentArray, int capacity )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2ChainShape* chain = b2GetChainShape( world, chainId );

	int count = b2MinInt( chain->count, capacity );
	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chain->shapeIndices[i];
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		segmentArray[i] = (b2ShapeId){ shapeId + 1, chainId.world0, shape->generation };
	}

	return count;
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
		case b2_chainSegmentShape:
			return b2ComputeSegmentAABB( &shape->chainSegment.segment, xf );
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
		case b2_chainSegmentShape:
			return b2Lerp( shape->chainSegment.segment.point1, shape->chainSegment.segment.point2, 0.5f );
		default:
			return b2Vec2_zero;
	}
}

// todo_erin maybe compute this on shape creation
float b2GetShapePerimeter( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return 2.0f * b2Length( b2Sub( shape->capsule.center1, shape->capsule.center2 ) ) +
				   2.0f * B2_PI * shape->capsule.radius;
		case b2_circleShape:
			return 2.0f * B2_PI * shape->circle.radius;
		case b2_polygonShape:
		{
			const b2Vec2* points = shape->polygon.vertices;
			int count = shape->polygon.count;
			float perimeter = 2.0f * B2_PI * shape->polygon.radius;
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
		case b2_chainSegmentShape:
			return 2.0f * b2Length( b2Sub( shape->chainSegment.segment.point1, shape->chainSegment.segment.point2 ) );
		default:
			return 0.0f;
	}
}

// This projects the shape perimeter onto an infinite line
float b2GetShapeProjectedPerimeter( const b2Shape* shape, b2Vec2 line )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			b2Vec2 axis = b2Sub( shape->capsule.center2, shape->capsule.center1 );
			float projectedLength = b2AbsFloat( b2Dot( axis, line ) );
			return projectedLength + 2.0f * shape->capsule.radius;
		}

		case b2_circleShape:
			return 2.0f * shape->circle.radius;

		case b2_polygonShape:
		{
			const b2Vec2* points = shape->polygon.vertices;
			int count = shape->polygon.count;
			B2_ASSERT( count > 0 );
			float value = b2Dot( points[0], line );
			float lower = value;
			float upper = value;
			for ( int i = 1; i < count; ++i )
			{
				value = b2Dot( points[i], line );
				lower = b2MinFloat( lower, value );
				upper = b2MaxFloat( upper, value );
			}

			return ( upper - lower ) + 2.0f * shape->polygon.radius;
		}

		case b2_segmentShape:
		{
			float value1 = b2Dot( shape->segment.point1, line );
			float value2 = b2Dot( shape->segment.point2, line );
			return b2AbsFloat( value2 - value1 );
		}

		case b2_chainSegmentShape:
		{
			float value1 = b2Dot( shape->chainSegment.segment.point1, line );
			float value2 = b2Dot( shape->chainSegment.segment.point2, line );
			return b2AbsFloat( value2 - value1 );
		}

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
			return (b2MassData){ 0 };
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
			float minExtent = B2_HUGE;
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

		case b2_chainSegmentShape:
		{
			extent.minExtent = 0.0f;
			b2Vec2 c1 = b2Sub( shape->chainSegment.segment.point1, localCenter );
			b2Vec2 c2 = b2Sub( shape->chainSegment.segment.point2, localCenter );
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
		case b2_chainSegmentShape:
			output = b2RayCastSegment( &localInput, &shape->chainSegment.segment, true );
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

	for ( int i = 0; i < localInput.proxy.count; ++i )
	{
		localInput.proxy.points[i] = b2InvTransformPoint( transform, input->proxy.points[i] );
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
		case b2_chainSegmentShape:
			output = b2ShapeCastSegment( &localInput, &shape->chainSegment.segment );
			break;
		default:
			return output;
	}

	output.point = b2TransformPoint( transform, output.point );
	output.normal = b2RotateVector( transform.q, output.normal );
	return output;
}

b2PlaneResult b2CollideMover( const b2Shape* shape, b2Transform transform, const b2Capsule* mover )
{
	b2Capsule localMover;
	localMover.center1 = b2InvTransformPoint( transform, mover->center1 );
	localMover.center2 = b2InvTransformPoint( transform, mover->center2 );
	localMover.radius = mover->radius;

	b2PlaneResult result = { 0 };
	switch ( shape->type )
	{
		case b2_capsuleShape:
			result = b2CollideMoverAndCapsule( &shape->capsule, &localMover );
			break;
		case b2_circleShape:
			result = b2CollideMoverAndCircle( &shape->circle, &localMover );
			break;
		case b2_polygonShape:
			result = b2CollideMoverAndPolygon( &shape->polygon, &localMover );
			break;
		case b2_segmentShape:
			result = b2CollideMoverAndSegment( &shape->segment, &localMover );
			break;
		case b2_chainSegmentShape:
			result = b2CollideMoverAndSegment( &shape->chainSegment.segment, &localMover );
			break;
		default:
			return result;
	}

	if ( result.hit == false )
	{
		return result;
	}

	result.plane.normal = b2RotateVector( transform.q, result.plane.normal );
	return result;
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

b2ShapeProxy b2MakeShapeDistanceProxy( const b2Shape* shape )
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
		case b2_chainSegmentShape:
			return b2MakeProxy( &shape->chainSegment.segment.point1, 2, 0.0f );
		default:
		{
			B2_ASSERT( false );
			b2ShapeProxy empty = { 0 };
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

b2WorldId b2Shape_GetWorld( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	return (b2WorldId){ shapeId.world0 + 1, world->generation };
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
	return shape->sensorIndex != B2_NULL_INDEX;
}

bool b2Shape_TestPoint( b2ShapeId shapeId, b2Vec2 point )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );

	b2Transform transform = b2GetBodyTransform( world, shape->bodyId );
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

// todo_erin untested
b2CastOutput b2Shape_RayCast( b2ShapeId shapeId, const b2RayCastInput* input )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );

	b2Transform transform = b2GetBodyTransform( world, shape->bodyId );

	// input in local coordinates
	b2RayCastInput localInput;
	localInput.origin = b2InvTransformPoint( transform, input->origin );
	localInput.translation = b2InvRotateVector( transform.q, input->translation );
	localInput.maxFraction = input->maxFraction;

	b2CastOutput output = { 0 };
	switch ( shape->type )
	{
		case b2_capsuleShape:
			output = b2RayCastCapsule( &localInput, &shape->capsule );
			break;

		case b2_circleShape:
			output = b2RayCastCircle( &localInput, &shape->circle );
			break;

		case b2_segmentShape:
			output = b2RayCastSegment( &localInput, &shape->segment, false );
			break;

		case b2_polygonShape:
			output = b2RayCastPolygon( &localInput, &shape->polygon );
			break;

		case b2_chainSegmentShape:
			output = b2RayCastSegment( &localInput, &shape->chainSegment.segment, true );
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

void b2Shape_SetDensity( b2ShapeId shapeId, float density, bool updateBodyMass )
{
	B2_ASSERT( b2IsValidFloat( density ) && density >= 0.0f );

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

	if ( updateBodyMass == true )
	{
		b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
		b2UpdateBodyMassData( world, body );
	}
}

float b2Shape_GetDensity( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->density;
}

void b2Shape_SetFriction( b2ShapeId shapeId, float friction )
{
	B2_ASSERT( b2IsValidFloat( friction ) && friction >= 0.0f );

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
	B2_ASSERT( b2IsValidFloat( restitution ) && restitution >= 0.0f );

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

void b2Shape_SetMaterial( b2ShapeId shapeId, int material )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	shape->userMaterialId = material;
}

int b2Shape_GetMaterial( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->userMaterialId;
}

b2SurfaceMaterial b2Shape_GetSurfaceMaterial( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return (b2SurfaceMaterial){
		.friction = shape->friction,
		.restitution = shape->restitution,
		.rollingResistance = shape->rollingResistance,
		.tangentSpeed = shape->tangentSpeed,
		.userMaterialId = shape->userMaterialId,
		.customColor = shape->customColor,
	};
}

void b2Shape_SetSurfaceMaterial( b2ShapeId shapeId, b2SurfaceMaterial surfaceMaterial )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	shape->friction = surfaceMaterial.friction;
	shape->restitution = surfaceMaterial.restitution;
	shape->rollingResistance = surfaceMaterial.rollingResistance;
	shape->tangentSpeed = surfaceMaterial.tangentSpeed;
	shape->userMaterialId = surfaceMaterial.userMaterialId;
	shape->customColor = surfaceMaterial.customColor;
}

b2Filter b2Shape_GetFilter( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->filter;
}

static void b2ResetProxy( b2World* world, b2Shape* shape, bool wakeBodies, bool destroyProxy )
{
	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );

	int shapeId = shape->id;

	// destroy all contacts associated with this shape
	int contactKey = body->headContactKey;
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
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
	bool destroyProxy = filter.categoryBits != shape->filter.categoryBits;

	shape->filter = filter;

	// need to wake bodies because a filter change may destroy contacts
	bool wakeBodies = true;
	b2ResetProxy( world, shape, wakeBodies, destroyProxy );

	// note: this does not immediately update sensor overlaps. Instead sensor
	// overlaps are updated the next time step
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

b2ChainSegment b2Shape_GetChainSegment( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	b2Shape* shape = b2GetShape( world, shapeId );
	B2_ASSERT( shape->type == b2_chainSegmentShape );
	return shape->chainSegment;
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
	if ( shape->type == b2_chainSegmentShape )
	{
		int chainId = shape->chainSegment.chainId;
		if ( chainId != B2_NULL_INDEX )
		{
			b2ChainShape* chain = b2ChainShapeArray_Get( &world->chainShapes, chainId );
			b2ChainId id = { chainId + 1, shapeId.world0, chain->generation };
			return id;
		}
	}

	return (b2ChainId){ 0 };
}

void b2Chain_SetFriction( b2ChainId chainId, float friction )
{
	B2_ASSERT( b2IsValidFloat( friction ) && friction >= 0.0f );

	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape( world, chainId );

	int materialCount = chainShape->materialCount;
	for ( int i = 0; i < materialCount; ++i )
	{
		chainShape->materials[i].friction = friction;
	}

	int count = chainShape->count;

	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chainShape->shapeIndices[i];
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shape->friction = friction;
	}
}

float b2Chain_GetFriction( b2ChainId chainId )
{
	b2World* world = b2GetWorld( chainId.world0 );
	b2ChainShape* chainShape = b2GetChainShape( world, chainId );
	return chainShape->materials[0].friction;
}

void b2Chain_SetRestitution( b2ChainId chainId, float restitution )
{
	B2_ASSERT( b2IsValidFloat( restitution ) );

	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape( world, chainId );

	int materialCount = chainShape->materialCount;
	for ( int i = 0; i < materialCount; ++i )
	{
		chainShape->materials[i].restitution = restitution;
	}

	int count = chainShape->count;

	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chainShape->shapeIndices[i];
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shape->restitution = restitution;
	}
}

float b2Chain_GetRestitution( b2ChainId chainId )
{
	b2World* world = b2GetWorld( chainId.world0 );
	b2ChainShape* chainShape = b2GetChainShape( world, chainId );
	return chainShape->materials[0].restitution;
}

void b2Chain_SetMaterial( b2ChainId chainId, int material )
{
	b2World* world = b2GetWorldLocked( chainId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape( world, chainId );
	int materialCount = chainShape->materialCount;
	for ( int i = 0; i < materialCount; ++i )
	{
		chainShape->materials[i].userMaterialId = material;
	}

	int count = chainShape->count;

	for ( int i = 0; i < count; ++i )
	{
		int shapeId = chainShape->shapeIndices[i];
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shape->userMaterialId = material;
	}
}

int b2Chain_GetMaterial( b2ChainId chainId )
{
	b2World* world = b2GetWorld( chainId.world0 );
	b2ChainShape* chainShape = b2GetChainShape( world, chainId );
	return chainShape->materials[0].userMaterialId;
}

int b2Shape_GetContactCapacity( b2ShapeId shapeId )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->sensorIndex != B2_NULL_INDEX )
	{
		return 0;
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );

	// Conservative and fast
	return body->contactCount;
}

int b2Shape_GetContactData( b2ShapeId shapeId, b2ContactData* contactData, int capacity )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->sensorIndex != B2_NULL_INDEX )
	{
		return 0;
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	int contactKey = body->headContactKey;
	int index = 0;
	while ( contactKey != B2_NULL_INDEX && index < capacity )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

		// Does contact involve this shape and is it touching?
		if ( ( contact->shapeIdA == shapeId.index1 - 1 || contact->shapeIdB == shapeId.index1 - 1 ) &&
			 ( contact->flags & b2_contactTouchingFlag ) != 0 )
		{
			b2Shape* shapeA = world->shapes.data + contact->shapeIdA;
			b2Shape* shapeB = world->shapes.data + contact->shapeIdB;

			contactData[index].shapeIdA = (b2ShapeId){ shapeA->id + 1, shapeId.world0, shapeA->generation };
			contactData[index].shapeIdB = (b2ShapeId){ shapeB->id + 1, shapeId.world0, shapeB->generation };

			b2ContactSim* contactSim = b2GetContactSim( world, contact );
			contactData[index].manifold = contactSim->manifold;
			index += 1;
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}

	B2_ASSERT( index <= capacity );

	return index;
}

int b2Shape_GetSensorCapacity( b2ShapeId shapeId )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->sensorIndex == B2_NULL_INDEX )
	{
		return 0;
	}

	b2Sensor* sensor = b2SensorArray_Get( &world->sensors, shape->sensorIndex );
	return sensor->overlaps2.count;
}

int b2Shape_GetSensorOverlaps( b2ShapeId shapeId, b2ShapeId* overlaps, int capacity )
{
	b2World* world = b2GetWorldLocked( shapeId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	if ( shape->sensorIndex == B2_NULL_INDEX )
	{
		return 0;
	}

	b2Sensor* sensor = b2SensorArray_Get( &world->sensors, shape->sensorIndex );

	int count = b2MinInt( sensor->overlaps2.count, capacity );
	b2ShapeRef* refs = sensor->overlaps2.data;
	for ( int i = 0; i < count; ++i )
	{
		overlaps[i] = (b2ShapeId){
			.index1 = refs[i].shapeId + 1,
			.world0 = shapeId.world0,
			.generation = refs[i].generation,
		};
	}

	return count;
}

b2AABB b2Shape_GetAABB( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return (b2AABB){ 0 };
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	return shape->aabb;
}

b2MassData b2Shape_GetMassData( b2ShapeId shapeId )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return (b2MassData){ 0 };
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	return b2ComputeShapeMass( shape );
}

b2Vec2 b2Shape_GetClosestPoint( b2ShapeId shapeId, b2Vec2 target )
{
	b2World* world = b2GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return (b2Vec2){ 0 };
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &target, 1, 0.0f );
	input.transformA = transform;
	input.transformB = b2Transform_identity;
	input.useRadii = true;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, NULL, 0 );

	return output.pointA;
}
