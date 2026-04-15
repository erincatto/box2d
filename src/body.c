// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"

#include "aabb.h"
#include "array.h"
#include "contact.h"
#include "container.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"

#include "box2d/box2d.h"
#include "box2d/id.h"

#include <string.h>

// Implement functions for b2BodyArray
B2_ARRAY_SOURCE( b2Body, b2Body )
B2_ARRAY_SOURCE( b2BodyState, b2BodyState )

static void b2LimitVelocity( b2Body* body, float maxLinearSpeed )
{
	float v2 = b2LengthSquared( body->linearVelocity );
	if ( v2 > maxLinearSpeed * maxLinearSpeed )
	{
		body->linearVelocity = b2MulSV( maxLinearSpeed / sqrtf( v2 ), body->linearVelocity );
	}
}

void b2RemoveBodyFromSet( b2SolverSet* set, b2BodyArray* bodies, int localIndex )
{
	B2_ASSERT( 0 <= localIndex && localIndex < set->bodyIds.count );
	int lastIndex = set->bodyIds.count - 1;
	set->bodyIds.data[localIndex] = set->bodyIds.data[lastIndex];
	b2Body* movedBody = b2BodyArray_Get( bodies, set->bodyIds.data[localIndex] );
	B2_ASSERT( movedBody->localIndex == lastIndex );
	movedBody->localIndex = localIndex;
	set->bodyIds.count -= 1;
}

// Get a validated body from a world using an id.
b2Body* b2GetBodyFullId( b2World* world, b2BodyId bodyId )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );

	// id index starts at one so that zero can represent null
	return b2BodyArray_Get( &world->bodies, bodyId.index1 - 1 );
}

b2Transform b2GetBodyTransformQuick( b2World* world, b2Body* body )
{
	B2_UNUSED( world );
	return body->transform;
}

b2Transform b2GetBodyTransform( b2World* world, int bodyId )
{
	b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
	return b2GetBodyTransformQuick( world, body );
}

// Create a b2BodyId from a raw id.
b2BodyId b2MakeBodyId( b2World* world, int bodyId )
{
	b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
	return (b2BodyId){ bodyId + 1, world->worldId, body->generation };
}

static void b2CreateIslandForBody( b2World* world, int setIndex, b2Body* body )
{
	B2_ASSERT( body->islandId == B2_NULL_INDEX );
	B2_ASSERT( setIndex != b2_disabledSet );

	b2Island* island = b2CreateIsland( world, setIndex );
	b2Array_Push( island->bodies, body->id );
	body->islandId = island->islandId;
	body->islandIndex = 0;

	b2ValidateIsland( world, island->islandId );
}

static void b2RemoveBodyFromIsland( b2World* world, b2Body* body )
{
	if ( body->islandId == B2_NULL_INDEX )
	{
		B2_ASSERT( body->islandIndex == B2_NULL_INDEX );
		return;
	}

	int islandId = body->islandId;
	b2Island* island = b2Array_Get( world->islands, islandId );
	{
		int localIndex = body->islandIndex;
		int movedBodyId = island->bodies.data[island->bodies.count - 1];
		island->bodies.data[localIndex] = movedBodyId;
		B2_VALIDATE( world->bodies.data[movedBodyId].islandIndex == island->bodies.count - 1 );
		world->bodies.data[movedBodyId].islandIndex = localIndex;
		island->bodies.count -= 1;
	}

	if ( island->bodies.count == 0 )
	{
		// Destroy empty island
		B2_ASSERT( island->contacts.count == 0 );
		B2_ASSERT( island->joints.count == 0 );

		// Free the island
		b2DestroyIsland( world, island->islandId );
	}
	else
	{
		b2ValidateIsland( world, islandId );
	}

	body->islandId = B2_NULL_INDEX;
	body->islandIndex = B2_NULL_INDEX;
}

static void b2DestroyBodyContacts( b2World* world, b2Body* body, bool wakeBodies )
{
	// Destroy the attached contacts
	int edgeKey = body->headContactKey;
	while ( edgeKey != B2_NULL_INDEX )
	{
		int contactId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
		edgeKey = contact->edges[edgeIndex].nextKey;
		b2DestroyContact( world, contact, wakeBodies );
	}

	b2ValidateSolverSets( world );
}

b2BodyId b2CreateBody( b2WorldId worldId, const b2BodyDef* def )
{
	B2_CHECK_DEF( def );
	B2_ASSERT( b2IsValidVec2( def->position ) );
	B2_ASSERT( b2IsValidRotation( def->rotation ) );
	B2_ASSERT( b2IsValidVec2( def->linearVelocity ) );
	B2_ASSERT( b2IsValidFloat( def->angularVelocity ) );
	B2_ASSERT( b2IsValidFloat( def->linearDamping ) && def->linearDamping >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->angularDamping ) && def->angularDamping >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->sleepThreshold ) && def->sleepThreshold >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def->gravityScale ) );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );

	if ( world->locked )
	{
		return b2_nullBodyId;
	}

	bool isAwake = ( def->isAwake || def->enableSleep == false ) && def->isEnabled;

	// determine the solver set
	int setId;
	if ( def->isEnabled == false )
	{
		// any body type can be disabled
		setId = b2_disabledSet;
	}
	else if ( def->type == b2_staticBody )
	{
		setId = b2_staticSet;
	}
	else if ( isAwake == true )
	{
		setId = b2_awakeSet;
	}
	else
	{
		// new set for a sleeping body in its own island
		setId = b2AllocId( &world->solverSetIdPool );
		if ( setId == world->solverSets.count )
		{
			// Create a zero initialized solver set. All sub-arrays are also zero initialized.
			b2SolverSetArray_Push( &world->solverSets, (b2SolverSet){ 0 } );
		}
		else
		{
			B2_ASSERT( world->solverSets.data[setId].setIndex == B2_NULL_INDEX );
		}

		world->solverSets.data[setId].setIndex = setId;
	}

	B2_ASSERT( 0 <= setId && setId < world->solverSets.count );

	int bodyId = b2AllocId( &world->bodyIdPool );

	if ( bodyId == world->bodies.count )
	{
		b2BodyArray_Push( &world->bodies, (b2Body){ 0 } );
	}
	else
	{
		B2_ASSERT( world->bodies.data[bodyId].id == B2_NULL_INDEX );
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
	uint16_t generation = body->generation;
	memset( body, 0, sizeof( b2Body ) );
	body->generation = generation;

	uint32_t lockFlags = 0;
	lockFlags |= def->motionLocks.linearX ? b2_lockLinearX : 0;
	lockFlags |= def->motionLocks.linearY ? b2_lockLinearY : 0;
	lockFlags |= def->motionLocks.angularZ ? b2_lockAngularZ : 0;

	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setId );
	b2Array_Push( set->bodyIds, bodyId );

	// if ( setId == b2_awakeSet )
	//{
	//	b2BodyState* bodyState = b2BodyStateArray_Add( &set->bodyStates );
	//	B2_ASSERT( ( (uintptr_t)bodyState & 0x1F ) == 0 );

	//	*bodyState = (b2BodyState){ 0 };
	//	bodyState->linearVelocity = def->linearVelocity;
	//	bodyState->angularVelocity = def->angularVelocity;
	//	bodyState->deltaRotation = b2Rot_identity;
	//	bodyState->flags = bodySim->flags;
	//}

	if ( def->name )
	{
		int i = 0;
		while ( i < B2_NAME_LENGTH - 1 && def->name[i] != 0 )
		{
			body->name[i] = def->name[i];
			i += 1;
		}

		while ( i < B2_NAME_LENGTH )
		{
			body->name[i] = 0;
			i += 1;
		}
	}
	else
	{
		memset( body->name, 0, B2_NAME_LENGTH * sizeof( char ) );
	}

	body->id = bodyId;
	body->transform.p = def->position;
	body->transform.q = def->rotation;
	body->center = def->position;
	body->rotation0 = body->transform.q;
	body->center0 = body->center;
	body->linearDamping = def->linearDamping;
	body->angularDamping = def->angularDamping;
	body->gravityScale = def->gravityScale;
	body->linearVelocity = def->linearVelocity;
	body->angularVelocity = def->angularVelocity;
	body->flags = lockFlags;
	body->flags |= def->isBullet ? b2_isBullet : 0;
	body->flags |= def->allowFastRotation ? b2_allowFastRotation : 0;
	body->flags |= def->type == b2_dynamicBody ? b2_dynamicFlag : 0;
	body->userData = def->userData;
	body->setIndex = setId;
	body->localIndex = set->bodyIds.count - 1;
	body->generation += 1;
	body->headShapeId = B2_NULL_INDEX;
	body->headChainId = B2_NULL_INDEX;
	body->headContactKey = B2_NULL_INDEX;
	body->headJointKey = B2_NULL_INDEX;
	body->islandId = B2_NULL_INDEX;
	body->islandIndex = B2_NULL_INDEX;
	body->bodyMoveIndex = B2_NULL_INDEX;
	body->sleepThreshold = def->sleepThreshold;
	body->minExtent = B2_HUGE;
	body->type = def->type;
	body->enableSleep = def->enableSleep;
	body->stateIndex = B2_NULL_INDEX;
	body->clusterIndex = B2_NULL_INDEX;
	body->previousClusterIndex = B2_NULL_INDEX;
	body->clusterLocalIndex = B2_NULL_INDEX;

	// dynamic and kinematic bodies that are enabled need a island
	if ( setId >= b2_awakeSet )
	{
		b2CreateIslandForBody( world, setId, body );

		if ( world->clusterManager.initialized )
		{
			b2FindAndLinkBodyCluster( world, body );
		}
	}

	b2ValidateSolverSets( world );

	b2BodyId id = { bodyId + 1, world->worldId, body->generation };
	return id;
}

bool b2WakeBody( b2World* world, b2Body* body )
{
	if ( body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, body->setIndex );
		b2ValidateSolverSets( world );
		return true;
	}

	return false;
}

void b2DestroyBody( b2BodyId bodyId )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	// Wake bodies attached to this body, even if this body is static.
	bool wakeBodies = true;

	// Destroy the attached joints
	int edgeKey = body->headJointKey;
	while ( edgeKey != B2_NULL_INDEX )
	{
		int jointId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		edgeKey = joint->edges[edgeIndex].nextKey;

		// Careful because this modifies the list being traversed
		b2DestroyJointInternal( world, joint, wakeBodies );
	}

	// Destroy all contacts attached to this body.
	b2DestroyBodyContacts( world, body, wakeBodies );

	// Destroy the attached shapes and their broad-phase proxies.
	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

		if ( shape->sensorIndex != B2_NULL_INDEX )
		{
			b2DestroySensor( world, shape );
		}

		b2DestroyShapeProxy( shape, &world->broadPhase );

		// Return shape to free list.
		b2FreeId( &world->shapeIdPool, shapeId );
		shape->id = B2_NULL_INDEX;

		shapeId = shape->nextShapeId;
	}

	// Destroy the attached chains. The associated shapes have already been destroyed above.
	int chainId = body->headChainId;
	while ( chainId != B2_NULL_INDEX )
	{
		b2ChainShape* chain = b2ChainShapeArray_Get( &world->chainShapes, chainId );

		b2FreeChainData( chain );

		// Return chain to free list.
		b2FreeId( &world->chainIdPool, chainId );
		chain->id = B2_NULL_INDEX;

		chainId = chain->nextChainId;
	}

	b2RemoveBodyFromIsland( world, body );

	// Remove from persistent cluster bodyIds
	if ( body->clusterLocalIndex != B2_NULL_INDEX )
	{
		b2ClusterUnlinkBody( world, body );
	}

	// Remove body sim from solver set that owns it
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, body->setIndex );
	b2RemoveBodyFromSet( set, &world->bodies, body->localIndex );

	if ( set->setIndex >= b2_firstSleepingSet && set->bodyIds.count == 0 )
	{
		// Remove sleeping solver set if it is empty
		b2DestroySolverSet( world, set->setIndex );
	}

	// Free body and id (preserve body generation)
	b2FreeId( &world->bodyIdPool, body->id );

	body->setIndex = B2_NULL_INDEX;
	body->localIndex = B2_NULL_INDEX;
	body->id = B2_NULL_INDEX;
	body->clusterIndex = B2_NULL_INDEX;
	body->previousClusterIndex = B2_NULL_INDEX;
	body->clusterLocalIndex = B2_NULL_INDEX;

	b2ValidateSolverSets( world );
}

int b2Body_GetContactCapacity( b2BodyId bodyId )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	// Conservative and fast
	return body->contactCount;
}

int b2Body_GetContactData( b2BodyId bodyId, b2ContactData* contactData, int capacity )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	int contactKey = body->headContactKey;
	int index = 0;
	while ( contactKey != B2_NULL_INDEX && index < capacity )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

		// Is contact touching?
		if ( contact->flags & b2_contactTouchingFlag )
		{
			b2Shape* shapeA = b2ShapeArray_Get( &world->shapes, contact->shapeIdA );
			b2Shape* shapeB = b2ShapeArray_Get( &world->shapes, contact->shapeIdB );

			contactData[index].contactId = (b2ContactId){ contact->contactId + 1, bodyId.world0, 0, contact->generation };
			contactData[index].shapeIdA = (b2ShapeId){ shapeA->id + 1, bodyId.world0, shapeA->generation };
			contactData[index].shapeIdB = (b2ShapeId){ shapeB->id + 1, bodyId.world0, shapeB->generation };

			contactData[index].manifold = contact->manifold;

			index += 1;
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}

	B2_ASSERT( index <= capacity );

	return index;
}

b2AABB b2Body_ComputeAABB( b2BodyId bodyId )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return (b2AABB){ 0 };
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body->headShapeId == B2_NULL_INDEX )
	{
		b2Transform transform = b2GetBodyTransform( world, body->id );
		return (b2AABB){ transform.p, transform.p };
	}

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, body->headShapeId );
	b2AABB aabb = shape->aabb;
	while ( shape->nextShapeId != B2_NULL_INDEX )
	{
		shape = b2ShapeArray_Get( &world->shapes, shape->nextShapeId );
		aabb = b2AABB_Union( aabb, shape->aabb );
	}

	return aabb;
}

void b2UpdateBodyMassData( b2World* world, b2Body* body )
{
	// Mass is no longer dirty
	body->flags &= ~b2_dirtyMass;

	// Compute mass data from shapes. Each shape has its own density.
	body->mass = 0.0f;
	body->inertia = 0.0f;

	body->invMass = 0.0f;
	body->invInertia = 0.0f;
	body->localCenter = b2Vec2_zero;
	body->minExtent = B2_HUGE;
	body->maxExtent = 0.0f;

	// Static and kinematic sims have zero mass.
	if ( body->type != b2_dynamicBody )
	{
		body->center = body->transform.p;
		body->center0 = body->center;

		// Need extents for kinematic bodies for sleeping to work correctly.
		if ( body->type == b2_kinematicBody )
		{
			int shapeId = body->headShapeId;
			while ( shapeId != B2_NULL_INDEX )
			{
				const b2Shape* s = b2ShapeArray_Get( &world->shapes, shapeId );

				b2ShapeExtent extent = b2ComputeShapeExtent( s, b2Vec2_zero );
				body->minExtent = b2MinFloat( body->minExtent, extent.minExtent );
				body->maxExtent = b2MaxFloat( body->maxExtent, extent.maxExtent );

				shapeId = s->nextShapeId;
			}
		}

		return;
	}

	int shapeCount = body->shapeCount;
	b2MassData* masses = b2AllocateArenaItem( &world->arena, shapeCount * sizeof( b2MassData ), "mass data" );

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int shapeId = body->headShapeId;
	int shapeIndex = 0;
	while ( shapeId != B2_NULL_INDEX )
	{
		const b2Shape* s = b2ShapeArray_Get( &world->shapes, shapeId );
		shapeId = s->nextShapeId;

		if ( s->density == 0.0f )
		{
			masses[shapeIndex] = (b2MassData){ 0 };
			shapeIndex += 1;
			continue;
		}

		b2MassData massData = b2ComputeShapeMass( s );
		body->mass += massData.mass;
		localCenter = b2MulAdd( localCenter, massData.mass, massData.center );

		masses[shapeIndex] = massData;
		shapeIndex += 1;
	}

	// Compute center of mass.
	if ( body->mass > 0.0f )
	{
		body->invMass = 1.0f / body->mass;
		localCenter = b2MulSV( body->invMass, localCenter );
	}

	// Second loop to accumulate the rotational inertia about the center of mass
	for ( shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
	{
		b2MassData massData = masses[shapeIndex];
		if ( massData.mass == 0.0f )
		{
			continue;
		}

		// Shift to center of mass. This is safe because it can only increase.
		b2Vec2 offset = b2Sub( localCenter, massData.center );
		float inertia = massData.rotationalInertia + massData.mass * b2Dot( offset, offset );
		body->inertia += inertia;
	}

	b2FreeArenaItem( &world->arena, masses );
	masses = NULL;

	B2_ASSERT( body->inertia >= 0.0f );

	if ( body->inertia > 0.0f )
	{
		body->invInertia = 1.0f / body->inertia;
	}
	else
	{
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = body->center;
	body->localCenter = localCenter;
	body->center = b2TransformPoint( body->transform, body->localCenter );
	body->center0 = body->center;

	// Update center of mass velocity
	b2Vec2 deltaLinear = b2CrossSV( body->angularVelocity, b2Sub( body->center, oldCenter ) );
	body->linearVelocity = b2Add( body->linearVelocity, deltaLinear );

	// Compute body extents relative to center of mass
	shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		const b2Shape* s = b2ShapeArray_Get( &world->shapes, shapeId );

		b2ShapeExtent extent = b2ComputeShapeExtent( s, localCenter );
		body->minExtent = b2MinFloat( body->minExtent, extent.minExtent );
		body->maxExtent = b2MaxFloat( body->maxExtent, extent.maxExtent );

		shapeId = s->nextShapeId;
	}
}

b2Vec2 b2Body_GetPosition( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return transform.p;
}

b2Rot b2Body_GetRotation( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return transform.q;
}

b2Transform b2Body_GetTransform( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return b2GetBodyTransformQuick( world, body );
}

b2Vec2 b2Body_GetLocalPoint( b2BodyId bodyId, b2Vec2 worldPoint )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2InvTransformPoint( transform, worldPoint );
}

b2Vec2 b2Body_GetWorldPoint( b2BodyId bodyId, b2Vec2 localPoint )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2TransformPoint( transform, localPoint );
}

b2Vec2 b2Body_GetLocalVector( b2BodyId bodyId, b2Vec2 worldVector )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2InvRotateVector( transform.q, worldVector );
}

b2Vec2 b2Body_GetWorldVector( b2BodyId bodyId, b2Vec2 localVector )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2RotateVector( transform.q, localVector );
}

void b2Body_SetTransform( b2BodyId bodyId, b2Vec2 position, b2Rot rotation )
{
	B2_ASSERT( b2IsValidVec2( position ) );
	B2_ASSERT( b2IsValidRotation( rotation ) );
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	b2World* world = b2GetWorld( bodyId.world0 );
	B2_ASSERT( world->locked == false );

	b2Body* body = b2GetBodyFullId( world, bodyId );

	body->transform.p = position;
	body->transform.q = rotation;
	body->center = b2TransformPoint( body->transform, body->localCenter );

	body->rotation0 = body->transform.q;
	body->center0 = body->center;

	b2BroadPhase* broadPhase = &world->broadPhase;

	b2Transform transform = body->transform;
	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;

	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		b2AABB aabb = b2ComputeShapeAABB( shape, transform );
		aabb.lowerBound.x -= speculativeDistance;
		aabb.lowerBound.y -= speculativeDistance;
		aabb.upperBound.x += speculativeDistance;
		aabb.upperBound.y += speculativeDistance;
		shape->aabb = aabb;

		if ( b2AABB_Contains( shape->fatAABB, aabb ) == false )
		{
			float margin = shape->aabbMargin;
			b2AABB fatAABB;
			fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
			fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
			fatAABB.upperBound.x = aabb.upperBound.x + margin;
			fatAABB.upperBound.y = aabb.upperBound.y + margin;
			shape->fatAABB = fatAABB;

			// They body could be disabled
			if ( shape->proxyKey != B2_NULL_INDEX )
			{
				b2BroadPhase_MoveProxy( broadPhase, shape->proxyKey, fatAABB );
			}
		}

		shapeId = shape->nextShapeId;
	}
}

b2Vec2 b2Body_GetLinearVelocity( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->linearVelocity;
}

float b2Body_GetAngularVelocity( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->angularVelocity;
}

void b2Body_SetLinearVelocity( b2BodyId bodyId, b2Vec2 linearVelocity )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body->type == b2_staticBody )
	{
		return;
	}

	body->linearVelocity = linearVelocity;
}

void b2Body_SetAngularVelocity( b2BodyId bodyId, float angularVelocity )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type == b2_staticBody || ( body->flags & b2_lockAngularZ ) )
	{
		return;
	}

	body->angularVelocity = angularVelocity;
}

void b2Body_SetTargetTransform( b2BodyId bodyId, b2Transform target, float timeStep, bool wake )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( body->type == b2_staticBody || timeStep <= 0.0f )
	{
		return;
	}

	if ( body->setIndex != b2_awakeSet && wake == false )
	{
		return;
	}

	// Compute linear velocity
	b2Vec2 center1 = body->center;
	b2Vec2 center2 = b2TransformPoint( target, body->localCenter );
	float invTimeStep = 1.0f / timeStep;
	b2Vec2 linearVelocity = b2MulSV( invTimeStep, b2Sub( center2, center1 ) );

	// Compute angular velocity
	b2Rot q1 = body->transform.q;
	b2Rot q2 = target.q;
	float deltaAngle = b2RelativeAngle( q1, q2 );
	float angularVelocity = invTimeStep * deltaAngle;

	// Early out if the body is asleep already and the desired movement is small
	if ( body->setIndex != b2_awakeSet )
	{
		float maxVelocity = b2Length( linearVelocity ) + b2AbsFloat( angularVelocity ) * body->maxExtent;

		// Return if velocity would be sleepy
		if ( maxVelocity < body->sleepThreshold )
		{
			return;
		}

		// Must wake for state to exist
		b2WakeBody( world, body );
	}

	B2_ASSERT( body->setIndex == b2_awakeSet );

	body->linearVelocity = linearVelocity;
	body->angularVelocity = angularVelocity;
}

b2Vec2 b2Body_GetLocalPointVelocity( b2BodyId bodyId, b2Vec2 localPoint )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Vec2 r = b2RotateVector( body->transform.q, b2Sub( localPoint, body->localCenter ) );
	b2Vec2 v = b2Add( body->linearVelocity, b2CrossSV( body->angularVelocity, r ) );
	return v;
}

b2Vec2 b2Body_GetWorldPointVelocity( b2BodyId bodyId, b2Vec2 worldPoint )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Vec2 r = b2Sub( worldPoint, body->center );
	b2Vec2 v = b2Add( body->linearVelocity, b2CrossSV( body->angularVelocity, r ) );
	return v;
}

void b2Body_ApplyForce( b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type != b2_dynamicBody || body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body->setIndex == b2_awakeSet )
	{
		body->force = b2Add( body->force, force );
		body->torque += b2Cross( b2Sub( point, body->center ), force );
	}
}

void b2Body_ApplyForceToCenter( b2BodyId bodyId, b2Vec2 force, bool wake )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type != b2_dynamicBody || body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body->setIndex == b2_awakeSet )
	{
		body->force = b2Add( body->force, force );
	}
}

void b2Body_ApplyTorque( b2BodyId bodyId, float torque, bool wake )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type != b2_dynamicBody || body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body->setIndex == b2_awakeSet )
	{
		body->torque += torque;
	}
}

void b2Body_ClearForces( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->force = b2Vec2_zero;
	body->torque = 0.0f;
}

void b2Body_ApplyLinearImpulse( b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type != b2_dynamicBody || body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body->setIndex == b2_awakeSet )
	{
		body->linearVelocity = b2MulAdd( body->linearVelocity, body->invMass, impulse );
		body->angularVelocity += body->invInertia * b2Cross( b2Sub( point, body->center ), impulse );

		b2LimitVelocity( body, world->maxLinearSpeed );
	}
}

void b2Body_ApplyLinearImpulseToCenter( b2BodyId bodyId, b2Vec2 impulse, bool wake )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type != b2_dynamicBody || body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body->setIndex == b2_awakeSet )
	{
		body->linearVelocity = b2MulAdd( body->linearVelocity, body->invMass, impulse );
		b2LimitVelocity( body, world->maxLinearSpeed );
	}
}

void b2Body_ApplyAngularImpulse( b2BodyId bodyId, float impulse, bool wake )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body->type != b2_dynamicBody || body->setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body->setIndex >= b2_firstSleepingSet )
	{
		// this will not invalidate body pointer
		b2WakeBody( world, body );
	}

	if ( body->setIndex == b2_awakeSet )
	{
		body->angularVelocity += body->invInertia * impulse;
	}
}

b2BodyType b2Body_GetType( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->type;
}

// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
// todo with a bit more effort I could support an option to let the body sleep
//
// Revised steps:
// 1 Skip disabled bodies
// 2 Destroy all contacts on the body
// 3 Wake the body
// 4 For all joints attached to the body
//  - wake attached bodies
//  - remove from island
//  - move to static set temporarily
// 5 Change the body type and transfer the body
// 6 If the body was static
//   - create an island for the body
//   Else if the body is becoming static
//   - remove it from the island
// 7 For all joints
//  - if either body is non-static
//    - link into island
//    - transfer to constraint graph
// 8 For all shapes
//  - Destroy proxy in old tree
//  - Create proxy in new tree
// Notes:
// - the implementation below tries to minimize the number of predicates, so some
//   operations may have no effect, such as transferring a joint to the same set
void b2Body_SetType( b2BodyId bodyId, b2BodyType type )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	b2BodyType originalType = body->type;
	if ( originalType == type )
	{
		return;
	}

	// Stage 1: skip disabled bodies
	if ( body->setIndex == b2_disabledSet )
	{
		// Disabled bodies don't change solver sets or islands when they change type.
		body->type = type;

		if ( type == b2_dynamicBody )
		{
			body->flags |= b2_dynamicFlag;
		}
		else
		{
			body->flags &= ~b2_dynamicFlag;
		}

		// Body type affects the mass properties
		b2UpdateBodyMassData( world, body );
		return;
	}

	// Stage 2: destroy all contacts but don't wake bodies (because we don't need to)
	bool wakeBodies = false;
	b2DestroyBodyContacts( world, body, wakeBodies );

	// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
	// all bodies in the same sleeping solver set.
	b2WakeBody( world, body );

	// Stage 4: move joints to temporary storage
	b2SolverSet* staticSet = b2SolverSetArray_Get( &world->solverSets, b2_staticSet );

	int jointKey = body->headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		jointKey = joint->edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if ( joint->setIndex == b2_disabledSet )
		{
			continue;
		}

		// Wake attached bodies. The b2WakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may have
		// no joints.
		b2Body* bodyA = b2BodyArray_Get( &world->bodies, joint->edges[0].bodyId );
		b2Body* bodyB = b2BodyArray_Get( &world->bodies, joint->edges[1].bodyId );
		b2WakeBody( world, bodyA );
		b2WakeBody( world, bodyB );

		// Remove joint from island
		b2UnlinkJoint( world, joint );

		// It is necessary to transfer all joints to the static set
		// so they can be added to the constraint graph below and acquire consistent colors.
		b2SolverSet* jointSourceSet = b2SolverSetArray_Get( &world->solverSets, joint->setIndex );
		b2TransferJoint( world, staticSet, jointSourceSet, joint );
	}

	// Stage 5: change the body type and transfer body
	body->type = type;

	if ( type == b2_dynamicBody )
	{
		body->flags |= b2_dynamicFlag;
	}
	else
	{
		body->flags &= ~b2_dynamicFlag;
	}

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2SolverSet* sourceSet = b2SolverSetArray_Get( &world->solverSets, body->setIndex );
	b2SolverSet* targetSet = type == b2_staticBody ? staticSet : awakeSet;

	// Transfer body
	b2TransferBody( world, targetSet, sourceSet, body );

	// Stage 6: update island participation for the body
	if ( originalType == b2_staticBody )
	{
		// Create island for body
		b2CreateIslandForBody( world, b2_awakeSet, body );

		// Assign cluster before stage 7 relinks joints (b2ClusterLinkJoint needs valid clusterIndex)
		b2FindAndLinkBodyCluster( world, body );
		b2SetBitGrow( &world->clusterManager.dirtyBodyBitSet, body->id );
	}
	else if ( type == b2_staticBody )
	{
		// Remove body from island.
		b2RemoveBodyFromIsland( world, body );

		// Remove from persistent cluster bodyIds
		if ( body->clusterLocalIndex != B2_NULL_INDEX )
		{
			b2ClusterUnlinkBody( world, body );
		}
		body->clusterIndex = B2_NULL_INDEX;
	}

	// Stage 7: Transfer joints to the target set
	jointKey = body->headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );

		jointKey = joint->edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if ( joint->setIndex == b2_disabledSet )
		{
			continue;
		}

		// All joints were transferred to the static set in an earlier stage
		B2_ASSERT( joint->setIndex == b2_staticSet );

		b2Body* bodyA = b2BodyArray_Get( &world->bodies, joint->edges[0].bodyId );
		b2Body* bodyB = b2BodyArray_Get( &world->bodies, joint->edges[1].bodyId );
		B2_ASSERT( bodyA->setIndex == b2_staticSet || bodyA->setIndex == b2_awakeSet );
		B2_ASSERT( bodyB->setIndex == b2_staticSet || bodyB->setIndex == b2_awakeSet );

		if ( bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody )
		{
			b2TransferJoint( world, awakeSet, staticSet, joint );
		}
	}

	// Recreate shape proxies in broadphase
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shapeId = shape->nextShapeId;
		b2DestroyShapeProxy( shape, &world->broadPhase );
		bool forcePairCreation = true;
		b2CreateShapeProxy( shape, &world->broadPhase, type, transform, forcePairCreation );
	}

	// Relink all joints
	jointKey = body->headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		jointKey = joint->edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		int otherBodyId = joint->edges[otherEdgeIndex].bodyId;
		b2Body* otherBody = b2BodyArray_Get( &world->bodies, otherBodyId );

		if ( otherBody->setIndex == b2_disabledSet )
		{
			continue;
		}

		if ( body->type != b2_dynamicBody && otherBody->type != b2_dynamicBody )
		{
			continue;
		}

		b2LinkJoint( world, joint );
	}

	// Body type affects the mass
	b2UpdateBodyMassData( world, body );

	b2ValidateSolverSets( world );
	b2ValidateIsland( world, body->islandId );
}

void b2Body_SetName( b2BodyId bodyId, const char* name )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( name )
	{
		int i = 0;
		while ( i < B2_NAME_LENGTH - 1 && name[i] != 0 )
		{
			body->name[i] = name[i];
			i += 1;
		}

		while ( i < B2_NAME_LENGTH )
		{
			body->name[i] = 0;
			i += 1;
		}
	}
	else
	{
		memset( body->name, 0, B2_NAME_LENGTH * sizeof( char ) );
	}
}

const char* b2Body_GetName( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->name;
}

void b2Body_SetUserData( b2BodyId bodyId, void* userData )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->userData = userData;
}

void* b2Body_GetUserData( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->userData;
}

float b2Body_GetMass( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->mass;
}

float b2Body_GetRotationalInertia( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->inertia;
}

b2Vec2 b2Body_GetLocalCenterOfMass( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->localCenter;
}

b2Vec2 b2Body_GetWorldCenterOfMass( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->center;
}

void b2Body_SetMassData( b2BodyId bodyId, b2MassData massData )
{
	B2_ASSERT( b2IsValidFloat( massData.mass ) && massData.mass >= 0.0f );
	B2_ASSERT( b2IsValidFloat( massData.rotationalInertia ) && massData.rotationalInertia >= 0.0f );
	B2_ASSERT( b2IsValidVec2( massData.center ) );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	body->mass = massData.mass;
	body->inertia = massData.rotationalInertia;
	body->localCenter = massData.center;

	b2Vec2 center = b2TransformPoint( body->transform, massData.center );
	body->center = center;
	body->center0 = center;

	body->invMass = body->mass > 0.0f ? 1.0f / body->mass : 0.0f;
	body->invInertia = body->inertia > 0.0f ? 1.0f / body->inertia : 0.0f;
}

b2MassData b2Body_GetMassData( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2MassData massData = { body->mass, body->localCenter, body->inertia };
	return massData;
}

void b2Body_ApplyMassFromShapes( b2BodyId bodyId )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2UpdateBodyMassData( world, body );
}

void b2Body_SetLinearDamping( b2BodyId bodyId, float linearDamping )
{
	B2_ASSERT( b2IsValidFloat( linearDamping ) && linearDamping >= 0.0f );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->linearDamping = linearDamping;
}

float b2Body_GetLinearDamping( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->linearDamping;
}

void b2Body_SetAngularDamping( b2BodyId bodyId, float angularDamping )
{
	B2_ASSERT( b2IsValidFloat( angularDamping ) && angularDamping >= 0.0f );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->angularDamping = angularDamping;
}

float b2Body_GetAngularDamping( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->angularDamping;
}

void b2Body_SetGravityScale( b2BodyId bodyId, float gravityScale )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	B2_ASSERT( b2IsValidFloat( gravityScale ) );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->gravityScale = gravityScale;
}

float b2Body_GetGravityScale( b2BodyId bodyId )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->gravityScale;
}

bool b2Body_IsAwake( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->setIndex == b2_awakeSet;
}

void b2Body_SetAwake( b2BodyId bodyId, bool awake )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( awake && body->setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}
	else if ( awake == false && body->setIndex == b2_awakeSet )
	{
		b2Island* island = b2Array_Get( world->islands, body->islandId );
		if ( island->constraintRemoveCount > 0 )
		{
			// Must split the island before sleeping. This is expensive.
			b2SplitIsland( world, body->islandId );
		}

		b2TrySleepIsland( world, body->islandId );
	}
}

void b2Body_WakeTouching( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	int contactKey = body->headContactKey;
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
		b2Shape* shapeA = b2ShapeArray_Get( &world->shapes, contact->shapeIdA );
		b2Shape* shapeB = b2ShapeArray_Get( &world->shapes, contact->shapeIdB );

		if ( shapeA->bodyId == bodyId.index1 - 1 )
		{
			b2Body* otherBody = b2BodyArray_Get( &world->bodies, shapeB->bodyId );
			b2WakeBody( world, otherBody );
		}
		else
		{
			b2Body* otherBody = b2BodyArray_Get( &world->bodies, shapeA->bodyId );
			b2WakeBody( world, otherBody );
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}
}

bool b2Body_IsEnabled( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->setIndex != b2_disabledSet;
}

bool b2Body_IsSleepEnabled( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->enableSleep;
}

void b2Body_SetSleepThreshold( b2BodyId bodyId, float sleepThreshold )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->sleepThreshold = sleepThreshold;
}

float b2Body_GetSleepThreshold( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->sleepThreshold;
}

void b2Body_EnableSleep( b2BodyId bodyId, bool enableSleep )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->enableSleep = enableSleep;

	if ( enableSleep == false )
	{
		b2WakeBody( world, body );
	}
}

// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
// The most challenging aspect is that joints may connect to bodies that are not disabled.
void b2Body_Disable( b2BodyId bodyId )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body->setIndex == b2_disabledSet )
	{
		return;
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	bool wakeBodies = true;
	b2DestroyBodyContacts( world, body, wakeBodies );

	// The current solver set of the body
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, body->setIndex );

	// Disabled bodies and connected joints are moved to the disabled set
	b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSets, b2_disabledSet );

	// Unlink joints and transfer them to the disabled set
	int jointKey = body->headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		jointKey = joint->edges[edgeIndex].nextKey;

		// joint may already be disabled by other body
		if ( joint->setIndex == b2_disabledSet )
		{
			continue;
		}

		B2_ASSERT( joint->setIndex == set->setIndex || set->setIndex == b2_staticSet );

		// Remove joint from island
		b2UnlinkJoint( world, joint );

		// Transfer joint to disabled set
		b2SolverSet* jointSet = b2SolverSetArray_Get( &world->solverSets, joint->setIndex );
		b2TransferJoint( world, disabledSet, jointSet, joint );
	}

	// Remove shapes from broad-phase
	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shapeId = shape->nextShapeId;
		b2DestroyShapeProxy( shape, &world->broadPhase );
	}

	// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
	b2RemoveBodyFromIsland( world, body );

	// Transfer body sim
	b2TransferBody( world, disabledSet, set, body );

	b2ValidateConnectivity( world );
	b2ValidateSolverSets( world );
}

void b2Body_Enable( b2BodyId bodyId )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body->setIndex != b2_disabledSet )
	{
		return;
	}

	b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSets, b2_disabledSet );
	int setId = body->type == b2_staticBody ? b2_staticSet : b2_awakeSet;
	b2SolverSet* targetSet = b2SolverSetArray_Get( &world->solverSets, setId );

	b2TransferBody( world, targetSet, disabledSet, body );

	b2Transform transform = b2GetBodyTransformQuick( world, body );

	// Add shapes to broad-phase
	b2BodyType proxyType = body->type;
	bool forcePairCreation = true;
	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shapeId = shape->nextShapeId;

		b2CreateShapeProxy( shape, &world->broadPhase, proxyType, transform, forcePairCreation );
	}

	if ( setId != b2_staticSet )
	{
		b2CreateIslandForBody( world, setId, body );
	}

	// Transfer joints. If the other body is disabled, don't transfer.
	// If the other body is sleeping, wake it.
	int jointKey = body->headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		B2_ASSERT( joint->setIndex == b2_disabledSet );
		B2_ASSERT( joint->islandId == B2_NULL_INDEX );

		jointKey = joint->edges[edgeIndex].nextKey;

		b2Body* bodyA = b2BodyArray_Get( &world->bodies, joint->edges[0].bodyId );
		b2Body* bodyB = b2BodyArray_Get( &world->bodies, joint->edges[1].bodyId );

		if ( bodyA->setIndex == b2_disabledSet || bodyB->setIndex == b2_disabledSet )
		{
			// one body is still disabled
			continue;
		}

		// Transfer joint first
		int jointSetId;
		if ( bodyA->setIndex == b2_staticSet && bodyB->setIndex == b2_staticSet )
		{
			jointSetId = b2_staticSet;
		}
		else if ( bodyA->setIndex == b2_staticSet )
		{
			jointSetId = bodyB->setIndex;
		}
		else
		{
			jointSetId = bodyA->setIndex;
		}

		b2SolverSet* jointSet = b2SolverSetArray_Get( &world->solverSets, jointSetId );
		b2TransferJoint( world, jointSet, disabledSet, joint );

		// Now that the joint is in the correct set, I can link the joint in the island.
		if ( jointSetId != b2_staticSet )
		{
			b2LinkJoint( world, joint );
		}
	}

	b2ValidateSolverSets( world );
}

void b2Body_SetMotionLocks( b2BodyId bodyId, b2MotionLocks locks )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	uint32_t newFlags = 0;
	newFlags |= locks.linearX ? b2_lockLinearX : 0;
	newFlags |= locks.linearY ? b2_lockLinearY : 0;
	newFlags |= locks.angularZ ? b2_lockAngularZ : 0;

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( ( body->flags & b2_allLocks ) != newFlags )
	{
		body->flags &= ~b2_allLocks;
		body->flags |= newFlags;

		if ( locks.linearX )
		{
			body->linearVelocity.x = 0.0f;
		}

		if ( locks.linearY )
		{
			body->linearVelocity.y = 0.0f;
		}

		if ( locks.angularZ )
		{
			body->angularVelocity = 0.0f;
		}
	}
}

b2MotionLocks b2Body_GetMotionLocks( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	b2MotionLocks locks;
	locks.linearX = ( body->flags & b2_lockLinearX );
	locks.linearY = ( body->flags & b2_lockLinearY );
	locks.angularZ = ( body->flags & b2_lockAngularZ );
	return locks;
}

void b2Body_SetBullet( b2BodyId bodyId, bool flag )
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( flag )
	{
		body->flags |= b2_isBullet;
	}
	else
	{
		body->flags &= ~b2_isBullet;
	}
}

bool b2Body_IsBullet( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return ( body->flags & b2_isBullet ) != 0;
}

void b2Body_EnableContactEvents( b2BodyId bodyId, bool flag )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shape->enableContactEvents = flag;
		shapeId = shape->nextShapeId;
	}
}

void b2Body_EnableHitEvents( b2BodyId bodyId, bool flag )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int shapeId = body->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		shape->enableHitEvents = flag;
		shapeId = shape->nextShapeId;
	}
}

b2WorldId b2Body_GetWorld( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	return (b2WorldId){ bodyId.world0 + 1, world->generation };
}

int b2Body_GetShapeCount( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->shapeCount;
}

int b2Body_GetShapes( b2BodyId bodyId, b2ShapeId* shapeArray, int capacity )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int shapeId = body->headShapeId;
	int shapeCount = 0;
	while ( shapeId != B2_NULL_INDEX && shapeCount < capacity )
	{
		b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
		b2ShapeId id = { shape->id + 1, bodyId.world0, shape->generation };
		shapeArray[shapeCount] = id;
		shapeCount += 1;

		shapeId = shape->nextShapeId;
	}

	return shapeCount;
}

int b2Body_GetJointCount( b2BodyId bodyId )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body->jointCount;
}

int b2Body_GetJoints( b2BodyId bodyId, b2JointId* jointArray, int capacity )
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int jointKey = body->headJointKey;

	int jointCount = 0;
	while ( jointKey != B2_NULL_INDEX && jointCount < capacity )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );

		b2JointId id = { jointId + 1, bodyId.world0, joint->generation };
		jointArray[jointCount] = id;
		jointCount += 1;

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return jointCount;
}

bool b2ShouldBodiesCollide( b2World* world, b2Body* bodyA, b2Body* bodyB )
{
	if ( bodyA->type != b2_dynamicBody && bodyB->type != b2_dynamicBody )
	{
		return false;
	}

	int jointKey;
	int otherBodyId;
	if ( bodyA->jointCount < bodyB->jointCount )
	{
		jointKey = bodyA->headJointKey;
		otherBodyId = bodyB->id;
	}
	else
	{
		jointKey = bodyB->headJointKey;
		otherBodyId = bodyA->id;
	}

	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;
		int otherEdgeIndex = edgeIndex ^ 1;

		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		if ( joint->collideConnected == false && joint->edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			return false;
		}

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return true;
}
