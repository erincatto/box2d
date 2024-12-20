// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"

#include "array.h"
#include "body.h"
#include "core.h"
#include "island.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"
#include "world.h"

#include "box2d/collision.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

B2_ARRAY_SOURCE( b2Contact, b2Contact );
B2_ARRAY_SOURCE( b2ContactSim, b2ContactSim );

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b2Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b2UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

static inline float b2MixFloats( float value1, float value2, b2MixingRule mixingRule )
{
	switch ( mixingRule )
	{
		case b2_mixAverage:
			return 0.5f * ( value1 + value2 );

		case b2_mixGeometricMean:
			return sqrtf( value1 * value2 );

		case b2_mixMultiply:
			return value1 * value2;

		case b2_mixMinimum:
			return value1 < value2 ? value1 : value2;

		case b2_mixMaximum:
			return value1 > value2 ? value1 : value2;

		default:
			B2_ASSERT( false );
			return 0.0f;
	}
}

// todo make relative for all
// typedef b2Manifold b2ManifoldFcn(const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xfB, b2SimplexCache* cache);
typedef b2Manifold b2ManifoldFcn( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
								  b2SimplexCache* cache );

struct b2ContactRegister
{
	b2ManifoldFcn* fcn;
	bool primary;
};

static struct b2ContactRegister s_registers[b2_shapeTypeCount][b2_shapeTypeCount];
static bool s_initialized = false;

static b2Manifold b2CircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideCircles( &shapeA->circle, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2CapsuleAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideCapsuleAndCircle( &shapeA->capsule, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2CapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									 b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideCapsules( &shapeA->capsule, xfA, &shapeB->capsule, xfB );
}

static b2Manifold b2PolygonAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollidePolygonAndCircle( &shapeA->polygon, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2PolygonAndCapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											   b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollidePolygonAndCapsule( &shapeA->polygon, xfA, &shapeB->capsule, xfB );
}

static b2Manifold b2PolygonManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									 b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollidePolygons( &shapeA->polygon, xfA, &shapeB->polygon, xfB );
}

static b2Manifold b2SegmentAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSegmentAndCircle( &shapeA->segment, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2SegmentAndCapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											   b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSegmentAndCapsule( &shapeA->segment, xfA, &shapeB->capsule, xfB );
}

static b2Manifold b2SegmentAndPolygonManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											   b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSegmentAndPolygon( &shapeA->segment, xfA, &shapeB->polygon, xfB );
}

static b2Manifold b2ChainSegmentAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
												   b2SimplexCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideChainSegmentAndCircle( &shapeA->chainSegment, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2ChainSegmentAndCapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB,
													b2Transform xfB, b2SimplexCache* cache )
{
	return b2CollideChainSegmentAndCapsule( &shapeA->chainSegment, xfA, &shapeB->capsule, xfB, cache );
}

static b2Manifold b2ChainSegmentAndPolygonManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB,
													b2Transform xfB, b2SimplexCache* cache )
{
	return b2CollideChainSegmentAndPolygon( &shapeA->chainSegment, xfA, &shapeB->polygon, xfB, cache );
}

static void b2AddType( b2ManifoldFcn* fcn, b2ShapeType type1, b2ShapeType type2 )
{
	B2_ASSERT( 0 <= type1 && type1 < b2_shapeTypeCount );
	B2_ASSERT( 0 <= type2 && type2 < b2_shapeTypeCount );

	s_registers[type1][type2].fcn = fcn;
	s_registers[type1][type2].primary = true;

	if ( type1 != type2 )
	{
		s_registers[type2][type1].fcn = fcn;
		s_registers[type2][type1].primary = false;
	}
}

void b2InitializeContactRegisters( void )
{
	if ( s_initialized == false )
	{
		b2AddType( b2CircleManifold, b2_circleShape, b2_circleShape );
		b2AddType( b2CapsuleAndCircleManifold, b2_capsuleShape, b2_circleShape );
		b2AddType( b2CapsuleManifold, b2_capsuleShape, b2_capsuleShape );
		b2AddType( b2PolygonAndCircleManifold, b2_polygonShape, b2_circleShape );
		b2AddType( b2PolygonAndCapsuleManifold, b2_polygonShape, b2_capsuleShape );
		b2AddType( b2PolygonManifold, b2_polygonShape, b2_polygonShape );
		b2AddType( b2SegmentAndCircleManifold, b2_segmentShape, b2_circleShape );
		b2AddType( b2SegmentAndCapsuleManifold, b2_segmentShape, b2_capsuleShape );
		b2AddType( b2SegmentAndPolygonManifold, b2_segmentShape, b2_polygonShape );
		b2AddType( b2ChainSegmentAndCircleManifold, b2_chainSegmentShape, b2_circleShape );
		b2AddType( b2ChainSegmentAndCapsuleManifold, b2_chainSegmentShape, b2_capsuleShape );
		b2AddType( b2ChainSegmentAndPolygonManifold, b2_chainSegmentShape, b2_polygonShape );
		s_initialized = true;
	}
}

void b2CreateContact( b2World* world, b2Shape* shapeA, b2Shape* shapeB )
{
	b2ShapeType type1 = shapeA->type;
	b2ShapeType type2 = shapeB->type;

	B2_ASSERT( 0 <= type1 && type1 < b2_shapeTypeCount );
	B2_ASSERT( 0 <= type2 && type2 < b2_shapeTypeCount );

	if ( s_registers[type1][type2].fcn == NULL )
	{
		// For example, no segment vs segment collision
		return;
	}

	if ( s_registers[type1][type2].primary == false )
	{
		// flip order
		b2CreateContact( world, shapeB, shapeA );
		return;
	}

	b2Body* bodyA = b2BodyArray_Get( &world->bodies, shapeA->bodyId );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, shapeB->bodyId );

	B2_ASSERT( bodyA->setIndex != b2_disabledSet && bodyB->setIndex != b2_disabledSet );
	B2_ASSERT( bodyA->setIndex != b2_staticSet || bodyB->setIndex != b2_staticSet );

	int setIndex;
	if ( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet )
	{
		setIndex = b2_awakeSet;
	}
	else
	{
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island
		setIndex = b2_disabledSet;
	}

	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );

	// Create contact key and contact
	int contactId = b2AllocId( &world->contactIdPool );
	if ( contactId == world->contacts.count )
	{
		b2ContactArray_Push( &world->contacts, ( b2Contact ){ 0 } );
	}

	int shapeIdA = shapeA->id;
	int shapeIdB = shapeB->id;

	b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
	contact->contactId = contactId;
	contact->setIndex = setIndex;
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = set->contactSims.count;
	contact->islandId = B2_NULL_INDEX;
	contact->islandPrev = B2_NULL_INDEX;
	contact->islandNext = B2_NULL_INDEX;
	contact->shapeIdA = shapeIdA;
	contact->shapeIdB = shapeIdB;
	contact->isMarked = false;
	contact->flags = 0;

	if ( shapeA->isSensor || shapeB->isSensor )
	{
		contact->flags |= b2_contactSensorFlag;
	}

	if ( shapeA->enableSensorEvents || shapeB->enableSensorEvents )
	{
		contact->flags |= b2_contactEnableSensorEvents;
	}

	if ( shapeA->enableContactEvents || shapeB->enableContactEvents )
	{
		contact->flags |= b2_contactEnableContactEvents;
	}

	// Connect to body A
	{
		contact->edges[0].bodyId = shapeA->bodyId;
		contact->edges[0].prevKey = B2_NULL_INDEX;
		contact->edges[0].nextKey = bodyA->headContactKey;

		int keyA = ( contactId << 1 ) | 0;
		int headContactKey = bodyA->headContactKey;
		if ( headContactKey != B2_NULL_INDEX )
		{
			b2Contact* headContact = b2ContactArray_Get( &world->contacts, headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyA;
		}
		bodyA->headContactKey = keyA;
		bodyA->contactCount += 1;
	}

	// Connect to body B
	{
		contact->edges[1].bodyId = shapeB->bodyId;
		contact->edges[1].prevKey = B2_NULL_INDEX;
		contact->edges[1].nextKey = bodyB->headContactKey;

		int keyB = ( contactId << 1 ) | 1;
		int headContactKey = bodyB->headContactKey;
		if ( bodyB->headContactKey != B2_NULL_INDEX )
		{
			b2Contact* headContact = b2ContactArray_Get( &world->contacts, headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyB;
		}
		bodyB->headContactKey = keyB;
		bodyB->contactCount += 1;
	}

	// Add to pair set for fast lookup
	uint64_t pairKey = B2_SHAPE_PAIR_KEY( shapeIdA, shapeIdB );
	b2AddKey( &world->broadPhase.pairSet, pairKey );

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	b2ContactSim* contactSim = b2ContactSimArray_Add( &set->contactSims );
	contactSim->contactId = contactId;

#if B2_VALIDATE
	contactSim->bodyIdA = shapeA->bodyId;
	contactSim->bodyIdB = shapeB->bodyId;
#endif

	contactSim->bodySimIndexA = B2_NULL_INDEX;
	contactSim->bodySimIndexB = B2_NULL_INDEX;
	contactSim->invMassA = 0.0f;
	contactSim->invIA = 0.0f;
	contactSim->invMassB = 0.0f;
	contactSim->invIB = 0.0f;
	contactSim->shapeIdA = shapeIdA;
	contactSim->shapeIdB = shapeIdB;
	contactSim->cache = b2_emptySimplexCache;
	contactSim->manifold = ( b2Manifold ){ 0 };
	contactSim->friction = b2MixFloats( shapeA->friction, shapeB->friction, world->frictionMixingRule );
	contactSim->restitution = b2MixFloats( shapeA->restitution, shapeB->restitution, world->restitutionMixingRule );
	contactSim->tangentSpeed = 0.0f;
	contactSim->simFlags = 0;

	if ( shapeA->enablePreSolveEvents || shapeB->enablePreSolveEvents )
	{
		contactSim->simFlags |= b2_simEnablePreSolveEvents;
	}
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
// - a shape becomes a sensor (check this!!!)
void b2DestroyContact( b2World* world, b2Contact* contact, bool wakeBodies )
{
	// Remove pair from set
	uint64_t pairKey = B2_SHAPE_PAIR_KEY( contact->shapeIdA, contact->shapeIdB );
	b2RemoveKey( &world->broadPhase.pairSet, pairKey );

	b2ContactEdge* edgeA = contact->edges + 0;
	b2ContactEdge* edgeB = contact->edges + 1;

	int bodyIdA = edgeA->bodyId;
	int bodyIdB = edgeB->bodyId;
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

	uint32_t flags = contact->flags;
	if ( ( flags & ( b2_contactTouchingFlag | b2_contactSensorTouchingFlag ) ) != 0 &&
		 ( flags & ( b2_contactEnableContactEvents | b2_contactEnableSensorEvents ) ) != 0 )
	{
		int16_t worldId = world->worldId;
		const b2Shape* shapeA = b2ShapeArray_Get( &world->shapes, contact->shapeIdA );
		const b2Shape* shapeB = b2ShapeArray_Get( &world->shapes, contact->shapeIdB );
		b2ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->revision };
		b2ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->revision };

		// Was touching?
		if ( ( flags & b2_contactTouchingFlag ) != 0 && ( flags & b2_contactEnableContactEvents ) != 0 )
		{
			B2_ASSERT( ( flags & b2_contactSensorFlag ) == 0 );
			b2ContactEndTouchEvent event = { shapeIdA, shapeIdB };
			b2ContactEndTouchEventArray_Push( world->contactEndEvents + world->endEventArrayIndex, event );
		}

		if ( ( flags & b2_contactSensorTouchingFlag ) != 0 && ( flags & b2_contactEnableSensorEvents ) != 0 )
		{
			B2_ASSERT( ( flags & b2_contactSensorFlag ) != 0 );
			B2_ASSERT( shapeA->isSensor == true || shapeB->isSensor == true );
			B2_ASSERT( shapeA->isSensor != shapeB->isSensor );

			b2SensorEndTouchEvent event;
			if (shapeA->isSensor)
			{
				event.sensorShapeId = shapeIdA;
				event.visitorShapeId = shapeIdB;
			}
			else
			{
				event.sensorShapeId = shapeIdB;
				event.visitorShapeId = shapeIdA;
			}
			b2SensorEndTouchEventArray_Push( world->sensorEndEvents + world->endEventArrayIndex, event );
		}
	}

	// Remove from body A
	if ( edgeA->prevKey != B2_NULL_INDEX )
	{
		b2Contact* prevContact = b2ContactArray_Get( &world->contacts, edgeA->prevKey >> 1 );
		b2ContactEdge* prevEdge = prevContact->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B2_NULL_INDEX )
	{
		b2Contact* nextContact = b2ContactArray_Get( &world->contacts, edgeA->nextKey >> 1 );
		b2ContactEdge* nextEdge = nextContact->edges + ( edgeA->nextKey & 1 );
		nextEdge->prevKey = edgeA->prevKey;
	}

	int contactId = contact->contactId;

	int edgeKeyA = ( contactId << 1 ) | 0;
	if ( bodyA->headContactKey == edgeKeyA )
	{
		bodyA->headContactKey = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if ( edgeB->prevKey != B2_NULL_INDEX )
	{
		b2Contact* prevContact = b2ContactArray_Get( &world->contacts, edgeB->prevKey >> 1 );
		b2ContactEdge* prevEdge = prevContact->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B2_NULL_INDEX )
	{
		b2Contact* nextContact = b2ContactArray_Get( &world->contacts, edgeB->nextKey >> 1 );
		b2ContactEdge* nextEdge = nextContact->edges + ( edgeB->nextKey & 1 );
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = ( contactId << 1 ) | 1;
	if ( bodyB->headContactKey == edgeKeyB )
	{
		bodyB->headContactKey = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	// Remove contact from the array that owns it
	if ( contact->islandId != B2_NULL_INDEX )
	{
		b2UnlinkContact( world, contact );
	}

	if ( contact->colorIndex != B2_NULL_INDEX )
	{
		// contact is an active constraint
		B2_ASSERT( contact->setIndex == b2_awakeSet );
		b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, contact->colorIndex, contact->localIndex );
	}
	else
	{
		// contact is non-touching or is sleeping or is a sensor
		B2_ASSERT( contact->setIndex != b2_awakeSet || ( contact->flags & b2_contactTouchingFlag ) == 0 ||
				   ( contact->flags & b2_contactSensorFlag ) != 0 );
		b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, contact->setIndex );
		int movedIndex = b2ContactSimArray_RemoveSwap( &set->contactSims, contact->localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			b2ContactSim* movedContactSim = set->contactSims.data + contact->localIndex;
			b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactSim->contactId );
			movedContact->localIndex = contact->localIndex;
		}
	}

	contact->contactId = B2_NULL_INDEX;
	contact->setIndex = B2_NULL_INDEX;
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = B2_NULL_INDEX;

	b2FreeId( &world->contactIdPool, contactId );

	if ( wakeBodies )
	{
		b2WakeBody( world, bodyA );
		b2WakeBody( world, bodyB );
	}
}

b2ContactSim* b2GetContactSim( b2World* world, b2Contact* contact )
{
	if ( contact->setIndex == b2_awakeSet && contact->colorIndex != B2_NULL_INDEX )
	{
		// contact lives in constraint graph
		B2_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = world->constraintGraph.colors + contact->colorIndex;
		return b2ContactSimArray_Get( &color->contactSims, contact->localIndex );
	}

	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, contact->setIndex );
	return b2ContactSimArray_Get( &set->contactSims, contact->localIndex );
}

bool b2ShouldShapesCollide( b2Filter filterA, b2Filter filterB )
{
	if ( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 )
	{
		return filterA.groupIndex > 0;
	}

	bool collide = ( filterA.maskBits & filterB.categoryBits ) != 0 && ( filterA.categoryBits & filterB.maskBits ) != 0;
	return collide;
}

static bool b2TestShapeOverlap( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
								b2SimplexCache* cache )
{
	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shapeA );
	input.proxyB = b2MakeShapeDistanceProxy( shapeB );
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	b2DistanceOutput output = b2ShapeDistance( cache, &input, NULL, 0 );

	return output.distance < 10.0f * FLT_EPSILON;
}

// Update the contact manifold and touching status. Also updates sensor overlap.
// Note: do not assume the shape AABBs are overlapping or are valid.
bool b2UpdateContact( b2World* world, b2ContactSim* contactSim, b2Shape* shapeA, b2Transform transformA, b2Vec2 centerOffsetA,
					  b2Shape* shapeB, b2Transform transformB, b2Vec2 centerOffsetB )
{
	bool touching;

	// Is this contact a sensor?
	if ( shapeA->isSensor || shapeB->isSensor )
	{
		// Sensors don't generate manifolds or hit events
		touching = b2TestShapeOverlap( shapeA, transformA, shapeB, transformB, &contactSim->cache );
	}
	else
	{
		// Save old manifold
		b2Manifold oldManifold = contactSim->manifold;

		// Compute new manifold
		b2ManifoldFcn* fcn = s_registers[shapeA->type][shapeB->type].fcn;
		contactSim->manifold = fcn( shapeA, transformA, shapeB, transformB, &contactSim->cache );

		int pointCount = contactSim->manifold.pointCount;
		touching = pointCount > 0;

		if ( touching && world->preSolveFcn && ( contactSim->simFlags & b2_simEnablePreSolveEvents ) != 0 )
		{
			b2ShapeId shapeIdA = { shapeA->id + 1, world->worldId, shapeA->revision };
			b2ShapeId shapeIdB = { shapeB->id + 1, world->worldId, shapeB->revision };

			// this call assumes thread safety
			touching = world->preSolveFcn( shapeIdA, shapeIdB, &contactSim->manifold, world->preSolveContext );
			if ( touching == false )
			{
				// disable contact
				pointCount = 0;
				contactSim->manifold.pointCount = 0;
			}
		}

		// This flag is for testing
		if (world->enableSpeculative == false && pointCount == 2)
		{
			if ( contactSim->manifold.points[0].separation > 1.5f * B2_LINEAR_SLOP )
			{
				contactSim->manifold.points[0] = contactSim->manifold.points[1];
				contactSim->manifold.pointCount = 1;
			}
			else if ( contactSim->manifold.points[0].separation > 1.5f * B2_LINEAR_SLOP )
			{
				contactSim->manifold.pointCount = 1;
			}

			pointCount = contactSim->manifold.pointCount;
		}

		if ( touching && ( shapeA->enableHitEvents || shapeB->enableHitEvents ) )
		{
			contactSim->simFlags |= b2_simEnableHitEvent;
		}
		else
		{
			contactSim->simFlags &= ~b2_simEnableHitEvent;
		}

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		int unmatchedCount = 0;
		for ( int i = 0; i < pointCount; ++i )
		{
			b2ManifoldPoint* mp2 = contactSim->manifold.points + i;

			// shift anchors to be center of mass relative
			mp2->anchorA = b2Sub( mp2->anchorA, centerOffsetA );
			mp2->anchorB = b2Sub( mp2->anchorB, centerOffsetB );

			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			mp2->maxNormalImpulse = 0.0f;
			mp2->normalVelocity = 0.0f;
			mp2->persisted = false;

			uint16_t id2 = mp2->id;

			for ( int j = 0; j < oldManifold.pointCount; ++j )
			{
				b2ManifoldPoint* mp1 = oldManifold.points + j;

				if ( mp1->id == id2 )
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					mp2->persisted = true;

					// clear old impulse
					mp1->normalImpulse = 0.0f;
					mp1->tangentImpulse = 0.0f;
					break;
				}
			}

			unmatchedCount += mp2->persisted ? 0 : 1;
		}

#if 0
		// todo I haven't found an improvement from this yet
		// If there are unmatched new contact points, apply any left over old impulse.
		if (unmatchedCount > 0)
		{
			float unmatchedNormalImpulse = 0.0f;
			float unmatchedTangentImpulse = 0.0f;
			for (int i = 0; i < oldManifold.pointCount; ++i)
			{
				b2ManifoldPoint* mp = oldManifold.points + i;
				unmatchedNormalImpulse += mp->normalImpulse;
				unmatchedTangentImpulse += mp->tangentImpulse;
			}

			float inverse = 1.0f / unmatchedCount;
			unmatchedNormalImpulse *= inverse;
			unmatchedTangentImpulse *= inverse;

			for ( int i = 0; i < pointCount; ++i )
			{
				b2ManifoldPoint* mp2 = contactSim->manifold.points + i;

				if (mp2->persisted)
				{
					continue;
				}

				mp2->normalImpulse = unmatchedNormalImpulse;
				mp2->tangentImpulse = unmatchedTangentImpulse;
			}
		}
#endif
	}

	if ( touching )
	{
		contactSim->simFlags |= b2_simTouchingFlag;
	}
	else
	{
		contactSim->simFlags &= ~b2_simTouchingFlag;
	}

	return touching;
}

b2Manifold b2ComputeManifold(b2Shape* shapeA, b2Transform transformA, b2Shape* shapeB, b2Transform transformB)
{
	b2ManifoldFcn* fcn = s_registers[shapeA->type][shapeB->type].fcn;
	b2SimplexCache cache = { 0 };
	return fcn( shapeA, transformA, shapeB, transformB, &cache );
}
