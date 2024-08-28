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

// Friction mixing law. The idea is to allow either shape to drive the friction to zero.
// For example, anything slides on ice.
static inline float b2MixFriction( float friction1, float friction2 )
{
	return sqrtf( friction1 * friction2 );
}

// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
// For example, a superball bounces on anything.
static inline float b2MixRestitution( float restitution1, float restitution2 )
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

// todo make relative for all
// typedef b2Manifold b2ManifoldFcn(const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xfB, b2DistanceCache* cache);
typedef b2Manifold b2ManifoldFcn( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
								  b2DistanceCache* cache );

struct b2ContactRegister
{
	b2ManifoldFcn* fcn;
	bool primary;
};

static struct b2ContactRegister s_registers[b2_shapeTypeCount][b2_shapeTypeCount];
static bool s_initialized = false;

static b2Manifold b2CircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideCircles( &shapeA->circle, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2CapsuleAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideCapsuleAndCircle( &shapeA->capsule, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2CapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									 b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideCapsules( &shapeA->capsule, xfA, &shapeB->capsule, xfB );
}

static b2Manifold b2PolygonAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollidePolygonAndCircle( &shapeA->polygon, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2PolygonAndCapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											   b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollidePolygonAndCapsule( &shapeA->polygon, xfA, &shapeB->capsule, xfB );
}

static b2Manifold b2PolygonManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									 b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollidePolygons( &shapeA->polygon, xfA, &shapeB->polygon, xfB );
}

static b2Manifold b2SegmentAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSegmentAndCircle( &shapeA->segment, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2SegmentAndCapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											   b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSegmentAndCapsule( &shapeA->segment, xfA, &shapeB->capsule, xfB );
}

static b2Manifold b2SegmentAndPolygonManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											   b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSegmentAndPolygon( &shapeA->segment, xfA, &shapeB->polygon, xfB );
}

static b2Manifold b2SmoothSegmentAndCircleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB,
													b2Transform xfB, b2DistanceCache* cache )
{
	B2_MAYBE_UNUSED( cache );
	return b2CollideSmoothSegmentAndCircle( &shapeA->smoothSegment, xfA, &shapeB->circle, xfB );
}

static b2Manifold b2SmoothSegmentAndCapsuleManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB,
													 b2Transform xfB, b2DistanceCache* cache )
{
	return b2CollideSmoothSegmentAndCapsule( &shapeA->smoothSegment, xfA, &shapeB->capsule, xfB, cache );
}

static b2Manifold b2SmoothSegmentAndPolygonManifold( const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB,
													 b2Transform xfB, b2DistanceCache* cache )
{
	return b2CollideSmoothSegmentAndPolygon( &shapeA->smoothSegment, xfA, &shapeB->polygon, xfB, cache );
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
		b2AddType( b2SmoothSegmentAndCircleManifold, b2_smoothSegmentShape, b2_circleShape );
		b2AddType( b2SmoothSegmentAndCapsuleManifold, b2_smoothSegmentShape, b2_capsuleShape );
		b2AddType( b2SmoothSegmentAndPolygonManifold, b2_smoothSegmentShape, b2_polygonShape );
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

	b2Body* bodyA = b2GetBody( world, shapeA->bodyId );
	b2Body* bodyB = b2GetBody( world, shapeB->bodyId );

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

	b2SolverSet* set = world->solverSetArray + setIndex;

	// Create contact key and contact
	int contactId = b2AllocId( &world->contactIdPool );
	if ( contactId == b2Array( world->contactArray ).count )
	{
		b2Array_Push( world->contactArray, ( b2Contact ){ 0 } );
	}

	int shapeIdA = shapeA->id;
	int shapeIdB = shapeB->id;

	b2Contact* contact = world->contactArray + contactId;
	contact->contactId = contactId;
	contact->setIndex = setIndex;
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = set->contacts.count;
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
			b2Contact* headContact = world->contactArray + ( headContactKey >> 1 );
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
			b2Contact* headContact = world->contactArray + ( headContactKey >> 1 );
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
	b2ContactSim* contactSim = b2AddContact( &set->contacts );
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
	contactSim->cache = b2_emptyDistanceCache;
	contactSim->manifold = ( b2Manifold ){ 0 };
	contactSim->friction = b2MixFriction( shapeA->friction, shapeB->friction );
	contactSim->restitution = b2MixRestitution( shapeA->restitution, shapeB->restitution );
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
	b2Body* bodyA = b2GetBody( world, bodyIdA );
	b2Body* bodyB = b2GetBody( world, bodyIdB );

	// if (contactListener && contact->IsTouching())
	//{
	//	contactListener->EndContact(contact);
	// }

	// Remove from body A
	if ( edgeA->prevKey != B2_NULL_INDEX )
	{
		b2Contact* prevContact = world->contactArray + ( edgeA->prevKey >> 1 );
		b2ContactEdge* prevEdge = prevContact->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B2_NULL_INDEX )
	{
		b2Contact* nextContact = world->contactArray + ( edgeA->nextKey >> 1 );
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
		b2Contact* prevContact = world->contactArray + ( edgeB->prevKey >> 1 );
		b2ContactEdge* prevEdge = prevContact->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B2_NULL_INDEX )
	{
		b2Contact* nextContact = world->contactArray + ( edgeB->nextKey >> 1 );
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
		b2SolverSet* set = world->solverSetArray + contact->setIndex;
		int movedIndex = b2RemoveContact( &set->contacts, contact->localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			b2ContactSim* movedContact = set->contacts.data + contact->localIndex;
			world->contactArray[movedContact->contactId].localIndex = contact->localIndex;
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
		B2_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount );
		b2GraphColor* color = world->constraintGraph.colors + contact->colorIndex;
		B2_ASSERT( 0 <= contact->localIndex && contact->localIndex < color->contacts.count );
		return color->contacts.data + contact->localIndex;
	}

	b2SolverSet* set = world->solverSetArray + contact->setIndex;
	B2_ASSERT( 0 <= contact->localIndex && contact->localIndex <= set->contacts.count );
	return set->contacts.data + contact->localIndex;
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
								b2DistanceCache* cache )
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
		b2Manifold oldManifold = contactSim->manifold;

		// Compute TOI
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
				contactSim->manifold.pointCount = 0;
			}
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
					break;
				}
			}
		}
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
