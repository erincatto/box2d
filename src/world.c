// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "world.h"

#include "aabb.h"
#include "array.h"
#include "bitset.h"
#include "body.h"
#include "broad_phase.h"
#include "constants.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"
#include "stack_allocator.h"

#include "box2d/box2d.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

_Static_assert( b2_maxWorlds > 0, "must be 1 or more" );
b2World b2_worlds[b2_maxWorlds];

B2_ARRAY_SOURCE( b2BodyMoveEvent, b2BodyMoveEvent );
B2_ARRAY_SOURCE( b2ContactBeginTouchEvent, b2ContactBeginTouchEvent );
B2_ARRAY_SOURCE( b2ContactEndTouchEvent, b2ContactEndTouchEvent );
B2_ARRAY_SOURCE( b2ContactHitEvent, b2ContactHitEvent );
B2_ARRAY_SOURCE( b2SensorBeginTouchEvent, b2SensorBeginTouchEvent );
B2_ARRAY_SOURCE( b2SensorEndTouchEvent, b2SensorEndTouchEvent );
B2_ARRAY_SOURCE( b2TaskContext, b2TaskContext );

b2World* b2GetWorldFromId( b2WorldId id )
{
	B2_ASSERT( 1 <= id.index1 && id.index1 <= b2_maxWorlds );
	b2World* world = b2_worlds + ( id.index1 - 1 );
	B2_ASSERT( id.index1 == world->worldId + 1 );
	B2_ASSERT( id.revision == world->revision );
	return world;
}

b2World* b2GetWorld( int index )
{
	B2_ASSERT( 0 <= index && index < b2_maxWorlds );
	b2World* world = b2_worlds + index;
	B2_ASSERT( world->worldId == index );
	return world;
}

b2World* b2GetWorldLocked( int index )
{
	B2_ASSERT( 0 <= index && index < b2_maxWorlds );
	b2World* world = b2_worlds + index;
	B2_ASSERT( world->worldId == index );
	if ( world->locked )
	{
		B2_ASSERT( false );
		return NULL;
	}

	return world;
}

static void* b2DefaultAddTaskFcn( b2TaskCallback* task, int count, int minRange, void* taskContext, void* userContext )
{
	B2_MAYBE_UNUSED( minRange );
	B2_MAYBE_UNUSED( userContext );
	task( 0, count, 0, taskContext );
	return NULL;
}

static void b2DefaultFinishTaskFcn( void* userTask, void* userContext )
{
	B2_MAYBE_UNUSED( userTask );
	B2_MAYBE_UNUSED( userContext );
}

b2WorldId b2CreateWorld( const b2WorldDef* def )
{
	_Static_assert( b2_maxWorlds < UINT16_MAX, "b2_maxWorlds limit exceeded" );
	b2CheckDef( def );

	int worldId = B2_NULL_INDEX;
	for ( int i = 0; i < b2_maxWorlds; ++i )
	{
		if ( b2_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}

	if ( worldId == B2_NULL_INDEX )
	{
		return ( b2WorldId ){ 0 };
	}

	b2InitializeContactRegisters();

	b2World* world = b2_worlds + worldId;
	uint16_t revision = world->revision;

	*world = ( b2World ){ 0 };

	world->worldId = (uint16_t)worldId;
	world->revision = revision;
	world->inUse = true;

	world->stackAllocator = b2CreateStackAllocator( 2048 );
	b2CreateBroadPhase( &world->broadPhase );
	b2CreateGraph( &world->constraintGraph, 16 );

	// pools
	world->bodyIdPool = b2CreateIdPool();
	world->bodies = b2BodyArray_Create( 16 );
	world->solverSets = b2SolverSetArray_Create( 8 );

	// add empty static, active, and disabled body sets
	world->solverSetIdPool = b2CreateIdPool();
	b2SolverSet set = { 0 };

	// static set
	set.setIndex = b2AllocId( &world->solverSetIdPool );
	b2SolverSetArray_Push( &world->solverSets, set );
	B2_ASSERT( world->solverSets.data[b2_staticSet].setIndex == b2_staticSet );

	// disabled set
	set.setIndex = b2AllocId( &world->solverSetIdPool );
	b2SolverSetArray_Push( &world->solverSets, set );
	B2_ASSERT( world->solverSets.data[b2_disabledSet].setIndex == b2_disabledSet );

	// awake set
	set.setIndex = b2AllocId( &world->solverSetIdPool );
	b2SolverSetArray_Push( &world->solverSets, set );
	B2_ASSERT( world->solverSets.data[b2_awakeSet].setIndex == b2_awakeSet );

	world->shapeIdPool = b2CreateIdPool();
	world->shapes = b2ShapeArray_Create( 16 );

	world->chainIdPool = b2CreateIdPool();
	world->chainShapes = b2ChainShapeArray_Create( 4 );

	world->contactIdPool = b2CreateIdPool();
	world->contacts = b2ContactArray_Create( 16 );

	world->jointIdPool = b2CreateIdPool();
	world->joints = b2JointArray_Create( 16 );

	world->islandIdPool = b2CreateIdPool();
	world->islands = b2IslandArray_Create( 8 );

	world->bodyMoveEvents = b2BodyMoveEventArray_Create( 4 );
	world->sensorBeginEvents = b2SensorBeginTouchEventArray_Create( 4 );
	world->sensorEndEvents[0] = b2SensorEndTouchEventArray_Create( 4 );
	world->sensorEndEvents[1] = b2SensorEndTouchEventArray_Create( 4 );
	world->contactBeginEvents = b2ContactBeginTouchEventArray_Create( 4 );
	world->contactEndEvents[0] = b2ContactEndTouchEventArray_Create( 4 );
	world->contactEndEvents[1] = b2ContactEndTouchEventArray_Create( 4 );
	world->contactHitEvents = b2ContactHitEventArray_Create( 4 );
	world->endEventArrayIndex = 0;

	world->stepIndex = 0;
	world->splitIslandId = B2_NULL_INDEX;
	world->activeTaskCount = 0;
	world->taskCount = 0;
	world->gravity = def->gravity;
	world->hitEventThreshold = def->hitEventThreshold;
	world->restitutionThreshold = def->restitutionThreshold;
	world->maxLinearVelocity = def->maximumLinearVelocity;
	world->contactPushoutVelocity = def->contactPushoutVelocity;
	world->contactHertz = def->contactHertz;
	world->contactDampingRatio = def->contactDampingRatio;
	world->jointHertz = def->jointHertz;
	world->jointDampingRatio = def->jointDampingRatio;
	world->frictionMixingRule = def->frictionMixingRule;
	world->restitutionMixingRule = def->restitutionMixingRule;
	world->enableSleep = def->enableSleep;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContinuous = def->enableContinuous;
	world->enableSpeculative = true;
	world->userTreeTask = NULL;
	world->userData = def->userData;

	if ( def->workerCount > 0 && def->enqueueTask != NULL && def->finishTask != NULL )
	{
		world->workerCount = b2MinInt( def->workerCount, b2_maxWorkers );
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = 1;
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->userTaskContext = NULL;
	}

	world->taskContexts = b2TaskContextArray_Create( world->workerCount );
	b2TaskContextArray_Resize( &world->taskContexts, world->workerCount );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		world->taskContexts.data[i].contactStateBitSet = b2CreateBitSet( 1024 );
		world->taskContexts.data[i].enlargedSimBitSet = b2CreateBitSet( 256 );
		world->taskContexts.data[i].awakeIslandBitSet = b2CreateBitSet( 256 );
	}

	world->debugBodySet = b2CreateBitSet( 256 );
	world->debugJointSet = b2CreateBitSet( 256 );
	world->debugContactSet = b2CreateBitSet( 256 );

	// add one to worldId so that 0 represents a null b2WorldId
	return ( b2WorldId ){ (uint16_t)( worldId + 1 ), world->revision };
}

void b2DestroyWorld( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );

	b2DestroyBitSet( &world->debugBodySet );
	b2DestroyBitSet( &world->debugJointSet );
	b2DestroyBitSet( &world->debugContactSet );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2DestroyBitSet( &world->taskContexts.data[i].contactStateBitSet );
		b2DestroyBitSet( &world->taskContexts.data[i].enlargedSimBitSet );
		b2DestroyBitSet( &world->taskContexts.data[i].awakeIslandBitSet );
	}

	b2TaskContextArray_Destroy( &world->taskContexts );

	b2BodyMoveEventArray_Destroy( &world->bodyMoveEvents );
	b2SensorBeginTouchEventArray_Destroy( &world->sensorBeginEvents );
	b2SensorEndTouchEventArray_Destroy( world->sensorEndEvents + 0 );
	b2SensorEndTouchEventArray_Destroy( world->sensorEndEvents + 1 );
	b2ContactBeginTouchEventArray_Destroy( &world->contactBeginEvents );
	b2ContactEndTouchEventArray_Destroy( world->contactEndEvents + 0);
	b2ContactEndTouchEventArray_Destroy( world->contactEndEvents + 1);
	b2ContactHitEventArray_Destroy( &world->contactHitEvents );

	int chainCapacity = world->chainShapes.count;
	for ( int i = 0; i < chainCapacity; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		if ( chain->id != B2_NULL_INDEX )
		{
			b2Free( chain->shapeIndices, chain->count * sizeof( int ) );
		}
		else
		{
			B2_ASSERT( chain->shapeIndices == NULL );
		}
	}

	b2BodyArray_Destroy( &world->bodies );
	b2ShapeArray_Destroy( &world->shapes );
	b2ChainShapeArray_Destroy( &world->chainShapes );
	b2ContactArray_Destroy( &world->contacts );
	b2JointArray_Destroy( &world->joints );
	b2IslandArray_Destroy( &world->islands );

	// Destroy solver sets
	int setCapacity = world->solverSets.count;
	for ( int i = 0; i < setCapacity; ++i )
	{
		b2SolverSet* set = world->solverSets.data + i;
		if ( set->setIndex != B2_NULL_INDEX )
		{
			b2DestroySolverSet( world, i );
		}
	}

	b2SolverSetArray_Destroy( &world->solverSets );

	b2DestroyGraph( &world->constraintGraph );
	b2DestroyBroadPhase( &world->broadPhase );

	b2DestroyIdPool( &world->bodyIdPool );
	b2DestroyIdPool( &world->shapeIdPool );
	b2DestroyIdPool( &world->chainIdPool );
	b2DestroyIdPool( &world->contactIdPool );
	b2DestroyIdPool( &world->jointIdPool );
	b2DestroyIdPool( &world->islandIdPool );
	b2DestroyIdPool( &world->solverSetIdPool );

	b2DestroyStackAllocator( &world->stackAllocator );

	// Wipe world but preserve revision
	uint16_t revision = world->revision;
	*world = ( b2World ){ 0 };
	world->worldId = 0;
	world->revision = revision + 1;
}

static void b2CollideTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( collide_task, "Collide Task", b2_colorDodgerBlue, true );

	b2StepContext* stepContext = context;
	b2World* world = stepContext->world;
	B2_ASSERT( (int)threadIndex < world->workerCount );
	b2TaskContext* taskContext = world->taskContexts.data + threadIndex;
	b2ContactSim** contactSims = stepContext->contacts;
	b2Shape* shapes = world->shapes.data;
	b2Body* bodies = world->bodies.data;

	B2_ASSERT( startIndex < endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactSim* contactSim = contactSims[i];

		int contactId = contactSim->contactId;

		b2Shape* shapeA = shapes + contactSim->shapeIdA;
		b2Shape* shapeB = shapes + contactSim->shapeIdB;

		// Do proxies still overlap?
		bool overlap = b2AABB_Overlaps( shapeA->fatAABB, shapeB->fatAABB );
		if ( overlap == false )
		{
			contactSim->simFlags |= b2_simDisjoint;
			contactSim->simFlags &= ~b2_simTouchingFlag;
			b2SetBit( &taskContext->contactStateBitSet, contactId );
		}
		else
		{
			bool wasTouching = ( contactSim->simFlags & b2_simTouchingFlag );

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = bodies + shapeA->bodyId;
			b2Body* bodyB = bodies + shapeB->bodyId;
			b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
			b2BodySim* bodySimB = b2GetBodySim( world, bodyB );

			// avoid cache misses in b2PrepareContactsTask
			contactSim->bodySimIndexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
			contactSim->invMassA = bodySimA->invMass;
			contactSim->invIA = bodySimA->invInertia;

			contactSim->bodySimIndexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;
			contactSim->invMassB = bodySimB->invMass;
			contactSim->invIB = bodySimB->invInertia;

			b2Transform transformA = bodySimA->transform;
			b2Transform transformB = bodySimB->transform;

			b2Vec2 centerOffsetA = b2RotateVector( transformA.q, bodySimA->localCenter );
			b2Vec2 centerOffsetB = b2RotateVector( transformB.q, bodySimB->localCenter );

			// This updates solid contacts and sensors
			bool touching =
				b2UpdateContact( world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB );

			// State changes that affect island connectivity. Also contact and sensor events.
			if ( touching == true && wasTouching == false )
			{
				contactSim->simFlags |= b2_simStartedTouching;
				b2SetBit( &taskContext->contactStateBitSet, contactId );
			}
			else if ( touching == false && wasTouching == true )
			{
				contactSim->simFlags |= b2_simStoppedTouching;
				b2SetBit( &taskContext->contactStateBitSet, contactId );
			}
		}
	}

	b2TracyCZoneEnd( collide_task );
}

static void b2UpdateTreesTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	B2_MAYBE_UNUSED( startIndex );
	B2_MAYBE_UNUSED( endIndex );
	B2_MAYBE_UNUSED( threadIndex );

	b2TracyCZoneNC( tree_task, "Rebuild Trees", b2_colorSnow, true );

	b2World* world = context;
	b2BroadPhase_RebuildTrees( &world->broadPhase );

	b2TracyCZoneEnd( tree_task );
}

static void b2AddNonTouchingContact( b2World* world, b2Contact* contact, b2ContactSim* contactSim )
{
	B2_ASSERT( contact->setIndex == b2_awakeSet );
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = set->contactSims.count;

	b2ContactSim* newContactSim = b2ContactSimArray_Add( &set->contactSims );
	memcpy( newContactSim, contactSim, sizeof( b2ContactSim ) );
}

static void b2RemoveNonTouchingContact( b2World* world, int setIndex, int localIndex )
{
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
	int movedIndex = b2ContactSimArray_RemoveSwap( &set->contactSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		b2ContactSim* movedContactSim = set->contactSims.data + localIndex;
		b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactSim->contactId );
		B2_ASSERT( movedContact->setIndex == setIndex );
		B2_ASSERT( movedContact->localIndex == movedIndex );
		B2_ASSERT( movedContact->colorIndex == B2_NULL_INDEX );
		movedContact->localIndex = localIndex;
	}
}

// Narrow-phase collision
static void b2Collide( b2StepContext* context )
{
	b2World* world = context->world;

	B2_ASSERT( world->workerCount > 0 );

	b2TracyCZoneNC( collide, "Collide", b2_colorDarkOrchid, true );

	// Tasks that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	world->userTreeTask = world->enqueueTaskFcn( &b2UpdateTreesTask, 1, 1, world, world->userTaskContext );
	world->taskCount += 1;
	world->activeTaskCount += world->userTreeTask == NULL ? 0 : 1;

	// gather contacts into a single array for easier parallel-for
	int contactCount = 0;
	b2GraphColor* graphColors = world->constraintGraph.colors;
	for ( int i = 0; i < b2_graphColorCount; ++i )
	{
		contactCount += graphColors[i].contactSims.count;
	}

	int nonTouchingCount = world->solverSets.data[b2_awakeSet].contactSims.count;
	contactCount += nonTouchingCount;

	if ( contactCount == 0 )
	{
		b2TracyCZoneEnd( collide );
		return;
	}

	b2ContactSim** contactSims = b2AllocateStackItem( &world->stackAllocator, contactCount * sizeof( b2ContactSim ), "contacts" );

	int contactIndex = 0;
	for ( int i = 0; i < b2_graphColorCount; ++i )
	{
		b2GraphColor* color = graphColors + i;
		int count = color->contactSims.count;
		b2ContactSim* base = color->contactSims.data;
		for ( int j = 0; j < count; ++j )
		{
			contactSims[contactIndex] = base + j;
			contactIndex += 1;
		}
	}

	{
		b2ContactSim* base = world->solverSets.data[b2_awakeSet].contactSims.data;
		for ( int i = 0; i < nonTouchingCount; ++i )
		{
			contactSims[contactIndex] = base + i;
			contactIndex += 1;
		}
	}

	B2_ASSERT( contactIndex == contactCount );

	context->contacts = contactSims;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b2GetIdCapacity( &world->contactIdPool );
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2SetBitCountAndClear( &world->taskContexts.data[i].contactStateBitSet, contactIdCapacity );
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int minRange = 64;
	void* userCollideTask = world->enqueueTaskFcn( &b2CollideTask, contactCount, minRange, context, world->userTaskContext );
	world->taskCount += 1;
	if ( userCollideTask != NULL )
	{
		world->finishTaskFcn( userCollideTask, world->userTaskContext );
	}

	b2FreeStackItem( &world->stackAllocator, contactSims );
	context->contacts = NULL;
	contactSims = NULL;

	// Serially update contact state
	b2TracyCZoneNC( contact_state, "Contact State", b2_colorCoral, true );

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world->taskContexts.data[0].contactStateBitSet;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world->taskContexts.data[i].contactStateBitSet );
	}

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );

	int endEventArrayIndex = world->endEventArrayIndex;

	const b2Shape* shapes = world->shapes.data;
	int16_t worldId = world->worldId;

	// Process contact state changes. Iterate over set bits
	for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
	{
		uint64_t bits = bitSet->bits[k];
		while ( bits != 0 )
		{
			uint32_t ctz = b2CTZ64( bits );
			int contactId = (int)( 64 * k + ctz );

			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );

			int colorIndex = contact->colorIndex;
			int localIndex = contact->localIndex;

			b2ContactSim* contactSim = NULL;
			if ( colorIndex != B2_NULL_INDEX )
			{
				// contact lives in constraint graph
				B2_ASSERT( 0 <= colorIndex && colorIndex < b2_graphColorCount );
				b2GraphColor* color = graphColors + colorIndex;
				contactSim = b2ContactSimArray_Get( &color->contactSims, localIndex );
			}
			else
			{
				contactSim = b2ContactSimArray_Get( &awakeSet->contactSims, localIndex );
			}

			const b2Shape* shapeA = shapes + contact->shapeIdA;
			const b2Shape* shapeB = shapes + contact->shapeIdB;
			b2ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->revision };
			b2ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->revision };
			uint32_t flags = contact->flags;
			uint32_t simFlags = contactSim->simFlags;

			if ( simFlags & b2_simDisjoint )
			{
				// Bounding boxes no longer overlap
				b2DestroyContact( world, contact, false );
				contact = NULL;
				contactSim = NULL;
			}
			else if ( simFlags & b2_simStartedTouching )
			{
				B2_ASSERT( contact->islandId == B2_NULL_INDEX );
				if ( ( flags & b2_contactSensorFlag ) != 0 )
				{
					// Contact is a sensor
					if ( ( flags & b2_contactEnableSensorEvents ) != 0 )
					{
						if ( shapeA->isSensor )
						{
							b2SensorBeginTouchEvent event = { shapeIdA, shapeIdB };
							b2SensorBeginTouchEventArray_Push( &world->sensorBeginEvents, event );
						}

						if ( shapeB->isSensor )
						{
							b2SensorBeginTouchEvent event = { shapeIdB, shapeIdA };
							b2SensorBeginTouchEventArray_Push( &world->sensorBeginEvents, event );
						}
					}

					contactSim->simFlags &= ~b2_simStartedTouching;
					contact->flags |= b2_contactSensorTouchingFlag;
				}
				else
				{
					// Contact is solid
					if ( flags & b2_contactEnableContactEvents )
					{
						b2ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactSim->manifold };
						b2ContactBeginTouchEventArray_Push( &world->contactBeginEvents, event );
					}

					B2_ASSERT( contactSim->manifold.pointCount > 0 );
					B2_ASSERT( contact->setIndex == b2_awakeSet );

					// Link first because this wakes colliding bodies and ensures the body sims
					// are in the correct place.
					contact->flags |= b2_contactTouchingFlag;
					b2LinkContact( world, contact );

					// Make sure these didn't change
					B2_ASSERT( contact->colorIndex == B2_NULL_INDEX );
					B2_ASSERT( contact->localIndex == localIndex );

					// Contact sim pointer may have become orphaned due to awake set growth,
					// so I just need to refresh it.
					contactSim = b2ContactSimArray_Get( &awakeSet->contactSims, localIndex );

					contactSim->simFlags &= ~b2_simStartedTouching;

					b2AddContactToGraph( world, contactSim, contact );
					b2RemoveNonTouchingContact( world, b2_awakeSet, localIndex );
					contactSim = NULL;
				}
			}
			else if ( simFlags & b2_simStoppedTouching )
			{
				contactSim->simFlags &= ~b2_simStoppedTouching;

				if ( ( flags & b2_contactSensorFlag ) != 0 )
				{
					// Contact is a sensor
					contact->flags &= ~b2_contactSensorTouchingFlag;

					if ( ( flags & b2_contactEnableSensorEvents ) != 0 )
					{
						if ( shapeA->isSensor )
						{
							b2SensorEndTouchEvent event = { shapeIdA, shapeIdB };
							b2SensorEndTouchEventArray_Push( world->sensorEndEvents + endEventArrayIndex, event );
						}

						if ( shapeB->isSensor )
						{
							b2SensorEndTouchEvent event = { shapeIdB, shapeIdA };
							b2SensorEndTouchEventArray_Push( world->sensorEndEvents + endEventArrayIndex, event );
						}
					}
				}
				else
				{
					// Contact is solid
					contact->flags &= ~b2_contactTouchingFlag;

					if ( contact->flags & b2_contactEnableContactEvents )
					{
						b2ContactEndTouchEvent event = { shapeIdA, shapeIdB };
						b2ContactEndTouchEventArray_Push( world->contactEndEvents + endEventArrayIndex, event );
					}

					B2_ASSERT( contactSim->manifold.pointCount == 0 );

					b2UnlinkContact( world, contact );
					int bodyIdA = contact->edges[0].bodyId;
					int bodyIdB = contact->edges[1].bodyId;

					b2AddNonTouchingContact( world, contact, contactSim );
					b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
					contact = NULL;
					contactSim = NULL;
				}
			}

			// Clear the smallest set bit
			bits = bits & ( bits - 1 );
		}
	}

	b2ValidateSolverSets( world );
	b2ValidateContacts( world );

	b2TracyCZoneEnd( contact_state );
	b2TracyCZoneEnd( collide );
}

void b2World_Step( b2WorldId worldId, float timeStep, int subStepCount )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	// 1. destroy shape
	// 2. step
	// 3. get events

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	b2BodyMoveEventArray_Clear( &world->bodyMoveEvents );
	b2SensorBeginTouchEventArray_Clear( &world->sensorBeginEvents );
	b2ContactBeginTouchEventArray_Clear( &world->contactBeginEvents );
	b2ContactHitEventArray_Clear( &world->contactHitEvents );

	world->profile = ( b2Profile ){ 0 };

	if ( timeStep == 0.0f )
	{
		// Swap end event array buffers
		world->endEventArrayIndex = 1 - world->endEventArrayIndex;
		b2SensorEndTouchEventArray_Clear( world->sensorEndEvents + world->endEventArrayIndex );
		b2ContactEndTouchEventArray_Clear( world->contactEndEvents + world->endEventArrayIndex );

		// todo would be useful to still process collision while paused
		return;
	}

	b2TracyCZoneNC( world_step, "Step", b2_colorBox2DGreen, true );

	world->locked = true;
	world->activeTaskCount = 0;
	world->taskCount = 0;

	b2Timer stepTimer = b2CreateTimer();

	// Update collision pairs and create contacts
	{
		b2Timer timer = b2CreateTimer();
		b2UpdateBroadPhasePairs( world );
		world->profile.pairs = b2GetMilliseconds( &timer );
	}

	b2StepContext context = { 0 };
	context.world = world;
	context.dt = timeStep;
	context.subStepCount = b2MaxInt( 1, subStepCount );

	if ( timeStep > 0.0f )
	{
		context.inv_dt = 1.0f / timeStep;
		context.h = timeStep / context.subStepCount;
		context.inv_h = context.subStepCount * context.inv_dt;
	}
	else
	{
		context.inv_dt = 0.0f;
		context.h = 0.0f;
		context.inv_h = 0.0f;
	}

	world->inv_h = context.inv_h;

	// Hertz values get reduced for large time steps
	float contactHertz = b2MinFloat( world->contactHertz, 0.25f * context.inv_h );
	float jointHertz = b2MinFloat( world->jointHertz, 0.125f * context.inv_h );

	context.contactSoftness = b2MakeSoft( contactHertz, world->contactDampingRatio, context.h );
	context.staticSoftness = b2MakeSoft( 2.0f * contactHertz, world->contactDampingRatio, context.h );
	context.jointSoftness = b2MakeSoft( jointHertz, world->jointDampingRatio, context.h );

	context.restitutionThreshold = world->restitutionThreshold;
	context.maxLinearVelocity = world->maxLinearVelocity;
	context.enableWarmStarting = world->enableWarmStarting;

	// Update contacts
	{
		b2Timer timer = b2CreateTimer();
		b2Collide( &context );
		world->profile.collide = b2GetMilliseconds( &timer );
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if ( context.dt > 0.0f )
	{
		b2Timer timer = b2CreateTimer();
		b2Solve( world, &context );
		world->profile.solve = b2GetMilliseconds( &timer );
	}

	world->locked = false;

	world->profile.step = b2GetMilliseconds( &stepTimer );

	B2_ASSERT( b2GetStackAllocation( &world->stackAllocator ) == 0 );

	// Ensure stack is large enough
	b2GrowStack( &world->stackAllocator );

	// Make sure all tasks that were started were also finished
	B2_ASSERT( world->activeTaskCount == 0 );

	b2TracyCZoneEnd( world_step );

	// Swap end event array buffers
	world->endEventArrayIndex = 1 - world->endEventArrayIndex;
	b2SensorEndTouchEventArray_Clear( world->sensorEndEvents + world->endEventArrayIndex );
	b2ContactEndTouchEventArray_Clear( world->contactEndEvents + world->endEventArrayIndex );
}

static void b2DrawShape( b2DebugDraw* draw, b2Shape* shape, b2Transform xf, b2HexColor color )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			b2Capsule* capsule = &shape->capsule;
			b2Vec2 p1 = b2TransformPoint( xf, capsule->center1 );
			b2Vec2 p2 = b2TransformPoint( xf, capsule->center2 );
			draw->DrawSolidCapsule( p1, p2, capsule->radius, color, draw->context );
		}
		break;

		case b2_circleShape:
		{
			b2Circle* circle = &shape->circle;
			xf.p = b2TransformPoint( xf, circle->center );
			draw->DrawSolidCircle( xf, circle->radius, color, draw->context );
		}
		break;

		case b2_polygonShape:
		{
			b2Polygon* poly = &shape->polygon;
			draw->DrawSolidPolygon( xf, poly->vertices, poly->count, poly->radius, color, draw->context );
		}
		break;

		case b2_segmentShape:
		{
			b2Segment* segment = &shape->segment;
			b2Vec2 p1 = b2TransformPoint( xf, segment->point1 );
			b2Vec2 p2 = b2TransformPoint( xf, segment->point2 );
			draw->DrawSegment( p1, p2, color, draw->context );
		}
		break;

		case b2_chainSegmentShape:
		{
			b2Segment* segment = &shape->chainSegment.segment;
			b2Vec2 p1 = b2TransformPoint( xf, segment->point1 );
			b2Vec2 p2 = b2TransformPoint( xf, segment->point2 );
			draw->DrawSegment( p1, p2, color, draw->context );
			draw->DrawPoint( p2, 4.0f, color, draw->context );
			draw->DrawSegment( p1, b2Lerp( p1, p2, 0.1f ), b2_colorPaleGreen, draw->context );
		}
		break;

		default:
			break;
	}
}

struct DrawContext
{
	b2World* world;
	b2DebugDraw* draw;
};

static bool DrawQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	struct DrawContext* drawContext = context;
	b2World* world = drawContext->world;
	b2DebugDraw* draw = drawContext->draw;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
	B2_ASSERT( shape->id == shapeId );

	b2SetBit( &world->debugBodySet, shape->bodyId );

	if ( draw->drawShapes )
	{
		b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
		b2BodySim* bodySim = b2GetBodySim( world, body );

		b2HexColor color;

		if ( shape->customColor != 0 )
		{
			color = shape->customColor;
		}
		else if ( body->type == b2_dynamicBody && bodySim->mass == 0.0f )
		{
			// Bad body
			color = b2_colorRed;
		}
		else if ( body->setIndex == b2_disabledSet )
		{
			color = b2_colorSlateGray;
		}
		else if ( shape->isSensor )
		{
			color = b2_colorWheat;
		}
		else if ( bodySim->isBullet && body->setIndex == b2_awakeSet )
		{
			color = b2_colorTurquoise;
		}
		else if ( body->isSpeedCapped )
		{
			color = b2_colorYellow;
		}
		else if ( bodySim->isFast )
		{
			color = b2_colorSalmon;
		}
		else if ( body->type == b2_staticBody )
		{
			color = b2_colorPaleGreen;
		}
		else if ( body->type == b2_kinematicBody )
		{
			color = b2_colorRoyalBlue;
		}
		else if ( body->setIndex == b2_awakeSet )
		{
			color = b2_colorPink;
		}
		else
		{
			color = b2_colorGray;
		}

		b2DrawShape( draw, shape, bodySim->transform, color );
	}

	if ( draw->drawAABBs )
	{
		b2AABB aabb = shape->fatAABB;

		b2Vec2 vs[4] = { { aabb.lowerBound.x, aabb.lowerBound.y },
						 { aabb.upperBound.x, aabb.lowerBound.y },
						 { aabb.upperBound.x, aabb.upperBound.y },
						 { aabb.lowerBound.x, aabb.upperBound.y } };

		draw->DrawPolygon( vs, 4, b2_colorGold, draw->context );
	}

	return true;
}

// todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
// solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
static void b2DrawWithBounds( b2World* world, b2DebugDraw* draw )
{
	B2_ASSERT( b2IsValidAABB( draw->drawingBounds ) );

	const float k_impulseScale = 1.0f;
	const float k_axisScale = 0.3f;
	b2HexColor speculativeColor = b2_colorGray3;
	b2HexColor addColor = b2_colorGreen;
	b2HexColor persistColor = b2_colorBlue;
	b2HexColor normalColor = b2_colorGray9;
	b2HexColor impulseColor = b2_colorMagenta;
	b2HexColor frictionColor = b2_colorYellow;

	b2HexColor graphColors[b2_graphColorCount] = { b2_colorRed,		  b2_colorOrange,	 b2_colorYellow, b2_colorGreen,
												   b2_colorCyan,	  b2_colorBlue,		 b2_colorViolet, b2_colorPink,
												   b2_colorChocolate, b2_colorGoldenrod, b2_colorCoral,	 b2_colorBlack };

	int bodyCapacity = b2GetIdCapacity( &world->bodyIdPool );
	b2SetBitCountAndClear( &world->debugBodySet, bodyCapacity );

	int jointCapacity = b2GetIdCapacity( &world->jointIdPool );
	b2SetBitCountAndClear( &world->debugJointSet, jointCapacity );

	int contactCapacity = b2GetIdCapacity( &world->contactIdPool );
	b2SetBitCountAndClear( &world->debugContactSet, contactCapacity );

	struct DrawContext drawContext = { world, draw };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Query( world->broadPhase.trees + i, draw->drawingBounds, b2_defaultMaskBits, DrawQueryCallback,
							 &drawContext );
	}

	uint32_t wordCount = world->debugBodySet.blockCount;
	uint64_t* bits = world->debugBodySet.bits;
	for ( uint32_t k = 0; k < wordCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = b2CTZ64( word );
			uint32_t bodyId = 64 * k + ctz;

			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );

			if ( draw->drawMass && body->type == b2_dynamicBody )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2Transform transform = { bodySim->center, bodySim->transform.q };
				draw->DrawTransform( transform, draw->context );

				b2Vec2 p = b2TransformPoint( transform, offset );

				char buffer[32];
				snprintf( buffer, 32, "  %.2f", bodySim->mass );
				draw->DrawString( p, buffer, draw->context );
			}

			if ( draw->drawJoints )
			{
				int jointKey = body->headJointKey;
				while ( jointKey != B2_NULL_INDEX )
				{
					int jointId = jointKey >> 1;
					int edgeIndex = jointKey & 1;
					b2Joint* joint = b2JointArray_Get( &world->joints, jointId );

					// avoid double draw
					if ( b2GetBit( &world->debugJointSet, jointId ) == false )
					{
						b2DrawJoint( draw, world, joint );
						b2SetBit( &world->debugJointSet, jointId );
					}
					else
					{
						// todo testing
						edgeIndex += 0;
					}

					jointKey = joint->edges[edgeIndex].nextKey;
				}
			}

			const float linearSlop = b2_linearSlop;
			if ( draw->drawContacts && body->type == b2_dynamicBody && body->setIndex == b2_awakeSet )
			{
				int contactKey = body->headContactKey;
				while ( contactKey != B2_NULL_INDEX )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;
					b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
					contactKey = contact->edges[edgeIndex].nextKey;

					if ( contact->setIndex != b2_awakeSet || contact->colorIndex == B2_NULL_INDEX )
					{
						continue;
					}

					// avoid double draw
					if ( b2GetBit( &world->debugContactSet, contactId ) == false )
					{
						B2_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount );

						b2GraphColor* gc = world->constraintGraph.colors + contact->colorIndex;
						b2ContactSim* contactSim = b2ContactSimArray_Get( &gc->contactSims, contact->localIndex );
						int pointCount = contactSim->manifold.pointCount;
						b2Vec2 normal = contactSim->manifold.normal;
						char buffer[32];

						for ( int j = 0; j < pointCount; ++j )
						{
							b2ManifoldPoint* point = contactSim->manifold.points + j;

							if ( draw->drawGraphColors )
							{
								// graph color
								float pointSize = contact->colorIndex == b2_overflowIndex ? 7.5f : 5.0f;
								draw->DrawPoint( point->point, pointSize, graphColors[contact->colorIndex], draw->context );
								// g_draw.DrawString(point->position, "%d", point->color);
							}
							else if ( point->separation > linearSlop )
							{
								// Speculative
								draw->DrawPoint( point->point, 5.0f, speculativeColor, draw->context );
							}
							else if ( point->persisted == false )
							{
								// Add
								draw->DrawPoint( point->point, 10.0f, addColor, draw->context );
							}
							else if ( point->persisted == true )
							{
								// Persist
								draw->DrawPoint( point->point, 5.0f, persistColor, draw->context );
							}

							if ( draw->drawContactNormals )
							{
								b2Vec2 p1 = point->point;
								b2Vec2 p2 = b2MulAdd( p1, k_axisScale, normal );
								draw->DrawSegment( p1, p2, normalColor, draw->context );
							}
							else if ( draw->drawContactImpulses )
							{
								b2Vec2 p1 = point->point;
								b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point->normalImpulse, normal );
								draw->DrawSegment( p1, p2, impulseColor, draw->context );
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.1f", 1000.0f * point->normalImpulse );
								draw->DrawString( p1, buffer, draw->context );
							}

							if ( draw->drawFrictionImpulses )
							{
								b2Vec2 tangent = b2RightPerp( normal );
								b2Vec2 p1 = point->point;
								b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point->tangentImpulse, tangent );
								draw->DrawSegment( p1, p2, frictionColor, draw->context );
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.1f", 1000.0f * point->tangentImpulse );
								draw->DrawString( p1, buffer, draw->context );
							}
						}

						b2SetBit( &world->debugContactSet, contactId );
					}
					else
					{
						// todo testing
						edgeIndex += 0;
					}

					contactKey = contact->edges[edgeIndex].nextKey;
				}
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}
}

void b2World_Draw( b2WorldId worldId, b2DebugDraw* draw )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	// todo it seems bounds drawing is fast enough for regular usage
	if ( draw->useDrawingBounds )
	{
		b2DrawWithBounds( world, draw );
		return;
	}

	if ( draw->drawShapes )
	{
		int setCount = world->solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
			int bodyCount = set->bodySims.count;
			for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
			{
				b2BodySim* bodySim = set->bodySims.data + bodyIndex;
				b2Body* body = b2BodyArray_Get( &world->bodies, bodySim->bodyId );
				B2_ASSERT( body->setIndex == setIndex );

				b2Transform xf = bodySim->transform;
				int shapeId = body->headShapeId;
				while ( shapeId != B2_NULL_INDEX )
				{
					b2Shape* shape = world->shapes.data + shapeId;
					b2HexColor color;

					if ( shape->customColor != 0 )
					{
						color = shape->customColor;
					}
					else if ( body->type == b2_dynamicBody && bodySim->mass == 0.0f )
					{
						// Bad body
						color = b2_colorRed;
					}
					else if ( body->setIndex == b2_disabledSet )
					{
						color = b2_colorSlateGray;
					}
					else if ( shape->isSensor )
					{
						color = b2_colorWheat;
					}
					else if ( bodySim->isBullet && body->setIndex == b2_awakeSet )
					{
						color = b2_colorTurquoise;
					}
					else if ( body->isSpeedCapped )
					{
						color = b2_colorYellow;
					}
					else if ( bodySim->isFast )
					{
						color = b2_colorSalmon;
					}
					else if ( body->type == b2_staticBody )
					{
						color = b2_colorPaleGreen;
					}
					else if ( body->type == b2_kinematicBody )
					{
						color = b2_colorRoyalBlue;
					}
					else if ( body->setIndex == b2_awakeSet )
					{
						color = b2_colorPink;
					}
					else
					{
						color = b2_colorGray;
					}

					b2DrawShape( draw, shape, xf, color );
					shapeId = shape->nextShapeId;
				}
			}
		}
	}

	if ( draw->drawJoints )
	{
		int count = world->joints.count;
		for ( int i = 0; i < count; ++i )
		{
			b2Joint* joint = world->joints.data + i;
			if ( joint->setIndex == B2_NULL_INDEX )
			{
				continue;
			}

			b2DrawJoint( draw, world, joint );
		}
	}

	if ( draw->drawAABBs )
	{
		b2HexColor color = b2_colorGold;

		int setCount = world->solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
			int bodyCount = set->bodySims.count;
			for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
			{
				b2BodySim* bodySim = set->bodySims.data + bodyIndex;

				char buffer[32];
				snprintf( buffer, 32, "%d", bodySim->bodyId );
				draw->DrawString( bodySim->center, buffer, draw->context );

				b2Body* body = b2BodyArray_Get( &world->bodies, bodySim->bodyId );
				B2_ASSERT( body->setIndex == setIndex );

				int shapeId = body->headShapeId;
				while ( shapeId != B2_NULL_INDEX )
				{
					b2Shape* shape = world->shapes.data + shapeId;
					b2AABB aabb = shape->fatAABB;

					b2Vec2 vs[4] = { { aabb.lowerBound.x, aabb.lowerBound.y },
									 { aabb.upperBound.x, aabb.lowerBound.y },
									 { aabb.upperBound.x, aabb.upperBound.y },
									 { aabb.lowerBound.x, aabb.upperBound.y } };

					draw->DrawPolygon( vs, 4, color, draw->context );

					shapeId = shape->nextShapeId;
				}
			}
		}
	}

	if ( draw->drawMass )
	{
		b2Vec2 offset = { 0.1f, 0.1f };
		int setCount = world->solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
			int bodyCount = set->bodySims.count;
			for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
			{
				b2BodySim* bodySim = set->bodySims.data + bodyIndex;

				b2Transform transform = { bodySim->center, bodySim->transform.q };
				draw->DrawTransform( transform, draw->context );

				b2Vec2 p = b2TransformPoint( transform, offset );

				char buffer[32];
				snprintf( buffer, 32, "  %.2f", bodySim->mass );
				draw->DrawString( p, buffer, draw->context );
			}
		}
	}

	if ( draw->drawContacts )
	{
		const float k_impulseScale = 1.0f;
		const float k_axisScale = 0.3f;
		const float linearSlop = b2_linearSlop;

		b2HexColor speculativeColor = b2_colorGray3;
		b2HexColor addColor = b2_colorGreen;
		b2HexColor persistColor = b2_colorBlue;
		b2HexColor normalColor = b2_colorGray9;
		b2HexColor impulseColor = b2_colorMagenta;
		b2HexColor frictionColor = b2_colorYellow;

		b2HexColor colors[b2_graphColorCount] = { b2_colorRed,		 b2_colorOrange,	b2_colorYellow, b2_colorGreen,
												  b2_colorCyan,		 b2_colorBlue,		b2_colorViolet, b2_colorPink,
												  b2_colorChocolate, b2_colorGoldenrod, b2_colorCoral,	b2_colorBlack };

		for ( int colorIndex = 0; colorIndex < b2_graphColorCount; ++colorIndex )
		{
			b2GraphColor* graphColor = world->constraintGraph.colors + colorIndex;

			int contactCount = graphColor->contactSims.count;
			for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
			{
				b2ContactSim* contact = graphColor->contactSims.data + contactIndex;
				int pointCount = contact->manifold.pointCount;
				b2Vec2 normal = contact->manifold.normal;
				char buffer[32];

				for ( int j = 0; j < pointCount; ++j )
				{
					b2ManifoldPoint* point = contact->manifold.points + j;

					if ( draw->drawGraphColors && 0 <= colorIndex && colorIndex <= b2_graphColorCount )
					{
						// graph color
						float pointSize = colorIndex == b2_overflowIndex ? 7.5f : 5.0f;
						draw->DrawPoint( point->point, pointSize, colors[colorIndex], draw->context );
						// g_draw.DrawString(point->position, "%d", point->color);
					}
					else if ( point->separation > linearSlop )
					{
						// Speculative
						draw->DrawPoint( point->point, 5.0f, speculativeColor, draw->context );
					}
					else if ( point->persisted == false )
					{
						// Add
						draw->DrawPoint( point->point, 10.0f, addColor, draw->context );
					}
					else if ( point->persisted == true )
					{
						// Persist
						draw->DrawPoint( point->point, 5.0f, persistColor, draw->context );
					}

					if ( draw->drawContactNormals )
					{
						b2Vec2 p1 = point->point;
						b2Vec2 p2 = b2MulAdd( p1, k_axisScale, normal );
						draw->DrawSegment( p1, p2, normalColor, draw->context );
					}
					else if ( draw->drawContactImpulses )
					{
						b2Vec2 p1 = point->point;
						b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point->normalImpulse, normal );
						draw->DrawSegment( p1, p2, impulseColor, draw->context );
						snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.2f", 1000.0f * point->normalImpulse );
						draw->DrawString( p1, buffer, draw->context );
					}

					if ( draw->drawFrictionImpulses )
					{
						b2Vec2 tangent = b2RightPerp( normal );
						b2Vec2 p1 = point->point;
						b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point->tangentImpulse, tangent );
						draw->DrawSegment( p1, p2, frictionColor, draw->context );
						snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.2f", point->normalImpulse );
						draw->DrawString( p1, buffer, draw->context );
					}
				}
			}
		}
	}
}

b2BodyEvents b2World_GetBodyEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return ( b2BodyEvents ){ 0 };
	}

	int count = world->bodyMoveEvents.count;
	b2BodyEvents events = { world->bodyMoveEvents.data, count };
	return events;
}

b2SensorEvents b2World_GetSensorEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return ( b2SensorEvents ){ 0 };
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world->endEventArrayIndex;

	int beginCount = world->sensorBeginEvents.count;
	int endCount = world->sensorEndEvents[endEventArrayIndex].count;

	b2SensorEvents events = {
		.beginEvents = world->sensorBeginEvents.data,
		.endEvents = world->sensorEndEvents[endEventArrayIndex].data,
		.beginCount = beginCount,
		.endCount = endCount,
	};
	return events;
}

b2ContactEvents b2World_GetContactEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return ( b2ContactEvents ){ 0 };
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world->endEventArrayIndex;

	int beginCount = world->contactBeginEvents.count;
	int endCount = world->contactEndEvents[endEventArrayIndex].count;
	int hitCount = world->contactHitEvents.count;

	b2ContactEvents events = {
		.beginEvents = world->contactBeginEvents.data,
		.endEvents = world->contactEndEvents[endEventArrayIndex].data,
		.hitEvents = world->contactHitEvents.data,
		.beginCount = beginCount,
		.endCount = endCount,
		.hitCount = hitCount,
	};

	return events;
}

bool b2World_IsValid( b2WorldId id )
{
	if ( id.index1 < 1 || b2_maxWorlds < id.index1 )
	{
		return false;
	}

	b2World* world = b2_worlds + ( id.index1 - 1 );

	if ( world->worldId != id.index1 - 1 )
	{
		// world is not allocated
		return false;
	}

	return id.revision == world->revision;
}

bool b2Body_IsValid( b2BodyId id )
{
	if ( id.world0 < 0 || b2_maxWorlds <= id.world0 )
	{
		// invalid world
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	if ( id.index1 < 1 || world->bodies.count < id.index1 )
	{
		// invalid index
		return false;
	}

	b2Body* body = world->bodies.data + ( id.index1 - 1 );
	if ( body->setIndex == B2_NULL_INDEX )
	{
		// this was freed
		return false;
	}

	B2_ASSERT( body->localIndex != B2_NULL_INDEX );

	if ( body->revision != id.revision )
	{
		// this id is orphaned
		return false;
	}

	return true;
}

bool b2Shape_IsValid( b2ShapeId id )
{
	if ( b2_maxWorlds <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int shapeId = id.index1 - 1;
	if ( shapeId < 0 || world->shapes.count <= shapeId )
	{
		return false;
	}

	b2Shape* shape = world->shapes.data + shapeId;
	if ( shape->id == B2_NULL_INDEX )
	{
		// shape is free
		return false;
	}

	B2_ASSERT( shape->id == shapeId );

	return id.revision == shape->revision;
}

bool b2Chain_IsValid( b2ChainId id )
{
	if ( id.world0 < 0 || b2_maxWorlds <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int chainId = id.index1 - 1;
	if ( chainId < 0 || world->chainShapes.count <= chainId )
	{
		return false;
	}

	b2ChainShape* chain = world->chainShapes.data + chainId;
	if ( chain->id == B2_NULL_INDEX )
	{
		// chain is free
		return false;
	}

	B2_ASSERT( chain->id == chainId );

	return id.revision == chain->revision;
}

bool b2Joint_IsValid( b2JointId id )
{
	if ( id.world0 < 0 || b2_maxWorlds <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int jointId = id.index1 - 1;
	if ( jointId < 0 || world->joints.count <= jointId )
	{
		return false;
	}

	b2Joint* joint = world->joints.data + jointId;
	if ( joint->jointId == B2_NULL_INDEX )
	{
		// joint is free
		return false;
	}

	B2_ASSERT( joint->jointId == jointId );

	return id.revision == joint->revision;
}

void b2World_EnableSleeping( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	if ( flag == world->enableSleep )
	{
		return;
	}

	world->enableSleep = flag;

	if ( flag == false )
	{
		int setCount = world->solverSets.count;
		for ( int i = b2_firstSleepingSet; i < setCount; ++i )
		{
			b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, i );
			if ( set->bodySims.count > 0 )
			{
				b2WakeSolverSet( world, i );
			}
		}
	}
}

bool b2World_IsSleepingEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->enableSleep;
}

void b2World_EnableWarmStarting( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->enableWarmStarting = flag;
}

bool b2World_IsWarmStartingEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->enableWarmStarting;
}

int b2World_GetAwakeBodyCount(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	return awakeSet->bodySims.count;
}

void b2World_EnableContinuous( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->enableContinuous = flag;
}

bool b2World_IsContinuousEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->enableContinuous;
}

void b2World_SetRestitutionThreshold( b2WorldId worldId, float value )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->restitutionThreshold = b2ClampFloat( value, 0.0f, FLT_MAX );
}

float b2World_GetRestitutionThreshold( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->restitutionThreshold;
}

void b2World_SetHitEventThreshold( b2WorldId worldId, float value )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->hitEventThreshold = b2ClampFloat( value, 0.0f, FLT_MAX );
}

float b2World_GetHitEventThreshold( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->hitEventThreshold;
}

void b2World_SetContactTuning( b2WorldId worldId, float hertz, float dampingRatio, float pushVelocity )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->contactHertz = b2ClampFloat( hertz, 0.0f, FLT_MAX );
	world->contactDampingRatio = b2ClampFloat( dampingRatio, 0.0f, FLT_MAX );
	world->contactPushoutVelocity = b2ClampFloat( pushVelocity, 0.0f, FLT_MAX );
}

void b2World_SetJointTuning( b2WorldId worldId, float hertz, float dampingRatio )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->jointHertz = b2ClampFloat( hertz, 0.0f, FLT_MAX );
	world->jointDampingRatio = b2ClampFloat( dampingRatio, 0.0f, FLT_MAX );
}

void b2World_SetMaximumLinearVelocity( b2WorldId worldId, float maximumLinearVelocity )
{
	B2_ASSERT( b2IsValidFloat( maximumLinearVelocity ) && maximumLinearVelocity > 0.0f );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	world->maxLinearVelocity = maximumLinearVelocity;
}

float b2World_GetMaximumLinearVelocity( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->maxLinearVelocity;
}

b2Profile b2World_GetProfile( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->profile;
}

b2Counters b2World_GetCounters( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Counters s = { 0 };
	s.bodyCount = b2GetIdCount( &world->bodyIdPool );
	s.shapeCount = b2GetIdCount( &world->shapeIdPool );
	s.contactCount = b2GetIdCount( &world->contactIdPool );
	s.jointCount = b2GetIdCount( &world->jointIdPool );
	s.islandCount = b2GetIdCount( &world->islandIdPool );

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;
	s.staticTreeHeight = b2DynamicTree_GetHeight( staticTree );

	b2DynamicTree* dynamicTree = world->broadPhase.trees + b2_dynamicBody;
	b2DynamicTree* kinematicTree = world->broadPhase.trees + b2_kinematicBody;
	s.treeHeight = b2MaxInt( b2DynamicTree_GetHeight( dynamicTree ), b2DynamicTree_GetHeight( kinematicTree ) );

	s.stackUsed = b2GetMaxStackAllocation( &world->stackAllocator );
	s.byteCount = b2GetByteCount();
	s.taskCount = world->taskCount;

	for ( int i = 0; i < b2_graphColorCount; ++i )
	{
		s.colorCounts[i] = world->constraintGraph.colors[i].contactSims.count + world->constraintGraph.colors[i].jointSims.count;
	}
	return s;
}

void b2World_SetUserData( b2WorldId worldId, void* userData )
{
	b2World* world = b2GetWorldFromId( worldId );
	world->userData = userData;
}

void* b2World_GetUserData( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->userData;
}

void b2World_DumpMemoryStats( b2WorldId worldId )
{
	FILE* file = fopen( "box2d_memory.txt", "w" );
	if ( file == NULL )
	{
		return;
	}

	b2World* world = b2GetWorldFromId( worldId );

	// id pools
	fprintf( file, "id pools\n" );
	fprintf( file, "body ids: %d\n", b2GetIdBytes( &world->bodyIdPool ) );
	fprintf( file, "solver set ids: %d\n", b2GetIdBytes( &world->solverSetIdPool ) );
	fprintf( file, "joint ids: %d\n", b2GetIdBytes( &world->jointIdPool ) );
	fprintf( file, "contact ids: %d\n", b2GetIdBytes( &world->contactIdPool ) );
	fprintf( file, "island ids: %d\n", b2GetIdBytes( &world->islandIdPool ) );
	fprintf( file, "shape ids: %d\n", b2GetIdBytes( &world->shapeIdPool ) );
	fprintf( file, "chain ids: %d\n", b2GetIdBytes( &world->chainIdPool ) );
	fprintf( file, "\n" );

	// world arrays
	fprintf( file, "world arrays\n" );
	fprintf( file, "bodies: %d\n", b2BodyArray_ByteCount( &world->bodies ) );
	fprintf( file, "solver sets: %d\n", b2SolverSetArray_ByteCount( &world->solverSets ) );
	fprintf( file, "joints: %d\n", b2JointArray_ByteCount( &world->joints ) );
	fprintf( file, "contacts: %d\n", b2ContactArray_ByteCount( &world->contacts ) );
	fprintf( file, "islands: %d\n", b2IslandArray_ByteCount( &world->islands ) );
	fprintf( file, "shapes: %d\n", b2ShapeArray_ByteCount( &world->shapes ) );
	fprintf( file, "chains: %d\n", b2ChainShapeArray_ByteCount( &world->chainShapes ) );
	fprintf( file, "\n" );

	// broad-phase
	fprintf( file, "broad-phase\n" );
	fprintf( file, "static tree: %d\n", b2DynamicTree_GetByteCount( world->broadPhase.trees + b2_staticBody ) );
	fprintf( file, "kinematic tree: %d\n", b2DynamicTree_GetByteCount( world->broadPhase.trees + b2_kinematicBody ) );
	fprintf( file, "dynamic tree: %d\n", b2DynamicTree_GetByteCount( world->broadPhase.trees + b2_dynamicBody ) );
	b2HashSet* moveSet = &world->broadPhase.moveSet;
	fprintf( file, "moveSet: %d (%d, %d)\n", b2GetHashSetBytes( moveSet ), moveSet->count, moveSet->capacity );
	fprintf( file, "moveArray: %d\n", b2IntArray_ByteCount( &world->broadPhase.moveArray ) );
	b2HashSet* pairSet = &world->broadPhase.pairSet;
	fprintf( file, "pairSet: %d (%d, %d)\n", b2GetHashSetBytes( pairSet ), pairSet->count, pairSet->capacity );
	fprintf( file, "\n" );

	// solver sets
	int bodySimCapacity = 0;
	int bodyStateCapacity = 0;
	int jointSimCapacity = 0;
	int contactSimCapacity = 0;
	int islandSimCapacity = 0;
	int solverSetCapacity = world->solverSets.count;
	for ( int i = 0; i < solverSetCapacity; ++i )
	{
		b2SolverSet* set = world->solverSets.data + i;
		if ( set->setIndex == B2_NULL_INDEX )
		{
			continue;
		}

		bodySimCapacity += set->bodySims.capacity;
		bodyStateCapacity += set->bodyStates.capacity;
		jointSimCapacity += set->jointSims.capacity;
		contactSimCapacity += set->contactSims.capacity;
		islandSimCapacity += set->islandSims.capacity;
	}

	fprintf( file, "solver sets\n" );
	fprintf( file, "body sim: %d\n", bodySimCapacity * (int)sizeof( b2BodySim ) );
	fprintf( file, "body state: %d\n", bodyStateCapacity * (int)sizeof( b2BodyState ) );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * (int)sizeof( b2JointSim ) );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * (int)sizeof( b2ContactSim ) );
	fprintf( file, "island sim: %d\n", islandSimCapacity * (int)sizeof( islandSimCapacity ) );
	fprintf( file, "\n" );

	// constraint graph
	int bodyBitSetBytes = 0;
	contactSimCapacity = 0;
	jointSimCapacity = 0;
	for ( int i = 0; i < b2_graphColorCount; ++i )
	{
		b2GraphColor* c = world->constraintGraph.colors + i;
		bodyBitSetBytes += b2GetBitSetBytes( &c->bodySet );
		contactSimCapacity += c->contactSims.capacity;
		jointSimCapacity += c->jointSims.capacity;
	}

	fprintf( file, "constraint graph\n" );
	fprintf( file, "body bit sets: %d\n", bodyBitSetBytes );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * (int)sizeof( b2JointSim ) );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * (int)sizeof( b2ContactSim ) );
	fprintf( file, "\n" );

	// stack allocator
	fprintf( file, "stack allocator: %d\n\n", world->stackAllocator.capacity );

	// chain shapes
	// todo

	fclose( file );
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	WorldQueryContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return true;
	}

	b2ShapeId id = { shapeId + 1, world->worldId, shape->revision };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b2TreeStats b2World_OverlapAABB( b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidAABB( aabb ) );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeQueryCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

typedef struct WorldOverlapContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	b2DistanceProxy proxy;
	b2Transform transform;
	void* userContext;
} WorldOverlapContext;

static bool TreeOverlapCallback( int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	WorldOverlapContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return true;
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = worldContext->proxy;
	input.proxyB = b2MakeShapeDistanceProxy( shape );
	input.transformA = worldContext->transform;
	input.transformB = transform;
	input.useRadii = true;

	b2DistanceCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	if ( output.distance > 0.0f )
	{
		return true;
	}

	b2ShapeId id = { shape->id + 1, world->worldId, shape->revision };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b2TreeStats b2World_OverlapPoint( b2WorldId worldId, b2Vec2 point, b2Transform transform, b2QueryFilter filter,
								  b2OverlapResultFcn* fcn, void* context )
{
	b2Circle circle = { point, 0.0f };
	return b2World_OverlapCircle( worldId, &circle, transform, filter, fcn, context );
}

b2TreeStats b2World_OverlapCircle( b2WorldId worldId, const b2Circle* circle, b2Transform transform, b2QueryFilter filter,
								   b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( transform.p ) );
	B2_ASSERT( b2IsValidRotation( transform.q ) );

	b2AABB aabb = b2ComputeCircleAABB( circle, transform );
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy( &circle->center, 1, circle->radius ), transform, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

b2TreeStats b2World_OverlapCapsule( b2WorldId worldId, const b2Capsule* capsule, b2Transform transform, b2QueryFilter filter,
									b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( transform.p ) );
	B2_ASSERT( b2IsValidRotation( transform.q ) );

	b2AABB aabb = b2ComputeCapsuleAABB( capsule, transform );
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy( &capsule->center1, 2, capsule->radius ), transform, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

b2TreeStats b2World_OverlapPolygon( b2WorldId worldId, const b2Polygon* polygon, b2Transform transform, b2QueryFilter filter,
									b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( transform.p ) );
	B2_ASSERT( b2IsValidRotation( transform.q ) );

	b2AABB aabb = b2ComputePolygonAABB( polygon, transform );
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy( polygon->vertices, polygon->count, polygon->radius ), transform, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

typedef struct WorldRayCastContext
{
	b2World* world;
	b2CastResultFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	void* userContext;
} WorldRayCastContext;

static float RayCastCallback( const b2RayCastInput* input, int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return input->maxFraction;
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	b2CastOutput output = b2RayCastShape( input, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world->worldId, shape->revision };
		float fraction = worldContext->fcn( id, output.point, output.normal, output.fraction, worldContext->userContext );
		worldContext->fraction = fraction;
		return fraction;
	}

	return input->maxFraction;
}

b2TreeStats b2World_CastRay( b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn,
							 void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2RayCastInput input = { origin, translation, 1.0f };

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_RayCast( world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
static float b2RayCastClosestFcn( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
{
	b2RayResult* rayResult = (b2RayResult*)context;
	rayResult->shapeId = shapeId;
	rayResult->point = point;
	rayResult->normal = normal;
	rayResult->fraction = fraction;
	rayResult->hit = true;
	return fraction;
}

b2RayResult b2World_CastRayClosest( b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter )
{
	b2RayResult result = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return result;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2RayCastInput input = { origin, translation, 1.0f };
	WorldRayCastContext worldContext = { world, b2RayCastClosestFcn, filter, 1.0f, &result };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_RayCast( world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext );
		result.nodeVisits += treeResult.nodeVisits;
		result.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return result;
		}

		input.maxFraction = worldContext.fraction;
	}

	return result;
}

static float ShapeCastCallback( const b2ShapeCastInput* input, int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return input->maxFraction;
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2CastOutput output = b2ShapeCastShape( input, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world->worldId, shape->revision };
		float fraction = worldContext->fcn( id, output.point, output.normal, output.fraction, worldContext->userContext );
		worldContext->fraction = fraction;
		return fraction;
	}

	return input->maxFraction;
}

b2TreeStats b2World_CastCircle( b2WorldId worldId, const b2Circle* circle, b2Transform originTransform, b2Vec2 translation,
								b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( originTransform.p ) );
	B2_ASSERT( b2IsValidRotation( originTransform.q ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	input.points[0] = b2TransformPoint( originTransform, circle->center );
	input.count = 1;
	input.radius = circle->radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_ShapeCast( world->broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

b2TreeStats b2World_CastCapsule( b2WorldId worldId, const b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation,
								 b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( originTransform.p ) );
	B2_ASSERT( b2IsValidRotation( originTransform.q ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	input.points[0] = b2TransformPoint( originTransform, capsule->center1 );
	input.points[1] = b2TransformPoint( originTransform, capsule->center2 );
	input.count = 2;
	input.radius = capsule->radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_ShapeCast( world->broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

b2TreeStats b2World_CastPolygon( b2WorldId worldId, const b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation,
								 b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( originTransform.p ) );
	B2_ASSERT( b2IsValidRotation( originTransform.q ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	for ( int i = 0; i < polygon->count; ++i )
	{
		input.points[i] = b2TransformPoint( originTransform, polygon->vertices[i] );
	}
	input.count = polygon->count;
	input.radius = polygon->radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_ShapeCast( world->broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

#if 0

void b2World_ShiftOrigin(b2WorldId worldId, b2Vec2 newOrigin)
{
	B2_ASSERT(m_locked == false);
	if (m_locked)
	{
		return;
	}

	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_xf.p -= newOrigin;
		b->m_sweep.c0 -= newOrigin;
		b->m_sweep.c -= newOrigin;
	}

	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void b2World_Dump()
{
	if (m_locked)
	{
		return;
	}

	b2OpenDump("box2d_dump.inl");

	b2Dump("b2Vec2 g(%.9g, %.9g);\n", m_gravity.x, m_gravity.y);
	b2Dump("m_world->SetGravity(g);\n");

	b2Dump("b2Body** sims = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);

	int32 i = 0;
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_islandIndex = i;
		b->Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j->Dump();
		b2Dump("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j->Dump();
		b2Dump("}\n");
	}

	b2Dump("b2Free(joints);\n");
	b2Dump("b2Free(sims);\n");
	b2Dump("joints = nullptr;\n");
	b2Dump("sims = nullptr;\n");

	b2CloseDump();
}
#endif

void b2World_SetCustomFilterCallback( b2WorldId worldId, b2CustomFilterFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	world->customFilterFcn = fcn;
	world->customFilterContext = context;
}

void b2World_SetPreSolveCallback( b2WorldId worldId, b2PreSolveFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b2World_SetGravity( b2WorldId worldId, b2Vec2 gravity )
{
	b2World* world = b2GetWorldFromId( worldId );
	world->gravity = gravity;
}

b2Vec2 b2World_GetGravity( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->gravity;
}

struct ExplosionContext
{
	b2World* world;
	b2Vec2 position;
	float radius;
	float falloff;
	float impulsePerLength;
};

static bool ExplosionCallback( int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	struct ExplosionContext* explosionContext = context;
	b2World* world = explosionContext->world;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );
	B2_ASSERT( body->type == b2_dynamicBody );

	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &explosionContext->position, 1, 0.0f );
	input.transformA = transform;
	input.transformB = b2Transform_identity;
	input.useRadii = true;

	b2DistanceCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	float radius = explosionContext->radius;
	float falloff = explosionContext->falloff;
	if ( output.distance > radius + falloff )
	{
		return true;
	}

	b2WakeBody( world, body );

	if ( body->setIndex != b2_awakeSet )
	{
		return true;
	}

	b2Vec2 closestPoint = output.pointA;
	if ( output.distance == 0.0f )
	{
		b2Vec2 localCentroid = b2GetShapeCentroid( shape );
		closestPoint = b2TransformPoint( transform, localCentroid );
	}

	b2Vec2 direction = b2Sub( closestPoint, explosionContext->position );
	if ( b2LengthSquared( direction ) > 100.0f * FLT_EPSILON * FLT_EPSILON )
	{
		direction = b2Normalize( direction );
	}
	else
	{
		direction = ( b2Vec2 ){ 1.0f, 0.0f };
	}

	b2Vec2 localLine = b2InvRotateVector( transform.q, b2LeftPerp( direction ) );
	float perimeter = b2GetShapeProjectedPerimeter( shape, localLine );
	float scale = 1.0f;
	if ( output.distance > radius && falloff > 0.0f )
	{
		scale = b2ClampFloat( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
	}

	float magnitude = explosionContext->impulsePerLength * perimeter * scale;
	b2Vec2 impulse = b2MulSV( magnitude, direction );

	int localIndex = body->localIndex;
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2BodyState* state = b2BodyStateArray_Get( &set->bodyStates, localIndex );
	b2BodySim* bodySim = b2BodySimArray_Get( &set->bodySims, localIndex );
	state->linearVelocity = b2MulAdd( state->linearVelocity, bodySim->invMass, impulse );
	state->angularVelocity += bodySim->invInertia * b2Cross( b2Sub( closestPoint, bodySim->center ), impulse );

	return true;
}

void b2World_Explode( b2WorldId worldId, const b2ExplosionDef* explosionDef )
{
	uint64_t maskBits = explosionDef->maskBits;
	b2Vec2 position = explosionDef->position;
	float radius = explosionDef->radius;
	float falloff = explosionDef->falloff;
	float impulsePerLength = explosionDef->impulsePerLength;

	B2_ASSERT( b2IsValidVec2( position ) );
	B2_ASSERT( b2IsValidFloat( radius ) && radius >= 0.0f );
	B2_ASSERT( b2IsValidFloat( falloff ) && falloff >= 0.0f );
	B2_ASSERT( b2IsValidFloat( impulsePerLength ) );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	struct ExplosionContext explosionContext = { world, position, radius, falloff, impulsePerLength };

	b2AABB aabb;
	aabb.lowerBound.x = position.x - ( radius + falloff );
	aabb.lowerBound.y = position.y - ( radius + falloff );
	aabb.upperBound.x = position.x + ( radius + falloff );
	aabb.upperBound.y = position.y + ( radius + falloff );

	b2DynamicTree_Query( world->broadPhase.trees + b2_dynamicBody, aabb, maskBits, ExplosionCallback, &explosionContext );
}

void b2World_RebuildStaticTree( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;
	b2DynamicTree_Rebuild( staticTree, true );
}

void b2World_EnableSpeculative(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId( worldId );
	world->enableSpeculative = flag;
}

#if B2_VALIDATE
// When validating islands ids I have to compare the root island
// ids because islands are not merged until the next time step.
static int b2GetRootIslandId( b2World* world, int islandId )
{
	if ( islandId == B2_NULL_INDEX )
	{
		return B2_NULL_INDEX;
	}

	b2Island* island = b2IslandArray_Get( &world->islands, islandId );

	int rootId = islandId;
	b2Island* rootIsland = island;
	while ( rootIsland->parentIsland != B2_NULL_INDEX )
	{
		b2Island* parent = b2IslandArray_Get( &world->islands, rootIsland->parentIsland );
		rootId = rootIsland->parentIsland;
		rootIsland = parent;
	}

	return rootId;
}

// This validates island graph connectivity for each body
void b2ValidateConnectivity( b2World* world )
{
	b2Body* bodies = world->bodies.data;
	int bodyCapacity = world->bodies.count;

	for ( int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex )
	{
		b2Body* body = bodies + bodyIndex;
		if ( body->id == B2_NULL_INDEX )
		{
			b2ValidateFreeId( &world->bodyIdPool, bodyIndex );
			continue;
		}

		B2_ASSERT( bodyIndex == body->id );

		// Need to get the root island because islands are not merged until the next time step
		int bodyIslandId = b2GetRootIslandId( world, body->islandId );
		int bodySetIndex = body->setIndex;

		int contactKey = body->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

			bool touching = ( contact->flags & b2_contactTouchingFlag ) != 0;
			if ( touching && ( contact->flags & b2_contactSensorFlag ) == 0 )
			{
				if ( bodySetIndex != b2_staticSet )
				{
					int contactIslandId = b2GetRootIslandId( world, contact->islandId );
					B2_ASSERT( contactIslandId == bodyIslandId );
				}
			}
			else
			{
				B2_ASSERT( contact->islandId == B2_NULL_INDEX );
			}

			contactKey = contact->edges[edgeIndex].nextKey;
		}

		int jointKey = body->headJointKey;
		while ( jointKey != B2_NULL_INDEX )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			b2Joint* joint = b2JointArray_Get( &world->joints, jointId );

			int otherEdgeIndex = edgeIndex ^ 1;

			b2Body* otherBody = b2BodyArray_Get( &world->bodies, joint->edges[otherEdgeIndex].bodyId );

			if ( bodySetIndex == b2_disabledSet || otherBody->setIndex == b2_disabledSet )
			{
				B2_ASSERT( joint->islandId == B2_NULL_INDEX );
			}
			else if ( bodySetIndex == b2_staticSet )
			{
				if ( otherBody->setIndex == b2_staticSet )
				{
					B2_ASSERT( joint->islandId == B2_NULL_INDEX );
				}
			}
			else
			{
				int jointIslandId = b2GetRootIslandId( world, joint->islandId );
				B2_ASSERT( jointIslandId == bodyIslandId );
			}

			jointKey = joint->edges[edgeIndex].nextKey;
		}
	}
}

// Validates solver sets, but not island connectivity
void b2ValidateSolverSets( b2World* world )
{
	B2_ASSERT( b2GetIdCapacity( &world->bodyIdPool ) == world->bodies.count );
	B2_ASSERT( b2GetIdCapacity( &world->contactIdPool ) == world->contacts.count );
	B2_ASSERT( b2GetIdCapacity( &world->jointIdPool ) == world->joints.count );
	B2_ASSERT( b2GetIdCapacity( &world->islandIdPool ) == world->islands.count );
	B2_ASSERT( b2GetIdCapacity( &world->solverSetIdPool ) == world->solverSets.count );

	int activeSetCount = 0;
	int totalBodyCount = 0;
	int totalJointCount = 0;
	int totalContactCount = 0;
	int totalIslandCount = 0;

	// Validate all solver sets
	int setCount = world->solverSets.count;
	for ( int setIndex = 0; setIndex < setCount; ++setIndex )
	{
		b2SolverSet* set = world->solverSets.data + setIndex;
		if ( set->setIndex != B2_NULL_INDEX )
		{
			activeSetCount += 1;

			if ( setIndex == b2_staticSet )
			{
				B2_ASSERT( set->contactSims.count == 0 );
				B2_ASSERT( set->islandSims.count == 0 );
				B2_ASSERT( set->bodyStates.count == 0 );
			}
			else if ( setIndex == b2_awakeSet )
			{
				B2_ASSERT( set->bodySims.count == set->bodyStates.count );
				B2_ASSERT( set->jointSims.count == 0 );
			}
			else if ( setIndex == b2_disabledSet )
			{
				B2_ASSERT( set->islandSims.count == 0 );
				B2_ASSERT( set->bodyStates.count == 0 );
			}
			else
			{
				B2_ASSERT( set->bodyStates.count == 0 );
			}

			// Validate bodies
			{
				b2Body* bodies = world->bodies.data;
				B2_ASSERT( set->bodySims.count >= 0 );
				totalBodyCount += set->bodySims.count;
				for ( int i = 0; i < set->bodySims.count; ++i )
				{
					b2BodySim* bodySim = set->bodySims.data + i;

					int bodyId = bodySim->bodyId;
					B2_ASSERT( 0 <= bodyId && bodyId < world->bodies.count );
					b2Body* body = bodies + bodyId;
					B2_ASSERT( body->setIndex == setIndex );
					B2_ASSERT( body->localIndex == i );
					B2_ASSERT( body->revision == body->revision );

					if ( setIndex == b2_disabledSet )
					{
						B2_ASSERT( body->headContactKey == B2_NULL_INDEX );
					}

					// Validate body shapes
					int prevShapeId = B2_NULL_INDEX;
					int shapeId = body->headShapeId;
					while ( shapeId != B2_NULL_INDEX )
					{
						b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );
						B2_ASSERT( shape->id == shapeId );
						B2_ASSERT( shape->prevShapeId == prevShapeId );

						if ( setIndex == b2_disabledSet )
						{
							B2_ASSERT( shape->proxyKey == B2_NULL_INDEX );
						}
						else if ( setIndex == b2_staticSet )
						{
							B2_ASSERT( B2_PROXY_TYPE( shape->proxyKey ) == b2_staticBody );
						}
						else
						{
							b2BodyType proxyType = B2_PROXY_TYPE( shape->proxyKey );
							B2_ASSERT( proxyType == b2_kinematicBody || proxyType == b2_dynamicBody );
						}

						prevShapeId = shapeId;
						shapeId = shape->nextShapeId;
					}

					// Validate body contacts
					int contactKey = body->headContactKey;
					while ( contactKey != B2_NULL_INDEX )
					{
						int contactId = contactKey >> 1;
						int edgeIndex = contactKey & 1;

						b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
						B2_ASSERT( contact->setIndex != b2_staticSet );
						B2_ASSERT( contact->edges[0].bodyId == bodyId || contact->edges[1].bodyId == bodyId );
						contactKey = contact->edges[edgeIndex].nextKey;
					}

					// Validate body joints
					int jointKey = body->headJointKey;
					while ( jointKey != B2_NULL_INDEX )
					{
						int jointId = jointKey >> 1;
						int edgeIndex = jointKey & 1;

						b2Joint* joint = b2JointArray_Get( &world->joints, jointId );

						int otherEdgeIndex = edgeIndex ^ 1;

						b2Body* otherBody = b2BodyArray_Get( &world->bodies, joint->edges[otherEdgeIndex].bodyId );

						if ( setIndex == b2_disabledSet || otherBody->setIndex == b2_disabledSet )
						{
							B2_ASSERT( joint->setIndex == b2_disabledSet );
						}
						else if ( setIndex == b2_staticSet && otherBody->setIndex == b2_staticSet )
						{
							B2_ASSERT( joint->setIndex == b2_staticSet );
						}
						else if ( setIndex == b2_awakeSet )
						{
							B2_ASSERT( joint->setIndex == b2_awakeSet );
						}
						else if ( setIndex >= b2_firstSleepingSet )
						{
							B2_ASSERT( joint->setIndex == setIndex );
						}

						b2JointSim* jointSim = b2GetJointSim( world, joint );
						B2_ASSERT( jointSim->jointId == jointId );
						B2_ASSERT( jointSim->bodyIdA == joint->edges[0].bodyId );
						B2_ASSERT( jointSim->bodyIdB == joint->edges[1].bodyId );

						jointKey = joint->edges[edgeIndex].nextKey;
					}
				}
			}

			// Validate contacts
			{
				B2_ASSERT( set->contactSims.count >= 0 );
				totalContactCount += set->contactSims.count;
				for ( int i = 0; i < set->contactSims.count; ++i )
				{
					b2ContactSim* contactSim = set->contactSims.data + i;
					b2Contact* contact = b2ContactArray_Get( &world->contacts, contactSim->contactId );
					if ( setIndex == b2_awakeSet )
					{
						// contact should be non-touching if awake
						// or it could be this contact hasn't been transferred yet
						B2_ASSERT( contactSim->manifold.pointCount == 0 ||
								   ( contactSim->simFlags & b2_simStartedTouching ) != 0 );
					}
					B2_ASSERT( contact->setIndex == setIndex );
					B2_ASSERT( contact->colorIndex == B2_NULL_INDEX );
					B2_ASSERT( contact->localIndex == i );
				}
			}

			// Validate joints
			{
				B2_ASSERT( set->jointSims.count >= 0 );
				totalJointCount += set->jointSims.count;
				for ( int i = 0; i < set->jointSims.count; ++i )
				{
					b2JointSim* jointSim = set->jointSims.data + i;
					b2Joint* joint = b2JointArray_Get( &world->joints, jointSim->jointId );
					B2_ASSERT( joint->setIndex == setIndex );
					B2_ASSERT( joint->colorIndex == B2_NULL_INDEX );
					B2_ASSERT( joint->localIndex == i );
				}
			}

			// Validate islands
			{
				B2_ASSERT( set->islandSims.count >= 0 );
				totalIslandCount += set->islandSims.count;
				for ( int i = 0; i < set->islandSims.count; ++i )
				{
					b2IslandSim* islandSim = set->islandSims.data + i;
					b2Island* island = b2IslandArray_Get( &world->islands, islandSim->islandId );
					B2_ASSERT( island->setIndex == setIndex );
					B2_ASSERT( island->localIndex == i );
				}
			}
		}
		else
		{
			B2_ASSERT( set->bodySims.count == 0 );
			B2_ASSERT( set->contactSims.count == 0 );
			B2_ASSERT( set->jointSims.count == 0 );
			B2_ASSERT( set->islandSims.count == 0 );
			B2_ASSERT( set->bodyStates.count == 0 );
		}
	}

	int setIdCount = b2GetIdCount( &world->solverSetIdPool );
	B2_ASSERT( activeSetCount == setIdCount );

	int bodyIdCount = b2GetIdCount( &world->bodyIdPool );
	B2_ASSERT( totalBodyCount == bodyIdCount );

	int islandIdCount = b2GetIdCount( &world->islandIdPool );
	B2_ASSERT( totalIslandCount == islandIdCount );

	// Validate constraint graph
	for ( int colorIndex = 0; colorIndex < b2_graphColorCount; ++colorIndex )
	{
		b2GraphColor* color = world->constraintGraph.colors + colorIndex;
		{
			B2_ASSERT( color->contactSims.count >= 0 );
			totalContactCount += color->contactSims.count;
			for ( int i = 0; i < color->contactSims.count; ++i )
			{
				b2ContactSim* contactSim = color->contactSims.data + i;
				b2Contact* contact = b2ContactArray_Get( &world->contacts, contactSim->contactId );
				// contact should be touching in the constraint graph or awaiting transfer to non-touching
				B2_ASSERT( contactSim->manifold.pointCount > 0 ||
						   ( contactSim->simFlags & ( b2_simStoppedTouching | b2_simDisjoint ) ) != 0 );
				B2_ASSERT( contact->setIndex == b2_awakeSet );
				B2_ASSERT( contact->colorIndex == colorIndex );
				B2_ASSERT( contact->localIndex == i );

				int bodyIdA = contact->edges[0].bodyId;
				int bodyIdB = contact->edges[1].bodyId;

				if ( colorIndex < b2_overflowIndex )
				{
					b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
					b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );
					B2_ASSERT( b2GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type != b2_staticBody ) );
					B2_ASSERT( b2GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type != b2_staticBody ) );
				}
			}
		}

		{
			B2_ASSERT( color->jointSims.count >= 0 );
			totalJointCount += color->jointSims.count;
			for ( int i = 0; i < color->jointSims.count; ++i )
			{
				b2JointSim* jointSim = color->jointSims.data + i;
				b2Joint* joint = b2JointArray_Get( &world->joints, jointSim->jointId );
				B2_ASSERT( joint->setIndex == b2_awakeSet );
				B2_ASSERT( joint->colorIndex == colorIndex );
				B2_ASSERT( joint->localIndex == i );

				int bodyIdA = joint->edges[0].bodyId;
				int bodyIdB = joint->edges[1].bodyId;

				if ( colorIndex < b2_overflowIndex )
				{
					b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
					b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );
					B2_ASSERT( b2GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type != b2_staticBody ) );
					B2_ASSERT( b2GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type != b2_staticBody ) );
				}
			}
		}
	}

	int contactIdCount = b2GetIdCount( &world->contactIdPool );
	B2_ASSERT( totalContactCount == contactIdCount );
	B2_ASSERT( totalContactCount == (int)world->broadPhase.pairSet.count );

	int jointIdCount = b2GetIdCount( &world->jointIdPool );
	B2_ASSERT( totalJointCount == jointIdCount );

// Validate shapes
// This is very slow on compounds
#if 0
	int shapeCapacity = b2Array(world->shapeArray).count;
	for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
	{
		b2Shape* shape = world->shapeArray + shapeIndex;
		if (shape->id != shapeIndex)
		{
			continue;
		}

		B2_ASSERT(0 <= shape->bodyId && shape->bodyId < b2Array(world->bodyArray).count);

		b2Body* body = world->bodyArray + shape->bodyId;
		B2_ASSERT(0 <= body->setIndex && body->setIndex < b2Array(world->solverSetArray).count);

		b2SolverSet* set = world->solverSetArray + body->setIndex;
		B2_ASSERT(0 <= body->localIndex && body->localIndex < set->sims.count);

		b2BodySim* bodySim = set->sims.data + body->localIndex;
		B2_ASSERT(bodySim->bodyId == shape->bodyId);

		bool found = false;
		int shapeCount = 0;
		int index = body->headShapeId;
		while (index != B2_NULL_INDEX)
		{
			b2CheckId(world->shapeArray, index);
			b2Shape* s = world->shapeArray + index;
			if (index == shapeIndex)
			{
				found = true;
			}

			index = s->nextShapeId;
			shapeCount += 1;
		}

		B2_ASSERT(found);
		B2_ASSERT(shapeCount == body->shapeCount);
	}
#endif
}

// Validate contact touching status.
void b2ValidateContacts( b2World* world )
{
	int contactCount = world->contacts.count;
	B2_ASSERT( contactCount == b2GetIdCapacity( &world->contactIdPool ) );
	int allocatedContactCount = 0;

	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactIndex );
		if ( contact->contactId == B2_NULL_INDEX )
		{
			continue;
		}

		B2_ASSERT( contact->contactId == contactIndex );

		allocatedContactCount += 1;

		bool touching = ( contact->flags & b2_contactTouchingFlag ) != 0;
		bool sensorTouching = ( contact->flags & b2_contactSensorTouchingFlag ) != 0;
		bool isSensor = ( contact->flags & b2_contactSensorFlag ) != 0;

		B2_ASSERT( touching == false || sensorTouching == false );
		B2_ASSERT( touching == false || isSensor == false );

		int setId = contact->setIndex;

		if ( setId == b2_awakeSet )
		{
			// If touching and not a sensor
			if ( touching && isSensor == false )
			{
				B2_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount );
			}
			else
			{
				B2_ASSERT( contact->colorIndex == B2_NULL_INDEX );
			}
		}
		else if ( setId >= b2_firstSleepingSet )
		{
			// Only touching contacts allowed in a sleeping set
			B2_ASSERT( touching == true && isSensor == false );
		}
		else
		{
			// Sleeping and non-touching contacts or sensor contacts belong in the disabled set
			B2_ASSERT( touching == false && setId == b2_disabledSet );
		}

		b2ContactSim* contactSim = b2GetContactSim( world, contact );
		B2_ASSERT( contactSim->contactId == contactIndex );
		B2_ASSERT( contactSim->bodyIdA == contact->edges[0].bodyId );
		B2_ASSERT( contactSim->bodyIdB == contact->edges[1].bodyId );

		// Sim touching is true for solid and sensor contacts
		bool simTouching = ( contactSim->simFlags & b2_simTouchingFlag ) != 0;
		B2_ASSERT( touching == simTouching || sensorTouching == simTouching );

		B2_ASSERT( 0 <= contactSim->manifold.pointCount && contactSim->manifold.pointCount <= 2 );
	}

	int contactIdCount = b2GetIdCount( &world->contactIdPool );
	B2_ASSERT( allocatedContactCount == contactIdCount );
}

#else

void b2ValidateConnectivity( b2World* world )
{
	B2_MAYBE_UNUSED( world );
}

void b2ValidateSolverSets( b2World* world )
{
	B2_MAYBE_UNUSED( world );
}

void b2ValidateContacts( b2World* world )
{
	B2_MAYBE_UNUSED( world );
}

#endif
