// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "physics_world.h"

#include "aabb.h"
#include "arena_allocator.h"
#include "bitset.h"
#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "parallel_for.h"
#include "recording.h"
#include "scheduler.h"
#include "sensor.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

_Static_assert( B2_MAX_WORLDS > 0, "must be 1 or more" );
_Static_assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
static b2World b2_worlds[B2_MAX_WORLDS];

static b2World* b2GetUnlockedWorldFromId( b2WorldId id )
{
	B2_ASSERT( 1 <= id.index1 && id.index1 <= B2_MAX_WORLDS );
	b2World* world = b2_worlds + ( id.index1 - 1 );
	B2_ASSERT( id.index1 == world->worldId + 1 );
	B2_ASSERT( id.generation == world->generation );

	// A world accessed from an id should not be locked
	if ( world->locked )
	{
		B2_ASSERT( false );
		return NULL;
	}
	return world;
}

b2World* b2GetWorldFromId( b2WorldId id )
{
	B2_ASSERT( 1 <= id.index1 && id.index1 <= B2_MAX_WORLDS );
	b2World* world = b2_worlds + ( id.index1 - 1 );
	B2_ASSERT( id.index1 == world->worldId + 1 );
	B2_ASSERT( id.generation == world->generation );
	return world;
}

b2World* b2GetWorld( int index )
{
	B2_ASSERT( 0 <= index && index < B2_MAX_WORLDS );
	b2World* world = b2_worlds + index;
	B2_ASSERT( world->worldId == index );
	return world;
}

b2World* b2GetWorldLocked( int index )
{
	B2_ASSERT( 0 <= index && index < B2_MAX_WORLDS );
	b2World* world = b2_worlds + index;
	B2_ASSERT( world->worldId == index );
	if ( world->locked )
	{
		B2_ASSERT( false );
		return NULL;
	}

	return world;
}

static void* b2DefaultAddTaskFcn( b2TaskCallback* task, void* taskContext, void* userContext )
{
	B2_UNUSED( userContext );
	task( taskContext );
	return NULL;
}

static void b2DefaultFinishTaskFcn( void* userTask, void* userContext )
{
	B2_UNUSED( userTask, userContext );
}

static float b2DefaultFrictionCallback( float frictionA, uint64_t materialA, float frictionB, uint64_t materialB )
{
	B2_UNUSED( materialA, materialB );
	return sqrtf( frictionA * frictionB );
}

static float b2DefaultRestitutionCallback( float restitutionA, uint64_t materialA, float restitutionB, uint64_t materialB )
{
	B2_UNUSED( materialA, materialB );
	return b2MaxFloat( restitutionA, restitutionB );
}

static void b2CreateWorkerContexts( b2World* world )
{
	b2Array_Create( world->taskContexts );
	b2Array_ResizeAndSetZero( world->taskContexts, world->workerCount );

	b2Array_Create( world->sensorTaskContexts );
	b2Array_ResizeAndSetZero( world->sensorTaskContexts, world->workerCount );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2Array_CreateN( world->taskContexts.data[i].sensorHits, 8 );
		world->taskContexts.data[i].contactStateBitSet = b2CreateBitSet( 1024 );
		world->taskContexts.data[i].hitEventBitSet = b2CreateBitSet( 1024 );
		world->taskContexts.data[i].hasHitEvents = false;
		world->taskContexts.data[i].jointStateBitSet = b2CreateBitSet( 1024 );
		world->taskContexts.data[i].enlargedSimBitSet = b2CreateBitSet( 256 );
		world->taskContexts.data[i].awakeIslandBitSet = b2CreateBitSet( 256 );
		world->taskContexts.data[i].splitIslandId = B2_NULL_INDEX;

		world->sensorTaskContexts.data[i].eventBits = b2CreateBitSet( 128 );
	}
}

static void b2DestroyWorkerContexts( b2World* world )
{
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2Array_Destroy( world->taskContexts.data[i].sensorHits );
		b2DestroyBitSet( &world->taskContexts.data[i].contactStateBitSet );
		b2DestroyBitSet( &world->taskContexts.data[i].hitEventBitSet );
		b2DestroyBitSet( &world->taskContexts.data[i].jointStateBitSet );
		b2DestroyBitSet( &world->taskContexts.data[i].enlargedSimBitSet );
		b2DestroyBitSet( &world->taskContexts.data[i].awakeIslandBitSet );

		b2DestroyBitSet( &world->sensorTaskContexts.data[i].eventBits );
	}

	b2Array_Destroy( world->taskContexts );
	b2Array_Destroy( world->sensorTaskContexts );
}

b2WorldId b2CreateWorld( const b2WorldDef* def )
{
	_Static_assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
	B2_CHECK_DEF( def );

	int worldId = B2_NULL_INDEX;
	for ( int i = 0; i < B2_MAX_WORLDS; ++i )
	{
		if ( b2_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}

	if ( worldId == B2_NULL_INDEX )
	{
		return (b2WorldId){ 0 };
	}

	b2InitializeContactRegisters();

	b2World* world = b2_worlds + worldId;
	uint16_t generation = world->generation;

	*world = (b2World){ 0 };

	world->worldId = (uint16_t)worldId;
	world->generation = generation;
	world->inUse = true;

	world->stack = b2CreateStack( 2048 );
	b2CreateBroadPhase( &world->broadPhase, &def->capacity );
	b2CreateGraph( &world->constraintGraph, &def->capacity );

	// pools
	world->bodyIdPool = b2CreateIdPool();

	int bodyCapacity = b2MaxInt( 16, def->capacity.staticBodyCount + def->capacity.dynamicBodyCount );
	b2Array_CreateN( world->bodies, bodyCapacity );
	b2Array_CreateN( world->solverSets, 8 );

	// add empty static, active, and disabled body sets
	world->solverSetIdPool = b2CreateIdPool();
	b2SolverSet set = { 0 };

	// static set
	set.setIndex = b2AllocId( &world->solverSetIdPool );
	b2Array_Push( world->solverSets, set );
	b2Array_Reserve( world->solverSets.data[b2_staticSet].bodySims, b2MaxInt( 16, def->capacity.staticBodyCount ) );
	B2_ASSERT( world->solverSets.data[b2_staticSet].setIndex == b2_staticSet );

	// disabled set
	set.setIndex = b2AllocId( &world->solverSetIdPool );
	b2Array_Push( world->solverSets, set );
	B2_ASSERT( world->solverSets.data[b2_disabledSet].setIndex == b2_disabledSet );

	// awake set
	set.setIndex = b2AllocId( &world->solverSetIdPool );
	b2Array_Push( world->solverSets, set );
	b2Array_Reserve( world->solverSets.data[b2_awakeSet].bodySims, b2MaxInt( 16, def->capacity.dynamicBodyCount ) );
	b2Array_Reserve( world->solverSets.data[b2_awakeSet].bodyStates, b2MaxInt( 16, def->capacity.dynamicBodyCount ) );
	b2Array_Reserve( world->solverSets.data[b2_awakeSet].contactSims, b2MaxInt( 16, def->capacity.contactCount ) );
	B2_ASSERT( world->solverSets.data[b2_awakeSet].setIndex == b2_awakeSet );

	world->shapeIdPool = b2CreateIdPool();

	int shapeCapacity = b2MaxInt( 16, def->capacity.staticShapeCount + def->capacity.dynamicShapeCount );
	b2Array_CreateN( world->shapes, shapeCapacity );

	world->chainIdPool = b2CreateIdPool();
	b2Array_CreateN( world->chainShapes, 4 );

	world->contactIdPool = b2CreateIdPool();
	b2Array_CreateN( world->contacts, b2MaxInt( 16, def->capacity.contactCount ) );

	world->jointIdPool = b2CreateIdPool();
	b2Array_CreateN( world->joints, 16 );

	world->islandIdPool = b2CreateIdPool();
	b2Array_CreateN( world->islands, b2MaxInt( 16, def->capacity.dynamicBodyCount ) );

	b2Array_CreateN( world->sensors, 4 );

	b2Array_CreateN( world->bodyMoveEvents, 4 );
	b2Array_CreateN( world->sensorBeginEvents, 4 );
	b2Array_CreateN( world->sensorEndEvents[0], 4 );
	b2Array_CreateN( world->sensorEndEvents[1], 4 );
	b2Array_CreateN( world->contactBeginEvents, 4 );
	b2Array_CreateN( world->contactEndEvents[0], 4 );
	b2Array_CreateN( world->contactEndEvents[1], 4 );
	b2Array_CreateN( world->contactHitEvents, 4 );
	b2Array_CreateN( world->jointEvents, 4 );
	world->endEventArrayIndex = 0;

	world->stepIndex = 0;
	world->splitIslandId = B2_NULL_INDEX;
	world->activeTaskCount = 0;
	world->taskCount = 0;
	world->gravity = def->gravity;
	world->hitEventThreshold = def->hitEventThreshold;
	world->restitutionThreshold = def->restitutionThreshold;
	world->maxLinearSpeed = def->maximumLinearSpeed;
	world->contactSpeed = def->contactSpeed;
	world->contactHertz = def->contactHertz;
	world->contactDampingRatio = def->contactDampingRatio;
	world->contactRecycleDistance = B2_CONTACT_RECYCLE_DISTANCE;

	if ( def->frictionCallback == NULL )
	{
		world->frictionCallback = b2DefaultFrictionCallback;
	}
	else
	{
		world->frictionCallback = def->frictionCallback;
	}

	if ( def->restitutionCallback == NULL )
	{
		world->restitutionCallback = b2DefaultRestitutionCallback;
	}
	else
	{
		world->restitutionCallback = def->restitutionCallback;
	}

	world->enableSleep = def->enableSleep;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContactSoftening = def->enableContactSoftening;
	world->enableContinuous = def->enableContinuous;
	world->enableSpeculative = true;
	world->userTreeTask = NULL;
	world->userData = def->userData;

	if ( def->workerCount > 0 && def->enqueueTask != NULL && def->finishTask != NULL )
	{
		// External task system
		world->workerCount = b2MinInt( def->workerCount, B2_MAX_WORKERS );
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->userTaskContext = def->userTaskContext;
		world->scheduler = NULL;
	}
	else if ( def->workerCount > 1 )
	{
		// Built-in scheduler
		world->workerCount = b2MinInt( def->workerCount, B2_MAX_WORKERS );
		world->scheduler = b2CreateScheduler( world->workerCount );
		world->enqueueTaskFcn = b2SchedulerEnqueueTask;
		world->finishTaskFcn = b2SchedulerFinishTask;
		world->userTaskContext = world->scheduler;
	}
	else
	{
		// Serial fallback
		world->workerCount = 1;
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->userTaskContext = NULL;
		world->scheduler = NULL;
	}

	b2CreateWorkerContexts( world );

	world->debugBodySet = b2CreateBitSet( 256 );
	world->debugJointSet = b2CreateBitSet( 256 );
	world->debugContactSet = b2CreateBitSet( 256 );
	world->debugIslandSet = b2CreateBitSet( 256 );

	// Recording is started by the host with b2World_StartRecording, never from the world def
	world->recording = NULL;

	// add one to worldId so that 0 represents a null b2WorldId
	return (b2WorldId){ (uint16_t)( worldId + 1 ), world->generation };
}

void b2DestroyWorld( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );

	// Detach any recording before teardown; the host owns and frees the recording buffer
	b2StopRecordingInternal( world );

	if ( world->scheduler != NULL )
	{
		b2DestroyScheduler( world->scheduler );
		world->scheduler = NULL;
	}

	b2DestroyBitSet( &world->debugBodySet );
	b2DestroyBitSet( &world->debugJointSet );
	b2DestroyBitSet( &world->debugContactSet );
	b2DestroyBitSet( &world->debugIslandSet );

	b2DestroyWorkerContexts( world );

	b2Array_Destroy( world->bodyMoveEvents );
	b2Array_Destroy( world->sensorBeginEvents );
	b2Array_Destroy( world->sensorEndEvents[0] );
	b2Array_Destroy( world->sensorEndEvents[1] );
	b2Array_Destroy( world->contactBeginEvents );
	b2Array_Destroy( world->contactEndEvents[0] );
	b2Array_Destroy( world->contactEndEvents[1] );
	b2Array_Destroy( world->contactHitEvents );
	b2Array_Destroy( world->jointEvents );

	int chainCapacity = world->chainShapes.count;
	for ( int i = 0; i < chainCapacity; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		if ( chain->id != B2_NULL_INDEX )
		{
			b2FreeChainData( chain );
		}
		else
		{
			B2_ASSERT( chain->shapeIndices == NULL );
			B2_ASSERT( chain->materials == NULL );
		}
	}

	int sensorCount = world->sensors.count;
	for ( int i = 0; i < sensorCount; ++i )
	{
		b2Array_Destroy( world->sensors.data[i].hits );
		b2Array_Destroy( world->sensors.data[i].overlaps1 );
		b2Array_Destroy( world->sensors.data[i].overlaps2 );
	}

	b2Array_Destroy( world->sensors );

	b2Array_Destroy( world->bodies );
	b2Array_Destroy( world->shapes );
	b2Array_Destroy( world->chainShapes );
	b2Array_Destroy( world->contacts );
	b2Array_Destroy( world->joints );

	for ( int i = 0; i < world->islands.count; ++i )
	{
		b2Array_Destroy( world->islands.data[i].bodies );
		b2Array_Destroy( world->islands.data[i].contacts );
		b2Array_Destroy( world->islands.data[i].joints );
	}
	b2Array_Destroy( world->islands );

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

	b2Array_Destroy( world->solverSets );

	b2DestroyGraph( &world->constraintGraph );
	b2DestroyBroadPhase( &world->broadPhase );

	b2DestroyIdPool( &world->bodyIdPool );
	b2DestroyIdPool( &world->shapeIdPool );
	b2DestroyIdPool( &world->chainIdPool );
	b2DestroyIdPool( &world->contactIdPool );
	b2DestroyIdPool( &world->jointIdPool );
	b2DestroyIdPool( &world->islandIdPool );
	b2DestroyIdPool( &world->solverSetIdPool );

	b2DestroyStack( &world->stack );

	// Wipe world but preserve generation
	uint16_t generation = world->generation;
	*world = (b2World){ 0 };
	world->worldId = 0;
	world->generation = generation + 1;
}

static inline float b2RelativeCos( b2Rot a, b2Rot b )
{
	return a.c * b.c + a.s * b.s;
}

static void b2CollideTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2TracyCZoneNC( collide_task, "Collide", b2_colorDodgerBlue, true );

	b2StepContext* stepContext = context;
	b2World* world = stepContext->world;
	b2TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b2ContactSim** contactSims = stepContext->contactSims;
	b2Shape* shapes = world->shapes.data;
	b2Body* bodies = world->bodies.data;

	B2_ASSERT( startIndex < endIndex );

	float recycleDistance = world->contactRecycleDistance;
	float speculativeDistance = B2_SPECULATIVE_DISTANCE;
	float recycleDistanceNonTouching = b2MinFloat( recycleDistance, speculativeDistance );

	for ( int contactIndex = startIndex; contactIndex < endIndex; ++contactIndex )
	{
		b2ContactSim* contactSim = contactSims[contactIndex];

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
			b2WorldTransform transformA = bodySimA->transform;
			b2WorldTransform transformB = bodySimB->transform;

			// These may not be skipped by relative transform check below
			contactSim->bodySimIndexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
			contactSim->invMassA = bodySimA->invMass;
			contactSim->invIA = bodySimA->invInertia;

			contactSim->bodySimIndexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;
			contactSim->invMassB = bodySimB->invMass;
			contactSim->invIB = bodySimB->invInertia;

			// Contact recycling optimization. Please cite this code if you use this optimization.
			// This is inspired by persistent contact manifolds used in some physics engines, such as PhysX.
			// However, this allows larger relative motion and has fewer tuning parameters (just one).
			if ( recycleDistance > 0.0f && ( contactSim->simFlags & b2_simRelativeTransformValid ) &&
				 ( contactSim->simFlags & b2_contactRecycleFlag ) )
			{
				b2Rot cachedQA = contactSim->cachedRotationA;
				b2Rot cachedQB = contactSim->cachedRotationB;
				b2Transform xfc = contactSim->cachedRelativePose;
				b2Transform xf = b2InvMulWorldTransforms( transformA, transformB );

				float cosA = b2RelativeCos( transformA.q, cachedQA );
				float cosB = b2RelativeCos( transformB.q, cachedQB );
				float minCos = b2MinFloat( cosA, cosB );

				float maxExtentA = bodyA->type == b2_staticBody ? 0.0f : bodySimA->maxExtent;
				float maxExtentB = bodyB->type == b2_staticBody ? 0.0f : bodySimB->maxExtent;
				float maxExtent = b2MaxFloat( maxExtentA, maxExtentB );
				float distance = b2Distance( xf.p, xfc.p );
				b2Rot qr = b2InvMulRot( xf.q, xfc.q );

				// This metric is used for fast bodies and sleeping. It comes from conservative advancement.
				// Note that qr.s == sin(theta) ~= theta for small angles.
				// Need a tighter tolerance for non-touching shapes so that contacts are not missed.
				float tolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

				if ( minCos > B2_CONTACT_RECYCLE_COS_ANGLE && distance + maxExtent * b2AbsFloat( qr.s ) < tolerance )
				{
					b2Rot dqA = b2MulRot( transformA.q, b2InvertRot( cachedQA ) );
					b2Rot dqB = b2MulRot( transformB.q, b2InvertRot( cachedQB ) );
					b2Vec2 normal = contactSim->manifold.normal;

					// Minimize round-off
					b2Vec2 dc = b2SubPos( bodySimB->center, bodySimA->center );

					for ( int i = 0; i < contactSim->manifold.pointCount; ++i )
					{
						// Keep anchors but update separation, same as sub-stepping. This eliminates jitter.
						b2ManifoldPoint* mp = contactSim->manifold.points + i;
						b2Vec2 rA = b2RotateVector( dqA, mp->anchorA );
						b2Vec2 rB = b2RotateVector( dqB, mp->anchorB );
						b2Vec2 dp = b2Add( dc, b2Sub( rB, rA ) );
						mp->separation = mp->baseSeparation + b2Dot( dp, normal );
						mp->persisted = true;
					}

					taskContext->recycledContactCount += 1;

					// Contact is recycled. This also skips updating other aspects of the contact
					// such as material parameters.
					continue;
				}
			}

			// Caching for contact recycling.
			contactSim->cachedRotationA = transformA.q;
			contactSim->cachedRotationB = transformB.q;
			contactSim->cachedRelativePose = b2InvMulWorldTransforms( transformA, transformB );
			contactSim->simFlags |= b2_simRelativeTransformValid;

			b2Vec2 centerOffsetA = b2RotateVector( transformA.q, bodySimA->localCenter );
			b2Vec2 centerOffsetB = b2RotateVector( transformB.q, bodySimB->localCenter );

			// This updates solid contacts
			bool touching =
				b2UpdateContact( world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB );

			// State changes that affect island connectivity. Also affects contact events.
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

			for ( int i = 0; i < contactSim->manifold.pointCount; ++i )
			{
				b2ManifoldPoint* mp = contactSim->manifold.points + i;
				mp->baseSeparation = mp->separation;
			}

			// To make this work, the time of impact code needs to adjust the target
			// distance based on the number of TOI events for a body.
			// if (touching && bodySimB->isFast)
			//{
			//	b2Manifold* manifold = &contactSim->manifold;
			//	int pointCount = manifold->pointCount;
			//	for (int i = 0; i < pointCount; ++i)
			//	{
			//		// trick the solver into pushing the fast shapes apart
			//		manifold->points[i].separation -= 0.25f * B2_SPECULATIVE_DISTANCE;
			//	}
			//}
		}
	}

	b2TracyCZoneEnd( collide_task );
}

static void b2AddNonTouchingContact( b2World* world, b2Contact* contact, b2ContactSim* contactSim )
{
	B2_ASSERT( contact->setIndex == b2_awakeSet );
	b2SolverSet* set = b2Array_Get( world->solverSets, b2_awakeSet );
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = set->contactSims.count;

	b2ContactSim* newContactSim = b2Array_Emplace( set->contactSims );
	memcpy( newContactSim, contactSim, sizeof( b2ContactSim ) );
}

static void b2RemoveNonTouchingContact( b2World* world, int setIndex, int localIndex )
{
	b2SolverSet* set = b2Array_Get( world->solverSets, setIndex );
	int movedIndex = b2Array_RemoveSwap( set->contactSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		b2ContactSim* movedContactSim = set->contactSims.data + localIndex;
		b2Contact* movedContact = b2Array_Get( world->contacts, movedContactSim->contactId );
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

	b2TracyCZoneNC( collide, "Narrow Phase", b2_colorDodgerBlue, true );

	// gather contacts into a single array for easier parallel-for
	int contactCount = 0;
	b2GraphColor* graphColors = world->constraintGraph.colors;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
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

	b2ContactSim** contactSims = b2StackAlloc( &world->stack, contactCount * sizeof( b2ContactSim* ), "contacts" );

	int contactIndex = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
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

	context->contactSims = contactSims;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b2GetIdCapacity( &world->contactIdPool );
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2SetBitCountAndClear( &world->taskContexts.data[i].contactStateBitSet, contactIdCapacity );
		world->taskContexts.data[i].recycledContactCount = 0;
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int minRange = 64;
	b2ParallelFor( world, &b2CollideTask, contactCount, minRange, context );

	b2StackFree( &world->stack, contactSims );
	context->contactSims = NULL;
	contactSims = NULL;

	// Serially update contact state
	// todo_erin bring this zone together with island merge
	b2TracyCZoneNC( contact_state, "Contact State", b2_colorLightSlateGray, true );

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world->taskContexts.data[0].contactStateBitSet;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world->taskContexts.data[i].contactStateBitSet );
	}

	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );

	int endEventArrayIndex = world->endEventArrayIndex;

	const b2Shape* shapes = world->shapes.data;
	uint16_t worldId = world->worldId;

	// Process contact state changes. Iterate over set bits
	for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
	{
		uint64_t bits = bitSet->bits[k];
		while ( bits != 0 )
		{
			uint32_t ctz = b2CTZ64( bits );
			int contactId = (int)( 64 * k + ctz );

			b2Contact* contact = b2Array_Get( world->contacts, contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );

			int colorIndex = contact->colorIndex;
			int localIndex = contact->localIndex;

			b2ContactSim* contactSim = NULL;
			if ( colorIndex != B2_NULL_INDEX )
			{
				// contact lives in constraint graph
				B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
				b2GraphColor* color = graphColors + colorIndex;
				contactSim = b2Array_Get( color->contactSims, localIndex );
			}
			else
			{
				contactSim = b2Array_Get( awakeSet->contactSims, localIndex );
			}

			const b2Shape* shapeA = shapes + contact->shapeIdA;
			const b2Shape* shapeB = shapes + contact->shapeIdB;
			b2ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
			b2ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };
			b2ContactId contactFullId = {
				.index1 = contactId + 1,
				.world0 = worldId,
				.padding = 0,
				.generation = contact->generation,
			};
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

				if ( flags & b2_contactEnableContactEvents )
				{
					b2ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b2Array_Push( world->contactBeginEvents, event );
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
				contactSim = b2Array_Get( awakeSet->contactSims, localIndex );

				contactSim->simFlags &= ~b2_simStartedTouching;

				// Add first for memcpy
				b2AddContactToGraph( world, contactSim, contact );

				// This destroys the contact sim
				b2RemoveNonTouchingContact( world, b2_awakeSet, localIndex );

				contactSim = NULL;
			}
			else if ( simFlags & b2_simStoppedTouching )
			{
				contactSim->simFlags &= ~b2_simStoppedTouching;
				contact->flags &= ~b2_contactTouchingFlag;

				if ( contact->flags & b2_contactEnableContactEvents )
				{
					b2ContactEndTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b2Array_Push( world->contactEndEvents[endEventArrayIndex], event );
				}

				B2_ASSERT( contactSim->manifold.pointCount == 0 );

				b2UnlinkContact( world, contact );
				int bodyIdA = contact->edges[0].bodyId;
				int bodyIdB = contact->edges[1].bodyId;

				// Add first for memcpy
				b2AddNonTouchingContact( world, contact, contactSim );
				b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
				contact = NULL;
				contactSim = NULL;
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
	B2_ASSERT( b2IsValidFloat( timeStep ) );
	B2_ASSERT( 0 < subStepCount );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		b2TracyCFrame;
		return;
	}

	// Record step inputs before simulation runs
	B2_REC( world, Step, worldId, timeStep, subStepCount );

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	b2Array_Clear( world->bodyMoveEvents );
	b2Array_Clear( world->sensorBeginEvents );
	b2Array_Clear( world->contactBeginEvents );
	b2Array_Clear( world->contactHitEvents );
	b2Array_Clear( world->jointEvents );

	world->profile = (b2Profile){ 0 };

	b2TracyCZoneNC( world_step, "Step", b2_colorBox2DGreen, true );

	world->locked = true;
	world->activeTaskCount = 0;
	world->taskCount = 0;

	if ( world->scheduler != NULL )
	{
		b2ResetScheduler( world->scheduler );
	}

	uint64_t stepTicks = b2GetTicks();

	{
		b2Capacity* c = &world->maxCapacity;
		c->staticShapeCount = b2MaxInt( c->staticShapeCount, world->broadPhase.trees[b2_staticBody].proxyCount );
		c->dynamicShapeCount = b2MaxInt( c->dynamicShapeCount, world->broadPhase.trees[b2_dynamicBody].proxyCount );

		int staticBodyCount = world->solverSets.data[b2_staticSet].bodySims.count;
		c->staticBodyCount = b2MaxInt( c->staticBodyCount, staticBodyCount );

		// this includes kinematic bodies
		int totalBodyCount = b2GetIdCount( &world->bodyIdPool );
		c->dynamicBodyCount = b2MaxInt( c->dynamicBodyCount, totalBodyCount - staticBodyCount );

		int totalContactCount = b2GetIdCount( &world->contactIdPool );
		c->contactCount = b2MaxInt( c->contactCount, totalContactCount );
	}

	// Update collision pairs and create contacts
	{
		uint64_t pairTicks = b2GetTicks();
		b2UpdateBroadPhasePairs( world );
		world->profile.pairs = b2GetMilliseconds( pairTicks );
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
	world->inv_dt = context.inv_dt;

	// Hertz values get reduced for large time steps
	float contactHertz = b2MinFloat( world->contactHertz, 0.125f * context.inv_h );
	context.contactSoftness = b2MakeSoft( contactHertz, world->contactDampingRatio, context.h );
	context.staticSoftness = b2MakeSoft( 2.0f * contactHertz, world->contactDampingRatio, context.h );

	context.restitutionThreshold = world->restitutionThreshold;
	context.maxLinearVelocity = world->maxLinearSpeed;
	context.enableWarmStarting = world->enableWarmStarting;

	// Narrow phase : update contacts
	{
		uint64_t collideTicks = b2GetTicks();
		b2Collide( &context );
		world->profile.collide = b2GetMilliseconds( collideTicks );
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if ( timeStep > 0.0f )
	{
		uint64_t solveTicks = b2GetTicks();
		b2Solve( world, &context );
		world->profile.solve = b2GetMilliseconds( solveTicks );
	}

	// Finish the tree task in case b2Solve didn't finish it
	if ( world->userTreeTask )
	{
		world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
		world->userTreeTask = NULL;
		world->activeTaskCount -= 1;
	}

	// Update sensors
	{
		uint64_t sensorTicks = b2GetTicks();
		b2OverlapSensors( world );
		world->profile.sensors = b2GetMilliseconds( sensorTicks );
	}

	world->profile.step = b2GetMilliseconds( stepTicks );

	B2_ASSERT( b2GetStackAllocation( &world->stack ) == 0 );

	// Ensure stack is large enough
	b2GrowStack( &world->stack );

	// Make sure all tasks that were started were also finished
	B2_ASSERT( world->activeTaskCount == 0 );

	b2TracyCZoneEnd( world_step );

	// Swap end event array buffers
	world->endEventArrayIndex = 1 - world->endEventArrayIndex;
	b2Array_Clear( world->sensorEndEvents[world->endEventArrayIndex] );
	b2Array_Clear( world->contactEndEvents[world->endEventArrayIndex] );

	if ( world->recording != NULL )
	{
		// Write the per-step StateHash while the world is still locked. Queries early return while
		// locked, so this keeps the shared recording buffer single-writer without a lock. StateHash
		// proves the simulation reproduced exactly on replay.
		uint64_t hash = b2HashWorldState( world );
		b2RecArgs_StateHash stateHash = { worldId, hash };
		b2RecWrite_StateHash( world->recording, &stateHash );

		// Grow the recorded bounds so a replay can frame the whole motion, not just frame 0
		b2AABB bounds;
		if ( b2ComputeWorldBounds( world, &bounds ) )
		{
			b2RecAccumulateBounds( world->recording, bounds );
		}
	}

	world->locked = false;

	b2TracyCFrame;
}

static void b2DrawShape( b2DebugDraw* draw, b2Shape* shape, b2WorldTransform transform, b2HexColor color, bool drawChainNormals )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			b2Capsule* capsule = &shape->capsule;
			b2Pos p1 = b2TransformWorldPoint( transform, capsule->center1 );
			b2Pos p2 = b2TransformWorldPoint( transform, capsule->center2 );
			draw->DrawSolidCapsuleFcn( p1, p2, capsule->radius, color, draw->context );
		}
		break;

		case b2_circleShape:
		{
			b2Circle* circle = &shape->circle;
			draw->DrawSolidCircleFcn( transform, circle->center, circle->radius, color, draw->context );
		}
		break;

		case b2_polygonShape:
		{
			b2Polygon* poly = &shape->polygon;
			draw->DrawSolidPolygonFcn( transform, poly->vertices, poly->count, poly->radius, color, draw->context );
		}
		break;

		case b2_segmentShape:
		{
			b2Segment* segment = &shape->segment;
			b2Pos p1 = b2TransformWorldPoint( transform, segment->point1 );
			b2Pos p2 = b2TransformWorldPoint( transform, segment->point2 );
			draw->DrawLineFcn( p1, p2, color, draw->context );
		}
		break;

		case b2_chainSegmentShape:
		{
			b2Segment* segment = &shape->chainSegment.segment;
			b2Pos p1 = b2TransformWorldPoint( transform, segment->point1 );
			b2Pos p2 = b2TransformWorldPoint( transform, segment->point2 );
			draw->DrawLineFcn( p1, p2, color, draw->context );
			draw->DrawPointFcn( p2, 4.0f, color, draw->context );

			if ( drawChainNormals )
			{
				b2Pos c = b2LerpPosition( p1, p2, 0.5f );
				b2Vec2 e = b2Normalize( b2SubPos( p2, p1 ) );
				b2Vec2 n = b2RightPerp( e );
				float L = 0.2f * b2GetLengthUnitsPerMeter();
				draw->DrawLineFcn( c, b2OffsetPos( c, b2MulSV( L, n ) ), b2_colorPaleGreen, draw->context );
			}
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

static bool DrawQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;

	struct DrawContext* drawContext = context;
	b2World* world = drawContext->world;
	b2DebugDraw* draw = drawContext->draw;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );
	B2_ASSERT( shape->id == shapeId );
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );

	b2SetBit( &world->debugBodySet, shape->bodyId );

	if ( draw->drawShapes )
	{
		b2HexColor color;

		if ( shape->material.customColor != 0 )
		{
			color = shape->material.customColor;
		}
		else if ( body->type == b2_dynamicBody && body->mass == 0.0f )
		{
			// Bad body
			color = b2_colorRed;
		}
		else if ( body->setIndex == b2_disabledSet )
		{
			color = b2_colorSlateGray;
		}
		else if ( shape->sensorIndex != B2_NULL_INDEX )
		{
			color = b2_colorWheat;
		}
		else if ( body->flags & b2_hadTimeOfImpact )
		{
			color = b2_colorLime;
		}
		else if ( ( bodySim->flags & b2_isBullet ) && body->setIndex == b2_awakeSet )
		{
			color = b2_colorTurquoise;
		}
		else if ( body->flags & b2_isSpeedCapped )
		{
			color = b2_colorYellow;
		}
		else if ( bodySim->flags & b2_isFast )
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

		b2DrawShape( draw, shape, bodySim->transform, color, draw->drawChainNormals );
	}

	if ( draw->drawBounds )
	{
		draw->DrawBoundsFcn( shape->fatAABB, b2_colorGold, draw->context );
	}

	return true;
}

// todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
// solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
void b2World_Draw( b2WorldId worldId, b2DebugDraw* draw )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_ASSERT( b2IsValidAABB( draw->drawingBounds ) );

	const float k_axisScale = 0.3f;
	b2HexColor speculativeColor = b2_colorGainsboro;
	b2HexColor addColor = b2_colorGreen;
	b2HexColor persistColor = b2_colorBlue;
	b2HexColor normalColor = b2_colorDimGray;
	b2HexColor impulseColor = b2_colorMagenta;
	b2HexColor frictionColor = b2_colorYellow;

	int bodyCapacity = b2GetIdCapacity( &world->bodyIdPool );
	b2SetBitCountAndClear( &world->debugBodySet, bodyCapacity );

	int jointCapacity = b2GetIdCapacity( &world->jointIdPool );
	b2SetBitCountAndClear( &world->debugJointSet, jointCapacity );

	int contactCapacity = b2GetIdCapacity( &world->contactIdPool );
	b2SetBitCountAndClear( &world->debugContactSet, contactCapacity );

	int islandCapacity = b2GetIdCapacity( &world->islandIdPool );
	b2SetBitCountAndClear( &world->debugIslandSet, islandCapacity );

	struct DrawContext drawContext = { world, draw };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_QueryAll( world->broadPhase.trees + i, draw->drawingBounds, DrawQueryCallback, &drawContext );
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

			b2Body* body = b2Array_Get( world->bodies, (int)bodyId );

			if ( draw->drawBodyNames && body->name[0] != 0 )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2WorldTransform transform = { bodySim->center, bodySim->transform.q };
				b2Pos p = b2TransformWorldPoint( transform, offset );
				draw->DrawStringFcn( p, body->name, b2_colorBlueViolet, draw->context );
			}

			if ( draw->drawMass && body->type == b2_dynamicBody )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2WorldTransform transform = { bodySim->center, bodySim->transform.q };
				draw->DrawLineFcn( bodySim->center0, bodySim->center, b2_colorWhiteSmoke, draw->context );
				draw->DrawTransformFcn( transform, draw->context );

				b2Pos p = b2TransformWorldPoint( transform, offset );
				char buffer[32];
				snprintf( buffer, 32, "  %.2f", body->mass );
				draw->DrawStringFcn( p, buffer, b2_colorWhite, draw->context );
			}

			if ( draw->drawJoints )
			{
				int jointKey = body->headJointKey;
				while ( jointKey != B2_NULL_INDEX )
				{
					int jointId = jointKey >> 1;
					int edgeIndex = jointKey & 1;
					b2Joint* joint = b2Array_Get( world->joints, jointId );

					// avoid double draw
					if ( b2GetBit( &world->debugJointSet, jointId ) == false )
					{
						b2DrawJoint( draw, world, joint );
						b2SetBit( &world->debugJointSet, jointId );
					}

					jointKey = joint->edges[edgeIndex].nextKey;
				}
			}

			const float linearSlop = B2_LINEAR_SLOP;
			if ( draw->drawContacts && body->type == b2_dynamicBody )
			{
				int contactKey = body->headContactKey;
				while ( contactKey != B2_NULL_INDEX )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;
					b2Contact* contact = b2Array_Get( world->contacts, contactId );
					contactKey = contact->edges[edgeIndex].nextKey;

					// avoid double draw
					if ( b2GetBit( &world->debugContactSet, contactId ) == false )
					{
						b2ContactSim* contactSim = b2GetContactSim( world, contact );
						b2Body* bodyA = b2Array_Get( world->bodies, contact->edges[0].bodyId );
						b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
						b2Body* bodyB = b2Array_Get( world->bodies, contact->edges[1].bodyId );
						b2BodySim* bodySimB = b2GetBodySim( world, bodyB );
						int pointCount = contactSim->manifold.pointCount;
						b2Vec2 normal = contactSim->manifold.normal;
						char buffer[32];

						for ( int j = 0; j < pointCount; ++j )
						{
							b2ManifoldPoint* mp = contactSim->manifold.points + j;

							b2Pos p;
							if ( draw->drawAnchorA )
							{
								p = b2OffsetPos( bodySimA->center, mp->anchorA );
							}
							else
							{
								p = b2OffsetPos( bodySimB->center, mp->anchorB );
							}

							if ( draw->drawGraphColors && contact->colorIndex != B2_NULL_INDEX )
							{
								// graph color
								float pointSize = contact->colorIndex == B2_OVERFLOW_INDEX ? 7.5f : 5.0f;
								draw->DrawPointFcn( p, pointSize, b2GetGraphColor( contact->colorIndex ), draw->context );
								// m_context->draw.DrawString(point->position, "%d", point->color);
							}
							else if ( mp->separation > linearSlop )
							{
								// Speculative
								draw->DrawPointFcn( p, 5.0f, speculativeColor, draw->context );
							}
							else if ( mp->persisted == false )
							{
								// Add
								draw->DrawPointFcn( p, 10.0f, addColor, draw->context );
							}
							else if ( mp->persisted == true )
							{
								// Persist
								draw->DrawPointFcn( p, 5.0f, persistColor, draw->context );
							}

							if ( draw->drawContactNormals )
							{
								b2Pos p1 = p;
								b2Pos p2 = b2OffsetPos( p1, b2MulSV( k_axisScale, normal ) );
								draw->DrawLineFcn( p1, p2, normalColor, draw->context );

								snprintf( buffer, B2_ARRAY_COUNT( buffer ), " %.2f", mp->separation );
								draw->DrawStringFcn( p1, buffer, b2_colorWhite, draw->context );
							}
							else if ( draw->drawContactForces )
							{
								// todo validate
								// multiply by one-half due to relax iteration
								float force = 0.5f * mp->totalNormalImpulse * world->inv_dt;
								b2Pos p1 = p;
								b2Pos p2 = b2OffsetPos( p1, b2MulSV( draw->forceScale * force, normal ) );
								draw->DrawLineFcn( p1, p2, impulseColor, draw->context );
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.1f", force );
								draw->DrawStringFcn( p1, buffer, b2_colorWhite, draw->context );
							}

							if ( draw->drawContactFeatures )
							{
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%d", mp->id );
								draw->DrawStringFcn( p, buffer, b2_colorOrange, draw->context );
							}

							if ( draw->drawFrictionForces )
							{
								float force = 0.5f * mp->tangentImpulse * world->inv_h;
								b2Vec2 tangent = b2RightPerp( normal );
								b2Pos p1 = p;
								b2Pos p2 = b2OffsetPos( p1, b2MulSV( draw->forceScale * force, tangent ) );
								draw->DrawLineFcn( p1, p2, frictionColor, draw->context );
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.1f", force );
								draw->DrawStringFcn( p1, buffer, b2_colorWhite, draw->context );
							}
						}

						b2SetBit( &world->debugContactSet, contactId );
					}

					contactKey = contact->edges[edgeIndex].nextKey;
				}
			}

			if ( draw->drawIslands )
			{
				int islandId = body->islandId;
				if ( islandId != B2_NULL_INDEX && b2GetBit( &world->debugIslandSet, islandId ) == false )
				{
					b2Island* island = world->islands.data + islandId;
					if ( island->setIndex == B2_NULL_INDEX )
					{
						continue;
					}

					int shapeCount = 0;
					b2AABB aabb = {
						.lowerBound = { FLT_MAX, FLT_MAX },
						.upperBound = { -FLT_MAX, -FLT_MAX },
					};

					for ( int bodyIndex = 0; bodyIndex < island->bodies.count; ++bodyIndex )
					{
						int islandBodyId = island->bodies.data[bodyIndex];
						b2Body* islandBody = b2Array_Get( world->bodies, islandBodyId );
						int shapeId = islandBody->headShapeId;
						while ( shapeId != B2_NULL_INDEX )
						{
							b2Shape* shape = b2Array_Get( world->shapes, shapeId );
							aabb = b2AABB_Union( aabb, shape->fatAABB );
							shapeCount += 1;
							shapeId = shape->nextShapeId;
						}
					}

					if ( shapeCount > 0 )
					{
						draw->DrawBoundsFcn( aabb, b2_colorOrangeRed, draw->context );
					}

					b2SetBit( &world->debugIslandSet, islandId );
				}
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}
}

bool b2ComputeWorldBounds( b2World* world, b2AABB* bounds )
{
	b2AABB worldBounds = { 0 };
	bool haveBounds = false;

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree* tree = world->broadPhase.trees + i;
		if ( b2DynamicTree_GetProxyCount( tree ) == 0 )
		{
			continue;
		}

		b2AABB treeBounds = b2DynamicTree_GetRootBounds( tree );
		worldBounds = haveBounds ? b2AABB_Union( worldBounds, treeBounds ) : treeBounds;
		haveBounds = true;
	}

	*bounds = worldBounds;
	return haveBounds;
}

b2AABB b2World_GetBounds( b2WorldId worldId )
{
	b2World* world = b2GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return (b2AABB){ 0 };
	}

	b2AABB bounds;
	b2ComputeWorldBounds( world, &bounds );
	return bounds;
}

b2BodyEvents b2World_GetBodyEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return (b2BodyEvents){ 0 };
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
		return (b2SensorEvents){ 0 };
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
		return (b2ContactEvents){ 0 };
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

b2JointEvents b2World_GetJointEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return (b2JointEvents){ 0 };
	}

	int count = world->jointEvents.count;
	b2JointEvents events = { world->jointEvents.data, count };
	return events;
}

bool b2World_IsValid( b2WorldId id )
{
	if ( id.index1 < 1 || B2_MAX_WORLDS < id.index1 )
	{
		return false;
	}

	b2World* world = b2_worlds + ( id.index1 - 1 );

	if ( world->worldId != id.index1 - 1 )
	{
		// world is not allocated
		return false;
	}

	return id.generation == world->generation;
}

bool b2Body_IsValid( b2BodyId id )
{
	if ( B2_MAX_WORLDS <= id.world0 )
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

	if ( body->generation != id.generation )
	{
		// this id is orphaned
		return false;
	}

	return true;
}

bool b2Shape_IsValid( b2ShapeId id )
{
	if ( B2_MAX_WORLDS <= id.world0 )
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

	return id.generation == shape->generation;
}

bool b2Chain_IsValid( b2ChainId id )
{
	if ( B2_MAX_WORLDS <= id.world0 )
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

	return id.generation == chain->generation;
}

bool b2Joint_IsValid( b2JointId id )
{
	if ( B2_MAX_WORLDS <= id.world0 )
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

	return id.generation == joint->generation;
}

bool b2Contact_IsValid( b2ContactId id )
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world->worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int contactId = id.index1 - 1;
	if ( contactId < 0 || world->contacts.count <= contactId )
	{
		return false;
	}

	b2Contact* contact = world->contacts.data + contactId;
	if ( contact->contactId == B2_NULL_INDEX )
	{
		// contact is free
		return false;
	}

	B2_ASSERT( contact->contactId == contactId );

	return id.generation == contact->generation;
}

void b2World_EnableSleeping( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_REC( world, WorldEnableSleeping, worldId, flag );

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
			b2SolverSet* set = b2Array_Get( world->solverSets, i );
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

	B2_REC( world, WorldEnableWarmStarting, worldId, flag );

	world->enableWarmStarting = flag;
}

bool b2World_IsWarmStartingEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->enableWarmStarting;
}

int b2World_GetAwakeBodyCount( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
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

	B2_REC( world, WorldEnableContinuous, worldId, flag );

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

	B2_REC( world, WorldSetRestitutionThreshold, worldId, value );

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

	B2_REC( world, WorldSetHitEventThreshold, worldId, value );

	world->hitEventThreshold = b2ClampFloat( value, 0.0f, FLT_MAX );
}

float b2World_GetHitEventThreshold( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->hitEventThreshold;
}

void b2World_SetContactTuning( b2WorldId worldId, float hertz, float dampingRatio, float pushSpeed )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_REC( world, WorldSetContactTuning, worldId, hertz, dampingRatio, pushSpeed );

	world->contactHertz = b2ClampFloat( hertz, 0.0f, FLT_MAX );
	world->contactDampingRatio = b2ClampFloat( dampingRatio, 0.0f, FLT_MAX );
	world->contactSpeed = b2ClampFloat( pushSpeed, 0.0f, FLT_MAX );
}

void b2World_SetContactRecycleDistance( b2WorldId worldId, float recycleDistance )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_REC( world, WorldSetContactRecycleDistance, worldId, recycleDistance );

	world->contactRecycleDistance = b2ClampFloat( recycleDistance, 0.0f, FLT_MAX );
}

float b2World_GetContactRecycleDistance( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->contactRecycleDistance;
}

void b2World_SetMaximumLinearSpeed( b2WorldId worldId, float maximumLinearSpeed )
{
	B2_ASSERT( b2IsValidFloat( maximumLinearSpeed ) && maximumLinearSpeed > 0.0f );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_REC( world, WorldSetMaximumLinearSpeed, worldId, maximumLinearSpeed );

	world->maxLinearSpeed = maximumLinearSpeed;
}

float b2World_GetMaximumLinearSpeed( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->maxLinearSpeed;
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

	s.stackUsed = b2GetMaxStackAllocation( &world->stack );
	s.byteCount = b2GetByteCount();
	s.taskCount = world->taskCount;

	s.recycledContactCount = 0;
	for ( int i = 0; i < world->workerCount; ++i )
	{
		s.recycledContactCount += world->taskContexts.data[i].recycledContactCount;
	}

	s.awakeContactCount = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* color = world->constraintGraph.colors + i;
		s.colorCounts[i] = color->contactSims.count + color->jointSims.count;
		s.awakeContactCount += color->contactSims.count;
	}
	s.awakeContactCount += world->solverSets.data[b2_awakeSet].contactSims.count;

	return s;
}

b2Capacity b2World_GetMaxCapacity( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world->maxCapacity;
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

void b2World_SetFrictionCallback( b2WorldId worldId, b2FrictionCallback* callback )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world->locked )
	{
		return;
	}

	if ( callback != NULL )
	{
		world->frictionCallback = callback;
	}
	else
	{
		world->frictionCallback = b2DefaultFrictionCallback;
	}
}

void b2World_SetRestitutionCallback( b2WorldId worldId, b2RestitutionCallback* callback )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world->locked )
	{
		return;
	}

	if ( callback != NULL )
	{
		world->restitutionCallback = callback;
	}
	else
	{
		world->restitutionCallback = b2DefaultRestitutionCallback;
	}
}

void b2World_SetWorkerCount( b2WorldId worldId, int count )
{
	b2World* world = b2GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	if ( count == world->workerCount )
	{
		return;
	}

	b2DestroyWorkerContexts( world );
	world->workerCount = b2ClampInt( count, 1, B2_MAX_WORKERS );
	b2CreateWorkerContexts( world );
}

int b2World_GetWorkerCount( b2WorldId worldId )
{
	b2World* world = b2GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return 0;
	}

	return world->workerCount;
}

void b2World_StartRecording( b2WorldId worldId, b2Recording* recording )
{
	// Must be a step boundary, so refuse a locked world
	b2World* world = b2GetUnlockedWorldFromId( worldId );

	if ( world == NULL || recording == NULL || world->recording != NULL )
	{
		return;
	}

	b2StartRecordingIntoBuffer( world, recording );
}

void b2World_StopRecording( b2WorldId worldId )
{
	b2World* world = b2GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	b2StopRecordingInternal( world );
}

void b2World_DumpMemoryStats( b2WorldId worldId )
{
	FILE* file = fopen( "box2d_memory.txt", "w" );
	if ( file == NULL )
	{
		return;
	}

	b2World* world = b2GetWorldFromId( worldId );

	int total = 0;

	// id pools
	int bodyIdBytes = b2GetIdBytes( &world->bodyIdPool );
	int solverSetIdBytes = b2GetIdBytes( &world->solverSetIdPool );
	int jointIdBytes = b2GetIdBytes( &world->jointIdPool );
	int contactIdBytes = b2GetIdBytes( &world->contactIdPool );
	int islandIdBytes = b2GetIdBytes( &world->islandIdPool );
	int shapeIdBytes = b2GetIdBytes( &world->shapeIdPool );
	int chainIdBytes = b2GetIdBytes( &world->chainIdPool );
	total += bodyIdBytes + solverSetIdBytes + jointIdBytes + contactIdBytes + islandIdBytes + shapeIdBytes + chainIdBytes;

	fprintf( file, "id pools\n" );
	fprintf( file, "body ids: %d\n", bodyIdBytes );
	fprintf( file, "solver set ids: %d\n", solverSetIdBytes );
	fprintf( file, "joint ids: %d\n", jointIdBytes );
	fprintf( file, "contact ids: %d\n", contactIdBytes );
	fprintf( file, "island ids: %d\n", islandIdBytes );
	fprintf( file, "shape ids: %d\n", shapeIdBytes );
	fprintf( file, "chain ids: %d\n", chainIdBytes );
	fprintf( file, "\n" );

	// Islands own per-island body/contact/joint link arrays
	int islandLinkBytes = 0;
	for ( int i = 0; i < world->islands.count; ++i )
	{
		b2Island* island = world->islands.data + i;
		islandLinkBytes += b2Array_ByteCount( island->bodies );
		islandLinkBytes += b2Array_ByteCount( island->contacts );
		islandLinkBytes += b2Array_ByteCount( island->joints );
	}

	// world arrays
	int bodyArrayBytes = b2Array_ByteCount( world->bodies );
	int solverSetArrayBytes = b2Array_ByteCount( world->solverSets );
	int jointArrayBytes = b2Array_ByteCount( world->joints );
	int contactArrayBytes = b2Array_ByteCount( world->contacts );
	int islandArrayBytes = b2Array_ByteCount( world->islands );
	int shapeArrayBytes = b2Array_ByteCount( world->shapes );
	int chainArrayBytes = b2Array_ByteCount( world->chainShapes );
	int sensorArrayBytes = b2Array_ByteCount( world->sensors );
	total += bodyArrayBytes + solverSetArrayBytes + jointArrayBytes + contactArrayBytes + islandArrayBytes + islandLinkBytes +
			 shapeArrayBytes + chainArrayBytes + sensorArrayBytes;

	fprintf( file, "world arrays\n" );
	fprintf( file, "bodies: %d\n", bodyArrayBytes );
	fprintf( file, "solver sets: %d\n", solverSetArrayBytes );
	fprintf( file, "joints: %d\n", jointArrayBytes );
	fprintf( file, "contacts: %d\n", contactArrayBytes );
	fprintf( file, "islands: %d\n", islandArrayBytes );
	fprintf( file, "island links: %d\n", islandLinkBytes );
	fprintf( file, "shapes: %d\n", shapeArrayBytes );
	fprintf( file, "chains: %d\n", chainArrayBytes );
	fprintf( file, "sensors: %d\n", sensorArrayBytes );
	fprintf( file, "\n" );

	// Chain shapes own index and surface material arrays
	int chainDataBytes = 0;
	for ( int i = 0; i < world->chainShapes.count; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		if ( chain->id == B2_NULL_INDEX )
		{
			continue;
		}

		chainDataBytes += chain->count * (int)sizeof( int );
		chainDataBytes += chain->materialCount * (int)sizeof( b2SurfaceMaterial );
	}

	// Sensors own overlap tracking arrays. The sensor array is dense.
	int sensorOverlapBytes = 0;
	for ( int i = 0; i < world->sensors.count; ++i )
	{
		b2Sensor* sensor = world->sensors.data + i;
		sensorOverlapBytes += b2Array_ByteCount( sensor->hits );
		sensorOverlapBytes += b2Array_ByteCount( sensor->overlaps1 );
		sensorOverlapBytes += b2Array_ByteCount( sensor->overlaps2 );
	}
	total += chainDataBytes + sensorOverlapBytes;

	fprintf( file, "owned arrays\n" );
	fprintf( file, "chain data: %d\n", chainDataBytes );
	fprintf( file, "sensor overlaps: %d\n", sensorOverlapBytes );
	fprintf( file, "\n" );

	// broad-phase
	int staticTreeBytes = b2DynamicTree_GetByteCount( world->broadPhase.trees + b2_staticBody );
	int kinematicTreeBytes = b2DynamicTree_GetByteCount( world->broadPhase.trees + b2_kinematicBody );
	int dynamicTreeBytes = b2DynamicTree_GetByteCount( world->broadPhase.trees + b2_dynamicBody );
	int movedBytes = 0;
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		movedBytes += b2GetBitSetBytes( &world->broadPhase.movedProxies[i] );
	}
	int moveArrayBytes = b2Array_ByteCount( world->broadPhase.moveArray );
	b2HashSet* pairSet = &world->broadPhase.pairSet;
	int pairSetBytes = b2GetHashSetBytes( pairSet );
	total += staticTreeBytes + kinematicTreeBytes + dynamicTreeBytes + movedBytes + moveArrayBytes + pairSetBytes;

	fprintf( file, "broad-phase\n" );
	fprintf( file, "static tree: %d\n", staticTreeBytes );
	fprintf( file, "kinematic tree: %d\n", kinematicTreeBytes );
	fprintf( file, "dynamic tree: %d\n", dynamicTreeBytes );
	fprintf( file, "movedProxies: %d\n", movedBytes );
	fprintf( file, "moveArray: %d\n", moveArrayBytes );
	fprintf( file, "pairSet: %d (%u, %u)\n", pairSetBytes, pairSet->count, pairSet->capacity );
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

	int setBodySimBytes = bodySimCapacity * (int)sizeof( b2BodySim );
	int setBodyStateBytes = bodyStateCapacity * (int)sizeof( b2BodyState );
	int setJointSimBytes = jointSimCapacity * (int)sizeof( b2JointSim );
	int setContactSimBytes = contactSimCapacity * (int)sizeof( b2ContactSim );
	int setIslandSimBytes = islandSimCapacity * (int)sizeof( b2IslandSim );
	total += setBodySimBytes + setBodyStateBytes + setJointSimBytes + setContactSimBytes + setIslandSimBytes;

	fprintf( file, "solver sets\n" );
	fprintf( file, "body sim: %d\n", setBodySimBytes );
	fprintf( file, "body state: %d\n", setBodyStateBytes );
	fprintf( file, "joint sim: %d\n", setJointSimBytes );
	fprintf( file, "contact sim: %d\n", setContactSimBytes );
	fprintf( file, "island sim: %d\n", setIslandSimBytes );
	fprintf( file, "\n" );

	// constraint graph
	int bodyBitSetBytes = 0;
	contactSimCapacity = 0;
	jointSimCapacity = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* c = world->constraintGraph.colors + i;
		bodyBitSetBytes += b2GetBitSetBytes( &c->bodySet );
		contactSimCapacity += c->contactSims.capacity;
		jointSimCapacity += c->jointSims.capacity;
	}

	int graphJointSimBytes = jointSimCapacity * (int)sizeof( b2JointSim );
	int graphContactSimBytes = contactSimCapacity * (int)sizeof( b2ContactSim );
	total += bodyBitSetBytes + graphJointSimBytes + graphContactSimBytes;

	fprintf( file, "constraint graph\n" );
	fprintf( file, "body bit sets: %d\n", bodyBitSetBytes );
	fprintf( file, "joint sim: %d\n", graphJointSimBytes );
	fprintf( file, "contact sim: %d\n", graphContactSimBytes );
	fprintf( file, "\n" );

	// Per worker task storage and its bit sets
	int taskContextBytes = b2Array_ByteCount( world->taskContexts );
	for ( int i = 0; i < world->taskContexts.count; ++i )
	{
		b2TaskContext* taskContext = world->taskContexts.data + i;
		taskContextBytes += b2Array_ByteCount( taskContext->sensorHits );
		taskContextBytes += b2GetBitSetBytes( &taskContext->contactStateBitSet );
		taskContextBytes += b2GetBitSetBytes( &taskContext->hitEventBitSet );
		taskContextBytes += b2GetBitSetBytes( &taskContext->jointStateBitSet );
		taskContextBytes += b2GetBitSetBytes( &taskContext->enlargedSimBitSet );
		taskContextBytes += b2GetBitSetBytes( &taskContext->awakeIslandBitSet );
	}

	int sensorTaskContextBytes = b2Array_ByteCount( world->sensorTaskContexts );
	for ( int i = 0; i < world->sensorTaskContexts.count; ++i )
	{
		b2SensorTaskContext* taskContext = world->sensorTaskContexts.data + i;
		sensorTaskContextBytes += b2GetBitSetBytes( &taskContext->eventBits );
	}
	total += taskContextBytes + sensorTaskContextBytes;

	fprintf( file, "task contexts\n" );
	fprintf( file, "worker: %d\n", taskContextBytes );
	fprintf( file, "sensor: %d\n", sensorTaskContextBytes );
	fprintf( file, "\n" );

	// Double buffered event arrays
	int eventBytes = 0;
	eventBytes += b2Array_ByteCount( world->bodyMoveEvents );
	eventBytes += b2Array_ByteCount( world->sensorBeginEvents );
	eventBytes += b2Array_ByteCount( world->contactBeginEvents );
	eventBytes += b2Array_ByteCount( world->sensorEndEvents[0] );
	eventBytes += b2Array_ByteCount( world->sensorEndEvents[1] );
	eventBytes += b2Array_ByteCount( world->contactEndEvents[0] );
	eventBytes += b2Array_ByteCount( world->contactEndEvents[1] );
	eventBytes += b2Array_ByteCount( world->contactHitEvents );
	eventBytes += b2Array_ByteCount( world->jointEvents );
	total += eventBytes;

	fprintf( file, "events: %d\n\n", eventBytes );

	// Debug draw bit sets
	int debugBytes = 0;
	debugBytes += b2GetBitSetBytes( &world->debugBodySet );
	debugBytes += b2GetBitSetBytes( &world->debugJointSet );
	debugBytes += b2GetBitSetBytes( &world->debugContactSet );
	debugBytes += b2GetBitSetBytes( &world->debugIslandSet );
	total += debugBytes;

	fprintf( file, "debug draw: %d\n\n", debugBytes );

	// stack allocator
	total += world->stack.capacity;
	fprintf( file, "stack allocator: %d\n\n", world->stack.capacity );

	fprintf( file, "total: %d\n", total );

	fclose( file );
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;

	WorldQueryContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return true;
	}

	b2ShapeId id = { shapeId + 1, world->worldId, shape->generation };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b2TreeStats b2World_OverlapAABB( b2WorldId worldId, b2Pos origin, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn,
								 void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidPosition( origin ) );
	B2_ASSERT( b2IsValidAABB( aabb ) );

	b2RecQueryWriter recWriter = { 0 };
	if ( world->recording != NULL )
	{
		b2RecQueryBegin( &recWriter, context );
		recWriter.userFcn.overlapFcn = fcn;
		b2RecW_WORLDID( &recWriter.buf, worldId );
		b2RecW_POSITION( &recWriter.buf, origin );
		b2RecW_AABB( &recWriter.buf, aabb );
		b2RecW_QUERYFILTER( &recWriter.buf, filter );
		recWriter.countOffset = b2RecReserveU32( &recWriter.buf );
		fcn = b2RecOverlapTrampoline;
		context = &recWriter;
	}

	// Lift to a world float box with outward rounding so the conservative tree test never misses
	b2AABB worldBox = b2OffsetAABB( aabb, origin );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world->broadPhase.trees + i, worldBox, filter.maskBits, TreeQueryCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	if ( world->recording != NULL )
	{
		b2RecPatchU32( &recWriter.buf, recWriter.countOffset, recWriter.hitCount );
		b2RecW_TREESTATS( &recWriter.buf, treeStats );
		b2RecQueryCommit( world->recording, 0xE0, &recWriter );
	}

	return treeStats;
}

typedef struct WorldOverlapContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	const b2ShapeProxy* proxy;
	b2Pos origin;
	void* userContext;
} WorldOverlapContext;

static bool TreeOverlapCallback( int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;

	WorldOverlapContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return true;
	}

	// Re-center on the query origin so the distance test stays in float precision far from the world origin
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2Transform transform = b2ToRelativeTransform( b2GetBodyTransformQuick( world, body ), worldContext->origin );

	b2DistanceInput input;
	input.proxyA = *worldContext->proxy;
	input.proxyB = b2MakeShapeDistanceProxy( shape );
	input.transform = transform;
	input.useRadii = true;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, NULL, 0 );

	float tolerance = 0.1f * B2_LINEAR_SLOP;
	if ( output.distance > tolerance )
	{
		return true;
	}

	b2ShapeId id = { shape->id + 1, world->worldId, shape->generation };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b2TreeStats b2World_OverlapShape( b2WorldId worldId, b2Pos origin, const b2ShapeProxy* proxy, b2QueryFilter filter,
								  b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidPosition( origin ) );

	b2RecQueryWriter recWriter = { 0 };
	if ( world->recording != NULL )
	{
		b2RecQueryBegin( &recWriter, context );
		recWriter.userFcn.overlapFcn = fcn;
		b2RecW_WORLDID( &recWriter.buf, worldId );
		b2RecW_POSITION( &recWriter.buf, origin );
		b2RecW_SHAPEPROXY( &recWriter.buf, *proxy );
		b2RecW_QUERYFILTER( &recWriter.buf, filter );
		recWriter.countOffset = b2RecReserveU32( &recWriter.buf );
		fcn = b2RecOverlapTrampoline;
		context = &recWriter;
	}

	// Relative box lifted to world float with outward rounding, conservative for the tree
	b2AABB aabb = b2OffsetAABB( b2MakeAABB( proxy->points, proxy->count, proxy->radius ), origin );
	WorldOverlapContext worldContext = {
		world, fcn, filter, proxy, origin, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	if ( world->recording != NULL )
	{
		b2RecPatchU32( &recWriter.buf, recWriter.countOffset, recWriter.hitCount );
		b2RecW_TREESTATS( &recWriter.buf, treeStats );
		b2RecQueryCommit( world->recording, 0xE1, &recWriter );
	}

	return treeStats;
}

typedef struct WorldRayCastContext
{
	b2World* world;
	b2CastResultFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	b2Pos origin;
	void* userContext;
} WorldRayCastContext;

static float RayCastCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return input->maxFraction;
	}

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2WorldTransform xf = b2GetBodyTransformQuick( world, body );

	// Re-center on the body so the per-shape cast stays in float precision far from the origin.
	// The tree traversal already used the truncated origin in input. Here we re-difference in full
	// precision against the body position.
	b2Pos base = xf.p;
	b2Transform transform = b2ToRelativeTransform( xf, base );
	b2RayCastInput localInput = *input;
	localInput.origin = b2SubPos( worldContext->origin, base );
	b2CastOutput output = b2RayCastShape( &localInput, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world->worldId, shape->generation };
		b2Pos point = b2OffsetPos( base, output.point );
		float fraction = worldContext->fcn( id, point, output.normal, output.fraction, worldContext->userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext->fraction = fraction;
		}

		return fraction;
	}

	return input->maxFraction;
}

b2TreeStats b2World_CastRay( b2WorldId worldId, b2Pos origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn,
							 void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidPosition( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2RecQueryWriter recWriter = { 0 };
	if ( world->recording != NULL )
	{
		b2RecQueryBegin( &recWriter, context );
		recWriter.userFcn.castFcn = fcn;
		b2RecW_WORLDID( &recWriter.buf, worldId );
		b2RecW_POSITION( &recWriter.buf, origin );
		b2RecW_VEC2( &recWriter.buf, translation );
		b2RecW_QUERYFILTER( &recWriter.buf, filter );
		recWriter.countOffset = b2RecReserveU32( &recWriter.buf );
		fcn = b2RecCastTrampoline;
		context = &recWriter;
	}

	// Tree traversal sees the origin truncated to float, displacing the ray by up to one
	// coordinate ULP, a graze sized miss tolerance at extreme range. Per-shape casts
	// re-difference against the full precision origin carried on the context.
	b2RayCastInput input = { b2ToVec2( origin ), translation, 1.0f };

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, origin, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_RayCast( world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		input.maxFraction = worldContext.fraction;
	}

	if ( world->recording != NULL )
	{
		b2RecPatchU32( &recWriter.buf, recWriter.countOffset, recWriter.hitCount );
		b2RecW_TREESTATS( &recWriter.buf, treeStats );
		b2RecQueryCommit( world->recording, 0xE2, &recWriter );
	}

	return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
static float b2RayCastClosestFcn( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	// Ignore initial overlap
	if ( fraction == 0.0f )
	{
		return -1.0f;
	}

	b2RayResult* rayResult = (b2RayResult*)context;
	rayResult->shapeId = shapeId;
	rayResult->point = point;
	rayResult->normal = normal;
	rayResult->fraction = fraction;
	rayResult->hit = true;
	return fraction;
}

b2RayResult b2World_CastRayClosest( b2WorldId worldId, b2Pos origin, b2Vec2 translation, b2QueryFilter filter )
{
	b2RayResult result = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return result;
	}

	B2_ASSERT( b2IsValidPosition( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2RayCastInput input = { b2ToVec2( origin ), translation, 1.0f };
	WorldRayCastContext worldContext = { world, b2RayCastClosestFcn, filter, 1.0f, origin, &result };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_RayCast( world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext );
		result.nodeVisits += treeResult.nodeVisits;
		result.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		input.maxFraction = worldContext.fraction;
	}

	if ( world->recording != NULL )
	{
		b2RecBuffer recBuf = { 0 };
		b2RecW_WORLDID( &recBuf, worldId );
		b2RecW_POSITION( &recBuf, origin );
		b2RecW_VEC2( &recBuf, translation );
		b2RecW_QUERYFILTER( &recBuf, filter );
		b2RecW_RAYRESULT( &recBuf, result );
		b2RecCommitRecord( world->recording, 0xE5, recBuf.data, recBuf.size );
		b2RecBufFree( &recBuf );
	}

	return result;
}

typedef struct WorldShapeCastContext
{
	b2World* world;
	b2CastResultFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	b2Pos origin;
	// origin relative input
	b2ShapeCastInput input;
	void* userContext;
} WorldShapeCastContext;

static float ShapeCastCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;

	WorldShapeCastContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return input->maxFraction;
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree.
	// The tree input is world float and would lose the cast far from the origin.
	b2ShapeCastInput localInput = worldContext->input;
	localInput.maxFraction = input->maxFraction;

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2WorldTransform transform = b2GetBodyTransformQuick( world, body );
	b2Transform localTransform = b2ToRelativeTransform( transform, worldContext->origin );

	b2CastOutput output = b2ShapeCastShape( &localInput, shape, localTransform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world->worldId, shape->generation };

		b2Pos point = b2OffsetPos( worldContext->origin, output.point );
		float fraction = worldContext->fcn( id, point, output.normal, output.fraction, worldContext->userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext->fraction = fraction;
		}

		return fraction;
	}

	return input->maxFraction;
}

b2TreeStats b2World_CastShape( b2WorldId worldId, b2Pos origin, const b2ShapeProxy* proxy, b2Vec2 translation,
							   b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidPosition( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2RecQueryWriter recWriter = { 0 };
	if ( world->recording != NULL )
	{
		b2RecQueryBegin( &recWriter, context );
		recWriter.userFcn.castFcn = fcn;
		b2RecW_WORLDID( &recWriter.buf, worldId );
		b2RecW_POSITION( &recWriter.buf, origin );
		b2RecW_SHAPEPROXY( &recWriter.buf, *proxy );
		b2RecW_VEC2( &recWriter.buf, translation );
		b2RecW_QUERYFILTER( &recWriter.buf, filter );
		recWriter.countOffset = b2RecReserveU32( &recWriter.buf );
		fcn = b2RecCastTrampoline;
		context = &recWriter;
	}

	WorldShapeCastContext worldContext = { 0 };
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = 1.0f;
	worldContext.origin = origin;
	worldContext.input.proxy = *proxy;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0f;
	worldContext.userContext = context;

	// Bound the proxy in origin relative space then lift to a conservative world float box. The
	// tree node boxes use the same directed rounding, so the swept box never clips a shape far
	// from the origin. Per shape casts re-difference at full precision against the carried origin.
	b2AABB localBox = b2MakeAABB( proxy->points, proxy->count, proxy->radius );
	b2AABB box = b2OffsetAABB( localBox, origin );
	b2BoxCastInput treeInput = { box, translation, 1.0f };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_BoxCast( world->broadPhase.trees + i, &treeInput, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}

	if ( world->recording != NULL )
	{
		b2RecPatchU32( &recWriter.buf, recWriter.countOffset, recWriter.hitCount );
		b2RecW_TREESTATS( &recWriter.buf, treeStats );
		b2RecQueryCommit( world->recording, 0xE3, &recWriter );
	}

	return treeStats;
}

typedef struct WorldMoverCastContext
{
	b2World* world;
	b2QueryFilter filter;
	float fraction;
	b2Pos origin;
	// origin relative input
	b2ShapeCastInput input;
} WorldMoverCastContext;

static float MoverCastCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;
	WorldMoverCastContext* worldContext = context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return worldContext->fraction;
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree
	b2ShapeCastInput localInput = worldContext->input;
	localInput.maxFraction = input->maxFraction;

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2Transform transform = b2ToRelativeTransform( b2GetBodyTransformQuick( world, body ), worldContext->origin );

	b2CastOutput output = b2ShapeCastShape( &localInput, shape, transform );
	if ( output.fraction == 0.0f )
	{
		// Ignore overlapping shapes
		return worldContext->fraction;
	}

	worldContext->fraction = output.fraction;
	return output.fraction;
}

float b2World_CastMover( b2WorldId worldId, b2Pos origin, const b2Capsule* mover, b2Vec2 translation, b2QueryFilter filter )
{
	B2_ASSERT( b2IsValidPosition( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );
	B2_ASSERT( mover->radius > 2.0f * B2_LINEAR_SLOP );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return 1.0f;
	}

	WorldMoverCastContext worldContext = { 0 };
	worldContext.world = world;
	worldContext.filter = filter;
	worldContext.fraction = 1.0f;
	worldContext.origin = origin;
	worldContext.input.proxy.points[0] = mover->center1;
	worldContext.input.proxy.points[1] = mover->center2;
	worldContext.input.proxy.count = 2;
	worldContext.input.proxy.radius = mover->radius;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0f;
	worldContext.input.canEncroach = true;

	// Bound the capsule in origin relative space then lift to a conservative world float box
	b2Vec2 centers[2] = { mover->center1, mover->center2 };
	b2AABB box = b2OffsetAABB( b2MakeAABB( centers, 2, mover->radius ), origin );
	b2BoxCastInput treeInput = { box, translation, 1.0f };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_BoxCast( world->broadPhase.trees + i, &treeInput, filter.maskBits, MoverCastCallback, &worldContext );

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}

	if ( world->recording != NULL )
	{
		b2RecBuffer recBuf = { 0 };
		b2RecW_WORLDID( &recBuf, worldId );
		b2RecW_POSITION( &recBuf, origin );
		b2RecW_CAPSULE( &recBuf, *mover );
		b2RecW_VEC2( &recBuf, translation );
		b2RecW_QUERYFILTER( &recBuf, filter );
		b2RecW_F32( &recBuf, worldContext.fraction );
		b2RecCommitRecord( world->recording, 0xE6, recBuf.data, recBuf.size );
		b2RecBufFree( &recBuf );
	}

	return worldContext.fraction;
}

typedef struct WorldMoverContext
{
	b2World* world;
	b2PlaneResultFcn* fcn;
	b2QueryFilter filter;
	b2Capsule mover;
	b2Pos origin;
	void* userContext;
} WorldMoverContext;

static bool TreeCollideCallback( int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;
	WorldMoverContext* worldContext = (WorldMoverContext*)context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return true;
	}

	// Re-center on the query origin, the mover and the resulting planes are origin relative
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2Transform transform = b2ToRelativeTransform( b2GetBodyTransformQuick( world, body ), worldContext->origin );

	b2PlaneResult result = b2CollideMover( &worldContext->mover, shape, transform );

	// todo handle deep overlap
	if ( result.hit && b2IsNormalized( result.plane.normal ) )
	{
		b2ShapeId id = { shape->id + 1, world->worldId, shape->generation };
		return worldContext->fcn( id, &result, worldContext->userContext );
	}

	return true;
}

// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
// not be worth it.
void b2World_CollideMover( b2WorldId worldId, b2Pos origin, const b2Capsule* mover, b2QueryFilter filter,
						   b2PlaneResultFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_ASSERT( b2IsValidPosition( origin ) );

	b2RecQueryWriter recWriter = { 0 };
	if ( world->recording != NULL )
	{
		b2RecQueryBegin( &recWriter, context );
		recWriter.userFcn.planeFcn = fcn;
		b2RecW_WORLDID( &recWriter.buf, worldId );
		b2RecW_POSITION( &recWriter.buf, origin );
		b2RecW_CAPSULE( &recWriter.buf, *mover );
		b2RecW_QUERYFILTER( &recWriter.buf, filter );
		recWriter.countOffset = b2RecReserveU32( &recWriter.buf );
		fcn = b2RecPlaneTrampoline;
		context = &recWriter;
	}

	b2Vec2 r = { mover->radius, mover->radius };

	// Relative box lifted to world float with outward rounding, conservative for the tree
	b2AABB relBox;
	relBox.lowerBound = b2Sub( b2Min( mover->center1, mover->center2 ), r );
	relBox.upperBound = b2Add( b2Max( mover->center1, mover->center2 ), r );
	b2AABB aabb = b2OffsetAABB( relBox, origin );

	WorldMoverContext worldContext = {
		world, fcn, filter, *mover, origin, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeCollideCallback, &worldContext );
	}

	if ( world->recording != NULL )
	{
		b2RecPatchU32( &recWriter.buf, recWriter.countOffset, recWriter.hitCount );
		// CollideMover returns void; no TREESTATS tail
		b2RecQueryCommit( world->recording, 0xE4, &recWriter );
	}
}

void b2World_SetCustomFilterCallback( b2WorldId worldId, b2CustomFilterFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( fcn != NULL && world->recording != NULL )
	{
		printf( "b2World_SetCustomFilterCallback: customFilter not supported while recording\n" );
		B2_ASSERT( false && "customFilter callbacks are not supported while recording" );
	}
	world->customFilterFcn = fcn;
	world->customFilterContext = context;
}

void b2World_SetPreSolveCallback( b2WorldId worldId, b2PreSolveFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( fcn != NULL && world->recording != NULL )
	{
		printf( "b2World_SetPreSolveCallback: preSolve not supported while recording\n" );
		B2_ASSERT( false && "preSolve callbacks are not supported while recording" );
	}
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b2World_SetGravity( b2WorldId worldId, b2Vec2 gravity )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_REC( world, WorldSetGravity, worldId, gravity );
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
	b2Pos position;
	float radius;
	float falloff;
	float impulsePerLength;
};

static bool ExplosionCallback( int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;
	struct ExplosionContext* explosionContext = context;
	b2World* world = explosionContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	B2_ASSERT( body->type == b2_dynamicBody );

	b2WorldTransform xf = b2GetBodyTransformQuick( world, body );

	// Re-center the explosion into the shape local frame so distance and direction stay precise
	// far from the origin. Everything below runs in that near-origin frame.
	b2Vec2 localPosition = b2InvTransformWorldPoint( xf, explosionContext->position );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &localPosition, 1, 0.0f );
	input.transform = b2Transform_identity;
	input.useRadii = true;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, NULL, 0 );

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
		closestPoint = b2GetShapeCentroid( shape );
	}

	b2Vec2 direction = b2Sub( closestPoint, localPosition );
	if ( b2LengthSquared( direction ) > 100.0f * FLT_EPSILON * FLT_EPSILON )
	{
		direction = b2Normalize( direction );
	}
	else
	{
		direction = (b2Vec2){ 1.0f, 0.0f };
	}

	b2Vec2 localLine = b2LeftPerp( direction );
	float perimeter = b2GetShapeProjectedPerimeter( shape, localLine );
	float scale = 1.0f;
	if ( output.distance > radius && falloff > 0.0f )
	{
		scale = b2ClampFloat( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
	}

	float magnitude = explosionContext->impulsePerLength * perimeter * scale;
	b2Vec2 impulse = b2MulSV( magnitude, b2RotateVector( xf.q, direction ) );

	int localIndex = body->localIndex;
	b2SolverSet* set = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* state = b2Array_Get( set->bodyStates, localIndex );
	b2BodySim* bodySim = b2Array_Get( set->bodySims, localIndex );
	state->linearVelocity = b2MulAdd( state->linearVelocity, bodySim->invMass, impulse );

	// Lever arm from the center of mass to the closest point, rotated to world
	b2Vec2 r = b2RotateVector( xf.q, b2Sub( closestPoint, bodySim->localCenter ) );
	state->angularVelocity += bodySim->invInertia * b2Cross( r, impulse );

	return true;
}

void b2World_Explode( b2WorldId worldId, const b2ExplosionDef* explosionDef )
{
	uint64_t maskBits = explosionDef->maskBits;
	b2Pos position = explosionDef->position;
	float radius = explosionDef->radius;
	float falloff = explosionDef->falloff;
	float impulsePerLength = explosionDef->impulsePerLength;

	B2_ASSERT( b2IsValidPosition( position ) );
	B2_ASSERT( b2IsValidFloat( radius ) && radius >= 0.0f );
	B2_ASSERT( b2IsValidFloat( falloff ) && falloff >= 0.0f );
	B2_ASSERT( b2IsValidFloat( impulsePerLength ) );

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	B2_REC( world, WorldExplode, worldId, *explosionDef );

	struct ExplosionContext explosionContext = { world, position, radius, falloff, impulsePerLength };

	// The broad-phase tree is float, so translate a local query box out to world with outward rounding
	float extent = radius + falloff;
	b2AABB localBox = { { -extent, -extent }, { extent, extent } };
	b2AABB aabb = b2OffsetAABB( localBox, position );

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

	B2_REC( world, WorldRebuildStaticTree, worldId );

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;
	b2DynamicTree_Rebuild( staticTree, true );
}

void b2World_EnableSpeculative( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_REC( world, WorldEnableSpeculative, worldId, flag );
	world->enableSpeculative = flag;
}

#if B2_ENABLE_VALIDATION
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

		b2ValidateUsedId( &world->bodyIdPool, bodyIndex );

		B2_ASSERT( bodyIndex == body->id );

		// Need to get the root island because islands are not merged until the next time step
		int bodyIslandId = body->islandId;
		int bodySetIndex = body->setIndex;

		int contactKey = body->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			b2Contact* contact = b2Array_Get( world->contacts, contactId );

			bool touching = ( contact->flags & b2_contactTouchingFlag ) != 0;
			if ( touching )
			{
				if ( bodySetIndex != b2_staticSet )
				{
					int contactIslandId = contact->islandId;
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

			b2Joint* joint = b2Array_Get( world->joints, jointId );

			int otherEdgeIndex = edgeIndex ^ 1;

			b2Body* otherBody = b2Array_Get( world->bodies, joint->edges[otherEdgeIndex].bodyId );

			if ( bodySetIndex == b2_disabledSet || otherBody->setIndex == b2_disabledSet )
			{
				B2_ASSERT( joint->islandId == B2_NULL_INDEX );
			}
			else if ( bodySetIndex == b2_staticSet )
			{
				// Intentional nesting
				if ( otherBody->setIndex == b2_staticSet )
				{
					B2_ASSERT( joint->islandId == B2_NULL_INDEX );
				}
			}
			else if ( body->type != b2_dynamicBody && otherBody->type != b2_dynamicBody )
			{
				B2_ASSERT( joint->islandId == B2_NULL_INDEX );
			}
			else
			{
				int jointIslandId = joint->islandId;
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
			else if ( setIndex == b2_disabledSet )
			{
				B2_ASSERT( set->islandSims.count == 0 );
				B2_ASSERT( set->bodyStates.count == 0 );
			}
			else if ( setIndex == b2_awakeSet )
			{
				B2_ASSERT( set->bodySims.count == set->bodyStates.count );
				B2_ASSERT( set->jointSims.count == 0 );
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

					uint32_t syncedFlags = body->flags & ~b2_bodyTransientFlags;
					B2_ASSERT( ( bodySim->flags & syncedFlags ) == syncedFlags );

					b2BodyState* bodyState = b2GetBodyState( world, body );
					if ( bodyState != NULL )
					{
						B2_ASSERT( ( bodyState->flags & syncedFlags ) == syncedFlags );
					}

					if ( body->type == b2_dynamicBody )
					{
						B2_ASSERT( body->flags & b2_dynamicFlag );
					}

					if ( setIndex == b2_disabledSet )
					{
						B2_ASSERT( body->headContactKey == B2_NULL_INDEX );
					}

					// Validate body shapes
					int prevShapeId = B2_NULL_INDEX;
					int shapeId = body->headShapeId;
					while ( shapeId != B2_NULL_INDEX )
					{
						b2Shape* shape = b2Array_Get( world->shapes, shapeId );
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

						b2Contact* contact = b2Array_Get( world->contacts, contactId );
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

						b2Joint* joint = b2Array_Get( world->joints, jointId );

						int otherEdgeIndex = edgeIndex ^ 1;

						b2Body* otherBody = b2Array_Get( world->bodies, joint->edges[otherEdgeIndex].bodyId );

						if ( setIndex == b2_disabledSet || otherBody->setIndex == b2_disabledSet )
						{
							B2_ASSERT( joint->setIndex == b2_disabledSet );
						}
						else if ( setIndex == b2_staticSet && otherBody->setIndex == b2_staticSet )
						{
							B2_ASSERT( joint->setIndex == b2_staticSet );
						}
						else if ( body->type != b2_dynamicBody && otherBody->type != b2_dynamicBody )
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
					b2Contact* contact = b2Array_Get( world->contacts, contactSim->contactId );
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
					b2Joint* joint = b2Array_Get( world->joints, jointSim->jointId );
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
					b2Island* island = b2Array_Get( world->islands, islandSim->islandId );
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
	for ( int colorIndex = 0; colorIndex < B2_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b2GraphColor* color = world->constraintGraph.colors + colorIndex;
		int bitCount = 0;

		B2_ASSERT( color->contactSims.count >= 0 );

		totalContactCount += color->contactSims.count;
		for ( int i = 0; i < color->contactSims.count; ++i )
		{
			b2ContactSim* contactSim = color->contactSims.data + i;
			b2Contact* contact = b2Array_Get( world->contacts, contactSim->contactId );
			// contact should be touching in the constraint graph or awaiting transfer to non-touching
			B2_ASSERT( contactSim->manifold.pointCount > 0 ||
					   ( contactSim->simFlags & ( b2_simStoppedTouching | b2_simDisjoint ) ) != 0 );
			B2_ASSERT( contact->setIndex == b2_awakeSet );
			B2_ASSERT( contact->colorIndex == colorIndex );
			B2_ASSERT( contact->localIndex == i );

			int bodyIdA = contact->edges[0].bodyId;
			int bodyIdB = contact->edges[1].bodyId;

			if ( colorIndex < B2_OVERFLOW_INDEX )
			{
				b2Body* bodyA = b2Array_Get( world->bodies, bodyIdA );
				b2Body* bodyB = b2Array_Get( world->bodies, bodyIdB );
				B2_ASSERT( b2GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b2_dynamicBody ) );
				B2_ASSERT( b2GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b2_dynamicBody ) );

				bitCount += bodyA->type == b2_dynamicBody ? 1 : 0;
				bitCount += bodyB->type == b2_dynamicBody ? 1 : 0;
			}
		}

		B2_ASSERT( color->jointSims.count >= 0 );
		totalJointCount += color->jointSims.count;
		for ( int i = 0; i < color->jointSims.count; ++i )
		{
			b2JointSim* jointSim = color->jointSims.data + i;
			b2Joint* joint = b2Array_Get( world->joints, jointSim->jointId );
			B2_ASSERT( joint->setIndex == b2_awakeSet );
			B2_ASSERT( joint->colorIndex == colorIndex );
			B2_ASSERT( joint->localIndex == i );

			int bodyIdA = joint->edges[0].bodyId;
			int bodyIdB = joint->edges[1].bodyId;

			if ( colorIndex < B2_OVERFLOW_INDEX )
			{
				b2Body* bodyA = b2Array_Get( world->bodies, bodyIdA );
				b2Body* bodyB = b2Array_Get( world->bodies, bodyIdB );
				B2_ASSERT( b2GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b2_dynamicBody ) );
				B2_ASSERT( b2GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b2_dynamicBody ) );

				bitCount += bodyA->type == b2_dynamicBody ? 1 : 0;
				bitCount += bodyB->type == b2_dynamicBody ? 1 : 0;
			}
		}

		// Validate the bit population for this graph color
		B2_ASSERT( bitCount == b2CountSetBits( &color->bodySet ) );
	}

	int contactIdCount = b2GetIdCount( &world->contactIdPool );
	B2_ASSERT( totalContactCount == contactIdCount );
	B2_ASSERT( totalContactCount == (int)world->broadPhase.pairSet.count );

	int jointIdCount = b2GetIdCount( &world->jointIdPool );
	B2_ASSERT( totalJointCount == jointIdCount );

// Validate shapes
// This is very slow on compounds
#if 0
	int shapeCapacity = world->shapes.count;
	for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
	{
		b2Shape* shape = world->shapes.data + shapeIndex;
		if (shape->id != shapeIndex)
		{
			continue;
		}

		b2Body* body = b2Array_Get(world->bodies, shape->bodyId);

		b2SolverSet* set = b2Array_Get(world->solverSets, body->setIndex);
		b2BodySim* bodySim = b2Array_Get(set->bodySims, body->localIndex);
		B2_ASSERT(bodySim->bodyId == shape->bodyId);

		bool found = false;
		int shapeCount = 0;
		int index = body->headShapeId;
		while (index != B2_NULL_INDEX)
		{
			b2Shape* s = b2Array_Get(world->shapes, index);
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
		b2Contact* contact = b2Array_Get( world->contacts, contactIndex );
		if ( contact->contactId == B2_NULL_INDEX )
		{
			continue;
		}

		B2_ASSERT( contact->contactId == contactIndex );

		allocatedContactCount += 1;

		bool touching = ( contact->flags & b2_contactTouchingFlag ) != 0;

		int setId = contact->setIndex;

		if ( setId == b2_awakeSet )
		{
			if ( touching )
			{
				B2_ASSERT( 0 <= contact->colorIndex && contact->colorIndex < B2_GRAPH_COLOR_COUNT );
			}
			else
			{
				B2_ASSERT( contact->colorIndex == B2_NULL_INDEX );
			}
		}
		else if ( setId >= b2_firstSleepingSet )
		{
			// Only touching contacts allowed in a sleeping set
			B2_ASSERT( touching == true );
		}
		else
		{
			// Sleeping and non-touching contacts belong in the disabled set
			B2_ASSERT( touching == false && setId == b2_disabledSet );
		}

		b2ContactSim* contactSim = b2GetContactSim( world, contact );
		B2_ASSERT( contactSim->contactId == contactIndex );
		B2_ASSERT( contactSim->bodyIdA == contact->edges[0].bodyId );
		B2_ASSERT( contactSim->bodyIdB == contact->edges[1].bodyId );

		bool simTouching = ( contactSim->simFlags & b2_simTouchingFlag ) != 0;
		B2_ASSERT( touching == simTouching );

		B2_ASSERT( 0 <= contactSim->manifold.pointCount && contactSim->manifold.pointCount <= 2 );
	}

	int contactIdCount = b2GetIdCount( &world->contactIdPool );
	B2_ASSERT( allocatedContactCount == contactIdCount );
}

#else

void b2ValidateConnectivity( b2World* world )
{
	B2_UNUSED( world );
}

void b2ValidateSolverSets( b2World* world )
{
	B2_UNUSED( world );
}

void b2ValidateContacts( b2World* world )
{
	B2_UNUSED( world );
}

#endif
