// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"

#include "arena_allocator.h"
#include "array.h"
#include "atomic.h"
#include "bitset.h"
#include "body.h"
#include "cluster.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>

// these are useful for solver testing
#define ITERATIONS 1
#define RELAX_ITERATIONS 1

// Compare to SDL_CPUPauseInstruction
#if ( defined( __GNUC__ ) || defined( __clang__ ) ) && ( defined( __i386__ ) || defined( __x86_64__ ) )
static inline void b2Pause( void )
{
	__asm__ __volatile__( "pause\n" );
}
#elif ( defined( __arm__ ) && defined( __ARM_ARCH ) && __ARM_ARCH >= 7 ) || defined( __aarch64__ )
static inline void b2Pause( void )
{
	__asm__ __volatile__( "yield" ::: "memory" );
}
#elif defined( _MSC_VER ) && ( defined( _M_IX86 ) || defined( _M_X64 ) )
static inline void b2Pause( void )
{
	_mm_pause();
}
#elif defined( _MSC_VER ) && ( defined( _M_ARM ) || defined( _M_ARM64 ) )
static inline void b2Pause( void )
{
	__yield();
}
#else
static inline void b2Pause( void )
{
}
#endif

typedef struct b2WorkerContext
{
	b2StepContext* context;
	int workerIndex;
	void* userTask;
} b2WorkerContext;

#define B2_MAX_CONTINUOUS_SENSOR_HITS 8

struct b2ContinuousContext
{
	b2World* world;
	b2Body* fastBody;
	b2Shape* fastShape;
	b2Vec2 centroid1, centroid2;
	b2Sweep sweep;
	float fraction;
	b2SensorHit sensorHits[B2_MAX_CONTINUOUS_SENSOR_HITS];
	float sensorFractions[B2_MAX_CONTINUOUS_SENSOR_HITS];
	int sensorCount;
};

#define B2_CORE_FRACTION 0.25f

// This is called from b2DynamicTree_Query for continuous collision
static bool b2ContinuousQueryCallback( int proxyId, uint64_t userData, void* context )
{
	B2_UNUSED( proxyId );

	int shapeId = (int)userData;

	struct b2ContinuousContext* continuousContext = context;
	b2Shape* fastShape = continuousContext->fastShape;
	b2Body* fastBody = continuousContext->fastBody;

	B2_ASSERT( fastShape->sensorIndex == B2_NULL_INDEX );

	// Skip same shape
	if ( shapeId == fastShape->id )
	{
		return true;
	}

	b2World* world = continuousContext->world;

	b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

	// Skip same body
	if ( shape->bodyId == fastShape->bodyId )
	{
		return true;
	}

	bool isSensor = shape->sensorIndex != B2_NULL_INDEX;

	// Skip sensors unless the shapes want sensor events
	if ( isSensor && ( shape->enableSensorEvents == false || fastShape->enableSensorEvents == false ) )
	{
		return true;
	}

	// Skip filtered shapes
	bool canCollide = b2ShouldShapesCollide( fastShape->filter, shape->filter );
	if ( canCollide == false )
	{
		return true;
	}

	b2Body* body = b2BodyArray_Get( &world->bodies, shape->bodyId );

	B2_ASSERT( body->type == b2_staticBody || ( fastBody->flags & b2_isBullet ) );

	// Skip bullets
	if ( body->flags & b2_isBullet )
	{
		return true;
	}

	// Skip filtered bodies
	canCollide = b2ShouldBodiesCollide( world, fastBody, body );
	if ( canCollide == false )
	{
		return true;
	}

	// Custom user filtering
	if ( shape->enableCustomFiltering || fastShape->enableCustomFiltering )
	{
		b2CustomFilterFcn* customFilterFcn = world->customFilterFcn;
		if ( customFilterFcn != NULL )
		{
			b2ShapeId idA = { shape->id + 1, world->worldId, shape->generation };
			b2ShapeId idB = { fastShape->id + 1, world->worldId, fastShape->generation };
			canCollide = customFilterFcn( idA, idB, world->customFilterContext );
			if ( canCollide == false )
			{
				return true;
			}
		}
	}

	// Early out on fast parallel movement over a chain shape.
	if ( shape->type == b2_chainSegmentShape )
	{
		b2Transform transform = body->transform;
		b2Vec2 p1 = b2TransformPoint( transform, shape->chainSegment.segment.point1 );
		b2Vec2 p2 = b2TransformPoint( transform, shape->chainSegment.segment.point2 );
		b2Vec2 e = b2Sub( p2, p1 );
		float length;
		e = b2GetLengthAndNormalize( &length, e );
		if ( length > B2_LINEAR_SLOP )
		{
			b2Vec2 c1 = continuousContext->centroid1;
			float separation1 = b2Cross( b2Sub( c1, p1 ), e );
			b2Vec2 c2 = continuousContext->centroid2;
			float separation2 = b2Cross( b2Sub( c2, p1 ), e );

			float coreDistance = B2_CORE_FRACTION * fastBody->minExtent;

			if ( separation1 < 0.0f || ( separation1 - separation2 < coreDistance && separation2 > coreDistance ) )
			{
				// Minimal clipping
				return true;
			}
		}
	}

	// todo_erin testing early out for segments
#if 0
	if ( shape->type == b2_segmentShape )
	{
		b2Transform transform = bodySim->transform;
		b2Vec2 p1 = b2TransformPoint( transform, shape->segment.point1 );
		b2Vec2 p2 = b2TransformPoint( transform, shape->segment.point2 );
		b2Vec2 e = b2Sub( p2, p1 );
		b2Vec2 c1 = continuousContext->centroid1;
		b2Vec2 c2 = continuousContext->centroid2;
		float offset1 = b2Cross( b2Sub( c1, p1 ), e );
		float offset2 = b2Cross( b2Sub( c2, p1 ), e );

		if ( offset1 > 0.0f && offset2 > 0.0f )
		{
			// Started behind or finished in front
			return true;
		}

		if ( offset1 < 0.0f && offset2 < 0.0f )
		{
			// Started behind or finished in front
			return true;
		}
	}
#endif

	b2TOIInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeShapeDistanceProxy( fastShape );
	input.sweepA = b2MakeSweep( body );
	input.sweepB = continuousContext->sweep;
	input.maxFraction = continuousContext->fraction;

	b2TOIOutput output = b2TimeOfImpact( &input );
	if ( isSensor )
	{
		// Only accept a sensor hit that is sooner than the current solid hit.
		if ( output.fraction <= continuousContext->fraction && continuousContext->sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS )
		{
			int index = continuousContext->sensorCount;

			// The hit shape is a sensor
			b2SensorHit sensorHit = {
				.sensorId = shape->id,
				.visitorId = fastShape->id,
			};

			continuousContext->sensorHits[index] = sensorHit;
			continuousContext->sensorFractions[index] = output.fraction;
			continuousContext->sensorCount += 1;
		}
	}
	else
	{
		float hitFraction = continuousContext->fraction;
		bool didHit = false;

		if ( 0.0f < output.fraction && output.fraction < continuousContext->fraction )
		{
			hitFraction = output.fraction;
			didHit = true;
		}
		else if ( 0.0f == output.fraction )
		{
			// fallback to TOI of a small circle around the fast shape centroid
			b2Vec2 centroid = b2GetShapeCentroid( fastShape );
			b2ShapeExtent extent = b2ComputeShapeExtent( fastShape, centroid );
			float radius = B2_CORE_FRACTION * extent.minExtent;
			input.proxyB = b2MakeProxy( &centroid, 1, radius );
			output = b2TimeOfImpact( &input );
			if ( 0.0f < output.fraction && output.fraction < continuousContext->fraction )
			{
				hitFraction = output.fraction;
				didHit = true;
			}
		}

		if ( didHit && ( shape->enablePreSolveEvents || fastShape->enablePreSolveEvents ) && world->preSolveFcn != NULL )
		{
			b2ShapeId shapeIdA = { shape->id + 1, world->worldId, shape->generation };
			b2ShapeId shapeIdB = { fastShape->id + 1, world->worldId, fastShape->generation };
			didHit = world->preSolveFcn( shapeIdA, shapeIdB, output.point, output.normal, world->preSolveContext );
		}

		if ( didHit )
		{
			fastBody->flags |= b2_hadTimeOfImpact;
			continuousContext->fraction = hitFraction;
		}
	}

	// Continue query
	return true;
}

static void b2SolveContinuous( b2World* world, b2Body* fastBody, b2TaskContext* taskContext )
{
	b2TracyCZoneNC( ccd, "CCD", b2_colorDarkGoldenRod, true );

	b2Sweep sweep = b2MakeSweep( fastBody );

	b2Transform xf1;
	xf1.q = sweep.q1;
	xf1.p = b2Sub( sweep.c1, b2RotateVector( sweep.q1, sweep.localCenter ) );

	b2Transform xf2;
	xf2.q = sweep.q2;
	xf2.p = b2Sub( sweep.c2, b2RotateVector( sweep.q2, sweep.localCenter ) );

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;
	b2DynamicTree* kinematicTree = world->broadPhase.trees + b2_kinematicBody;
	b2DynamicTree* dynamicTree = world->broadPhase.trees + b2_dynamicBody;

	struct b2ContinuousContext context = { 0 };
	context.world = world;
	context.sweep = sweep;
	context.fastBody = fastBody;
	context.fraction = 1.0f;

	bool isBullet = ( fastBody->flags & b2_isBullet ) != 0;

	int shapeId = fastBody->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* fastShape = b2ShapeArray_Get( &world->shapes, shapeId );
		shapeId = fastShape->nextShapeId;

		context.fastShape = fastShape;
		context.centroid1 = b2TransformPoint( xf1, fastShape->localCentroid );
		context.centroid2 = b2TransformPoint( xf2, fastShape->localCentroid );

		b2AABB box1 = fastShape->aabb;
		b2AABB box2 = b2ComputeShapeAABB( fastShape, xf2 );

		// Store this to avoid double computation in the case there is no impact event
		fastShape->aabb = box2;

		// No continuous collision for sensors (but still need the updated bounds)
		if ( fastShape->sensorIndex != B2_NULL_INDEX )
		{
			continue;
		}

		b2AABB sweptBox = b2AABB_Union( box1, box2 );

		b2DynamicTree_Query( staticTree, sweptBox, B2_DEFAULT_MASK_BITS, b2ContinuousQueryCallback, &context );

		if ( isBullet )
		{
			b2DynamicTree_Query( kinematicTree, sweptBox, B2_DEFAULT_MASK_BITS, b2ContinuousQueryCallback, &context );
			b2DynamicTree_Query( dynamicTree, sweptBox, B2_DEFAULT_MASK_BITS, b2ContinuousQueryCallback, &context );
		}
	}

	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;

	if ( context.fraction < 1.0f )
	{
		// Handle time of impact event
		b2Rot q = b2NLerp( sweep.q1, sweep.q2, context.fraction );
		b2Vec2 c = b2Lerp( sweep.c1, sweep.c2, context.fraction );
		b2Vec2 origin = b2Sub( c, b2RotateVector( q, sweep.localCenter ) );

		// Advance body
		b2Transform transform = { origin, q };
		fastBody->transform = transform;
		fastBody->center = c;
		fastBody->rotation0 = q;
		fastBody->center0 = c;

		// Update body move event
		b2BodyMoveEvent* event = b2BodyMoveEventArray_Get( &world->bodyMoveEvents, fastBody->localIndex );
		event->transform = transform;

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = fastBody->headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

			// Must recompute aabb at the interpolated transform
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

				shape->enlargedAABB = true;
				fastBody->flags |= b2_enlargeBounds;
			}

			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		// No time of impact event

		// Advance body
		fastBody->rotation0 = fastBody->transform.q;
		fastBody->center0 = fastBody->center;

		// Prepare AABBs for broad-phase
		shapeId = fastBody->headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

			// shape->aabb is still valid from above

			if ( b2AABB_Contains( shape->fatAABB, shape->aabb ) == false )
			{
				float margin = shape->aabbMargin;
				b2AABB fatAABB;
				fatAABB.lowerBound.x = shape->aabb.lowerBound.x - margin;
				fatAABB.lowerBound.y = shape->aabb.lowerBound.y - margin;
				fatAABB.upperBound.x = shape->aabb.upperBound.x + margin;
				fatAABB.upperBound.y = shape->aabb.upperBound.y + margin;
				shape->fatAABB = fatAABB;

				shape->enlargedAABB = true;
				fastBody->flags |= b2_enlargeBounds;
			}

			shapeId = shape->nextShapeId;
		}
	}

	// Push sensor hits on the the task context for serial processing.
	for ( int i = 0; i < context.sensorCount; ++i )
	{
		// Skip any sensor hits that occurred after a solid hit
		if ( context.sensorFractions[i] < context.fraction )
		{
			b2SensorHitArray_Push( &taskContext->sensorHits, context.sensorHits[i] );
		}
	}

	b2TracyCZoneEnd( ccd );
}

static void b2FinalizeBodiesTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( finalize_transforms, "Transforms", b2_colorMediumSeaGreen, true );

	b2StepContext* stepContext = context;
	b2World* world = stepContext->world;
	b2ClusterManager* clusterManager = &world->clusterManager;

	B2_ASSERT( (int)threadIndex < world->workerCount );

	bool enableSleep = world->enableSleep;
	b2BodyState* states = stepContext->states;
	b2ClusterBody* clusterBodies = stepContext->clusterBodies;
	b2Body* bodies = world->bodies.data;
	float timeStep = stepContext->dt;
	float invTimeStep = stepContext->inv_dt;

	uint16_t worldId = world->worldId;

	// The body move event array should already have the correct size
	B2_ASSERT( endIndex <= world->bodyMoveEvents.count );
	b2BodyMoveEvent* moveEvents = world->bodyMoveEvents.data;

	b2BitSet* enlargedSimBitSet = &world->taskContexts.data[threadIndex].enlargedSimBitSet;
	b2BitSet* awakeIslandBitSet = &world->taskContexts.data[threadIndex].awakeIslandBitSet;
	b2TaskContext* taskContext = world->taskContexts.data + threadIndex;

	bool enableContinuous = world->enableContinuous;

	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;

	B2_ASSERT( startIndex <= endIndex );

	for ( int stateIndex = startIndex; stateIndex < endIndex; ++stateIndex )
	{
		b2BodyState* state = states + stateIndex;
		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

		if ( state->flags & b2_lockLinearX )
		{
			v.x = 0.0f;
		}

		if ( state->flags & b2_lockLinearY )
		{
			v.y = 0.0f;
		}

		if ( state->flags & b2_lockAngularZ )
		{
			w = 0.0f;
		}

		B2_VALIDATE( 0 <= state->bodyId && state->bodyId < world->bodies.count );
		b2Body* body = bodies + state->bodyId;
		int awakeIndex = body->localIndex;
		body->bodyMoveIndex = awakeIndex;

		// Reset state index for safety
		body->stateIndex = B2_NULL_INDEX;

		if ( b2IsValidVec2( v ) == false )
		{
			b2Log( "bad body: %s\n", body->name );
		}

		B2_ASSERT( b2IsValidVec2( v ) );
		B2_ASSERT( b2IsValidFloat( w ) );

		body->linearVelocity = v;
		body->angularVelocity = w;
		body->center = b2Add( body->center, state->deltaPosition );

		// Find best cluster
		int bestIndex = 0;
		float minDistSqr = b2DistanceSquared( body->center, clusterManager->clusters[0].center );
		for ( int clusterIndex = 1; clusterIndex < B2_CLUSTER_COUNT; ++clusterIndex )
		{
			float distSqr = b2DistanceSquared( body->center, clusterManager->clusters[clusterIndex].center );
			if ( distSqr < minDistSqr )
			{
				bestIndex = clusterIndex;
				minDistSqr = distSqr;
			}
		}

		body->clusterIndex = (int16_t)bestIndex;

		clusterBodies[stateIndex] = (b2ClusterBody){
			.position = body->center,
			.clusterIndex = (int16_t)bestIndex,
		};

		body->transform.q = b2NormalizeRot( b2MulRot( state->deltaRotation, body->transform.q ) );

		// Use the velocity of the farthest point on the body to account for rotation.
		float maxVelocity = b2Length( v ) + b2AbsFloat( w ) * body->maxExtent;

		// Sleep needs to observe position correction as well as true velocity.
		float maxDeltaPosition = b2Length( state->deltaPosition ) + b2AbsFloat( state->deltaRotation.s ) * body->maxExtent;

		// Position correction is not as important for sleep as true velocity.
		float positionSleepFactor = 0.5f;

		float sleepVelocity = b2MaxFloat( maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition );

		body->transform.p = b2Sub( body->center, b2RotateVector( body->transform.q, body->localCenter ) );

		// cache miss here, however I need the shape list below
		moveEvents[awakeIndex].transform = body->transform;
		moveEvents[awakeIndex].bodyId = (b2BodyId){ state->bodyId + 1, worldId, body->generation };
		moveEvents[awakeIndex].userData = body->userData;
		moveEvents[awakeIndex].fellAsleep = false;

		// reset applied force and torque
		body->force = b2Vec2_zero;
		body->torque = 0.0f;

		// If you hit this then it means you deferred mass computation but never called b2Body_ApplyMassFromShapes
		B2_ASSERT( ( body->flags & b2_dirtyMass ) == 0 );

		body->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );
		body->flags |= ( body->flags & ( b2_isSpeedCapped | b2_hadTimeOfImpact ) );
		body->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );

		if ( enableSleep == false || body->enableSleep == false || sleepVelocity > body->sleepThreshold )
		{
			// Body is not sleepy
			body->sleepTime = 0.0f;

			if ( body->type == b2_dynamicBody && enableContinuous && maxVelocity * timeStep > 0.5f * body->minExtent )
			{
				// This flag is only retained for debug draw
				body->flags |= b2_isFast;

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if ( body->flags & b2_isBullet )
				{
					int bulletIndex = b2AtomicFetchAddInt( &stepContext->bulletBodyCount, 1 );
					stepContext->bulletBodies[bulletIndex] = body;
				}
				else
				{
					b2SolveContinuous( world, body, taskContext );
				}
			}
			else
			{
				// Body is safe to advance
				body->center0 = body->center;
				body->rotation0 = body->transform.q;
			}
		}
		else
		{
			// Body is safe to advance and is falling asleep
			body->center0 = body->center;
			body->rotation0 = body->transform.q;
			body->sleepTime += timeStep;
		}

		// Any single body in an island can keep it awake
		b2Island* island = b2Array_Get( world->islands, body->islandId );
		if ( body->sleepTime < B2_TIME_TO_SLEEP )
		{
			// keep island awake
			int islandIndex = island->localIndex;
			b2SetBit( awakeIslandBitSet, islandIndex );
		}
		else if ( island->constraintRemoveCount > 0 )
		{
			// body wants to sleep but its island needs splitting first
			if ( body->sleepTime > taskContext->splitSleepTime )
			{
				// pick the sleepiest candidate
				taskContext->splitIslandId = body->islandId;
				taskContext->splitSleepTime = body->sleepTime;
			}
		}

		// Update shapes AABBs
		b2Transform transform = body->transform;
		bool isFast = ( body->flags & b2_isFast ) != 0;
		int shapeId = body->headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

			if ( isFast )
			{
				// For fast non-bullet bodies the AABB has already been updated in b2SolveContinuous
				// For fast bullet bodies the AABB will be updated at a later stage

				// Add to enlarged shapes regardless of AABB changes.
				// Bit-set to keep the move array sorted
				b2SetBit( enlargedSimBitSet, stateIndex );
			}
			else
			{
				b2AABB aabb = b2ComputeShapeAABB( shape, transform );
				aabb.lowerBound.x -= speculativeDistance;
				aabb.lowerBound.y -= speculativeDistance;
				aabb.upperBound.x += speculativeDistance;
				aabb.upperBound.y += speculativeDistance;
				shape->aabb = aabb;

				B2_ASSERT( shape->enlargedAABB == false );

				if ( b2AABB_Contains( shape->fatAABB, aabb ) == false )
				{
					float margin = shape->aabbMargin;
					b2AABB fatAABB;
					fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
					fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
					fatAABB.upperBound.x = aabb.upperBound.x + margin;
					fatAABB.upperBound.y = aabb.upperBound.y + margin;
					shape->fatAABB = fatAABB;

					shape->enlargedAABB = true;

					// Bit-set to keep the move array sorted
					b2SetBit( enlargedSimBitSet, stateIndex );
				}
			}

			shapeId = shape->nextShapeId;
		}
	}

	b2TracyCZoneEnd( finalize_transforms );
}

// Helper: main thread solves borders whose adjacent clusters are both complete.
// Solve borders in fixed index order, eagerly starting each border as soon as both its
// adjacent clusters are done. Never skip ahead — this ensures deterministic solve order
// even when borders share a cluster and modify overlapping body state.
static void b2SolveBordersWhenReady( b2StepContext* context, bool useBias, bool isRestitution, b2SolverStage* stage )
{
	b2TracyCZoneNC( solver_borders, "Solve Borders", b2_colorMintCream, true );

	b2BorderConstraints* borders = context->borders;
	int borderCount = context->borderCount;
	b2ClusterSolveData* clusterData = context->clusterData;

	if ( borderCount == 0 )
	{
		b2TracyCZoneEnd( solver_borders );
		return;
	}

	for ( int bi = 0; bi < borderCount; ++bi )
	{
		b2BorderConstraints* border = borders + bi;

		// Spin-wait until both adjacent clusters have finished solving
		while ( b2AtomicLoadInt( &clusterData[border->clusterA].solveComplete ) < 2 ||
				b2AtomicLoadInt( &clusterData[border->clusterB].solveComplete ) < 2 )
		{
			b2Pause();
		}

		if ( isRestitution )
		{
			if ( border->contactCount > 0 )
			{
				b2ApplyContactRestitution( context, border->contactConstraints, border->contactCount,
										   context->world->restitutionThreshold );
			}
		}
		else
		{
			if ( border->contactCount > 0 )
			{
				b2SolveContactConstraints( context, border->contactConstraints, border->contactCount, context->inv_h,
										   context->world->contactSpeed, useBias );
			}
			for ( int k = 0; k < border->jointCount; ++k )
			{
				b2SolveJoint( border->joints[k], context, useBias );
			}

			if ( stage->storeImpulses && border->contactCount > 0 )
			{
				b2StoreContactImpulses( border->contacts, border->contactConstraints, border->contactCount );
			}
		}
	}

	b2TracyCZoneEnd( solver_borders );
}

static void b2SolveWorkerClusters( b2StepContext* context, int workerIndex, bool useBias, bool isRestitution,
								   b2SolverStage* stage )
{
	b2TracyCZoneNC( solver_clusters, "Solve Clusters", b2_colorLemonChiffon, true );

	b2ClusterSolveData* clusterData = context->clusterData;

	for ( int order = 0; order < 2; ++order )
	{
		for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
		{
			bool assigned = ( context->clusterWorkerMap[c] == workerIndex );
			if ( ( order == 0 && !assigned ) || ( order == 1 && assigned ) )
			{
				continue;
			}

			b2ClusterSolveData* cd = clusterData + c;

			if ( b2AtomicCompareExchangeInt( &cd->solveComplete, 0, 1 ) )
			{
				if ( stage->integratePositions )
				{
					float h = context->h;
					for ( int i = 0; i < cd->bodyCount; ++i )
					{
						b2BodyState* state = cd->states + i;

						if ( state->flags & b2_lockLinearX )
						{
							state->linearVelocity.x = 0.0f;
						}

						if ( state->flags & b2_lockLinearY )
						{
							state->linearVelocity.y = 0.0f;
						}

						if ( state->flags & b2_lockAngularZ )
						{
							state->angularVelocity = 0.0f;
						}

						state->deltaPosition = b2MulAdd( state->deltaPosition, h, state->linearVelocity );
						state->deltaRotation = b2IntegrateRotation( state->deltaRotation, h * state->angularVelocity );
					}
				}

				if ( isRestitution )
				{
					if ( cd->contactCount > 0 )
					{
						b2ApplyContactRestitution( context, cd->contactConstraints, cd->contactCount,
												   context->world->restitutionThreshold );
					}
				}
				else
				{
					if ( cd->contactCount > 0 )
					{
						b2SolveContactConstraints( context, cd->contactConstraints, cd->contactCount, context->inv_h,
												   context->world->contactSpeed, useBias );
					}

					for ( int k = 0; k < cd->jointCount; ++k )
					{
						b2SolveJoint( cd->joints[k], context, useBias );
					}

					if ( stage->storeImpulses && cd->contactCount > 0 )
					{
						b2StoreContactImpulses( cd->contacts, cd->contactConstraints, cd->contactCount );
					}
				}

				b2AtomicStoreInt( &cd->solveComplete, 2 );
			}
		}
	}

	b2TracyCZoneEnd( solver_clusters );
}

static void b2PrepareWorkerClusters( b2StepContext* context, int workerIndex )
{
	b2TracyCZoneNC( prepare_clusters, "Prepare Clusters", b2_colorDarkOrange, true );

	b2ClusterSolveData* clusterData = context->clusterData;
	b2World* world = context->world;
	b2Cluster* clusters = world->clusterManager.clusters;

	b2Vec2 g = world->gravity;

	for ( int order = 0; order < 2; ++order )
	{
		for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
		{
			bool assigned = ( context->clusterWorkerMap[c] == workerIndex );
			if ( ( order == 0 && !assigned ) || ( order == 1 && assigned ) )
			{
				continue;
			}

			b2ClusterSolveData* cd = clusterData + c;

			if ( b2AtomicCompareExchangeInt( &cd->prepareComplete, 0, 1 ) )
			{
				// Populate body state in cluster order
				int stateIndex = clusters[c].stateOffset;
				int clusterBodyCount = clusters[c].bodyIds.count;

				for ( int j = 0; j < clusterBodyCount; ++j )
				{
					int bodyId = clusters[c].bodyIds.data[j];
					b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );

					body->stateIndex = stateIndex;

					b2BodyState* state = context->states + stateIndex;
					state->linearVelocity = body->linearVelocity;
					state->angularVelocity = body->angularVelocity;
					state->force = b2MulAdd( body->force, body->gravityScale * body->mass, g );
					state->torque = body->torque;
					state->invMass = body->invMass;
					state->invInertia = body->invInertia;
					state->flags = body->flags;
					state->bodyId = bodyId;
					state->deltaPosition = b2Vec2_zero;
					state->deltaRotation = b2Rot_identity;

					stateIndex += 1;
				}

				if ( cd->contactCount > 0 )
				{
					// Pass bodySims to remap constraint indices to local cluster indices
					b2PrepareContactConstraints( context, cd->contacts, cd->contactConstraints, cd->contactCount );
				}

				for ( int j = 0; j < cd->jointCount; ++j )
				{
					b2PrepareJoint( cd->joints[j], context );
				}

				b2AtomicStoreInt( &cd->prepareComplete, 2 );
			}
		}
	}

	b2TracyCZoneEnd( prepare_clusters );
}

static void b2PrepareBordersWhenReady( b2StepContext* context )
{
	b2TracyCZoneNC( prepare_borders, "Prepare Borders", b2_colorMintCream, true );

	b2BorderConstraints* borders = context->borders;
	int borderCount = context->borderCount;
	b2ClusterSolveData* clusterData = context->clusterData;

	if ( borderCount == 0 )
	{
		b2TracyCZoneEnd( prepare_borders );
		return;
	}

	for ( int bi = 0; bi < borderCount; ++bi )
	{
		b2BorderConstraints* border = borders + bi;

		// Spin-wait until both adjacent clusters have finished preparation
		while ( b2AtomicLoadInt( &clusterData[border->clusterA].prepareComplete ) < 2 ||
				b2AtomicLoadInt( &clusterData[border->clusterB].prepareComplete ) < 2 )
		{
			b2Pause();
		}

		if ( border->contactCount > 0 )
		{
			b2PrepareContactConstraints( context, border->contacts, border->contactConstraints, border->contactCount );
		}

		for ( int j = 0; j < border->jointCount; ++j )
		{
			b2PrepareJoint( border->joints[j], context );
		}
	}

	b2TracyCZoneEnd( prepare_borders );
}

static void b2ExecuteClusterPreparePhase( b2StepContext* context, b2SolverStage* stage, uint32_t syncBits )
{
	B2_UNUSED( stage );
	b2ClusterSolveData* clusterData = context->clusterData;

	// Reset cluster preparation flags
	for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
	{
		b2AtomicStoreInt( &clusterData[c].prepareComplete, 0 );
	}

	// Signal workers
	b2AtomicStoreU32( &context->atomicSyncBits, syncBits );

	// Main thread prepares its own clusters
	b2PrepareWorkerClusters( context, 0 );

	// Main thread prepares borders as adjacent clusters complete
	b2PrepareBordersWhenReady( context );

	// Wait for all clusters to complete preparation
	for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
	{
		while ( b2AtomicLoadInt( &clusterData[c].prepareComplete ) < 2 )
		{
			b2Pause();
		}
	}
}

static void b2WarmStartWorkerClusters( b2StepContext* context, int workerIndex )
{
	b2TracyCZoneNC( warm_start_clusters, "Warm Start Clusters", b2_colorNavy, true );

	b2ClusterSolveData* clusterData = context->clusterData;

	float h = context->h;
	float maxLinearSpeed = context->maxLinearVelocity;
	float maxAngularSpeed = B2_MAX_ROTATION * context->inv_dt;
	float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for ( int order = 0; order < 2; ++order )
	{
		for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
		{
			bool assigned = ( context->clusterWorkerMap[c] == workerIndex );
			if ( ( order == 0 && !assigned ) || ( order == 1 && assigned ) )
			{
				continue;
			}

			b2ClusterSolveData* cd = clusterData + c;

			if ( b2AtomicCompareExchangeInt( &cd->warmStartComplete, 0, 1 ) )
			{
				// Integrate velocities for this cluster's bodies
				for ( int i = 0; i < cd->bodyCount; ++i )
				{
					b2BodyState* state = cd->states + i;

					b2Vec2 v = state->linearVelocity;
					float w = state->angularVelocity;

					v = b2MulAdd( v, h * state->invMass, state->force );
					w += h * state->invInertia * state->torque;

					if ( b2Dot( v, v ) > maxLinearSpeedSquared )
					{
						float ratio = maxLinearSpeed / b2Length( v );
						v = b2MulSV( ratio, v );
						state->flags |= b2_isSpeedCapped;
					}

					if ( w * w > maxAngularSpeedSquared && ( state->flags & b2_allowFastRotation ) == 0 )
					{
						float ratio = maxAngularSpeed / b2AbsFloat( w );
						w *= ratio;
						state->flags |= b2_isSpeedCapped;
					}

					if ( state->flags & b2_lockLinearX )
					{
						v.x = 0.0f;
					}

					if ( state->flags & b2_lockLinearY )
					{
						v.y = 0.0f;
					}

					if ( state->flags & b2_lockAngularZ )
					{
						w = 0.0f;
					}

					state->linearVelocity = v;
					state->angularVelocity = w;
				}

				// Warm start constraints for this cluster
				if ( cd->contactCount > 0 )
				{
					b2WarmStartContactConstraints( context, cd->contactConstraints, cd->contactCount );
				}

				for ( int k = 0; k < cd->jointCount; ++k )
				{
					b2WarmStartJoint( cd->joints[k], context );
				}

				b2AtomicStoreInt( &cd->warmStartComplete, 2 );
			}
		}
	}

	b2TracyCZoneEnd( warm_start_clusters );
}

static void b2WarmStartBordersWhenReady( b2StepContext* context )
{
	b2TracyCZoneNC( warm_start_borders, "Warm Start Borders", b2_colorNavy, true );

	b2BorderConstraints* borders = context->borders;
	int borderCount = context->borderCount;
	b2ClusterSolveData* clusterData = context->clusterData;

	if ( borderCount == 0 )
	{
		b2TracyCZoneEnd( warm_start_borders );
		return;
	}

	for ( int bi = 0; bi < borderCount; ++bi )
	{
		b2BorderConstraints* border = borders + bi;

		// Spin-wait until both adjacent clusters have finished warm start
		while ( b2AtomicLoadInt( &clusterData[border->clusterA].warmStartComplete ) < 2 ||
				b2AtomicLoadInt( &clusterData[border->clusterB].warmStartComplete ) < 2 )
		{
			b2Pause();
		}

		if ( border->contactCount > 0 )
		{
			b2WarmStartContactConstraints( context, border->contactConstraints, border->contactCount );
		}

		for ( int k = 0; k < border->jointCount; ++k )
		{
			b2WarmStartJoint( border->joints[k], context );
		}
	}

	b2TracyCZoneEnd( warm_start_borders );
}

static void b2ExecuteClusterWarmStartPhase( b2StepContext* context, b2SolverStage* stage, uint32_t syncBits )
{
	B2_UNUSED( stage );
	b2ClusterSolveData* clusterData = context->clusterData;

	// Reset cluster warm start flags
	for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
	{
		b2AtomicStoreInt( &clusterData[c].warmStartComplete, 0 );
	}

	// Signal workers
	b2AtomicStoreU32( &context->atomicSyncBits, syncBits );

	// Main thread warm starts its own clusters
	b2WarmStartWorkerClusters( context, 0 );

	// Main thread warm starts borders as adjacent clusters complete
	b2WarmStartBordersWhenReady( context );

	// Wait for all clusters to complete warm start
	for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
	{
		while ( b2AtomicLoadInt( &clusterData[c].warmStartComplete ) < 2 )
		{
			b2Pause();
		}
	}
}

// Helper: main thread executes a cluster phase (solve, relax, or restitution).
// Signals workers, solves own clusters, solves borders, waits for workers.
static void b2ExecuteClusterPhase( b2StepContext* context, b2SolverStage* stage, uint32_t syncBits, bool useBias,
								   bool isRestitution )
{
	b2ClusterSolveData* clusterData = context->clusterData;

	// Reset cluster completion flags
	for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
	{
		b2AtomicStoreInt( &clusterData[c].solveComplete, 0 );
	}

	// Signal workers
	b2AtomicStoreU32( &context->atomicSyncBits, syncBits );

	// Main thread solves its own clusters
	b2SolveWorkerClusters( context, 0, useBias, isRestitution, stage );

	// Main thread solves borders as adjacent clusters complete
	b2SolveBordersWhenReady( context, useBias, isRestitution, stage );

	// Wait for all clusters to complete their solve work
	for ( int c = 0; c < B2_CLUSTER_COUNT; ++c )
	{
		while ( b2AtomicLoadInt( &clusterData[c].solveComplete ) < 2 )
		{
			b2Pause();
		}
	}
}

// This should not use the thread index because thread 0 can be called twice by enkiTS.
static void b2SolverTask( int startIndex, int endIndex, uint32_t threadIndexIgnore, void* taskContext )
{
	B2_UNUSED( startIndex, endIndex, threadIndexIgnore );

	b2WorkerContext* workerContext = taskContext;
	int workerIndex = workerContext->workerIndex;
	b2StepContext* context = workerContext->context;
	b2SolverStage* stages = context->stages;
	b2Profile* profile = &context->world->profile;

	if ( workerIndex == 0 )
	{
		// Main thread synchronizes the workers and does work itself.
		uint64_t ticks = b2GetTicks();

		int stageIndex = 0;

		// Prepare cluster and border constraints (parallel across workers)
		{
			uint32_t syncBits = ( 1 << 16 ) | stageIndex;
			B2_ASSERT( stages[stageIndex].type == b2_stagePrepareClusters );
			b2ExecuteClusterPreparePhase( context, stages + stageIndex, syncBits );
			stageIndex += 1;
		}

		profile->prepareConstraints += b2GetMillisecondsAndReset( &ticks );

		// This is used to push the syncBits forward across substeps
		int clusterSyncIndex = 1;

		int subStepCount = context->subStepCount;
		for ( int subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex )
		{
			int iterationStageIndex = stageIndex;

			// Integrate velocities and warm start cluster/border constraints (parallel across workers)
			{
				uint32_t syncBits = ( clusterSyncIndex << 16 ) | iterationStageIndex;
				B2_ASSERT( stages[iterationStageIndex].type == b2_stageWarmStartClusters );
				b2ExecuteClusterWarmStartPhase( context, stages + iterationStageIndex, syncBits );
				clusterSyncIndex += 1;
			}
			iterationStageIndex += 1;

			profile->warmStart += b2GetMillisecondsAndReset( &ticks );

			// Solve constraints
			bool useBias = true;
			bool isRestitution = false;
			for ( int j = 0; j < ITERATIONS; ++j )
			{
				// Cluster solve phase
				uint32_t syncBits = ( clusterSyncIndex << 16 ) | iterationStageIndex;
				B2_ASSERT( stages[iterationStageIndex].type == b2_stageSolveClusters );
				b2ExecuteClusterPhase( context, stages + iterationStageIndex, syncBits, useBias, isRestitution );
				clusterSyncIndex += 1;
			}
			iterationStageIndex += 1;

			profile->solveImpulses += b2GetMillisecondsAndReset( &ticks );

			// Relax constraints (first iteration also integrates positions)
			useBias = false;
			isRestitution = false;
			for ( int j = 0; j < RELAX_ITERATIONS; ++j )
			{
				bool isFirstRelax = ( j == 0 );
				bool isLastRelax = ( subStepIndex == subStepCount - 1 ) && ( j == RELAX_ITERATIONS - 1 );
				stages[iterationStageIndex].integratePositions = isFirstRelax;
				stages[iterationStageIndex].storeImpulses = isLastRelax;
				uint32_t syncBits = ( clusterSyncIndex << 16 ) | iterationStageIndex;
				B2_ASSERT( stages[iterationStageIndex].type == b2_stageRelaxClusters );
				b2ExecuteClusterPhase( context, stages + iterationStageIndex, syncBits, useBias, isRestitution );
				clusterSyncIndex += 1;
			}
			iterationStageIndex += 1;

			profile->relaxImpulses += b2GetMillisecondsAndReset( &ticks );
		}

		// Advance stage index past sub-step stages:
		// warm start clusters + solve clusters + relax clusters
		stageIndex += 3;

		// Restitution
		{
			uint32_t syncBits = ( clusterSyncIndex << 16 ) | stageIndex;
			B2_ASSERT( stages[stageIndex].type == b2_stageRestitutionClusters );
			bool useBias = false;
			bool isRestitution = true;
			b2ExecuteClusterPhase( context, stages + stageIndex, syncBits, useBias, isRestitution );
			stageIndex += 1;
		}

		profile->applyRestitution += b2GetMillisecondsAndReset( &ticks );

		// Signal workers to finish
		b2AtomicStoreU32( &context->atomicSyncBits, UINT_MAX );

		B2_ASSERT( stageIndex == context->stageCount );
		return;
	}

	// Worker spins and waits for work
	uint32_t lastSyncBits = 0;
	while ( true )
	{
		uint32_t syncBits;
		int spinCount = 0;
		while ( ( syncBits = b2AtomicLoadU32( &context->atomicSyncBits ) ) == lastSyncBits )
		{
			if ( spinCount > 5 )
			{
				b2Yield();
				spinCount = 0;
			}
			else
			{
				b2Pause();
				b2Pause();
				spinCount += 1;
			}
		}

		if ( syncBits == UINT_MAX )
		{
			// sentinel hit
			break;
		}

		int stageIndex = syncBits & 0xFFFF;
		B2_ASSERT( stageIndex < context->stageCount );

		b2SolverStage* stage = stages + stageIndex;

		// Branch on stage type: cluster phases vs parallel-for phases
		if ( stage->type == b2_stagePrepareClusters )
		{
			// Cluster prepare phase: prepare all clusters assigned to this worker
			b2PrepareWorkerClusters( context, workerIndex );
		}
		else if ( stage->type == b2_stageWarmStartClusters )
		{
			// Cluster warm start phase: warm start all clusters assigned to this worker
			b2WarmStartWorkerClusters( context, workerIndex );
		}
		else if ( stage->type == b2_stageSolveClusters || stage->type == b2_stageRelaxClusters ||
				  stage->type == b2_stageRestitutionClusters )
		{
			// Cluster phase: solve all clusters assigned to this worker
			bool useBias = ( stage->type == b2_stageSolveClusters );
			bool isRestitution = ( stage->type == b2_stageRestitutionClusters );

			b2SolveWorkerClusters( context, workerIndex, useBias, isRestitution, stage );
		}
		else
		{
			B2_ASSERT( false );
		}

		lastSyncBits = syncBits;
	}
}

static void b2BulletBodyTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	B2_UNUSED( threadIndex );

	b2TracyCZoneNC( bullet_body_task, "Bullet", b2_colorLightSkyBlue, true );

	b2StepContext* stepContext = context;
	b2TaskContext* taskContext = b2TaskContextArray_Get( &stepContext->world->taskContexts, threadIndex );

	B2_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2SolveContinuous( stepContext->world, stepContext->bulletBodies[i], taskContext );
	}

	b2TracyCZoneEnd( bullet_body_task );
}

#if B2_SIMD_WIDTH == 8
#define B2_SIMD_SHIFT 3
#elif B2_SIMD_WIDTH == 4
#define B2_SIMD_SHIFT 2
#else
#define B2_SIMD_SHIFT 0
#endif

// Solve with graph coloring
void b2Solve( b2World* world, b2StepContext* stepContext )
{
	world->stepIndex += 1;

	// Are there any awake bodies? This scenario should not be important for profiling.
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	int awakeBodyCount = awakeSet->bodyIds.count;
	if ( awakeBodyCount == 0 )
	{
		// Nothing to simulate, however the tree rebuild must be finished.
		if ( world->userTreeTask != NULL )
		{
			world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
			world->userTreeTask = NULL;
			world->activeTaskCount -= 1;
		}

		b2ValidateNoEnlarged( &world->broadPhase );
		return;
	}

	// Solve constraints using cluster-based solver
	{
		b2TracyCZoneNC( prepare_stages, "Prepare Stages", b2_colorDarkOrange, true );
		uint64_t prepareTicks = b2GetTicks();

		// Prepare buffers for bullets
		b2AtomicStoreInt( &stepContext->bulletBodyCount, 0 );
		stepContext->bulletBodies = b2AllocateArenaItem( &world->arena, awakeBodyCount * sizeof( b2Body* ), "bullet bodies" );

		// prepare for move events
		b2BodyMoveEventArray_Resize( &world->bodyMoveEvents, awakeBodyCount );

		int workerCount = world->workerCount;

		// Stage setup: prepare joints, prepare clusters, warm start clusters (includes velocity integration),
		// solve clusters, relax clusters (includes position integration), restitution
		int stageCount = 0;
		stageCount += 1; // b2_stagePrepareClusters
		stageCount += 1; // b2_stageWarmStartClusters (reused per sub-step, integrates velocities then warm starts)
		stageCount += 1; // b2_stageSolveClusters (reused per sub-step per iteration)
		stageCount += 1; // b2_stageRelaxClusters (reused per sub-step per iteration)
		stageCount += 1; // b2_stageRestitutionClusters

		// Body states
		stepContext->states = b2AllocateArenaItem( &world->arena, awakeBodyCount * sizeof( b2BodyState ), "states" );
		stepContext->clusterBodies =
			b2AllocateArenaItem( &world->arena, awakeBodyCount * sizeof( b2ClusterBody ), "cluster body" );

#if B2_ENABLE_VALIDATION
		for (int i = 0; i < awakeBodyCount; ++i)
		{
			stepContext->clusterBodies[i].position = b2Vec2_zero;
			stepContext->clusterBodies[i].clusterIndex = B2_NULL_INDEX;
		}
#endif

		// Classify constraints into clusters and borders
		stepContext->clusterData =
			b2AllocateArenaItem( &world->arena, B2_CLUSTER_COUNT * sizeof( b2ClusterSolveData ), "cluster solve data" );
		memset( stepContext->clusterData, 0, B2_CLUSTER_COUNT * sizeof( b2ClusterSolveData ) );

		// Compute spatial clusters before solving so constraint classification can read clusterIndex
		b2ComputeClusters( world );
		b2ClassifyConstraints( world, stepContext );

		// Assign clusters to workers using LPT (Longest Processing Time) heuristic
		// for load-balanced scheduling. Same worker handles a cluster across all phases
		// (prepare, warm start, solve, relax) preserving L2 cache affinity.
		{
			// Compute per-cluster work estimate
			int clusterWork[B2_CLUSTER_COUNT];
			int sortedIndices[B2_CLUSTER_COUNT];
			for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
			{
				b2ClusterSolveData* cd = stepContext->clusterData + i;
				clusterWork[i] = cd->contactCount + cd->jointCount;
				sortedIndices[i] = i;
			}

			// Sort cluster indices by work descending (insertion sort, only 16 elements)
			for ( int i = 1; i < B2_CLUSTER_COUNT; ++i )
			{
				int key = sortedIndices[i];
				int keyWork = clusterWork[key];
				int j = i - 1;
				while ( j >= 0 && clusterWork[sortedIndices[j]] < keyWork )
				{
					sortedIndices[j + 1] = sortedIndices[j];
					j -= 1;
				}
				sortedIndices[j + 1] = key;
			}

			// Greedily assign each cluster (heaviest first) to the least-loaded worker
			int workerLoad[B2_MAX_WORKERS] = { 0 };

			// Seed worker 0 (main thread) with border work since it solves all borders
			{
				b2BorderConstraints* borders = stepContext->borders;
				int borderCount = stepContext->borderCount;
				for ( int i = 0; i < borderCount; ++i )
				{
					workerLoad[0] += borders[i].contactCount + borders[i].jointCount;
				}
			}

			for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
			{
				int clusterIndex = sortedIndices[i];

				// Find worker with minimum load
				int minLoad = workerLoad[0];
				int minWorker = 0;
				for ( int w = 1; w < workerCount; ++w )
				{
					if ( workerLoad[w] < minLoad )
					{
						minLoad = workerLoad[w];
						minWorker = w;
					}
				}

				stepContext->clusterWorkerMap[clusterIndex] = minWorker;
				workerLoad[minWorker] += clusterWork[clusterIndex];
			}
		}

		// Must do no arena allocations during split task
		b2SolverStage* stages = b2AllocateArenaItem( &world->arena, stageCount * sizeof( b2SolverStage ), "stages" );

		// Split an awake island
		void* splitIslandTask = NULL;
		if ( world->splitIslandId != B2_NULL_INDEX )
		{
			splitIslandTask = world->enqueueTaskFcn( &b2SplitIslandTask, 1, 1, world, world->userTaskContext );
			world->taskCount += 1;
			world->activeTaskCount += splitIslandTask == NULL ? 0 : 1;
		}

		// Set up stages
		b2SolverStage* stage = stages;

		// Prepare clusters (cluster phase, runs once)
		stage->type = b2_stagePrepareClusters;
		stage += 1;

		// Warm start clusters (cluster phase, reused per sub-step; integrates velocities then warm starts)
		stage->type = b2_stageWarmStartClusters;
		stage += 1;

		// Solve clusters (cluster phase, reused per sub-step per iteration)
		stage->type = b2_stageSolveClusters;
		stage->integratePositions = false;
		stage += 1;

		// Relax clusters (cluster phase, reused per sub-step per iteration)
		stage->type = b2_stageRelaxClusters;
		stage->integratePositions = false;
		stage += 1;

		// Restitution clusters
		stage->type = b2_stageRestitutionClusters;
		stage->integratePositions = false;
		stage += 1;

		B2_ASSERT( (int)( stage - stages ) == stageCount );

		B2_ASSERT( workerCount <= B2_MAX_WORKERS );
		b2WorkerContext workerContext[B2_MAX_WORKERS];

		stepContext->workerCount = workerCount;
		stepContext->stageCount = stageCount;
		stepContext->stages = stages;
		b2AtomicStoreU32( &stepContext->atomicSyncBits, 0 );

		world->profile.prepareStages = b2GetMillisecondsAndReset( &prepareTicks );
		b2TracyCZoneEnd( prepare_stages );

		b2TracyCZoneNC( solve_constraints, "Solve Constraints", b2_colorIndigo, true );
		uint64_t constraintTicks = b2GetTicks();

		// Must use worker index because thread 0 can be assigned multiple tasks by enkiTS
		int jointIdCapacity = b2GetIdCapacity( &world->jointIdPool );
		for ( int i = 0; i < workerCount; ++i )
		{
			b2TaskContext* taskContext = b2TaskContextArray_Get( &world->taskContexts, i );
			b2SetBitCountAndClear( &taskContext->jointStateBitSet, jointIdCapacity );

			workerContext[i].context = stepContext;
			workerContext[i].workerIndex = i;
			workerContext[i].userTask = world->enqueueTaskFcn( b2SolverTask, 1, 1, workerContext + i, world->userTaskContext );
			world->taskCount += 1;
			world->activeTaskCount += workerContext[i].userTask == NULL ? 0 : 1;
		}

		// Finish island split
		if ( splitIslandTask != NULL )
		{
			world->finishTaskFcn( splitIslandTask, world->userTaskContext );
			world->activeTaskCount -= 1;
		}
		world->splitIslandId = B2_NULL_INDEX;

		// Finish constraint solve
		for ( int i = 0; i < workerCount; ++i )
		{
			if ( workerContext[i].userTask != NULL )
			{
				world->finishTaskFcn( workerContext[i].userTask, world->userTaskContext );
				world->activeTaskCount -= 1;
			}
		}

		b2FreeArenaItem( &world->arena, stages );
		stepContext->stages = NULL;

		world->profile.solveConstraints = b2GetMillisecondsAndReset( &constraintTicks );
		b2TracyCZoneEnd( solve_constraints );

		b2TracyCZoneNC( update_transforms, "Update Transforms", b2_colorMediumSeaGreen, true );
		uint64_t transformTicks = b2GetTicks();

		// Prepare contact, enlarged body, and island bit sets used in body finalization.
		int awakeIslandCount = awakeSet->islandSims.count;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			b2TaskContext* taskContext = world->taskContexts.data + i;
			b2SensorHitArray_Clear( &taskContext->sensorHits );
			b2SetBitCountAndClear( &taskContext->enlargedSimBitSet, awakeBodyCount );
			b2SetBitCountAndClear( &taskContext->awakeIslandBitSet, awakeIslandCount );
			taskContext->splitIslandId = B2_NULL_INDEX;
			taskContext->splitSleepTime = 0.0f;
		}

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		void* finalizeBodiesTask =
			world->enqueueTaskFcn( b2FinalizeBodiesTask, awakeBodyCount, 64, stepContext, world->userTaskContext );
		world->taskCount += 1;
		if ( finalizeBodiesTask != NULL )
		{
			world->finishTaskFcn( finalizeBodiesTask, world->userTaskContext );
		}

		// Update clusters. Bodies are assigned at the beginning of the solve to ensure they
		// all exist and are awake. This just computes the new centers.
		b2Cluster* clusters = world->clusterManager.clusters;
		int clusterBodyCounts[B2_CLUSTER_COUNT] = {0};
		for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
		{
			// Done with the bodies
			clusters[i].bodyIds.count = 0;
			clusters[i].accumulator = b2Vec2_zero;
		}

		// Serial for determinism
		for ( int i = 0; i < awakeBodyCount; ++i )
		{
			b2ClusterBody* clusterBody = stepContext->clusterBodies + i;
			int clusterIndex = clusterBody->clusterIndex;
			B2_ASSERT( 0 <= clusterIndex && clusterIndex < B2_CLUSTER_COUNT );
			b2Cluster* cluster = clusters + clusterIndex;
			cluster->accumulator = b2Add( cluster->accumulator, clusterBody->position );
			clusterBodyCounts[clusterIndex] += 1;
		}

		// Compute cluster centers
		for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
		{
			if ( clusterBodyCounts[i] > 0 )
			{
				clusters[i].center = b2MulSV( 1.0f / clusterBodyCounts[i], clusters[i].accumulator );
			}

			clusters[i].accumulator = b2Vec2_zero;
		}

		// Free arena allocations in reverse order (LIFO)
		// Free classify allocations in reverse: borders (last-to-first), then clusters (last-to-first)
		{
			b2BorderConstraints* borders = stepContext->borders;
			int borderCount = stepContext->borderCount;
			b2ClusterSolveData* clusterData = stepContext->clusterData;

			for ( int i = borderCount - 1; i >= 0; --i )
			{
				b2BorderConstraints* border = borders + i;
				if ( border->contactConstraints != NULL )
				{
					b2FreeArenaItem( &world->arena, border->contactConstraints );
				}
				if ( border->joints != NULL )
				{
					b2FreeArenaItem( &world->arena, border->joints );
				}
				if ( border->contacts != NULL )
				{
					b2FreeArenaItem( &world->arena, border->contacts );
				}
			}

			if ( borders != NULL )
			{
				b2FreeArenaItem( &world->arena, borders );
			}

			for ( int i = B2_CLUSTER_COUNT - 1; i >= 0; --i )
			{
				b2ClusterSolveData* cd = clusterData + i;
				if ( cd->contactConstraints != NULL )
				{
					b2FreeArenaItem( &world->arena, cd->contactConstraints );
				}
				if ( cd->joints != NULL )
				{
					b2FreeArenaItem( &world->arena, cd->joints );
				}
				if ( cd->contacts != NULL )
				{
					b2FreeArenaItem( &world->arena, cd->contacts );
				}
			}

			b2FreeArenaItem( &world->arena, clusterData );
			stepContext->clusterData = NULL;
		}

		world->profile.transforms = b2GetMilliseconds( transformTicks );
		b2TracyCZoneEnd( update_transforms );
	}

	// Report joint events
	{
		b2TracyCZoneNC( joint_events, "Joint Events", b2_colorPeru, true );
		uint64_t jointEventTicks = b2GetTicks();

		// Gather bits for all joints that have force/torque events
		b2BitSet* jointStateBitSet = &world->taskContexts.data[0].jointStateBitSet;
		for ( int i = 1; i < world->workerCount; ++i )
		{
			b2InPlaceUnion( jointStateBitSet, &world->taskContexts.data[i].jointStateBitSet );
		}

		{
			uint32_t wordCount = jointStateBitSet->blockCount;
			uint64_t* bits = jointStateBitSet->bits;

			b2Joint* jointArray = world->joints.data;
			uint16_t worldIndex0 = world->worldId;

			for ( uint32_t k = 0; k < wordCount; ++k )
			{
				uint64_t word = bits[k];
				while ( word != 0 )
				{
					uint32_t ctz = b2CTZ64( word );
					int jointId = (int)( 64 * k + ctz );

					B2_ASSERT( jointId < world->joints.capacity );

					b2Joint* joint = jointArray + jointId;

					B2_ASSERT( joint->setIndex == b2_awakeSet );

					b2JointEvent event = {
						.jointId =
							{
								.index1 = jointId + 1,
								.world0 = worldIndex0,
								.generation = joint->generation,
							},
						.userData = joint->userData,
					};

					b2JointEventArray_Push( &world->jointEvents, event );

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		world->profile.jointEvents = b2GetMilliseconds( jointEventTicks );
		b2TracyCZoneEnd( joint_events );
	}

	// Report hit events
	// todo_erin perhaps optimize this with a bitset
	// todo_erin perhaps do this in parallel with other work below
	{
		b2TracyCZoneNC( hit_events, "Hit Events", b2_colorRosyBrown, true );
		uint64_t hitTicks = b2GetTicks();

		B2_ASSERT( world->contactHitEvents.count == 0 );

		float threshold = world->hitEventThreshold;
		b2SolverSet* awakeSet2 = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
		int contactCount = awakeSet2->contactSims.count;
		b2ContactSim* contactSims = awakeSet2->contactSims.data;
		for ( int j = 0; j < contactCount; ++j )
		{
			b2ContactSim* contactSim = contactSims + j;
			if ( contactSim->manifold.pointCount == 0 )
			{
				continue;
			}

			if ( ( contactSim->simFlags & b2_simEnableHitEvent ) == 0 )
			{
				continue;
			}

			b2ContactHitEvent event = { 0 };
			event.approachSpeed = threshold;

			bool hit = false;
			int pointCount = contactSim->manifold.pointCount;
			for ( int k = 0; k < pointCount; ++k )
			{
				b2ManifoldPoint* mp = contactSim->manifold.points + k;
				float approachSpeed = -mp->normalVelocity;

				// Need to check total impulse because the point may be speculative and not colliding
				if ( approachSpeed > event.approachSpeed && mp->totalNormalImpulse > 0.0f )
				{
					event.approachSpeed = approachSpeed;
					event.point = mp->clipPoint;
					hit = true;
				}
			}

			if ( hit == true )
			{
				event.normal = contactSim->manifold.normal;

				b2Shape* shapeA = b2ShapeArray_Get( &world->shapes, contactSim->shapeIdA );
				b2Shape* shapeB = b2ShapeArray_Get( &world->shapes, contactSim->shapeIdB );

				event.shapeIdA = (b2ShapeId){ shapeA->id + 1, world->worldId, shapeA->generation };
				event.shapeIdB = (b2ShapeId){ shapeB->id + 1, world->worldId, shapeB->generation };

				b2Contact* contact = b2ContactArray_Get( &world->contacts, contactSim->contactId );

				event.contactId = (b2ContactId){
					.index1 = contact->contactId + 1,
					.world0 = world->worldId,
					.padding = 0,
					.generation = contact->generation,
				};

				b2ContactHitEventArray_Push( &world->contactHitEvents, event );
			}
		}

		world->profile.hitEvents = b2GetMilliseconds( hitTicks );
		b2TracyCZoneEnd( hit_events );
	}

	{
		b2TracyCZoneNC( refit_bvh, "Refit BVH", b2_colorFireBrick, true );
		uint64_t refitTicks = b2GetTicks();

		// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
		// broad-phase.
		if ( world->userTreeTask != NULL )
		{
			world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
			world->userTreeTask = NULL;
			world->activeTaskCount -= 1;
		}

		b2ValidateNoEnlarged( &world->broadPhase );

		// Gather bits for all sim bodies that have enlarged AABBs
		b2BitSet* enlargedBodyBitSet = &world->taskContexts.data[0].enlargedSimBitSet;
		for ( int i = 1; i < world->workerCount; ++i )
		{
			b2InPlaceUnion( enlargedBodyBitSet, &world->taskContexts.data[i].enlargedSimBitSet );
		}

		// Enlarge broad-phase proxies and build move array
		// Apply shape AABB changes to broad-phase. This also create the move array which must be
		// in deterministic order. I'm tracking sim bodies because the number of shape ids can be huge.
		// This has to happen before bullets are processed.
		// Also collect moved body ids for lazy cluster reassignment next step.
		{
			b2BroadPhase* broadPhase = &world->broadPhase;
			uint32_t wordCount = enlargedBodyBitSet->blockCount;
			uint64_t* bits = enlargedBodyBitSet->bits;

			// Fast array access is important here
			b2Body* bodyArray = world->bodies.data;
			b2BodyState* states = stepContext->states;
			b2Shape* shapeArray = world->shapes.data;

			for ( uint32_t k = 0; k < wordCount; ++k )
			{
				uint64_t word = bits[k];
				while ( word != 0 )
				{
					uint32_t ctz = b2CTZ64( word );
					uint32_t stateIndex = 64 * k + ctz;

					b2BodyState* state = states + stateIndex;
					b2Body* body = bodyArray + state->bodyId;
					int shapeId = body->headShapeId;
					if ( ( body->flags & ( b2_isBullet | b2_isFast ) ) == ( b2_isBullet | b2_isFast ) )
					{
						// Fast bullet bodies don't have their final AABB yet
						while ( shapeId != B2_NULL_INDEX )
						{
							b2Shape* shape = shapeArray + shapeId;

							// Shape is fast. It's aabb will be enlarged in continuous collision.
							// Update the move array here for determinism because bullets are processed
							// below in non-deterministic order.
							b2BufferMove( broadPhase, shape->proxyKey );

							shapeId = shape->nextShapeId;
						}
					}
					else
					{
						while ( shapeId != B2_NULL_INDEX )
						{
							b2Shape* shape = shapeArray + shapeId;

							// The AABB may not have been enlarged, despite the body being flagged as enlarged.
							// For example, a body with multiple shapes may have not have all shapes enlarged.
							// A fast body may have been flagged as enlarged despite having no shapes enlarged.
							if ( shape->enlargedAABB )
							{
								b2BroadPhase_EnlargeProxy( broadPhase, shape->proxyKey, shape->fatAABB );
								shape->enlargedAABB = false;
							}

							shapeId = shape->nextShapeId;
						}
					}

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		b2ValidateBroadphase( &world->broadPhase );

		world->profile.refit = b2GetMilliseconds( refitTicks );
		b2TracyCZoneEnd( refit_bvh );
	}

	int bulletBodyCount = b2AtomicLoadInt( &stepContext->bulletBodyCount );
	if ( bulletBodyCount > 0 )
	{
		b2TracyCZoneNC( bullets, "Bullets", b2_colorLightYellow, true );
		uint64_t bulletTicks = b2GetTicks();

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		int minRange = 8;
		void* userBulletBodyTask =
			world->enqueueTaskFcn( &b2BulletBodyTask, bulletBodyCount, minRange, stepContext, world->userTaskContext );
		world->taskCount += 1;
		if ( userBulletBodyTask != NULL )
		{
			world->finishTaskFcn( userBulletBodyTask, world->userTaskContext );
		}

		// Serially enlarge broad-phase proxies for bullet shapes
		b2BroadPhase* broadPhase = &world->broadPhase;
		b2DynamicTree* dynamicTree = broadPhase->trees + b2_dynamicBody;

		// Fast array access is important here
		b2Shape* shapeArray = world->shapes.data;

		// Serially enlarge broad-phase proxies for bullet shapes
		b2Body** bulletBodies = stepContext->bulletBodies;

		// This loop has non-deterministic order but it shouldn't affect the result
		for ( int i = 0; i < bulletBodyCount; ++i )
		{
			b2Body* bulletBody = bulletBodies[i];
			if ( ( bulletBody->flags & b2_enlargeBounds ) == 0 )
			{
				continue;
			}

			// Clear flag
			bulletBody->flags &= ~b2_enlargeBounds;

			int shapeId = bulletBody->headShapeId;
			while ( shapeId != B2_NULL_INDEX )
			{
				b2Shape* shape = shapeArray + shapeId;
				if ( shape->enlargedAABB == false )
				{
					shapeId = shape->nextShapeId;
					continue;
				}

				// Clear flag
				shape->enlargedAABB = false;

				int proxyKey = shape->proxyKey;
				int proxyId = B2_PROXY_ID( proxyKey );
				B2_ASSERT( B2_PROXY_TYPE( proxyKey ) == b2_dynamicBody );

				// all fast bullet shapes should already be in the move buffer
				B2_ASSERT( b2ContainsKey( &broadPhase->moveSet, proxyKey + 1 ) );

				b2DynamicTree_EnlargeProxy( dynamicTree, proxyId, shape->fatAABB );

				shapeId = shape->nextShapeId;
			}
		}

		world->profile.bullets = b2GetMilliseconds( bulletTicks );
		b2TracyCZoneEnd( bullets );
	}

	b2FreeArenaItem( &world->arena, stepContext->clusterBodies );
	stepContext->clusterBodies = NULL;

	b2FreeArenaItem( &world->arena, stepContext->states );
	stepContext->states = NULL;

	// Need to free this even if no bullets got processed.
	b2FreeArenaItem( &world->arena, stepContext->bulletBodies );
	stepContext->bulletBodies = NULL;
	b2AtomicStoreInt( &stepContext->bulletBodyCount, 0 );

	// Report sensor hits. This may include bullets sensor hits.
	{
		b2TracyCZoneNC( sensor_hits, "Sensor Hits", b2_colorPowderBlue, true );
		uint64_t sensorHitTicks = b2GetTicks();

		int workerCount = world->workerCount;
		B2_ASSERT( workerCount == world->taskContexts.count );

		for ( int i = 0; i < workerCount; ++i )
		{
			b2TaskContext* taskContext = world->taskContexts.data + i;
			int hitCount = taskContext->sensorHits.count;
			b2SensorHit* hits = taskContext->sensorHits.data;

			for ( int j = 0; j < hitCount; ++j )
			{
				b2SensorHit hit = hits[j];
				b2Shape* sensorShape = b2ShapeArray_Get( &world->shapes, hit.sensorId );
				b2Shape* visitor = b2ShapeArray_Get( &world->shapes, hit.visitorId );

				b2Sensor* sensor = b2SensorArray_Get( &world->sensors, sensorShape->sensorIndex );
				b2Visitor shapeRef = {
					.shapeId = hit.visitorId,
					.generation = visitor->generation,
				};
				b2VisitorArray_Push( &sensor->hits, shapeRef );
			}
		}

		world->profile.sensorHits = b2GetMilliseconds( sensorHitTicks );
		b2TracyCZoneEnd( sensor_hits );
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if ( world->enableSleep == true )
	{
		b2TracyCZoneNC( sleep_islands, "Island Sleep", b2_colorLightSlateGray, true );
		uint64_t sleepTicks = b2GetTicks();

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		B2_ASSERT( world->splitIslandId == B2_NULL_INDEX );
		float splitSleepTimer = 0.0f;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			b2TaskContext* taskContext = world->taskContexts.data + i;
			if ( taskContext->splitIslandId != B2_NULL_INDEX && taskContext->splitSleepTime >= splitSleepTimer )
			{
				B2_ASSERT( taskContext->splitSleepTime > 0.0f );

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if ( taskContext->splitSleepTime == splitSleepTimer && taskContext->splitIslandId < world->splitIslandId )
				{
					continue;
				}

				world->splitIslandId = taskContext->splitIslandId;
				splitSleepTimer = taskContext->splitSleepTime;
			}
		}

		b2BitSet* awakeIslandBitSet = &world->taskContexts.data[0].awakeIslandBitSet;
		for ( int i = 1; i < world->workerCount; ++i )
		{
			b2InPlaceUnion( awakeIslandBitSet, &world->taskContexts.data[i].awakeIslandBitSet );
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		b2IslandSim* islands = awakeSet->islandSims.data;
		int count = awakeSet->islandSims.count;
		for ( int islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1 )
		{
			if ( b2GetBit( awakeIslandBitSet, islandIndex ) == true )
			{
				// this island is still awake
				continue;
			}

			b2IslandSim* island = islands + islandIndex;
			int islandId = island->islandId;

			b2TrySleepIsland( world, islandId );
		}

		b2ValidateSolverSets( world );

		world->profile.sleepIslands = b2GetMilliseconds( sleepTicks );
		b2TracyCZoneEnd( sleep_islands );
	}
}
