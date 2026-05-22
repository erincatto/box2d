// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"

#include "arena_allocator.h"
#include "atomic.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "parallel_for.h"
#include "physics_world.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// these are useful for solver testing
#define ITERATIONS 1
#define RELAX_ITERATIONS 1

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

// Integrate velocities and apply damping
static void b2IntegrateVelocitiesTask( b2SolverBlock block, b2StepContext* context )
{
	b2TracyCZoneNC( integrate_velocity, "IntVel", b2_colorDeepPink, true );

	b2BodyState* states = context->states;
	b2BodySim* sims = context->sims;

	B2_VALIDATE( block.startIndex + block.count <= context->world->solverSets.data[b2_awakeSet].bodyStates.count );

	b2Vec2 gravity = context->world->gravity;
	float h = context->h;

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b2BodySim* sim = sims + i;
		b2BodyState* state = states + i;

		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

		// Apply forces, torque, gravity, and damping
		// Apply damping.
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		float linearDamping = 1.0f / ( 1.0f + h * sim->linearDamping );
		float angularDamping = 1.0f / ( 1.0f + h * sim->angularDamping );

		// Gravity scale will be zero for kinematic bodies
		float gravityScale = sim->invMass > 0.0f ? sim->gravityScale : 0.0f;

		// lvd = h * im * f + h * g
		b2Vec2 linearVelocityDelta = b2Add( b2MulSV( h * sim->invMass, sim->force ), b2MulSV( h * gravityScale, gravity ) );
		float angularVelocityDelta = h * sim->invInertia * sim->torque;

		v = b2MulAdd( linearVelocityDelta, linearDamping, v );
		w = angularVelocityDelta + angularDamping * w;

		state->linearVelocity = v;
		state->angularVelocity = w;
	}

	b2TracyCZoneEnd( integrate_velocity );
}

static void b2IntegratePositionsTask( b2SolverBlock block, b2StepContext* context )
{
	b2TracyCZoneNC( integrate_positions, "IntPos", b2_colorDarkSeaGreen, true );

	B2_VALIDATE( block.startIndex + block.count <= context->world->solverSets.data[b2_awakeSet].bodyStates.count );

	b2BodyState* states = context->states;
	float h = context->h;
	float maxLinearSpeed = context->maxLinearVelocity;
	float maxAngularSpeed = B2_MAX_ROTATION * context->inv_dt;
	float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b2BodyState* state = states + i;

		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

		// Motion locks - these can be viewed as a constraint that comes last
		v.x = ( state->flags & b2_lockLinearX ) ? 0.0f : v.x;
		v.y = ( state->flags & b2_lockLinearY ) ? 0.0f : v.y;
		w = ( state->flags & b2_lockAngularZ ) ? 0.0f : w;

		// Clamp to max linear speed
		if ( b2Dot( v, v ) > maxLinearSpeedSquared )
		{
			float ratio = maxLinearSpeed / b2Length( v );
			v = b2MulSV( ratio, v );
			state->flags |= b2_isSpeedCapped;
		}

		// Clamp to max angular speed
		if ( w * w > maxAngularSpeedSquared && ( state->flags & b2_allowFastRotation ) == 0 )
		{
			float ratio = maxAngularSpeed / b2AbsFloat( w );
			w *= ratio;
			state->flags |= b2_isSpeedCapped;
		}

		state->linearVelocity = v;
		state->angularVelocity = w;
		state->deltaPosition = b2MulAdd( state->deltaPosition, h, state->linearVelocity );
		state->deltaRotation = b2IntegrateRotation( state->deltaRotation, h * state->angularVelocity );
	}

	b2TracyCZoneEnd( integrate_positions );
}

#define B2_MAX_CONTINUOUS_SENSOR_HITS 8

struct b2ContinuousContext
{
	b2World* world;
	b2BodySim* fastBodySim;
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
	b2BodySim* fastBodySim = continuousContext->fastBodySim;

	B2_ASSERT( fastShape->sensorIndex == B2_NULL_INDEX );

	// Skip same shape
	if ( shapeId == fastShape->id )
	{
		return true;
	}

	b2World* world = continuousContext->world;
	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	// Skip same body
	if ( shape->bodyId == fastShape->bodyId )
	{
		return true;
	}

	// Skip sensors unless the shapes want sensor events
	bool isSensor = shape->sensorIndex != B2_NULL_INDEX;
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

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );

	b2BodySim* bodySim = b2GetBodySim( world, body );
	B2_ASSERT( body->type == b2_staticBody || ( fastBodySim->flags & b2_isBullet ) );

	// Skip bullets
	if ( bodySim->flags & b2_isBullet )
	{
		return true;
	}

	// Skip filtered bodies
	b2Body* fastBody = b2Array_Get( world->bodies, fastBodySim->bodyId );
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
		b2Transform transform = bodySim->transform;
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

			float coreDistance = B2_CORE_FRACTION * fastBodySim->minExtent;

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
	input.sweepA = b2MakeSweep( bodySim );
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
			fastBodySim->flags |= b2_hadTimeOfImpact;
			continuousContext->fraction = hitFraction;
		}
	}

	// Continue query
	return true;
}

// Continuous collision of dynamic versus static
static void b2SolveContinuous( b2World* world, int bodySimIndex, b2TaskContext* taskContext )
{
	b2TracyCZoneNC( ccd, "CCD", b2_colorDarkGoldenRod, true );

	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodySim* fastBodySim = b2Array_Get( awakeSet->bodySims, bodySimIndex );
	B2_ASSERT( fastBodySim->flags & b2_isFast );

	b2Sweep sweep = b2MakeSweep( fastBodySim );

	b2Transform xf1;
	xf1.q = sweep.q1;
	xf1.p = b2Sub( sweep.c1, b2RotateVector( sweep.q1, sweep.localCenter ) );

	b2Transform xf2;
	xf2.q = sweep.q2;
	xf2.p = b2Sub( sweep.c2, b2RotateVector( sweep.q2, sweep.localCenter ) );

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;
	b2DynamicTree* kinematicTree = world->broadPhase.trees + b2_kinematicBody;
	b2DynamicTree* dynamicTree = world->broadPhase.trees + b2_dynamicBody;
	b2Body* fastBody = b2Array_Get( world->bodies, fastBodySim->bodyId );

	struct b2ContinuousContext context = { 0 };
	context.world = world;
	context.sweep = sweep;
	context.fastBodySim = fastBodySim;
	context.fraction = 1.0f;

	bool isBullet = ( fastBodySim->flags & b2_isBullet ) != 0;

	int shapeId = fastBody->headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* fastShape = b2Array_Get( world->shapes, shapeId );
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
		fastBodySim->transform = transform;
		fastBodySim->center = c;
		fastBodySim->rotation0 = q;
		fastBodySim->center0 = c;

		// Update body move event
		b2BodyMoveEvent* event = b2Array_Get( world->bodyMoveEvents, bodySimIndex );
		event->transform = transform;

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = fastBody->headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2Array_Get( world->shapes, shapeId );

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
				fastBodySim->flags |= b2_enlargeBounds;
			}

			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		// No time of impact event

		// Advance body
		fastBodySim->rotation0 = fastBodySim->transform.q;
		fastBodySim->center0 = fastBodySim->center;

		// Prepare AABBs for broad-phase
		shapeId = fastBody->headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2Array_Get( world->shapes, shapeId );

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
				fastBodySim->flags |= b2_enlargeBounds;
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
			b2Array_Push( taskContext->sensorHits, context.sensorHits[i] );
		}
	}

	b2TracyCZoneEnd( ccd );
}

// Implements b2ParallelForCallback
static void b2FinalizeBodiesTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2TracyCZoneNC( finalize_transforms, "Finalize", b2_colorMediumSeaGreen, true );

	b2StepContext* stepContext = context;
	b2World* world = stepContext->world;
	b2Body* bodies = world->bodies.data;
	b2BodyState* states = stepContext->states;
	b2BodySim* sims = stepContext->sims;

	B2_ASSERT( endIndex <= world->bodyMoveEvents.count );

	bool enableSleep = world->enableSleep;
	bool enableContinuous = world->enableContinuous;
	float timeStep = stepContext->dt;
	float invTimeStep = stepContext->inv_dt;
	uint16_t worldId = world->worldId;

	// The body move event array should already have the correct size
	b2BodyMoveEvent* moveEvents = world->bodyMoveEvents.data;

	b2TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b2BitSet* enlargedSimBitSet = &taskContext->enlargedSimBitSet;
	b2BitSet* awakeIslandBitSet = &taskContext->awakeIslandBitSet;

	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;

	for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
	{
		b2BodyState* state = states + simIndex;
		b2BodySim* sim = sims + simIndex;

		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

		if ( b2IsValidVec2( v ) == false || b2IsValidFloat( w ) == false )
		{
			b2Body* debugBody = bodies + sim->bodyId;
			b2Log( "unstable: %s\n", debugBody->name );
		}

		B2_ASSERT( b2IsValidVec2( v ) );
		B2_ASSERT( b2IsValidFloat( w ) );

		sim->center = b2Add( sim->center, state->deltaPosition );
		sim->transform.q = b2NormalizeRot( b2MulRot( state->deltaRotation, sim->transform.q ) );

		// Use the velocity of the farthest point on the body to account for rotation.
		float maxVelocity = b2Length( v ) + b2AbsFloat( w ) * sim->maxExtent;

		// Sleep needs to observe position correction as well as true velocity.
		float maxDeltaPosition = b2Length( state->deltaPosition ) + b2AbsFloat( state->deltaRotation.s ) * sim->maxExtent;

		// Position correction is not as important for sleep as true velocity.
		float positionSleepFactor = 0.5f;
		float sleepVelocity = b2MaxFloat( maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition );

		// reset state deltas
		state->deltaPosition = b2Vec2_zero;
		state->deltaRotation = b2Rot_identity;

		sim->transform.p = b2Sub( sim->center, b2RotateVector( sim->transform.q, sim->localCenter ) );

		// cache miss here, however I need the shape list below
		b2Body* body = bodies + sim->bodyId;
		body->bodyMoveIndex = simIndex;
		moveEvents[simIndex].transform = sim->transform;
		moveEvents[simIndex].bodyId = (b2BodyId){ sim->bodyId + 1, worldId, body->generation };
		moveEvents[simIndex].userData = body->userData;
		moveEvents[simIndex].fellAsleep = false;

		// reset applied force and torque
		sim->force = b2Vec2_zero;
		sim->torque = 0.0f;

		// If you hit this then it means you deferred mass computation but never called b2Body_ApplyMassFromShapes
		B2_ASSERT( ( body->flags & b2_dirtyMass ) == 0 );

		body->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );
		body->flags |= ( sim->flags & ( b2_isSpeedCapped | b2_hadTimeOfImpact ) );
		body->flags |= ( state->flags & ( b2_isSpeedCapped | b2_hadTimeOfImpact ) );
		sim->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );
		state->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );

		if ( enableSleep == false || body->enableSleep == false || sleepVelocity > body->sleepThreshold )
		{
			// Body is not sleepy
			body->sleepTime = 0.0f;

			const float safetyFactor = 0.5f;
			float maxMotion = b2MaxFloat( maxDeltaPosition, maxVelocity * timeStep );
			if ( body->type == b2_dynamicBody && enableContinuous && maxMotion > safetyFactor * sim->minExtent )
			{
				// This flag is only retained for debug draw
				sim->flags |= b2_isFast;

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if ( sim->flags & b2_isBullet )
				{
					int bulletIndex = b2AtomicFetchAddInt( &stepContext->bulletBodyCount, 1 );
					stepContext->bulletBodies[bulletIndex] = simIndex;
				}
				else
				{
					b2SolveContinuous( world, simIndex, taskContext );
				}
			}
			else
			{
				// Body is safe to advance
				sim->center0 = sim->center;
				sim->rotation0 = sim->transform.q;
			}
		}
		else
		{
			// Body is safe to advance and is falling asleep
			sim->center0 = sim->center;
			sim->rotation0 = sim->transform.q;
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
		b2Transform transform = sim->transform;
		bool isFast = ( sim->flags & b2_isFast ) != 0;
		int shapeId = body->headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2Array_Get( world->shapes, shapeId );

			if ( isFast )
			{
				// For fast non-bullet bodies the AABB has already been updated in b2SolveContinuous
				// For fast bullet bodies the AABB will be updated at a later stage

				// Add to enlarged shapes regardless of AABB changes.
				// Bit-set to keep the move array sorted
				b2SetBit( enlargedSimBitSet, simIndex );
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
					b2SetBit( enlargedSimBitSet, simIndex );
				}
			}

			shapeId = shape->nextShapeId;
		}
	}

	b2TracyCZoneEnd( finalize_transforms );
}

typedef struct b2BlockDim
{
	// number of items per block (except last block)
	int size;

	// total number of blocks
	int count;
} b2BlockDim;

// A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
// Each block of work has a minimum number of elements (block size). This in turn may limit the
// number of blocks. If there are many elements then the block size is increased so there are
// still at most M blocks of work per worker. M is a tunable number that has two goals:
// 1. keep M small to reduce overhead
// 2. keep M large enough for other workers to be able to steal work
// The block size is a power of two to make math efficient.
static inline b2BlockDim b2ComputeBlockCount( int itemCount, int minSize, int maxBlockCount )
{
	b2BlockDim dim = { 0 };
	if ( itemCount == 0 )
	{
		return dim;
	}

	if ( itemCount <= minSize * maxBlockCount )
	{
		dim.size = minSize;
	}
	else
	{
		dim.size = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
	}

	dim.count = ( itemCount + dim.size - 1 ) / dim.size;

	B2_ASSERT( dim.count >= 1 );
	B2_ASSERT( dim.size * dim.count >= itemCount );

	return dim;
}

// Initialize solver blocks for a contiguous range of items. Computes block size internally
// from the same parameters used by b2ComputeBlockCount. The atomic claim counter is zeroed
// so workers can CAS (0, 1) on the first stage that owns these blocks.
static void b2InitBlocks( b2SyncBlock* blocks, b2BlockDim dim, int itemCount, uint8_t blockType, uint8_t colorIndex )
{
	if ( dim.count == 0 )
	{
		return;
	}

	B2_ASSERT( itemCount >= dim.count );

	// Compute the number of elements per block
	int blockSize = dim.size;

	// Simulation too big
	B2_ASSERT( blockSize <= UINT16_MAX );

	for ( int i = 0; i < dim.count; ++i )
	{
		blocks[i].block.startIndex = i * blockSize;
		blocks[i].block.count = (uint16_t)blockSize;
		blocks[i].block.blockType = blockType;
		blocks[i].block.colorIndex = colorIndex;
		b2AtomicStoreInt( &blocks[i].syncIndex, 0 );
	}

	// The last block may not be full
	blocks[dim.count - 1].block.count = (uint16_t)( itemCount - ( dim.count - 1 ) * blockSize );

	B2_VALIDATE( blocks[dim.count - 1].block.count <= blockSize );
	B2_VALIDATE( ( dim.count - 1 ) * dim.size + blocks[dim.count - 1].block.count == itemCount );
}

static inline b2SolverStage* b2InitStage( b2SolverStage* stage, b2SolverStageType type, b2SyncBlock* blocks, int blockCount,
										  uint8_t colorIndex )
{
	stage->type = type;
	stage->blocks = blocks;
	stage->blockCount = blockCount;
	stage->colorIndex = colorIndex;
	b2AtomicStoreInt( &stage->completionCount, 0 );
	return stage + 1;
}

// Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
// All iterations of a given color share the same b2SyncBlock array so the per-block syncIndex
// grows monotonically across stages within that color.
static b2SolverStage* b2InitColorStages( b2SolverStage* stage, b2SolverStageType type, int iterations, int activeColorCount,
										 b2SyncBlock** colorBlocks, int* colorBlockCounts, int* activeColorIndices )
{
	for ( int j = 0; j < iterations; ++j )
	{
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage = b2InitStage( stage, type, colorBlocks[i], colorBlockCounts[i], (uint8_t)activeColorIndices[i] );
		}
	}
	return stage;
}

static void b2ExecuteBlock( b2SolverStage* stage, b2StepContext* context, b2SolverBlock block, int workerIndex )
{
	b2SolverStageType stageType = stage->type;
	b2SolverBlockType blockType = block.blockType;

	switch ( stageType )
	{
		case b2_stagePrepareJoints:
			b2PrepareJointsTask( block, context );
			break;

		case b2_stagePrepareContacts:
			b2PrepareContactsTask( block, context );
			break;

		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask( block, context );
			break;

		case b2_stageWarmStart:
			if ( blockType == b2_graphContactBlock )
			{
				b2WarmStartContactsTask( block, context );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				b2WarmStartJointsTask( block, context );
			}
			break;

		case b2_stageSolve:
			if ( blockType == b2_graphContactBlock )
			{
				bool useBias = true;
				b2SolveContactsTask( block, context, useBias );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				bool useBias = true;
				b2SolveJointsTask( block, context, useBias, workerIndex );
			}
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask( block, context );
			break;

		case b2_stageRelax:
			if ( blockType == b2_graphContactBlock )
			{
				bool useBias = false;
				b2SolveContactsTask( block, context, useBias );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				bool useBias = false;
				b2SolveJointsTask( block, context, useBias, workerIndex );
			}
			break;

		case b2_stageRestitution:
			if ( blockType == b2_graphContactBlock )
			{
				b2ApplyRestitutionTask( block, context );
			}
			break;

		case b2_stageStoreImpulses:
			b2StoreImpulsesTask( block, context, workerIndex );
			break;
	}
}

// This staggers the worker start indices so they avoid touching the same solver blocks
static inline int GetWorkerStartIndex( int workerIndex, int blockCount, int workerCount )
{
	if ( blockCount <= workerCount )
	{
		return workerIndex < blockCount ? workerIndex : B2_NULL_INDEX;
	}

	int blocksPerWorker = blockCount / workerCount;
	int remainder = blockCount - blocksPerWorker * workerCount;
	return blocksPerWorker * workerIndex + b2MinInt( remainder, workerIndex );
}

// Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.
static void b2ExecuteStage( b2SolverStage* stage, b2StepContext* context, int previousSyncIndex, int syncIndex, int workerIndex )
{
	int completedCount = 0;
	b2SyncBlock* blocks = stage->blocks;
	int blockCount = stage->blockCount;

	int startIndex = GetWorkerStartIndex( workerIndex, blockCount, context->workerCount );
	if ( startIndex == B2_NULL_INDEX )
	{
		return;
	}

	B2_ASSERT( 0 <= startIndex && startIndex < blockCount );

	int blockIndex = startIndex;
	for ( int i = 0; i < blockCount; ++i )
	{
		if ( b2AtomicCompareExchangeInt( &blocks[blockIndex].syncIndex, previousSyncIndex, syncIndex ) )
		{
			B2_ASSERT( stage->type != b2_stagePrepareContacts || syncIndex < 2 );
			B2_ASSERT( completedCount < blockCount );

			// Pass the descriptor by value -- the wrapping b2SyncBlock holds the atomic
			// syncIndex but we only copy .block, so the struct copy never aliases the CAS target.
			b2ExecuteBlock( stage, context, blocks[blockIndex].block, workerIndex );
			completedCount += 1;
		}

		blockIndex += 1;
		if ( blockIndex >= blockCount )
		{
			blockIndex = 0;
		}
	}

	(void)b2AtomicFetchAddInt( &stage->completionCount, completedCount );
}

// Execute a stage on worker 0 (main thread).
static void b2ExecuteMainStage( b2SolverStage* stage, b2StepContext* context, uint32_t syncBits )
{
	int blockCount = stage->blockCount;
	if ( blockCount == 0 )
	{
		return;
	}

	int workerIndex = 0;

	if ( blockCount == 1 )
	{
		b2ExecuteBlock( stage, context, stage->blocks[0].block, workerIndex );
	}
	else
	{
		b2AtomicStoreU32( &context->atomicSyncBits, syncBits );

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B2_ASSERT( syncIndex > 0 );
		int previousSyncIndex = syncIndex - 1;

		b2ExecuteStage( stage, context, previousSyncIndex, syncIndex, workerIndex );

		// Spin waiting for thieves to finish
		while ( b2AtomicLoadInt( &stage->completionCount ) != blockCount )
		{
			b2Pause();
		}

		b2AtomicStoreInt( &stage->completionCount, 0 );
	}
}

// Parallel solver task
static void b2SolverTask( void* taskContext )
{
	b2WorkerContext* workerContext = taskContext;
	int workerIndex = workerContext->workerIndex;
	b2StepContext* context = workerContext->context;
	int activeColorCount = context->activeColorCount;
	b2SolverStage* stages = context->stages;
	b2Profile* profile = &context->world->profile;

	if ( workerIndex == 0 )
	{
		// The orchestrator slot is a race. The calling thread of b2World_Step also enters here
		// as worker 0, so progress is guaranteed even if the user's task system schedules tasks
		// out of order, has fewer threads than workerCount, or runs the task synchronously
		// inside enqueueTaskFcn. Whoever wins the CAS becomes the orchestrator; the loser
		// returns and lets the spinner-only path handle workers >0.
		if ( b2AtomicCompareExchangeInt( &context->mainClaimed, 0, 1 ) == false )
		{
			return;
		}

		// Main thread synchronizes the workers and does work itself.
		//
		// This single task is able to fully complete all work even if all other workers are
		// blocked, so a fully serial task system still drives the simulation forward.

		// Stages are re-used by loops so that I don't need more stages for large substep counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		Stage sequence
		b2_stagePrepareJoints,
		b2_stagePrepareContacts,
		b2_stageIntegrateVelocities,
		b2_stageWarmStart,
		b2_stageSolve,
		b2_stageIntegratePositions,
		b2_stageRelax,
		b2_stageRestitution,
		b2_stageStoreImpulses
		*/

		uint64_t ticks = b2GetTicks();

		int bodySyncIndex = 1;
		int stageIndex = 0;

		// Prepare joint constraints
		uint32_t jointSyncIndex = 1;
		uint32_t syncBits = ( jointSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stagePrepareJoints );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		jointSyncIndex += 1;

		// Prepare contact constraints
		uint32_t contactSyncIndex = 1;
		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stagePrepareContacts );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		contactSyncIndex += 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		b2PrepareJoints_Overflow( context );
		b2PrepareContacts_Overflow( context );

		profile->prepareConstraints += b2GetMillisecondsAndReset( &ticks );

		int graphSyncIndex = 1;
		int subStepCount = context->subStepCount;
		for ( int subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex )
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int iterationStageIndex = stageIndex;

			// Integrate velocities
			syncBits = ( bodySyncIndex << 16 ) | iterationStageIndex;
			B2_ASSERT( stages[iterationStageIndex].type == b2_stageIntegrateVelocities );
			b2ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			profile->integrateVelocities += b2GetMillisecondsAndReset( &ticks );

			// Warm start constraints
			b2WarmStartJoints_Overflow( context );
			b2WarmStartContacts_Overflow( context );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterationStageIndex;
				B2_ASSERT( stages[iterationStageIndex].type == b2_stageWarmStart );
				b2ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
				iterationStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->warmStart += b2GetMillisecondsAndReset( &ticks );

			// Solve constraints
			bool useBias = true;
			for ( int j = 0; j < ITERATIONS; ++j )
			{
				// Overflow constraints have lower priority. Typically these are dynamic-vs-dynamic.
				b2SolveJoints_Overflow( context, useBias );
				b2SolveContacts_Overflow( context, useBias );

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( graphSyncIndex << 16 ) | iterationStageIndex;
					B2_ASSERT( stages[iterationStageIndex].type == b2_stageSolve );
					b2ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			profile->solveImpulses += b2GetMillisecondsAndReset( &ticks );

			// Integrate positions
			B2_ASSERT( stages[iterationStageIndex].type == b2_stageIntegratePositions );
			syncBits = ( bodySyncIndex << 16 ) | iterationStageIndex;
			b2ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			profile->integratePositions += b2GetMillisecondsAndReset( &ticks );

			// Relax constraints
			useBias = false;
			for ( int j = 0; j < RELAX_ITERATIONS; ++j )
			{
				b2SolveJoints_Overflow( context, useBias );
				b2SolveContacts_Overflow( context, useBias );
				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( graphSyncIndex << 16 ) | iterationStageIndex;
					B2_ASSERT( stages[iterationStageIndex].type == b2_stageRelax );
					b2ExecuteMainStage( stages + iterationStageIndex, context, syncBits );
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			profile->relaxImpulses += b2GetMillisecondsAndReset( &ticks );
		}

		// Advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

		// Restitution
		{
			b2ApplyRestitution_Overflow( context );

			int iterStageIndex = stageIndex;
			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageRestitution );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			// graphSyncIndex += 1;
			stageIndex += activeColorCount;
		}

		profile->applyRestitution += b2GetMillisecondsAndReset( &ticks );

		// Store impulses
		b2StoreImpulses_Overflow( context );

		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stageStoreImpulses );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );

		profile->storeImpulses += b2GetMillisecondsAndReset( &ticks );

		// Signal workers to finish
		b2AtomicStoreU32( &context->atomicSyncBits, UINT_MAX );

		B2_ASSERT( stageIndex + 1 == context->stageCount );
		return;
	}

	// Worker spins and waits for work
	uint32_t lastSyncBits = 0;
	// uint64_t maxSpinTime = 10;
	while ( true )
	{
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		// todo improve this spinner
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
				// Using the cycle counter helps to account for variation in mm_pause timing across different
				// CPUs. However, this is X64 only.
				// uint64_t prev = __rdtsc();
				// do
				//{
				//	_mm_pause();
				//}
				// while ((__rdtsc() - prev) < maxSpinTime);
				// maxSpinTime += 10;

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

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B2_ASSERT( syncIndex > 0 );

		int previousSyncIndex = syncIndex - 1;

		b2SolverStage* stage = stages + stageIndex;
		b2ExecuteStage( stage, context, previousSyncIndex, syncIndex, workerIndex );

		lastSyncBits = syncBits;
	}
}

static void b2BulletBodyTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2TracyCZoneNC( bullet_body_task, "Bullet", b2_colorLightSkyBlue, true );

	b2StepContext* stepContext = context;
	b2TaskContext* taskContext = stepContext->world->taskContexts.data + workerIndex;

	B2_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		int simIndex = stepContext->bulletBodies[i];
		b2SolveContinuous( stepContext->world, simIndex, taskContext );
	}

	b2TracyCZoneEnd( bullet_body_task );
}

// Solve with graph coloring
void b2Solve( b2World* world, b2StepContext* stepContext )
{
	// Only count steps that advance the simulation
	world->stepIndex += 1;

	// Are there any awake bodies? This scenario should not be important for profiling.
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	int awakeBodyCount = awakeSet->bodySims.count;
	if ( awakeBodyCount == 0 )
	{
		b2ValidateNoEnlarged( &world->broadPhase );
		return;
	}

	// Solve constraints using graph coloring
	{
		b2TracyCZoneNC( solver_setup, "Solver Setup", b2_colorDarkOrange, true );
		uint64_t setupTicks = b2GetTicks();

		// Prepare buffers for bullets
		b2AtomicStoreInt( &stepContext->bulletBodyCount, 0 );
		stepContext->bulletBodies = b2StackAlloc( &world->stack, awakeBodyCount * sizeof( int ), "bullet bodies" );

		b2ConstraintGraph* graph = &world->constraintGraph;
		b2GraphColor* colors = graph->colors;

		stepContext->sims = awakeSet->bodySims.data;
		stepContext->states = awakeSet->bodyStates.data;

		// count contacts, joints, and colors
		int activeColorCount = 0;
		for ( int i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i )
		{
			int perColorContactCount = colors[i].contactSims.count;
			int perColorJointCount = colors[i].jointSims.count;
			int occupancyCount = perColorContactCount + perColorJointCount;
			activeColorCount += occupancyCount > 0 ? 1 : 0;
		}

		// prepare for move events
		b2Array_Resize( world->bodyMoveEvents, awakeBodyCount );

		int workerCount = world->workerCount;

		// Target 4 blocks per worker to allow work stealing
		const int maxBlockCount = 4 * workerCount;

		// Body blocks are for parallel iteration over bodies directly (integration, update transforms)
		int minBodiesPerBlock = 32;
		b2BlockDim bodyDim = b2ComputeBlockCount( awakeBodyCount, minBodiesPerBlock, maxBlockCount );

		const int minContactsPerBlock = 4;
		const int minJointsPerBlock = 4;

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of wide contact blocks and joint blocks
		int activeColorIndices[B2_GRAPH_COLOR_COUNT];
		int colorContactCounts[B2_GRAPH_COLOR_COUNT];
		int colorJointCounts[B2_GRAPH_COLOR_COUNT];
		b2BlockDim graphContactDims[B2_GRAPH_COLOR_COUNT];
		b2BlockDim graphJointDims[B2_GRAPH_COLOR_COUNT];
		int graphBlockCount = 0;

		// c is the active color index
		int wideContactCount = 0;
		int jointCount = 0;
		int c = 0;
		for ( int i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i )
		{
			int colorContactCount = colors[i].contactSims.count;
			int colorJointCount = colors[i].jointSims.count;

			if ( colorContactCount + colorJointCount == 0 )
			{
				continue;
			}

			activeColorIndices[c] = i;

			// Ceiling for wide constraint count
			int colorContactCountW = colorContactCount > 0 ? ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1 : 0;
			wideContactCount += colorContactCountW;
			colorContactCounts[c] = colorContactCountW;

			colorJointCounts[c] = colorJointCount;
			jointCount += colorJointCount;

			// Graph solver block dimensions
			graphContactDims[c] = b2ComputeBlockCount( colorContactCountW, minContactsPerBlock, maxBlockCount );
			graphJointDims[c] = b2ComputeBlockCount( colorJointCount, minJointsPerBlock, maxBlockCount );
			graphBlockCount += graphContactDims[c].count + graphJointDims[c].count;

			c += 1;
		}
		activeColorCount = c;

		// Prepare and store run as one flat parallel-for over the entire wide constraint range,
		// partitioned into uniformly sized blocks. Color info is consulted inside the task via
		// a small span array, so blocks do not need to honor color boundaries here.
		b2BlockDim contactPrepareDim = b2ComputeBlockCount( wideContactCount, minContactsPerBlock, maxBlockCount );
		b2BlockDim jointPrepareDim = b2ComputeBlockCount( jointCount, minJointsPerBlock, maxBlockCount );

		int wideContactConstraintByteCount = b2GetWideContactConstraintByteCount();
		struct b2ContactConstraintWide* wideContactConstraints =
			b2StackAlloc( &world->stack, wideContactCount * wideContactConstraintByteCount, "contact constraint" );

		b2GraphColor* overflow = colors + B2_OVERFLOW_INDEX;
		int overflowCount = overflow->contactSims.count;
		b2ContactConstraint* overflowContacts =
			b2StackAlloc( &world->stack, overflowCount * sizeof( b2ContactConstraint ), "overflow contact constraint" );
		overflow->overflowConstraints = overflowContacts;

		// Build the span table for the flat prepare/store parallel-for while I slice the
		// wide constraint buffer across colors. One entry per active color plus a sentinel
		// at wideContactCount.
		b2ContactPrepareSpan contactPrepareSpans[B2_GRAPH_COLOR_COUNT + 1];
		b2JointPrepareSpan jointPrepareSpans[B2_GRAPH_COLOR_COUNT + 1];

		// Distribute transient constraints to each graph color and prepare spans
		{
			int wideBase = 0;
			int jointBase = 0;
			for ( int i = 0; i < activeColorCount; ++i )
			{
				int j = activeColorIndices[i];
				b2GraphColor* color = colors + j;

				int colorContactCount = color->contactSims.count;
				contactPrepareSpans[i].start = wideBase;
				contactPrepareSpans[i].count = colorContactCount;
				contactPrepareSpans[i].contacts = color->contactSims.data;

				if ( colorContactCount == 0 )
				{
					color->wideConstraints = NULL;
					color->wideConstraintCount = 0;
				}
				else
				{
					color->wideConstraints = (struct b2ContactConstraintWide*)( (uint8_t*)wideContactConstraints +
																				wideBase * wideContactConstraintByteCount );

					int colorContactCountW = ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1;
					color->wideConstraintCount = colorContactCountW;

					// Zero remainder lanes in the tail wide slot so prepare workers don't need to
					// initialize them.
					if ( ( colorContactCount & ( B2_SIMD_WIDTH - 1 ) ) != 0 )
					{
						memset( (uint8_t*)color->wideConstraints + ( colorContactCountW - 1 ) * wideContactConstraintByteCount, 0,
								wideContactConstraintByteCount );
					}

					wideBase += colorContactCountW;
				}

				jointPrepareSpans[i].start = jointBase;
				jointPrepareSpans[i].count = color->jointSims.count;
				jointPrepareSpans[i].joints = color->jointSims.data;
				jointBase += color->jointSims.count;
			}

			// Sentinel
			contactPrepareSpans[activeColorCount].start = wideContactCount;
			contactPrepareSpans[activeColorCount].count = 0;
			contactPrepareSpans[activeColorCount].contacts = NULL;
			B2_ASSERT( wideBase == wideContactCount );

			jointPrepareSpans[activeColorCount].start = jointCount;
			jointPrepareSpans[activeColorCount].count = 0;
			jointPrepareSpans[activeColorCount].joints = NULL;
			B2_ASSERT( jointBase == jointCount );
		}

		int stageCount = 0;

		// b2_stagePrepareJoints
		stageCount += 1;
		// b2_stagePrepareContacts
		stageCount += 1;
		// b2_stageIntegrateVelocities
		stageCount += 1;
		// b2_stageWarmStart
		stageCount += activeColorCount;
		// b2_stageSolve
		stageCount += ITERATIONS * activeColorCount;
		// b2_stageIntegratePositions
		stageCount += 1;
		// b2_stageRelax
		stageCount += RELAX_ITERATIONS * activeColorCount;
		// b2_stageRestitution
		stageCount += activeColorCount;
		// b2_stageStoreImpulses
		stageCount += 1;

		b2SolverStage* stages = b2StackAlloc( &world->stack, stageCount * sizeof( b2SolverStage ), "stages" );
		b2SyncBlock* bodyBlocks = b2StackAlloc( &world->stack, bodyDim.count * sizeof( b2SyncBlock ), "body blocks" );
		b2SyncBlock* contactBlocks =
			b2StackAlloc( &world->stack, contactPrepareDim.count * sizeof( b2SyncBlock ), "contact blocks" );
		b2SyncBlock* jointBlocks = b2StackAlloc( &world->stack, jointPrepareDim.count * sizeof( b2SyncBlock ), "joint blocks" );
		b2SyncBlock* graphBlocks = b2StackAlloc( &world->stack, graphBlockCount * sizeof( b2SyncBlock ), "graph blocks" );

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		void* splitIslandTask = NULL;
		if ( world->splitIslandId != B2_NULL_INDEX )
		{
			if ( world->taskCount < B2_MAX_TASKS )
			{
				splitIslandTask = world->enqueueTaskFcn( &b2SplitIslandTask, world, world->userTaskContext );
				world->taskCount += 1;
				world->activeTaskCount += splitIslandTask == NULL ? 0 : 1;
			}
			else
			{
				b2SplitIslandTask( world );
			}
		}

		// Prepare body blocks
		b2InitBlocks( bodyBlocks, bodyDim, awakeBodyCount, b2_bodyBlock, UINT8_MAX );

		// Prepare blocks as a single flat parallel-for over the whole constraint range.
		// The task walks spans to decode flat slot indices back to per-color arrays.
		b2InitBlocks( contactBlocks, contactPrepareDim, wideContactCount, b2_contactBlock, UINT8_MAX );
		b2InitBlocks( jointBlocks, jointPrepareDim, jointCount, b2_jointBlock, UINT8_MAX );

		// Prepare graph work blocks. Each color gets joint blocks followed by contact blocks.
		b2SyncBlock* graphColorBlocks[B2_GRAPH_COLOR_COUNT] = { 0 };
		b2SyncBlock* baseGraphBlock = graphBlocks;
		int graphBlockCounts[B2_GRAPH_COLOR_COUNT] = { 0 };
		for ( int i = 0; i < activeColorCount; ++i )
		{
			graphColorBlocks[i] = baseGraphBlock;

			uint8_t colorIndex = (uint8_t)activeColorIndices[i];
			b2InitBlocks( baseGraphBlock, graphJointDims[i], colorJointCounts[i], b2_graphJointBlock, colorIndex );
			baseGraphBlock += graphJointDims[i].count;

			b2InitBlocks( baseGraphBlock, graphContactDims[i], colorContactCounts[i], b2_graphContactBlock, colorIndex );
			baseGraphBlock += graphContactDims[i].count;

			graphBlockCounts[i] = graphJointDims[i].count + graphContactDims[i].count;
		}

		B2_ASSERT( (ptrdiff_t)( baseGraphBlock - graphBlocks ) == graphBlockCount );

		b2SolverStage* stage = stages;
		stage = b2InitStage( stage, b2_stagePrepareJoints, jointBlocks, jointPrepareDim.count, UINT8_MAX );
		stage = b2InitStage( stage, b2_stagePrepareContacts, contactBlocks, contactPrepareDim.count, UINT8_MAX );
		stage = b2InitStage( stage, b2_stageIntegrateVelocities, bodyBlocks, bodyDim.count, UINT8_MAX );
		stage = b2InitColorStages( stage, b2_stageWarmStart, 1, activeColorCount, graphColorBlocks, graphBlockCounts,
								   activeColorIndices );
		stage = b2InitColorStages( stage, b2_stageSolve, ITERATIONS, activeColorCount, graphColorBlocks, graphBlockCounts,
								   activeColorIndices );
		stage = b2InitStage( stage, b2_stageIntegratePositions, bodyBlocks, bodyDim.count, UINT8_MAX );
		stage = b2InitColorStages( stage, b2_stageRelax, RELAX_ITERATIONS, activeColorCount, graphColorBlocks, graphBlockCounts,
								   activeColorIndices );
		stage = b2InitColorStages( stage, b2_stageRestitution, 1, activeColorCount, graphColorBlocks, graphBlockCounts,
								   activeColorIndices );
		stage = b2InitStage( stage, b2_stageStoreImpulses, contactBlocks, contactPrepareDim.count, UINT8_MAX );

		B2_ASSERT( (int)( stage - stages ) == stageCount );

		B2_ASSERT( workerCount <= B2_MAX_WORKERS );
		b2WorkerContext workerContext[B2_MAX_WORKERS];

		stepContext->graph = graph;
		stepContext->activeColorCount = activeColorCount;
		stepContext->workerCount = workerCount;
		stepContext->stageCount = stageCount;
		stepContext->stages = stages;
		stepContext->wideContactConstraints = wideContactConstraints;
		stepContext->contactPrepareSpans = contactPrepareSpans;
		stepContext->wideContactCount = wideContactCount;
		stepContext->jointPrepareSpans = jointPrepareSpans;
		b2AtomicStoreU32( &stepContext->atomicSyncBits, 0 );
		b2AtomicStoreInt( &stepContext->mainClaimed, 0 );

		world->profile.solverSetup = b2GetMillisecondsAndReset( &setupTicks );
		b2TracyCZoneEnd( solver_setup );

		b2TracyCZoneNC( solve_constraints, "Solve Constraints", b2_colorIndigo, true );
		uint64_t constraintTicks = b2GetTicks();

		int jointIdCapacity = b2GetIdCapacity( &world->jointIdPool );
		int contactIdCapacity = b2GetIdCapacity( &world->contactIdPool );
		for ( int i = 0; i < workerCount; ++i )
		{
			b2TaskContext* taskContext = b2Array_Get( world->taskContexts, i );
			b2SetBitCountAndClear( &taskContext->jointStateBitSet, jointIdCapacity );
			b2SetBitCountAndClear( &taskContext->hitEventBitSet, contactIdCapacity );
			taskContext->hasHitEvents = false;

			workerContext[i].context = stepContext;
			workerContext[i].workerIndex = i;

			if ( world->taskCount < B2_MAX_TASKS )
			{
				workerContext[i].userTask = world->enqueueTaskFcn( &b2SolverTask, workerContext + i, world->userTaskContext );
				world->taskCount += 1;
				world->activeTaskCount += workerContext[i].userTask == NULL ? 0 : 1;
			}
			else
			{
				workerContext[i].userTask = NULL;
				b2SolverTask( workerContext + i );
			}
		}

		// The calling thread of b2World_Step also enters b2SolverTask as worker 0 and races for the
		// orchestrator slot via the CAS inside. This guarantees progress even when the user's task
		// system can't run the queued worker 0 promptly: it might schedule out of order, have fewer
		// threads than workerCount, or invert priority by parking the calling thread in finishTaskFcn.
		// Whoever wins the CAS becomes the orchestrator; the loser returns and lets the spinner-only
		// path handle workers >0.
		b2WorkerContext callerContext = { stepContext, 0, NULL };
		b2SolverTask( &callerContext );

		// Finish constraint solve
		for ( int i = 0; i < workerCount; ++i )
		{
			if ( workerContext[i].userTask != NULL )
			{
				world->finishTaskFcn( workerContext[i].userTask, world->userTaskContext );
				world->activeTaskCount -= 1;
			}
		}

		// Finish island split
		if ( splitIslandTask != NULL )
		{
			world->finishTaskFcn( splitIslandTask, world->userTaskContext );
			world->activeTaskCount -= 1;
		}
		world->splitIslandId = B2_NULL_INDEX;

		world->profile.constraints = b2GetMillisecondsAndReset( &constraintTicks );
		b2TracyCZoneEnd( solve_constraints );

		b2TracyCZoneNC( update_transforms, "Update Transforms", b2_colorMediumSeaGreen, true );
		uint64_t transformTicks = b2GetTicks();

		// Prepare contact, enlarged body, and island bit sets used in body finalization.
		int awakeIslandCount = awakeSet->islandSims.count;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			b2TaskContext* taskContext = world->taskContexts.data + i;
			taskContext->sensorHits.count = 0;
			b2SetBitCountAndClear( &taskContext->enlargedSimBitSet, awakeBodyCount );
			b2SetBitCountAndClear( &taskContext->awakeIslandBitSet, awakeIslandCount );
			taskContext->splitIslandId = B2_NULL_INDEX;
			taskContext->splitSleepTime = 0.0f;
		}

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		b2ParallelFor( world, &b2FinalizeBodiesTask, awakeBodyCount, 64, stepContext );

		b2StackFree( &world->stack, graphBlocks );
		b2StackFree( &world->stack, jointBlocks );
		b2StackFree( &world->stack, contactBlocks );
		b2StackFree( &world->stack, bodyBlocks );
		b2StackFree( &world->stack, stages );
		b2StackFree( &world->stack, overflowContacts );
		b2StackFree( &world->stack, wideContactConstraints );

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

					b2Array_Push( world->jointEvents, event );

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		world->profile.jointEvents = b2GetMilliseconds( jointEventTicks );
		b2TracyCZoneEnd( joint_events );
	}

	// Report hit events
	{
		b2TracyCZoneNC( hit_events, "Hit Events", b2_colorRosyBrown, true );
		uint64_t hitTicks = b2GetTicks();

		B2_ASSERT( world->contactHitEvents.count == 0 );

		// Fast path: if no worker flagged any hit-event candidates during b2StoreImpulsesTask, skip entirely.
		bool anyHitEvents = false;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			if ( world->taskContexts.data[i].hasHitEvents )
			{
				anyHitEvents = true;
				break;
			}
		}

		if ( anyHitEvents )
		{
			// Union per-worker bits into worker 0's bit set.
			b2BitSet* hitEventBitSet = &world->taskContexts.data[0].hitEventBitSet;
			for ( int i = 1; i < world->workerCount; ++i )
			{
				if ( world->taskContexts.data[i].hasHitEvents )
				{
					b2InPlaceUnion( hitEventBitSet, &world->taskContexts.data[i].hitEventBitSet );
				}
			}

			float threshold = world->hitEventThreshold;
			b2GraphColor* colors = world->constraintGraph.colors;
			b2Contact* contactArray = world->contacts.data;
			b2Shape* shapeArray = world->shapes.data;
			uint16_t worldId = world->worldId;

			uint32_t wordCount = hitEventBitSet->blockCount;
			uint64_t* bits = hitEventBitSet->bits;
			for ( uint32_t k = 0; k < wordCount; ++k )
			{
				uint64_t word = bits[k];
				while ( word != 0 )
				{
					uint32_t ctz = b2CTZ64( word );
					int contactId = (int)( 64 * k + ctz );

					b2Contact* contact = contactArray + contactId;
					B2_ASSERT( contact->setIndex == b2_awakeSet && contact->colorIndex != B2_NULL_INDEX );

					b2GraphColor* color = colors + contact->colorIndex;
					b2ContactSim* contactSim = color->contactSims.data + contact->localIndex;

					b2ContactHitEvent event = { 0 };
					event.approachSpeed = threshold;

					bool found = false;
					int pointCount = contactSim->manifold.pointCount;
					for ( int p = 0; p < pointCount; ++p )
					{
						b2ManifoldPoint* mp = contactSim->manifold.points + p;
						float approachSpeed = -mp->normalVelocity;

						// Need to check total impulse because the point may be speculative and not colliding
						if ( approachSpeed > event.approachSpeed && mp->totalNormalImpulse > 0.0f )
						{
							event.approachSpeed = approachSpeed;
							// Using the clip point here is somewhat questionable
							event.point = mp->clipPoint;
							found = true;
						}
					}

					B2_VALIDATE( found );

					if ( found == true )
					{
						event.normal = contactSim->manifold.normal;

						b2Shape* shapeA = shapeArray + contactSim->shapeIdA;
						b2Shape* shapeB = shapeArray + contactSim->shapeIdB;

						event.shapeIdA = (b2ShapeId){ shapeA->id + 1, worldId, shapeA->generation };
						event.shapeIdB = (b2ShapeId){ shapeB->id + 1, worldId, shapeB->generation };

						event.contactId = (b2ContactId){
							.index1 = contact->contactId + 1,
							.world0 = worldId,
							.padding = 0,
							.generation = contact->generation,
						};

						b2Array_Push( world->contactHitEvents, event );
					}

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
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
		{
			b2BroadPhase* broadPhase = &world->broadPhase;
			uint32_t wordCount = enlargedBodyBitSet->blockCount;
			uint64_t* bits = enlargedBodyBitSet->bits;

			// Fast array access is important here
			b2Body* bodyArray = world->bodies.data;
			b2BodySim* bodySimArray = awakeSet->bodySims.data;
			b2Shape* shapeArray = world->shapes.data;

			for ( uint32_t k = 0; k < wordCount; ++k )
			{
				uint64_t word = bits[k];
				while ( word != 0 )
				{
					uint32_t ctz = b2CTZ64( word );
					uint32_t bodySimIndex = 64 * k + ctz;

					b2BodySim* bodySim = bodySimArray + bodySimIndex;

					b2Body* body = bodyArray + bodySim->bodyId;

					int shapeId = body->headShapeId;
					if ( ( bodySim->flags & ( b2_isBullet | b2_isFast ) ) == ( b2_isBullet | b2_isFast ) )
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
		b2ParallelFor( world, &b2BulletBodyTask, bulletBodyCount, minRange, stepContext );

		// Serially enlarge broad-phase proxies for bullet shapes
		b2BroadPhase* broadPhase = &world->broadPhase;
		b2DynamicTree* dynamicTree = broadPhase->trees + b2_dynamicBody;

		// Fast array access is important here
		b2Body* bodyArray = world->bodies.data;
		b2BodySim* bodySimArray = awakeSet->bodySims.data;
		b2Shape* shapeArray = world->shapes.data;

		// Serially enlarge broad-phase proxies for bullet shapes
		int* bulletBodySimIndices = stepContext->bulletBodies;

		// This loop has non-deterministic order but it shouldn't affect the result
		for ( int i = 0; i < bulletBodyCount; ++i )
		{
			b2BodySim* bulletBodySim = bodySimArray + bulletBodySimIndices[i];
			if ( ( bulletBodySim->flags & b2_enlargeBounds ) == 0 )
			{
				continue;
			}

			// Clear flag
			bulletBodySim->flags &= ~b2_enlargeBounds;

			int bodyId = bulletBodySim->bodyId;
			B2_ASSERT( 0 <= bodyId && bodyId < world->bodies.count );
			b2Body* bulletBody = bodyArray + bodyId;

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

	// Need to free this even if no bullets got processed.
	b2StackFree( &world->stack, stepContext->bulletBodies );
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
				b2Shape* sensorShape = b2Array_Get( world->shapes, hit.sensorId );
				b2Shape* visitor = b2Array_Get( world->shapes, hit.visitorId );

				b2Sensor* sensor = b2Array_Get( world->sensors, sensorShape->sensorIndex );
				b2Visitor shapeRef = {
					.shapeId = hit.visitorId,
					.generation = visitor->generation,
				};
				b2Array_Push( sensor->hits, shapeRef );
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
