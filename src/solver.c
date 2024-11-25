// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"

#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "shape.h"
#include "solver_set.h"
#include "stack_allocator.h"
#include "world.h"

#include <limits.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>

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
//#include <immintrin.h>
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
static void b2IntegrateVelocitiesTask( int startIndex, int endIndex, b2StepContext* context )
{
	b2TracyCZoneNC( integrate_velocity, "IntVel", b2_colorDeepPink, true );

	b2BodyState* states = context->states;
	b2BodySim* sims = context->sims;

	b2Vec2 gravity = context->world->gravity;
	float h = context->h;
	float maxLinearSpeed = context->maxLinearVelocity;
	float maxAngularSpeed = b2_maxRotation * context->inv_dt;
	float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
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
		b2Vec2 linearVelocityDelta = b2MulSV( h * sim->invMass, b2MulAdd( sim->force, sim->mass * sim->gravityScale, gravity ) );
		float angularVelocityDelta = h * sim->invInertia * sim->torque;

		v = b2MulAdd( linearVelocityDelta, linearDamping, v );
		w = angularVelocityDelta + angularDamping * w;

		// Clamp to max linear speed
		if ( b2Dot( v, v ) > maxLinearSpeedSquared )
		{
			float ratio = maxLinearSpeed / b2Length( v );
			v = b2MulSV( ratio, v );
			sim->isSpeedCapped = true;
		}

		// Clamp to max angular speed
		if ( w * w > maxAngularSpeedSquared && sim->allowFastRotation == false )
		{
			float ratio = maxAngularSpeed / b2AbsFloat( w );
			w *= ratio;
			sim->isSpeedCapped = true;
		}

		state->linearVelocity = v;
		state->angularVelocity = w;
	}

	b2TracyCZoneEnd( integrate_velocity );
}

static void b2PrepareJointsTask( int startIndex, int endIndex, b2StepContext* context )
{
	b2TracyCZoneNC( prepare_joints, "PrepJoints", b2_colorOldLace, true );

	b2JointSim** joints = context->joints;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2JointSim* joint = joints[i];
		b2PrepareJoint( joint, context );
	}

	b2TracyCZoneEnd( prepare_joints );
}

static void b2WarmStartJointsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex )
{
	b2TracyCZoneNC( warm_joints, "WarmJoints", b2_colorGold, true );

	b2GraphColor* color = context->graph->colors + colorIndex;
	b2JointSim* joints = color->jointSims.data;
	B2_ASSERT( 0 <= startIndex && startIndex < color->jointSims.count );
	B2_ASSERT( startIndex <= endIndex && endIndex <= color->jointSims.count );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2JointSim* joint = joints + i;
		b2WarmStartJoint( joint, context );
	}

	b2TracyCZoneEnd( warm_joints );
}

static void b2SolveJointsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias )
{
	b2TracyCZoneNC( solve_joints, "SolveJoints", b2_colorLemonChiffon, true );

	b2GraphColor* color = context->graph->colors + colorIndex;
	b2JointSim* joints = color->jointSims.data;
	B2_ASSERT( 0 <= startIndex && startIndex < color->jointSims.count );
	B2_ASSERT( startIndex <= endIndex && endIndex <= color->jointSims.count );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2JointSim* joint = joints + i;
		b2SolveJoint( joint, context, useBias );
	}

	b2TracyCZoneEnd( solve_joints );
}

static void b2IntegratePositionsTask( int startIndex, int endIndex, b2StepContext* context )
{
	b2TracyCZoneNC( integrate_positions, "IntPos", b2_colorDarkSeaGreen, true );

	b2BodyState* states = context->states;
	float h = context->h;

	B2_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2BodyState* state = states + i;
		state->deltaRotation = b2IntegrateRotation( state->deltaRotation, h * state->angularVelocity );
		state->deltaPosition = b2MulAdd( state->deltaPosition, h, state->linearVelocity );
	}

	b2TracyCZoneEnd( integrate_positions );
}

struct b2ContinuousContext
{
	b2World* world;
	b2BodySim* fastBodySim;
	b2Shape* fastShape;
	b2Vec2 centroid1, centroid2;
	b2Sweep sweep;
	float fraction;
};

// This is called from b2DynamicTree_Query for continuous collision
static bool b2ContinuousQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	struct b2ContinuousContext* continuousContext = context;
	b2Shape* fastShape = continuousContext->fastShape;
	b2BodySim* fastBodySim = continuousContext->fastBodySim;

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

	// Skip sensors
	if ( shape->isSensor == true )
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

	b2BodySim* bodySim = b2GetBodySim( world, body );
	B2_ASSERT( body->type == b2_staticBody || fastBodySim->isBullet );

	// Skip bullets
	if ( bodySim->isBullet )
	{
		return true;
	}

	// Skip filtered bodies
	b2Body* fastBody = b2BodyArray_Get( &world->bodies, fastBodySim->bodyId );
	canCollide = b2ShouldBodiesCollide( world, fastBody, body );
	if ( canCollide == false )
	{
		return true;
	}

	// Custom user filtering
	b2CustomFilterFcn* customFilterFcn = world->customFilterFcn;
	if ( customFilterFcn != NULL )
	{
		b2ShapeId idA = { shape->id + 1, world->worldId, shape->revision };
		b2ShapeId idB = { fastShape->id + 1, world->worldId, fastShape->revision };
		canCollide = customFilterFcn( idA, idB, world->customFilterContext );
		if ( canCollide == false )
		{
			return true;
		}
	}

	// Prevent pausing on chain segment junctions
	if ( shape->type == b2_chainSegmentShape )
	{
		b2Transform transform = bodySim->transform;
		b2Vec2 p1 = b2TransformPoint( transform, shape->chainSegment.segment.point1 );
		b2Vec2 p2 = b2TransformPoint( transform, shape->chainSegment.segment.point2 );
		b2Vec2 e = b2Sub( p2, p1 );
		b2Vec2 c1 = continuousContext->centroid1;
		b2Vec2 c2 = continuousContext->centroid2;
		float offset1 = b2Cross( b2Sub( c1, p1 ), e );
		float offset2 = b2Cross( b2Sub( c2, p1 ), e );

		if ( offset1 < 0.0f || offset2 > 0.0f )
		{
			// Started behind or finished in front
			return true;
		}
	}

	b2TOIInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeShapeDistanceProxy( fastShape );
	input.sweepA = b2MakeSweep( bodySim );
	input.sweepB = continuousContext->sweep;
	input.tMax = continuousContext->fraction;

	float hitFraction = continuousContext->fraction;

	bool didHit = false;
	b2TOIOutput output = b2TimeOfImpact( &input );
	if ( 0.0f < output.t && output.t < continuousContext->fraction )
	{
		hitFraction = output.t;
		didHit = true;
	}
	else if ( 0.0f == output.t )
	{
		// fallback to TOI of a small circle around the fast shape centroid
		b2Vec2 centroid = b2GetShapeCentroid( fastShape );
		input.proxyB = b2MakeProxy( &centroid, 1, b2_speculativeDistance );
		output = b2TimeOfImpact( &input );
		if ( 0.0f < output.t && output.t < continuousContext->fraction )
		{
			hitFraction = output.t;
			didHit = true;
		}
	}

	if ( didHit && ( shape->enablePreSolveEvents || fastShape->enablePreSolveEvents ) )
	{
		// Pre-solve is expensive because I need to compute a temporary manifold
		b2Transform transformA = b2GetSweepTransform( &input.sweepA, hitFraction );
		b2Transform transformB = b2GetSweepTransform( &input.sweepB, hitFraction );
		b2Manifold manifold = b2ComputeManifold( shape, transformA, fastShape, transformB );
		b2ShapeId shapeIdA = { shape->id + 1, world->worldId, shape->revision };
		b2ShapeId shapeIdB = { fastShape->id + 1, world->worldId, fastShape->revision };

		// The user may modify the temporary manifold here but it doesn't matter. They will be able to
		// modify the real manifold in the discrete solver.
		didHit = world->preSolveFcn( shapeIdA, shapeIdB, &manifold, world->preSolveContext );
	}

	if ( didHit )
	{
		continuousContext->fraction = hitFraction;
	}

	return true;
}

// Continuous collision of dynamic versus static
static void b2SolveContinuous( b2World* world, int bodySimIndex )
{
	b2TracyCZoneNC( fast_body, "Fast Body", b2_colorDarkGoldenrod, true );

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2BodySim* fastBodySim = b2BodySimArray_Get( &awakeSet->bodySims, bodySimIndex );
	B2_ASSERT( fastBodySim->isFast );

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
	b2Body* fastBody = b2BodyArray_Get( &world->bodies, fastBodySim->bodyId );

	struct b2ContinuousContext context;
	context.world = world;
	context.sweep = sweep;
	context.fastBodySim = fastBodySim;
	context.fraction = 1.0f;

	bool isBullet = fastBodySim->isBullet;

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
		b2AABB box = b2AABB_Union( box1, box2 );

		// Store this to avoid double computation in the case there is no impact event
		fastShape->aabb = box2;

		// No continuous collision for sensors
		if ( fastShape->isSensor )
		{
			continue;
		}

		b2DynamicTree_Query( staticTree, box, b2_defaultMaskBits, b2ContinuousQueryCallback, &context );

		if ( isBullet )
		{
			b2DynamicTree_Query( kinematicTree, box, b2_defaultMaskBits, b2ContinuousQueryCallback, &context );
			b2DynamicTree_Query( dynamicTree, box, b2_defaultMaskBits, b2ContinuousQueryCallback, &context );
		}
	}

	const float speculativeDistance = b2_speculativeDistance;
	const float aabbMargin = b2_aabbMargin;

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

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the
		// AABB may not need enlargement.

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
				b2AABB fatAABB;
				fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
				fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
				fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
				fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
				shape->fatAABB = fatAABB;

				shape->enlargedAABB = true;
				fastBodySim->enlargeAABB = true;
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
			b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

			// shape->aabb is still valid from above

			if ( b2AABB_Contains( shape->fatAABB, shape->aabb ) == false )
			{
				b2AABB fatAABB;
				fatAABB.lowerBound.x = shape->aabb.lowerBound.x - aabbMargin;
				fatAABB.lowerBound.y = shape->aabb.lowerBound.y - aabbMargin;
				fatAABB.upperBound.x = shape->aabb.upperBound.x + aabbMargin;
				fatAABB.upperBound.y = shape->aabb.upperBound.y + aabbMargin;
				shape->fatAABB = fatAABB;

				shape->enlargedAABB = true;
				fastBodySim->enlargeAABB = true;
			}

			shapeId = shape->nextShapeId;
		}
	}

	b2TracyCZoneEnd( fast_body );
}

static void b2FinalizeBodiesTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( finalize_bodies, "FinalizeBodies", b2_colorViolet, true );

	b2StepContext* stepContext = context;
	b2World* world = stepContext->world;
	bool enableSleep = world->enableSleep;
	b2BodyState* states = stepContext->states;
	b2BodySim* sims = stepContext->sims;
	b2Body* bodies = world->bodies.data;
	float timeStep = stepContext->dt;
	float invTimeStep = stepContext->inv_dt;

	uint16_t worldId = world->worldId;

	// The body move event array has should already have the correct size
	B2_ASSERT( endIndex <= world->bodyMoveEvents.count );
	b2BodyMoveEvent* moveEvents = world->bodyMoveEvents.data;

	b2BitSet* enlargedSimBitSet = &world->taskContexts.data[threadIndex].enlargedSimBitSet;
	b2BitSet* awakeIslandBitSet = &world->taskContexts.data[threadIndex].awakeIslandBitSet;
	b2TaskContext* taskContext = world->taskContexts.data + threadIndex;

	bool enableContinuous = world->enableContinuous;

	const float speculativeDistance = b2_speculativeDistance;
	const float aabbMargin = b2_aabbMargin;

	B2_ASSERT( startIndex <= endIndex );

	for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
	{
		b2BodyState* state = states + simIndex;
		b2BodySim* sim = sims + simIndex;

		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

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
		moveEvents[simIndex].bodyId = ( b2BodyId ){ sim->bodyId + 1, worldId, body->revision };
		moveEvents[simIndex].userData = body->userData;
		moveEvents[simIndex].fellAsleep = false;

		// reset applied force and torque
		sim->force = b2Vec2_zero;
		sim->torque = 0.0f;

		body->isSpeedCapped = sim->isSpeedCapped;
		sim->isSpeedCapped = false;

		sim->isFast = false;

		if ( enableSleep == false || body->enableSleep == false || sleepVelocity > body->sleepThreshold )
		{
			// Body is not sleepy
			body->sleepTime = 0.0f;

			const float saftetyFactor = 0.5f;
			if ( body->type == b2_dynamicBody && enableContinuous && maxVelocity * timeStep > saftetyFactor * sim->minExtent )
			{
				// This flag is only retained for debug draw
				sim->isFast = true;

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if ( sim->isBullet )
				{
					int bulletIndex = atomic_fetch_add( &stepContext->bulletBodyCount, 1 );
					stepContext->bulletBodies[bulletIndex] = simIndex;
				}
				else
				{
					b2SolveContinuous( world, simIndex );
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
		b2Island* island = b2IslandArray_Get( &world->islands, body->islandId );
		if ( body->sleepTime < b2_timeToSleep )
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
		bool isFast = sim->isFast;
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
					b2AABB fatAABB;
					fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
					fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
					fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
					fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
					shape->fatAABB = fatAABB;

					shape->enlargedAABB = true;

					// Bit-set to keep the move array sorted
					b2SetBit( enlargedSimBitSet, simIndex );
				}
			}

			shapeId = shape->nextShapeId;
		}
	}

	b2TracyCZoneEnd( finalize_bodies );
}

/*
 typedef enum b2SolverStageType
{
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageIntegrateVelocities,
	b2_stageWarmStart,
	b2_stageSolve,
	b2_stageIntegratePositions,
	b2_stageRelax,
	b2_stageRestitution,
	b2_stageStoreImpulses
} b2SolverStageType;

typedef enum b2SolverBlockType
{
	b2_bodyBlock,
	b2_jointBlock,
	b2_contactBlock,
	b2_graphJointBlock,
	b2_graphContactBlock
} b2SolverBlockType;
*/

static void b2ExecuteBlock( b2SolverStage* stage, b2StepContext* context, b2SolverBlock* block )
{
	b2SolverStageType stageType = stage->type;
	b2SolverBlockType blockType = block->blockType;
	int startIndex = block->startIndex;
	int endIndex = startIndex + block->count;

	switch ( stageType )
	{
		case b2_stagePrepareJoints:
			b2PrepareJointsTask( startIndex, endIndex, context );
			break;

		case b2_stagePrepareContacts:
			b2PrepareContactsTask( startIndex, endIndex, context );
			break;

		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask( startIndex, endIndex, context );
			break;

		case b2_stageWarmStart:
			if ( context->world->enableWarmStarting )
			{
				if ( blockType == b2_graphContactBlock )
				{
					b2WarmStartContactsTask( startIndex, endIndex, context, stage->colorIndex );
				}
				else if ( blockType == b2_graphJointBlock )
				{
					b2WarmStartJointsTask( startIndex, endIndex, context, stage->colorIndex );
				}
			}
			break;

		case b2_stageSolve:
			if ( blockType == b2_graphContactBlock )
			{
				b2SolveContactsTask( startIndex, endIndex, context, stage->colorIndex, true );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				b2SolveJointsTask( startIndex, endIndex, context, stage->colorIndex, true );
			}
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask( startIndex, endIndex, context );
			break;

		case b2_stageRelax:
			if ( blockType == b2_graphContactBlock )
			{
				b2SolveContactsTask( startIndex, endIndex, context, stage->colorIndex, false );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				b2SolveJointsTask( startIndex, endIndex, context, stage->colorIndex, false );
			}
			break;

		case b2_stageRestitution:
			if ( blockType == b2_graphContactBlock )
			{
				b2ApplyRestitutionTask( startIndex, endIndex, context, stage->colorIndex );
			}
			break;

		case b2_stageStoreImpulses:
			b2StoreImpulsesTask( startIndex, endIndex, context );
			break;
	}
}

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

static void b2ExecuteStage( b2SolverStage* stage, b2StepContext* context, int previousSyncIndex, int syncIndex, int workerIndex )
{
	int completedCount = 0;
	b2SolverBlock* blocks = stage->blocks;
	int blockCount = stage->blockCount;

	int expectedSyncIndex = previousSyncIndex;

	int startIndex = GetWorkerStartIndex( workerIndex, blockCount, context->workerCount );
	if ( startIndex == B2_NULL_INDEX )
	{
		return;
	}

	B2_ASSERT( 0 <= startIndex && startIndex < blockCount );

	int blockIndex = startIndex;

	// Caution: this can change expectedSyncIndex
	while ( atomic_compare_exchange_strong( &blocks[blockIndex].syncIndex, &expectedSyncIndex, syncIndex ) == true )
	{
		B2_ASSERT( stage->type != b2_stagePrepareContacts || syncIndex < 2 );

		B2_ASSERT( completedCount < blockCount );

		b2ExecuteBlock( stage, context, blocks + blockIndex );

		completedCount += 1;
		blockIndex += 1;
		if ( blockIndex >= blockCount )
		{
			// Keep looking for work
			blockIndex = 0;
		}

		expectedSyncIndex = previousSyncIndex;
	}

	// Search backwards for blocks
	blockIndex = startIndex - 1;
	while ( true )
	{
		if ( blockIndex < 0 )
		{
			blockIndex = blockCount - 1;
		}

		expectedSyncIndex = previousSyncIndex;

		// Caution: this can change expectedSyncIndex
		if ( atomic_compare_exchange_strong( &blocks[blockIndex].syncIndex, &expectedSyncIndex, syncIndex ) == false )
		{
			break;
		}

		b2ExecuteBlock( stage, context, blocks + blockIndex );
		completedCount += 1;
		blockIndex -= 1;
	}

	(void)atomic_fetch_add( &stage->completionCount, completedCount );
}

static void b2ExecuteMainStage( b2SolverStage* stage, b2StepContext* context, uint32_t syncBits )
{
	int blockCount = stage->blockCount;
	if ( blockCount == 0 )
	{
		return;
	}

	if ( blockCount == 1 )
	{
		b2ExecuteBlock( stage, context, stage->blocks );
	}
	else
	{
		atomic_store( &context->atomicSyncBits, syncBits );

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B2_ASSERT( syncIndex > 0 );
		int previousSyncIndex = syncIndex - 1;

		b2ExecuteStage( stage, context, previousSyncIndex, syncIndex, 0 );

		// todo consider using the cycle counter as well
		while ( atomic_load( &stage->completionCount ) != blockCount )
		{
			b2Pause();
		}

		atomic_store( &stage->completionCount, 0 );
	}
}

// This should not use the thread index because thread 0 can be called twice by enkiTS.
void b2SolverTask( int startIndex, int endIndex, uint32_t threadIndexDontUse, void* taskContext )
{
	B2_MAYBE_UNUSED( startIndex );
	B2_MAYBE_UNUSED( endIndex );
	B2_MAYBE_UNUSED( threadIndexDontUse );

	b2WorkerContext* workerContext = taskContext;
	int workerIndex = workerContext->workerIndex;
	b2StepContext* context = workerContext->context;
	int activeColorCount = context->activeColorCount;
	b2SolverStage* stages = context->stages;
	b2Profile* profile = &context->world->profile;

	if ( workerIndex == 0 )
	{
		// Main thread synchronizes the workers and does work itself.
		//
		// Stages are re-used by loops so that I don't need more stages for large iteration counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
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

		b2Timer timer = b2CreateTimer();

		int bodySyncIndex = 1;
		int stageIndex = 0;

		// This stage loops over all awake joints
		uint32_t jointSyncIndex = 1;
		uint32_t syncBits = ( jointSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stagePrepareJoints );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		jointSyncIndex += 1;

		// This stage loops over all contact constraints
		uint32_t contactSyncIndex = 1;
		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stagePrepareContacts );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		contactSyncIndex += 1;

		int graphSyncIndex = 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		b2PrepareOverflowJoints( context );
		b2PrepareOverflowContacts( context );

		profile->prepareConstraints += b2GetMillisecondsAndReset( &timer );

		int subStepCount = context->subStepCount;
		for ( int i = 0; i < subStepCount; ++i )
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int iterStageIndex = stageIndex;

			// integrate velocities
			syncBits = ( bodySyncIndex << 16 ) | iterStageIndex;
			B2_ASSERT( stages[iterStageIndex].type == b2_stageIntegrateVelocities );
			b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
			iterStageIndex += 1;
			bodySyncIndex += 1;

			profile->integrateVelocities += b2GetMillisecondsAndReset( &timer );

			// warm start constraints
			b2WarmStartOverflowJoints( context );
			b2WarmStartOverflowContacts( context );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageWarmStart );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->warmStart += b2GetMillisecondsAndReset( &timer );

			// solve constraints
			bool useBias = true;
			b2SolveOverflowJoints( context, useBias );
			b2SolveOverflowContacts( context, useBias );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageSolve );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->solveVelocities += b2GetMillisecondsAndReset( &timer );

			// integrate positions
			B2_ASSERT( stages[iterStageIndex].type == b2_stageIntegratePositions );
			syncBits = ( bodySyncIndex << 16 ) | iterStageIndex;
			b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
			iterStageIndex += 1;
			bodySyncIndex += 1;

			profile->integratePositions += b2GetMillisecondsAndReset( &timer );

			// relax constraints
			useBias = false;
			b2SolveOverflowJoints( context, useBias );
			b2SolveOverflowContacts( context, useBias );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageRelax );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->relaxVelocities += b2GetMillisecondsAndReset( &timer );
		}

		// advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + activeColorCount + 1 + activeColorCount;

		// Restitution
		{
			b2ApplyOverflowRestitution( context );

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

		profile->applyRestitution += b2GetMillisecondsAndReset( &timer );

		b2StoreOverflowImpulses( context );

		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stageStoreImpulses );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );

		profile->storeImpulses += b2GetMillisecondsAndReset( &timer );

		// Signal workers to finish
		atomic_store( &context->atomicSyncBits, UINT_MAX );

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
		uint32_t syncBits;
		int spinCount = 0;
		while ( ( syncBits = atomic_load( &context->atomicSyncBits ) ) == lastSyncBits )
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
				//	b2Pause();
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

static void b2BulletBodyTask( int startIndex, int endIndex, uint32_t threadIndex, void* taskContext )
{
	B2_MAYBE_UNUSED( threadIndex );

	b2TracyCZoneNC( bullet_body_task, "Bullet Body Task", b2_colorLightSkyBlue, true );

	b2StepContext* stepContext = taskContext;

	B2_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		int simIndex = stepContext->bulletBodies[i];
		b2SolveContinuous( stepContext->world, simIndex );
	}

	b2TracyCZoneEnd( bullet_body_task );
}

#if B2_SIMD_WIDTH == 8
#define B2_SIMD_SHIFT 3
#else
#define B2_SIMD_SHIFT 2
#endif

// Solve with graph coloring
void b2Solve( b2World* world, b2StepContext* stepContext )
{
	b2Timer timer = b2CreateTimer();

	world->stepIndex += 1;

	b2MergeAwakeIslands( world );

	world->profile.buildIslands = b2GetMillisecondsAndReset( &timer );

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	int awakeBodyCount = awakeSet->bodySims.count;
	if ( awakeBodyCount == 0 )
	{
		// Nothing to simulate, however I must still finish the broad-phase rebuild.
		if ( world->userTreeTask != NULL )
		{
			world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
			world->userTreeTask = NULL;
			world->activeTaskCount -= 1;
		}

		b2ValidateNoEnlarged( &world->broadPhase );
		return;
	}

	b2TracyCZoneNC( solve, "Solve", b2_colorIndigo, true );

	// Prepare buffers for continuous collision (fast bodies)
	stepContext->bulletBodyCount = 0;
	stepContext->bulletBodies = b2AllocateStackItem( &world->stackAllocator, awakeBodyCount * sizeof( int ), "bullet bodies" );

	b2TracyCZoneNC( graph_solver, "Graph", b2_colorSeaGreen, true );

	// Solve constraints using graph coloring
	{
		b2TracyCZoneNC( prepare_stages, "Prepare Stages", b2_colorDarkOrange, true );

		b2ConstraintGraph* graph = &world->constraintGraph;
		b2GraphColor* colors = graph->colors;

		stepContext->sims = awakeSet->bodySims.data;
		stepContext->states = awakeSet->bodyStates.data;

		// count contacts, joints, and colors
		int awakeContactCount = 0;
		int awakeJointCount = 0;
		int activeColorCount = 0;
		for ( int i = 0; i < b2_graphColorCount - 1; ++i )
		{
			int perColorContactCount = colors[i].contactSims.count;
			int perColorJointCount = colors[i].jointSims.count;
			int occupancyCount = perColorContactCount + perColorJointCount;
			activeColorCount += occupancyCount > 0 ? 1 : 0;
			awakeContactCount += perColorContactCount;
			awakeJointCount += perColorJointCount;
		}

		// Deal with void**
		{
			b2BodyMoveEventArray_Resize( &world->bodyMoveEvents, awakeBodyCount );
		}

		// Each worker receives at most M blocks of work. The workers may receive less than there is not sufficient work.
		// Each block of work has a minimum number of elements (block size). This in turn may limit number of blocks.
		// If there are many elements then the block size is increased so there are still at most M blocks of work per worker.
		// M is a tunable number that has two goals:
		// 1. keep M small to reduce overhead
		// 2. keep M large enough for other workers to be able to steal work
		// The block size is a power of two to make math efficient.

		int workerCount = world->workerCount;
		const int blocksPerWorker = 4;
		const int maxBlockCount = blocksPerWorker * workerCount;

		// Configure blocks for tasks that parallel-for bodies
		int bodyBlockSize = 1 << 5;
		int bodyBlockCount;
		if ( awakeBodyCount > bodyBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			bodyBlockSize = awakeBodyCount / maxBlockCount;
			bodyBlockCount = maxBlockCount;
		}
		else
		{
			bodyBlockCount = ( ( awakeBodyCount - 1 ) >> 5 ) + 1;
		}

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of SIMD contact blocks and joint blocks
		int activeColorIndices[b2_graphColorCount];

		int colorContactCounts[b2_graphColorCount];
		int colorContactBlockSizes[b2_graphColorCount];
		int colorContactBlockCounts[b2_graphColorCount];

		int colorJointCounts[b2_graphColorCount];
		int colorJointBlockSizes[b2_graphColorCount];
		int colorJointBlockCounts[b2_graphColorCount];

		int graphBlockCount = 0;

		// c is the active color index
		int simdContactCount = 0;
		int c = 0;
		for ( int i = 0; i < b2_graphColorCount - 1; ++i )
		{
			int colorContactCount = colors[i].contactSims.count;
			int colorJointCount = colors[i].jointSims.count;

			if ( colorContactCount + colorJointCount > 0 )
			{
				activeColorIndices[c] = i;

				// 4/8-way SIMD
				int colorContactCountSIMD = colorContactCount > 0 ? ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1 : 0;

				colorContactCounts[c] = colorContactCountSIMD;

				// determine the number of contact work blocks for this color
				if ( colorContactCountSIMD > blocksPerWorker * maxBlockCount )
				{
					// too many contact blocks
					colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
					colorContactBlockCounts[c] = maxBlockCount;
				}
				else if ( colorContactCountSIMD > 0 )
				{
					// dividing by blocksPerWorker (4)
					colorContactBlockSizes[c] = blocksPerWorker;
					colorContactBlockCounts[c] = ( ( colorContactCountSIMD - 1 ) >> 2 ) + 1;
				}
				else
				{
					// no contacts in this color
					colorContactBlockSizes[c] = 0;
					colorContactBlockCounts[c] = 0;
				}

				colorJointCounts[c] = colorJointCount;

				// determine number of joint work blocks for this color
				if ( colorJointCount > blocksPerWorker * maxBlockCount )
				{
					// too many joint blocks
					colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
					colorJointBlockCounts[c] = maxBlockCount;
				}
				else if ( colorJointCount > 0 )
				{
					// dividing by blocksPerWorker (4)
					colorJointBlockSizes[c] = blocksPerWorker;
					colorJointBlockCounts[c] = ( ( colorJointCount - 1 ) >> 2 ) + 1;
				}
				else
				{
					colorJointBlockSizes[c] = 0;
					colorJointBlockCounts[c] = 0;
				}

				graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
				simdContactCount += colorContactCountSIMD;
				c += 1;
			}
		}
		activeColorCount = c;

		// Gather contact pointers for easy parallel-for traversal. Some may be NULL due to SIMD remainders.
		b2ContactSim** contacts = b2AllocateStackItem(
			&world->stackAllocator, B2_SIMD_WIDTH * simdContactCount * sizeof( b2ContactSim* ), "contact pointers" );

		// Gather joint pointers for easy parallel-for traversal.
		b2JointSim** joints =
			b2AllocateStackItem( &world->stackAllocator, awakeJointCount * sizeof( b2JointSim* ), "joint pointers" );

		int simdConstraintSize = b2GetContactConstraintSIMDByteCount();
		b2ContactConstraintSIMD* simdContactConstraints =
			b2AllocateStackItem( &world->stackAllocator, simdContactCount * simdConstraintSize, "contact constraint" );

		int overflowContactCount = colors[b2_overflowIndex].contactSims.count;
		b2ContactConstraint* overflowContactConstraints = b2AllocateStackItem(
			&world->stackAllocator, overflowContactCount * sizeof( b2ContactConstraint ), "overflow contact constraint" );

		graph->colors[b2_overflowIndex].overflowConstraints = overflowContactConstraints;

		// Distribute transient constraints to each graph color and build flat arrays of contact and joint pointers
		{
			int contactBase = 0;
			int jointBase = 0;
			for ( int i = 0; i < activeColorCount; ++i )
			{
				int j = activeColorIndices[i];
				b2GraphColor* color = colors + j;

				int colorContactCount = color->contactSims.count;

				if ( colorContactCount == 0 )
				{
					color->simdConstraints = NULL;
				}
				else
				{
					color->simdConstraints =
						(b2ContactConstraintSIMD*)( (uint8_t*)simdContactConstraints + contactBase * simdConstraintSize );

					for ( int k = 0; k < colorContactCount; ++k )
					{
						contacts[B2_SIMD_WIDTH * contactBase + k] = color->contactSims.data + k;
					}

					// remainder
					int colorContactCountSIMD = ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1;
					for ( int k = colorContactCount; k < B2_SIMD_WIDTH * colorContactCountSIMD; ++k )
					{
						contacts[B2_SIMD_WIDTH * contactBase + k] = NULL;
					}

					contactBase += colorContactCountSIMD;
				}

				int colorJointCount = color->jointSims.count;
				for ( int k = 0; k < colorJointCount; ++k )
				{
					joints[jointBase + k] = color->jointSims.data + k;
				}
				jointBase += colorJointCount;
			}

			B2_ASSERT( contactBase == simdContactCount );
			B2_ASSERT( jointBase == awakeJointCount );
		}

		// Define work blocks for preparing contacts and storing contact impulses
		int contactBlockSize = blocksPerWorker;
		int contactBlockCount = simdContactCount > 0 ? ( ( simdContactCount - 1 ) >> 2 ) + 1 : 0;
		if ( simdContactCount > contactBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			contactBlockSize = simdContactCount / maxBlockCount;
			contactBlockCount = maxBlockCount;
		}

		// Define work blocks for preparing joints
		int jointBlockSize = blocksPerWorker;
		int jointBlockCount = awakeJointCount > 0 ? ( ( awakeJointCount - 1 ) >> 2 ) + 1 : 0;
		if ( awakeJointCount > jointBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			jointBlockSize = awakeJointCount / maxBlockCount;
			jointBlockCount = maxBlockCount;
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
		stageCount += activeColorCount;
		// b2_stageIntegratePositions
		stageCount += 1;
		// b2_stageRelax
		stageCount += activeColorCount;
		// b2_stageRestitution
		stageCount += activeColorCount;
		// b2_stageStoreImpulses
		stageCount += 1;

		b2SolverStage* stages = b2AllocateStackItem( &world->stackAllocator, stageCount * sizeof( b2SolverStage ), "stages" );
		b2SolverBlock* bodyBlocks =
			b2AllocateStackItem( &world->stackAllocator, bodyBlockCount * sizeof( b2SolverBlock ), "body blocks" );
		b2SolverBlock* contactBlocks =
			b2AllocateStackItem( &world->stackAllocator, contactBlockCount * sizeof( b2SolverBlock ), "contact blocks" );
		b2SolverBlock* jointBlocks =
			b2AllocateStackItem( &world->stackAllocator, jointBlockCount * sizeof( b2SolverBlock ), "joint blocks" );
		b2SolverBlock* graphBlocks =
			b2AllocateStackItem( &world->stackAllocator, graphBlockCount * sizeof( b2SolverBlock ), "graph blocks" );

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		void* splitIslandTask = NULL;
		if ( world->splitIslandId != B2_NULL_INDEX )
		{
			splitIslandTask = world->enqueueTaskFcn( &b2SplitIslandTask, 1, 1, world, world->userTaskContext );
			world->taskCount += 1;
			world->activeTaskCount += splitIslandTask == NULL ? 0 : 1;
		}

		// Prepare body work blocks
		for ( int i = 0; i < bodyBlockCount; ++i )
		{
			b2SolverBlock* block = bodyBlocks + i;
			block->startIndex = i * bodyBlockSize;
			block->count = (int16_t)bodyBlockSize;
			block->blockType = b2_bodyBlock;
			block->syncIndex = 0;
		}
		bodyBlocks[bodyBlockCount - 1].count = (int16_t)( awakeBodyCount - ( bodyBlockCount - 1 ) * bodyBlockSize );

		// Prepare joint work blocks
		for ( int i = 0; i < jointBlockCount; ++i )
		{
			b2SolverBlock* block = jointBlocks + i;
			block->startIndex = i * jointBlockSize;
			block->count = (int16_t)jointBlockSize;
			block->blockType = b2_jointBlock;
			block->syncIndex = 0;
		}

		if ( jointBlockCount > 0 )
		{
			jointBlocks[jointBlockCount - 1].count = (int16_t)( awakeJointCount - ( jointBlockCount - 1 ) * jointBlockSize );
		}

		// Prepare contact work blocks
		for ( int i = 0; i < contactBlockCount; ++i )
		{
			b2SolverBlock* block = contactBlocks + i;
			block->startIndex = i * contactBlockSize;
			block->count = (int16_t)contactBlockSize;
			block->blockType = b2_contactBlock;
			block->syncIndex = 0;
		}

		if ( contactBlockCount > 0 )
		{
			contactBlocks[contactBlockCount - 1].count =
				(int16_t)( simdContactCount - ( contactBlockCount - 1 ) * contactBlockSize );
		}

		// Prepare graph work blocks
		b2SolverBlock* graphColorBlocks[b2_graphColorCount];
		b2SolverBlock* baseGraphBlock = graphBlocks;

		for ( int i = 0; i < activeColorCount; ++i )
		{
			graphColorBlocks[i] = baseGraphBlock;

			int colorJointBlockCount = colorJointBlockCounts[i];
			int colorJointBlockSize = colorJointBlockSizes[i];
			for ( int j = 0; j < colorJointBlockCount; ++j )
			{
				b2SolverBlock* block = baseGraphBlock + j;
				block->startIndex = j * colorJointBlockSize;
				block->count = (int16_t)colorJointBlockSize;
				block->blockType = b2_graphJointBlock;
				block->syncIndex = 0;
			}

			if ( colorJointBlockCount > 0 )
			{
				baseGraphBlock[colorJointBlockCount - 1].count =
					(int16_t)( colorJointCounts[i] - ( colorJointBlockCount - 1 ) * colorJointBlockSize );
				baseGraphBlock += colorJointBlockCount;
			}

			int colorContactBlockCount = colorContactBlockCounts[i];
			int colorContactBlockSize = colorContactBlockSizes[i];
			for ( int j = 0; j < colorContactBlockCount; ++j )
			{
				b2SolverBlock* block = baseGraphBlock + j;
				block->startIndex = j * colorContactBlockSize;
				block->count = (int16_t)colorContactBlockSize;
				block->blockType = b2_graphContactBlock;
				block->syncIndex = 0;
			}

			if ( colorContactBlockCount > 0 )
			{
				baseGraphBlock[colorContactBlockCount - 1].count =
					(int16_t)( colorContactCounts[i] - ( colorContactBlockCount - 1 ) * colorContactBlockSize );
				baseGraphBlock += colorContactBlockCount;
			}
		}

		B2_ASSERT( (ptrdiff_t)(baseGraphBlock - graphBlocks) == graphBlockCount );

		b2SolverStage* stage = stages;

		// Prepare joints
		stage->type = b2_stagePrepareJoints;
		stage->blocks = jointBlocks;
		stage->blockCount = jointBlockCount;
		stage->colorIndex = -1;
		stage->completionCount = 0;
		stage += 1;

		// Prepare contacts
		stage->type = b2_stagePrepareContacts;
		stage->blocks = contactBlocks;
		stage->blockCount = contactBlockCount;
		stage->colorIndex = -1;
		stage->completionCount = 0;
		stage += 1;

		// Integrate velocities
		stage->type = b2_stageIntegrateVelocities;
		stage->blocks = bodyBlocks;
		stage->blockCount = bodyBlockCount;
		stage->colorIndex = -1;
		stage->completionCount = 0;
		stage += 1;

		// Warm start
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b2_stageWarmStart;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
			stage->colorIndex = activeColorIndices[i];
			stage->completionCount = 0;
			stage += 1;
		}

		// Solve graph
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b2_stageSolve;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
			stage->colorIndex = activeColorIndices[i];
			stage->completionCount = 0;
			stage += 1;
		}

		// Integrate positions
		stage->type = b2_stageIntegratePositions;
		stage->blocks = bodyBlocks;
		stage->blockCount = bodyBlockCount;
		stage->colorIndex = -1;
		stage->completionCount = 0;
		stage += 1;

		// Relax constraints
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b2_stageRelax;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
			stage->colorIndex = activeColorIndices[i];
			stage->completionCount = 0;
			stage += 1;
		}

		// Restitution
		// Note: joint blocks mixed in, could have joint limit restitution
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage->type = b2_stageRestitution;
			stage->blocks = graphColorBlocks[i];
			stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
			stage->colorIndex = activeColorIndices[i];
			stage->completionCount = 0;
			stage += 1;
		}

		// Store impulses
		stage->type = b2_stageStoreImpulses;
		stage->blocks = contactBlocks;
		stage->blockCount = contactBlockCount;
		stage->colorIndex = -1;
		stage->completionCount = 0;
		stage += 1;

		B2_ASSERT( (int)( stage - stages ) == stageCount );

		B2_ASSERT( workerCount <= b2_maxWorkers );
		b2WorkerContext workerContext[b2_maxWorkers];

		stepContext->graph = graph;
		stepContext->joints = joints;
		stepContext->contacts = contacts;
		stepContext->simdContactConstraints = simdContactConstraints;
		stepContext->activeColorCount = activeColorCount;
		stepContext->workerCount = workerCount;
		stepContext->stageCount = stageCount;
		stepContext->stages = stages;
		stepContext->atomicSyncBits = 0;

		world->profile.prepareTasks = b2GetMillisecondsAndReset( &timer );

		b2TracyCZoneEnd( prepare_stages );

		// Must use worker index because thread 0 can be assigned multiple tasks by enkiTS
		for ( int i = 0; i < workerCount; ++i )
		{
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

		world->profile.solverTasks = b2GetMillisecondsAndReset( &timer );

		// Prepare contact, enlarged body, and island bit sets used in body finalization.
		int awakeIslandCount = awakeSet->islandSims.count;
		for ( int i = 0; i < world->workerCount; ++i )
		{
			b2TaskContext* taskContext = world->taskContexts.data + i;
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

		world->profile.finalizeBodies = b2GetMillisecondsAndReset( &timer );

		b2FreeStackItem( &world->stackAllocator, graphBlocks );
		b2FreeStackItem( &world->stackAllocator, jointBlocks );
		b2FreeStackItem( &world->stackAllocator, contactBlocks );
		b2FreeStackItem( &world->stackAllocator, bodyBlocks );
		b2FreeStackItem( &world->stackAllocator, stages );
		b2FreeStackItem( &world->stackAllocator, overflowContactConstraints );
		b2FreeStackItem( &world->stackAllocator, simdContactConstraints );
		b2FreeStackItem( &world->stackAllocator, joints );
		b2FreeStackItem( &world->stackAllocator, contacts );
	}

	b2TracyCZoneEnd( graph_solver );
	world->profile.solveConstraints = b2GetMillisecondsAndReset( &timer );

	// Report hit events
	// todo perhaps optimize this with a bitset
	{
		b2TracyCZoneNC( hit_events, "Hit", b2_colorVioletRed, true );

		B2_ASSERT( world->contactHitEvents.count == 0 );

		float threshold = world->hitEventThreshold;
		b2GraphColor* colors = world->constraintGraph.colors;
		for ( int i = 0; i < b2_graphColorCount; ++i )
		{
			b2GraphColor* color = colors + i;
			int contactCount = color->contactSims.count;
			b2ContactSim* contactSims = color->contactSims.data;
			for ( int j = 0; j < contactCount; ++j )
			{
				b2ContactSim* contactSim = contactSims + j;
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

					// Need to check max impulse because the point may be speculative and not colliding
					if ( approachSpeed > event.approachSpeed && mp->maxNormalImpulse > 0.0f )
					{
						event.approachSpeed = approachSpeed;
						event.point = mp->point;
						hit = true;
					}
				}

				if ( hit == true )
				{
					event.normal = contactSim->manifold.normal;

					b2Shape* shapeA = b2ShapeArray_Get( &world->shapes, contactSim->shapeIdA );
					b2Shape* shapeB = b2ShapeArray_Get( &world->shapes, contactSim->shapeIdB );

					event.shapeIdA = ( b2ShapeId ){ shapeA->id + 1, world->worldId, shapeA->revision };
					event.shapeIdB = ( b2ShapeId ){ shapeB->id + 1, world->worldId, shapeB->revision };

					b2ContactHitEventArray_Push( &world->contactHitEvents, event );
				}
			}
		}

		b2TracyCZoneEnd( hit_events );
	}

	world->profile.hitEvents = b2GetMillisecondsAndReset( &timer );

	// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the broad-phase.
	if ( world->userTreeTask != NULL )
	{
		world->finishTaskFcn( world->userTreeTask, world->userTaskContext );
		world->userTreeTask = NULL;
		world->activeTaskCount -= 1;
	}

	b2ValidateNoEnlarged( &world->broadPhase );

	b2TracyCZoneNC( broad_phase, "Broadphase", b2_colorPurple, true );

	b2TracyCZoneNC( enlarge_proxies, "Enlarge Proxies", b2_colorDarkTurquoise, true );

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
				if (bodySim->isBullet && bodySim->isFast)
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

	world->profile.broadphase = b2GetMillisecondsAndReset( &timer );

	b2TracyCZoneEnd( enlarge_proxies );
	b2TracyCZoneEnd( broad_phase );

	if ( stepContext->bulletBodyCount > 0 )
	{
		b2TracyCZoneNC( bullets, "Bullets", b2_colorDarkGoldenrod, true );

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		int minRange = 8;
		void* userBulletBodyTask = world->enqueueTaskFcn( &b2BulletBodyTask, stepContext->bulletBodyCount, minRange, stepContext,
														  world->userTaskContext );
		world->taskCount += 1;
		if ( userBulletBodyTask != NULL )
		{
			world->finishTaskFcn( userBulletBodyTask, world->userTaskContext );
		}

		b2TracyCZoneEnd( bullets );
	}

	// Serially enlarge broad-phase proxies for bullet shapes
	{
		b2TracyCZoneNC( broad_phase_bullet, "Broadphase", b2_colorPurple, true );
		b2TracyCZoneNC( enlarge_proxies_bullet, "Enlarge Proxies", b2_colorDarkTurquoise, true );

		b2BroadPhase* broadPhase = &world->broadPhase;
		b2DynamicTree* dynamicTree = broadPhase->trees + b2_dynamicBody;

		// Fast array access is important here
		b2Body* bodyArray = world->bodies.data;
		b2BodySim* bodySimArray = awakeSet->bodySims.data;
		b2Shape* shapeArray = world->shapes.data;

		// Serially enlarge broad-phase proxies for bullet shapes
		int* bulletBodySimIndices = stepContext->bulletBodies;
		int bulletBodyCount = stepContext->bulletBodyCount;

		// This loop has non-deterministic order but it shouldn't affect the result
		for ( int i = 0; i < bulletBodyCount; ++i )
		{
			b2BodySim* bulletBodySim = bodySimArray + bulletBodySimIndices[i];
			if ( bulletBodySim->enlargeAABB == false )
			{
				continue;
			}

			// clear flag
			bulletBodySim->enlargeAABB = false;

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

				// clear flag
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

		b2TracyCZoneEnd( enlarge_proxies_bullet );
		b2TracyCZoneEnd( broad_phase_bullet );
	}

	b2FreeStackItem( &world->stackAllocator, stepContext->bulletBodies );
	stepContext->bulletBodies = NULL;
	stepContext->bulletBodyCount = 0;

	world->profile.continuous = b2GetMillisecondsAndReset( &timer );

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	if ( world->enableSleep == true )
	{
		b2TracyCZoneNC( sleep_islands, "Island Sleep", b2_colorGainsboro, true );

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

		b2TracyCZoneEnd( sleep_islands );
	}

	world->profile.sleepIslands = b2GetMillisecondsAndReset( &timer );
	b2TracyCZoneEnd( solve );
}
