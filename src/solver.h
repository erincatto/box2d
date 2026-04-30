// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// Solver work is partitioned into fixed-size blocks that worker threads claim
// in parallel via atomic CAS on a per-block syncIndex. The descriptor (b2SolverBlock)
// and the atomic counter sit in a wrapping b2SyncBlock so the CAS-winner can
// pass the descriptor by value into stage tasks without aliasing the atomic
// memory other threads are CAS-writing. Three properties of this design
// matter for performance:
//
// 1. Distributed contention. Per-block atomic syncIndex avoids the cache line stampede
//    that a single shared fetch_add counter would cause. Once a worker
//    settles into a block range, its CAS targets live in its own L1.
//
// 2. Monotonic syncIndex across iterations. Iterative stages (warm start,
//    solve, relax) reuse the same block array every sub-step iteration.
//    syncIndex grows each iteration; workers CAS (prev, prev+1), so the
//    main thread never touches any per-block state between iterations.
//    Non-iterative stages simply use syncIndex 1.
//
// 3. L2 affinity across iterations. Each worker picks a start offset from
//    its workerIndex, then scans forward and (after wrap) backward:
//
//      blocks:   [0] [1] [2] [3] [4] [5] [6] [7]
//                 ^           ^           ^   ^
//                 W0          W1          W2  W3   <- start offsets
//
//    W0 claims 0,1,2,3 (forward), W1 claims 4,5, etc. Under balanced load
//    each worker re-hits the same block range every iteration, keeping that
//    range's hot data resident in its L2. A failed CAS means a neighbour
//    already claimed the block, so the stealing worker stops -- preserving
//    locality under mild imbalance while still draining the queue.
//
// A graph color stage lays out joint blocks first, then contact blocks:
//
//      stage->blocks ->
//        +------+------+------+------+------+------+------+
//        |  J0  |  J1  |  J2  |  C0  |  C1  |  C2  |  C3  |
//        +------+------+------+------+------+------+------+
//        <-- graphJointBlocks --><---- graphContactBlocks ---->
//
// Each block carries its type so the dispatcher routes J-blocks to the joint
// solver and C-blocks to the SIMD contact solver; both kinds run concurrently
// within the stage -- no barrier between them. The type tag lives on the
// block (not the stage) so that mixed-type stages can keep the concurrency.
//
// The solver threading model is inspired by https://github.com/bepu/bepuphysics2

#pragma once

#include "core.h"

#include "box2d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

#if B2_SIMD_WIDTH == 8
#define B2_SIMD_SHIFT 3
#elif B2_SIMD_WIDTH == 4
#define B2_SIMD_SHIFT 2
#else
#define B2_SIMD_SHIFT 0
#endif

typedef struct b2BodySim b2BodySim;
typedef struct b2BodyState b2BodyState;
typedef struct b2ContactSim b2ContactSim;
typedef struct b2ContactConstraintWide b2ContactConstraintWide;
typedef struct b2JointSim b2JointSim;
typedef struct b2World b2World;

// Solver stages. Prepare joints and prepare contacts are split up
// because there is no need to store joint impulses.
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

// Solver block describes a multithreaded unit of work.
typedef struct b2SolverBlock
{
	int startIndex;
	uint16_t count;
	// b2SolverBlockType
	uint8_t blockType;
	uint8_t colorIndex;
} b2SolverBlock;

// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
// monotonically allowing the solver block to be re-used across sub-steps.
typedef struct b2SyncBlock
{
	b2SolverBlock block;
	b2AtomicInt syncIndex;
} b2SyncBlock;

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
typedef struct b2SolverStage
{
	b2SyncBlock* blocks;
	b2SolverStageType type;
	int blockCount;
	uint8_t colorIndex;
	b2AtomicInt completionCount;
} b2SolverStage;

// Constraint softness
typedef struct b2Softness
{
	float biasRate;
	float massScale;
	float impulseScale;
} b2Softness;

// Prepare/store run as a flat parallel-for over the whole wide-constraint
// range. Each span maps a slice of that range back to the owning color's
// contacts so workers can decode flat wide-slot indices without touching
// graph state. The spans array has one entry per active color plus a sentinel
// whose start == wideContactCount.
typedef struct b2ContactPrepareSpan
{
	int start;
	int count;
	b2ContactSim* contacts;
} b2ContactPrepareSpan;

// Similar for joints
typedef struct b2JointPrepareSpan
{
	int start;
	int count;
	b2JointSim* joints;
} b2JointPrepareSpan;

// Context for a time step. Recreated each time step.
typedef struct b2StepContext
{
	// time step
	float dt;

	// inverse time step (0 if dt == 0).
	float inv_dt;

	// sub-step
	float h;
	float inv_h;

	int subStepCount;

	b2Softness contactSoftness;
	b2Softness staticSoftness;

	float restitutionThreshold;
	float maxLinearVelocity;

	struct b2World* world;
	struct b2ConstraintGraph* graph;

	// shortcut to body states from awake set
	b2BodyState* states;

	// shortcut to body sims from awake set
	b2BodySim* sims;

	// array of all shape ids for shapes that have enlarged AABBs
	int* enlargedShapes;
	int enlargedShapeCount;

	// Array of bullet bodies that need continuous collision handling
	int* bulletBodies;
	b2AtomicInt bulletBodyCount;

	// contact pointers for simplified parallel-for access.
	// - parallel-for collide with no gaps, includes touching and non-touching
	b2ContactSim** contactSims;

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. wideContactConstraints is the contiguous base
	// pointer; per-color slices live at colors[i].wideConstraints.
	b2ContactConstraintWide* wideContactConstraints;
	b2ContactPrepareSpan* contactPrepareSpans;
	int wideContactCount;
	
	b2JointPrepareSpan* jointPrepareSpans;
	int jointCount;

	int activeColorCount;
	int workerCount;

	b2SolverStage* stages;
	int stageCount;
	bool enableWarmStarting;

	// padding to prevent false sharing
	char padding1[64];

	// This atomic is central to multi-threaded solver task synchronization.
	// It prevents ABA problems by monotonically growing as the solver advances.
	// This means a delayed worker thread will catch up without repeating already completed
	// work (causing a race condition).
	// sync index (16-bits) | stage type (16-bits)
	b2AtomicU32 atomicSyncBits;

	// padding to prevent false sharing
	char padding2[64];

	// Race flag claimed by whichever runner reaches b2SolverTask with workerIndex 0 first.
	// The calling thread of b2World_Step also races for this slot so the orchestrator can
	// always make progress, regardless of how the user's task system schedules tasks (out
	// of order, fewer threads than workers, or synchronously inside enqueueTaskFcn). The
	// loser of the race no-ops as workerIndex 0.
	b2AtomicInt mainClaimed;

	// padding to prevent false sharing
	char padding3[64];

} b2StepContext;

static inline b2Softness b2MakeSoft( float hertz, float zeta, float h )
{
	if ( hertz == 0.0f )
	{
		return (b2Softness){
			.biasRate = 0.0f,
			.massScale = 0.0f,
			.impulseScale = 0.0f,
		};
	}

	float omega = 2.0f * B2_PI * hertz;
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / ( 1.0f + a2 );

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	return (b2Softness){
		.biasRate = omega / a1,
		.massScale = a2 * a3,
		.impulseScale = a3,
	};
}

void b2Solve( b2World* world, b2StepContext* stepContext );
