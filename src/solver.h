// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b2BodySim b2BodySim;
typedef struct b2BodyState b2BodyState;
typedef struct b2ContactSim b2ContactSim;
typedef struct b2JointSim b2JointSim;
typedef struct b2World b2World;

typedef struct b2Softness
{
	float biasRate;
	float massScale;
	float impulseScale;
} b2Softness;

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

// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker
// claims a block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention
// on a single block index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver
// iteration) the same block of work is executed once per iteration and the atomic sync index is shared across iterations, so it
// increases monotonically.
typedef struct b2SolverBlock
{
	int startIndex;
	int16_t count;
	int16_t blockType; // b2SolverBlockType
	// todo consider false sharing of this atomic
	_Atomic int syncIndex;
} b2SolverBlock;

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
typedef struct b2SolverStage
{
	b2SolverStageType type;
	b2SolverBlock* blocks;
	int blockCount;
	int colorIndex;
	// todo consider false sharing of this atomic
	_Atomic int completionCount;
} b2SolverStage;

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

	b2Softness jointSoftness;
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

	// Array of fast bodies that need continuous collision handling
	int* fastBodies;
	_Atomic int fastBodyCount;

	// Array of bullet bodies that need continuous collision handling
	int* bulletBodies;
	_Atomic int bulletBodyCount;

	// joint pointers for simplified parallel-for access.
	b2JointSim** joints;

	// contact pointers for simplified parallel-for access.
	// - parallel-for collide with no gaps
	// - parallel-for prepare and store contacts with NULL gaps for SIMD remainders
	// despite being an array of pointers, these are contiguous sub-arrays corresponding
	// to constraint graph colors
	b2ContactSim** contacts;

	struct b2ContactConstraintSIMD* simdContactConstraints;
	int activeColorCount;
	int workerCount;

	b2SolverStage* stages;
	int stageCount;
	bool enableWarmStarting;

	// todo padding to prevent false sharing
	char dummy1[64];

	// sync index (16-bits) | stage type (16-bits)
	_Atomic unsigned int atomicSyncBits;

	char dummy2[64];

} b2StepContext;

static inline b2Softness b2MakeSoft( float hertz, float zeta, float h )
{
	if ( hertz == 0.0f )
	{
		return ( b2Softness ){ 0.0f, 1.0f, 0.0f };
	}

	float omega = 2.0f * b2_pi * hertz;
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / ( 1.0f + a2 );
	return ( b2Softness ){ omega / a1, a2 * a3, a3 };
}

void b2Solve( b2World* world, b2StepContext* stepContext );
