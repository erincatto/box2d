// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b2Body b2Body;
typedef struct b2Contact b2Contact;
typedef struct b2World b2World;

typedef struct b2Softness
{
	float biasRate;
	float massScale;
	float impulseScale;
} b2Softness;

typedef enum b2SolverStageType
{
	b2_stageSolveClusters,
	b2_stageRelaxClusters,
	b2_stageRestitutionClusters,
	b2_stagePrepareClusters,
	b2_stageWarmStartClusters,
} b2SolverStageType;

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
typedef struct b2SolverStage
{
	b2SolverStageType type;
	bool storeImpulses;
	bool integratePositions;
} b2SolverStage;

typedef struct b2ClusterBody
{
	b2Vec2 position;
	int clusterIndex;
} b2ClusterBody;

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

	struct b2BodyState* states;
	b2ClusterBody* clusterBodies;

	// array of all shape ids for shapes that have enlarged AABBs
	int* enlargedShapes;
	int enlargedShapeCount;

	// Array of bullet bodies that need continuous collision handling
	b2Body** bulletBodies;
	b2AtomicInt bulletBodyCount;

	// contact pointers for simplified parallel-for access.
	// - parallel-for collide with no gaps
	// - parallel-for prepare and store contacts with NULL gaps for SIMD remainders
	// despite being an array of pointers, these are contiguous sub-arrays corresponding
	// to constraint graph colors
	b2Contact** contacts;

	int workerCount;

	b2SolverStage* stages;
	int stageCount;
	bool enableWarmStarting;

	// Cluster solver data (transient, allocated from arena each step)
	struct b2ClusterSolveData* clusterData;
	struct b2BorderConstraints* borders;
	int borderCount;
	int clusterWorkerMap[B2_CLUSTER_COUNT];

	// todo padding to prevent false sharing
	char dummy1[64];

	// sync index (16-bits) | stage type (16-bits)
	b2AtomicU32 atomicSyncBits;

	char dummy2[64];

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
