// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

typedef struct b2World b2World;

typedef struct b2Cluster
{
	b2IntArray bodyIndices;
	b2Vec2 center;
	b2Vec2 accumulator;
} b2Cluster;

typedef struct b2ClusterManager
{
	b2Cluster clusters[B2_CLUSTER_COUNT];
} b2ClusterManager;

void b2CreateClusters( b2ClusterManager* manager );
void b2DestroyClusters( b2ClusterManager* manager );

void b2ComputeClusters( b2World* world );
