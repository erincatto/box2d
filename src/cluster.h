// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "atomic.h"
#include "bitset.h"
#include "container.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

typedef struct b2BodyState b2BodyState;
typedef struct b2ContactConstraint b2ContactConstraint;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

b2DeclareArray( b2BodyState );

// Maximum number of cluster pair borders: C(16,2) = 120
#define B2_MAX_BORDERS ( B2_CLUSTER_COUNT * ( B2_CLUSTER_COUNT - 1 ) / 2 )

typedef struct b2Cluster
{
	b2ArrayC( int ) bodyIds;

	// Persistent arrays of touching contact/joint IDs classified as interior to this cluster
	b2ArrayC( int ) contactIds;
	b2ArrayC( int ) jointIds;

	b2Vec2 center;
	b2Vec2 accumulator;
	int stateOffset;
} b2Cluster;

// Persistent border constraint storage between two clusters
typedef struct b2PersistentBorder
{
	b2ArrayC( int ) contactIds;
	b2ArrayC( int ) jointIds;
} b2PersistentBorder;

// Interior constraints for one cluster, allocated from the arena each step
typedef struct b2ClusterSolveData
{
	int* contactIds;
	b2ContactConstraint* contactConstraints;
	int contactCount;

	int* jointIds;
	int jointCount;

	// Pointer to cluster's body ids
	int* bodyIds;

	// Pointer to sub-array in b2StepContext::states
	b2BodyState* states;
	int bodyCount;

	char dummy1[64];

	// Signaled by the worker when this cluster's solve phase is done
	b2AtomicInt solveComplete;

	// Signaled by the worker when this cluster's prepare phase is done
	b2AtomicInt prepareComplete;

	// Signaled by the worker when this cluster's warm start phase is done
	b2AtomicInt warmStartComplete;
} b2ClusterSolveData;

// Border constraints between two clusters (clusterA < clusterB)
typedef struct b2BorderConstraints
{
	int clusterA;
	int clusterB;

	int* contactIds;
	int contactCount;

	int* jointIds;
	int jointCount;

	b2ContactConstraint* contactConstraints;
} b2BorderConstraints;

typedef struct b2ClusterManager
{
	b2Cluster clusters[B2_CLUSTER_COUNT];
	b2PersistentBorder borders[B2_MAX_BORDERS];

	// Bodies whose clusterIndex changed since last reclassification
	b2BitSet dirtyBodyBitSet;

	bool initialized;
	int bitCapacity;
} b2ClusterManager;

void b2CreateClusters( b2ClusterManager* manager );
void b2DestroyClusters( b2ClusterManager* manager );

void b2ComputeClusters( b2World* world );

// Cluster slot encoding for back-references on contacts/joints:
// 0..B2_CLUSTER_COUNT-1: interior constraint in that cluster
// B2_CLUSTER_COUNT..B2_CLUSTER_COUNT+B2_MAX_BORDERS-1: border constraint at flat index (slot - B2_CLUSTER_COUNT)
// -1: not classified
#define B2_CLUSTER_SLOT_NONE ( -1 )

// Link/unlink a touching contact into/from the persistent cluster arrays
void b2ClusterLinkContact( b2World* world, int contactId, int bodyIdA, int bodyIdB );
void b2ClusterUnlinkContact( b2World* world, int contactId );

// Link/unlink a joint into/from the persistent cluster arrays
void b2ClusterLinkJoint( b2World* world, int jointId, int bodyIdA, int bodyIdB );
void b2ClusterUnlinkJoint( b2World* world, int jointId );

// Reclassify constraints for bodies that changed cluster membership.
// Only touches constraints of dirty bodies. Clears the dirty bitset.
void b2ReclassifyDirtyConstraints( b2World* world, b2StepContext* context );

// Build transient arena-allocated solve data from persistent cluster arrays.
void b2BuildSolveData( b2World* world, b2StepContext* context );
