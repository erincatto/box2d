// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "cluster.h"

#include "array.h"
#include "body.h"
#include "physics_world.h"
#include "solver_set.h"

void b2CreateClusters( b2ClusterManager* manager )
{
	*manager = (b2ClusterManager){ 0 };

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2Cluster* cluster = manager->clusters + i;
		cluster->bodyIndices = b2IntArray_Create( 16 );
	}
}

void b2DestroyClusters( b2ClusterManager* manager )
{
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2Cluster* cluster = manager->clusters + i;
		b2IntArray_Destroy( &cluster->bodyIndices );
	}
}

void b2ComputeClusters( b2World* world )
{
	b2Cluster* clusters = world->clusterManager.clusters;

	// Clear
	for (int i = 0; i < B2_CLUSTER_COUNT; ++i)
	{
		b2IntArray_Clear( &clusters[i].bodyIndices );
	}

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	int awakeCount = awakeSet->bodySims.count;
	b2BodySim* bodySims = awakeSet->bodySims.data;

	int seedCount = b2MinInt( awakeCount, B2_CLUSTER_COUNT );
	for (int i = 0; i < seedCount; ++i)
	{
		clusters[i].center = bodySims[i].center;
		b2IntArray_Push( &clusters[i].bodyIndices, i );
		bodySims[i].clusterIndex = i;
	}

	if (awakeCount < B2_CLUSTER_COUNT)
	{
		return;
	}

	for (int iteration = 0; iteration < 32; ++iteration)
	{
		for (int i = 0; i < B2_CLUSTER_COUNT; ++i)
		{
			clusters[i].accumulator = b2Vec2_zero;
			clusters[i].bodyIndices.count = 0;
		}

		for (int i = 0; i < awakeCount; ++i)
		{
			b2Vec2 p = bodySims[i].center;

			float minDistanceSquared = b2DistanceSquared( p, clusters[0].center );
			int bestIndex = 0;

			for (int j = 1; j < B2_CLUSTER_COUNT; ++j)
			{
				float distanceSquared = b2DistanceSquared(p, clusters[j].center);
				if (distanceSquared < minDistanceSquared)
				{
					minDistanceSquared = distanceSquared;
					bestIndex = j;
				}
			}

			bodySims[i].clusterIndex = bestIndex;
			b2IntArray_Push( &clusters[bestIndex].bodyIndices, bestIndex );
			clusters[bestIndex].accumulator = b2Add( clusters[bestIndex].accumulator, p );
		}

		for (int i = 0; i < B2_CLUSTER_COUNT; ++i)
		{
			int clusterBodyCount = clusters[i].bodyIndices.count;
			if ( clusterBodyCount > 0 )
			{
				clusters[i].center = b2MulSV( 1.0f / clusterBodyCount, clusters[i].accumulator );
			}
		}
	}
}
