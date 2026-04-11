// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "cluster.h"

#include "arena_allocator.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "ctz.h"
#include "joint.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

#include <string.h>

void b2CreateClusters( b2ClusterManager* manager )
{
	*manager = (b2ClusterManager){ 0 };

	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2Cluster* cluster = manager->clusters + i;
		b2Array_CreateN( cluster->bodyIds, 16 );
	}
}

void b2DestroyClusters( b2ClusterManager* manager )
{
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2Cluster* cluster = manager->clusters + i;
		b2Array_Destroy( cluster->bodyIds );
	}
}

void b2ComputeClusters( b2World* world )
{
	b2ClusterManager* manager = &world->clusterManager;
	b2Cluster* clusters = manager->clusters;

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	int awakeCount = awakeSet->bodyIds.count;
	if ( awakeCount == 0 )
	{
		return;
	}

	if ( manager->initialized == false )
	{
		int seedCount = b2MinInt(awakeCount, B2_CLUSTER_COUNT);
		for ( int i = 0; i < seedCount; ++i )
		{
			int bodyId = awakeSet->bodyIds.data[i];
			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
			clusters[i].center = body->center;
		}

		manager->initialized = true;
	}

	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		clusters[i].bodyIds.count = 0;
	}

	// Populate clusters
	// Possible optimizations:
	// 1. store the cluster index with the body ids
	// 2. assign bodies to rolling clusters at creation, bumping to a new cluster after every N bodies created
	int* bodyIds = awakeSet->bodyIds.data;
	for ( int i = 0; i < awakeCount; ++i )
	{
		b2Body* body = b2BodyArray_Get( &world->bodies, bodyIds[i] );
		int clusterIndex = body->clusterIndex;

		if (clusterIndex == B2_NULL_INDEX)
		{
			b2Vec2 center = body->center;
			float minDistSqr = b2DistanceSquared( center, clusters[0].center );
			clusterIndex = 0;

			for ( int j = 1; j < B2_CLUSTER_COUNT; ++j )
			{
				float distSqr = b2DistanceSquared( center, clusters[j].center );
				if ( distSqr < minDistSqr )
				{
					clusterIndex = j;
					minDistSqr = distSqr;
				}
			}

			body->clusterIndex = (int16_t)clusterIndex;
		}

		B2_ASSERT( 0 <= clusterIndex && clusterIndex < B2_CLUSTER_COUNT );
		b2Array_Push( clusters[clusterIndex].bodyIds, body->id );
	}

	// Re-seed empty clusters
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		int count = clusters[i].bodyIds.count;
		if ( count == 0 && awakeCount >= B2_CLUSTER_COUNT )
		{
			// Re-seed empty cluster from the body furthest from its assigned center
			// todo consider choosing an arbitrary yet deterministic body to avoid iteration
			float maxDistanceSquared = -1.0f;
			b2Body* maxBody = NULL;
			for ( int b = 0; b < awakeCount; ++b )
			{
				int bodyId = bodyIds[b];
				b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
				int ci = body->clusterIndex;
				float d = b2DistanceSquared( body->center, clusters[ci].center );
				if ( d > maxDistanceSquared )
				{
					maxDistanceSquared = d;
					maxBody = body;
				}
			}

			if ( maxBody != NULL )
			{
				clusters[i].center = maxBody->center;
			}
		}
	}

	// Compute state offsets for each cluster. Aids in parallel state population.
	int stateOffset = 0;
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		clusters[i].stateOffset = stateOffset;
		stateOffset += clusters[i].bodyIds.count;
	}

	B2_ASSERT( stateOffset == awakeCount );
}

// Convert a pair (a, b) with a < b into a linear border index
static inline int b2GetBorderIndex( int a, int b )
{
	B2_ASSERT( a < b );
	// Row-major upper triangular index: sum of (B2_CLUSTER_COUNT - 1) + ... + (B2_CLUSTER_COUNT - a) + (b - a - 1)
	return a * ( 2 * B2_CLUSTER_COUNT - a - 1 ) / 2 + ( b - a - 1 );
}

void b2ClassifyConstraints( b2World* world, b2StepContext* context )
{
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );

	// Temporary counts for sizing
	int clusterContactCounts[B2_CLUSTER_COUNT] = { 0 };
	int clusterJointCounts[B2_CLUSTER_COUNT] = { 0 };

	// Use a flat array for border counts, indexed by b2GetBorderIndex
	int borderContactCounts[B2_MAX_BORDERS] = { 0 };
	int borderJointCounts[B2_MAX_BORDERS] = { 0 };

	// First pass: count contacts per cluster/border
	{
		int awakeContactCount = awakeSet->contactSims.count;
		b2ContactSim* awakeContactSims = awakeSet->contactSims.data;

		for ( int i = 0; i < awakeContactCount; ++i )
		{
			b2ContactSim* contactSim = awakeContactSims + i;

			// Skip non-touching contacts
			if ( contactSim->manifold.pointCount == 0 )
				continue;

			int idA = contactSim->bodyIdA;
			int idB = contactSim->bodyIdB;
			b2Body* bodyA = b2BodyArray_Get( &world->bodies, idA );
			b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );
			b2BodyType typeA = bodyA->type;
			b2BodyType typeB = bodyB->type;

			if ( typeA == b2_staticBody )
			{
				int clusterIdx = bodyB->clusterIndex;
				clusterContactCounts[clusterIdx] += 1;
			}
			else if ( typeB == b2_staticBody )
			{
				int clusterIdx = bodyA->clusterIndex;
				clusterContactCounts[clusterIdx] += 1;
			}
			else
			{
				int clusterA = bodyA->clusterIndex;
				int clusterB = bodyB->clusterIndex;

				if ( clusterA == clusterB )
				{
					clusterContactCounts[clusterA] += 1;
				}
				else
				{
					int a = clusterA < clusterB ? clusterA : clusterB;
					int b = clusterA < clusterB ? clusterB : clusterA;
					int borderIdx = b2GetBorderIndex( a, b );
					borderContactCounts[borderIdx] += 1;
				}
			}
		}
	}

	// First pass: count joints per cluster/border
	{
		int awakeJointCount = awakeSet->jointSims.count;
		b2JointSim* awakeJointSims = awakeSet->jointSims.data;

		for ( int i = 0; i < awakeJointCount; ++i )
		{
			b2JointSim* jointSim = awakeJointSims + i;
			int bodyIdA = jointSim->bodyIdA;
			int bodyIdB = jointSim->bodyIdB;

			b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
			b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );
			b2BodyType typeA = bodyA->type;
			b2BodyType typeB = bodyB->type;

			if ( typeA == b2_staticBody )
			{
				int clusterIdx = bodyB->clusterIndex;
				clusterJointCounts[clusterIdx] += 1;
			}
			else if ( typeB == b2_staticBody )
			{
				int clusterIdx = bodyA->clusterIndex;
				clusterJointCounts[clusterIdx] += 1;
			}
			else
			{
				int clusterA = bodyA->clusterIndex;
				int clusterB = bodyB->clusterIndex;

				if ( clusterA == clusterB )
				{
					clusterJointCounts[clusterA] += 1;
				}
				else
				{
					int a = clusterA < clusterB ? clusterA : clusterB;
					int b = clusterA < clusterB ? clusterB : clusterA;
					int borderIdx = b2GetBorderIndex( a, b );
					borderJointCounts[borderIdx] += 1;
				}
			}
		}
	}

	// Allocate cluster solve data arrays from arena
	b2ClusterManager* manager = &world->clusterManager;
	int stateIndex = 0;
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2ClusterSolveData* cd = context->clusterData + i;
		b2Cluster* cluster = manager->clusters + i;

		int cc = clusterContactCounts[i];
		int jc = clusterJointCounts[i];

		cd->contacts = ( cc > 0 ) ? b2AllocateArenaItem( &world->arena, cc * sizeof( b2ContactSim* ), "cluster contacts" ) : NULL;
		cd->contactCount = 0;

		cd->joints = ( jc > 0 ) ? b2AllocateArenaItem( &world->arena, jc * sizeof( b2JointSim* ), "cluster joints" ) : NULL;
		cd->jointCount = 0;

		cd->contactConstraints =
			( cc > 0 ) ? b2AllocateArenaItem( &world->arena, cc * sizeof( b2ContactConstraint ), "cluster contact constraints" )
					   : NULL;

		// Allocate per-cluster local body state array for L1 cache locality
		int bodyCount = cluster->bodyIds.count;
		cd->bodyCount = bodyCount;
		cd->bodyIds = cluster->bodyIds.data;
		cd->states = context->states + stateIndex;
		stateIndex += bodyCount;

		b2AtomicStoreInt( &cd->solveComplete, 0 );
	}

	B2_ASSERT( stateIndex == awakeSet->bodyIds.count );

	// Count non-empty borders and allocate
	int borderCount = 0;
	for ( int i = 0; i < B2_MAX_BORDERS; ++i )
	{
		if ( borderContactCounts[i] > 0 || borderJointCounts[i] > 0 )
		{
			borderCount += 1;
		}
	}

	context->borderCount = borderCount;
	context->borders = ( borderCount > 0 ) ? b2AllocateArenaItem( &world->arena, borderCount * sizeof( b2BorderConstraints ),
																  "border constraints" )
										   : NULL;

	// Fill in border data
	int borderWriteIndex = 0;
	for ( int a = 0; a < B2_CLUSTER_COUNT; ++a )
	{
		for ( int b = a + 1; b < B2_CLUSTER_COUNT; ++b )
		{
			int flatIdx = b2GetBorderIndex( a, b );
			int cc = borderContactCounts[flatIdx];
			int jc = borderJointCounts[flatIdx];

			if ( cc == 0 && jc == 0 )
			{
				continue;
			}

			b2BorderConstraints* border = context->borders + borderWriteIndex;
			border->clusterA = a;
			border->clusterB = b;

			border->contacts =
				( cc > 0 ) ? b2AllocateArenaItem( &world->arena, cc * sizeof( b2ContactSim* ), "border contacts" ) : NULL;
			border->contactCount = 0;

			border->joints =
				( jc > 0 ) ? b2AllocateArenaItem( &world->arena, jc * sizeof( b2JointSim* ), "border joints" ) : NULL;
			border->jointCount = 0;

			border->contactConstraints = ( cc > 0 ) ? b2AllocateArenaItem( &world->arena, cc * sizeof( b2ContactConstraint ),
																		   "border contact constraints" )
													: NULL;

			// Store the border write index in the flat array for the second pass
			// Reuse borderContactCounts as a mapping from flat index -> border write index
			borderContactCounts[flatIdx] = borderWriteIndex;
			borderWriteIndex += 1;
		}
	}

	// Second pass: distribute contact pointers to clusters and borders
	{
		int awakeContactCount = awakeSet->contactSims.count;
		b2ContactSim* awakeContactSims = awakeSet->contactSims.data;

		for ( int i = 0; i < awakeContactCount; ++i )
		{
			b2ContactSim* contactSim = awakeContactSims + i;

			// Skip non-touching contacts
			if ( contactSim->manifold.pointCount == 0 )
				continue;

			int idA = contactSim->bodyIdA;
			int idB = contactSim->bodyIdB;
			b2Body* bodyA = b2BodyArray_Get( &world->bodies, idA );
			b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );
			b2BodyType typeA = bodyA->type;
			b2BodyType typeB = bodyB->type;

			if ( typeA == b2_staticBody )
			{
				b2ClusterSolveData* cd = context->clusterData + bodyB->clusterIndex;
				cd->contacts[cd->contactCount++] = contactSim;
			}
			else if ( typeB == b2_staticBody )
			{
				b2ClusterSolveData* cd = context->clusterData + bodyA->clusterIndex;
				cd->contacts[cd->contactCount++] = contactSim;
			}
			else
			{
				int clusterA = bodyA->clusterIndex;
				int clusterB = bodyB->clusterIndex;

				if ( clusterA == clusterB )
				{
					b2ClusterSolveData* cd = context->clusterData + clusterA;
					cd->contacts[cd->contactCount++] = contactSim;
				}
				else
				{
					int a = clusterA < clusterB ? clusterA : clusterB;
					int b = clusterA < clusterB ? clusterB : clusterA;
					int flatIdx = b2GetBorderIndex( a, b );
					// borderContactCounts was repurposed as the mapping
					int bIdx = borderContactCounts[flatIdx];
					b2BorderConstraints* border = context->borders + bIdx;
					border->contacts[border->contactCount++] = contactSim;
				}
			}
		}
	}

	// Second pass: distribute joint pointers to clusters and borders
	{
		int awakeJointCount = awakeSet->jointSims.count;
		b2JointSim* awakeJointSims = awakeSet->jointSims.data;

		for ( int i = 0; i < awakeJointCount; ++i )
		{
			b2JointSim* jointSim = awakeJointSims + i;
			int bodyIdA = jointSim->bodyIdA;
			int bodyIdB = jointSim->bodyIdB;

			b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
			b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

			b2BodyType typeA = bodyA->type;
			b2BodyType typeB = bodyB->type;

			if ( typeA == b2_staticBody )
			{
				b2ClusterSolveData* cd = context->clusterData + bodyB->clusterIndex;
				cd->joints[cd->jointCount++] = jointSim;
			}
			else if ( typeB == b2_staticBody )
			{
				b2ClusterSolveData* cd = context->clusterData + bodyA->clusterIndex;
				cd->joints[cd->jointCount++] = jointSim;
			}
			else
			{
				int clusterA = bodyA->clusterIndex;
				int clusterB = bodyB->clusterIndex;

				if ( clusterA == clusterB )
				{
					b2ClusterSolveData* cd = context->clusterData + clusterA;
					cd->joints[cd->jointCount++] = jointSim;
				}
				else
				{
					int a = clusterA < clusterB ? clusterA : clusterB;
					int b = clusterA < clusterB ? clusterB : clusterA;
					int flatIdx = b2GetBorderIndex( a, b );
					int bIdx = borderContactCounts[flatIdx];
					b2BorderConstraints* border = context->borders + bIdx;
					border->joints[border->jointCount++] = jointSim;
				}
			}
		}
	}
}
