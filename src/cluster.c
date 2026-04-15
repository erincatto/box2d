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
		b2Array_CreateN( cluster->contactIds, 16 );
		b2Array_CreateN( cluster->jointIds, 16 );
	}

	for ( int i = 0; i < B2_MAX_BORDERS; ++i )
	{
		b2PersistentBorder* border = manager->borders + i;
		b2Array_CreateN( border->contactIds, 0 );
		b2Array_CreateN( border->jointIds, 0 );
	}

	manager->dirtyBodyBitSet = b2CreateBitSet( 256 );
}

void b2DestroyClusters( b2ClusterManager* manager )
{
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2Cluster* cluster = manager->clusters + i;
		b2Array_Destroy( cluster->bodyIds );
		b2Array_Destroy( cluster->contactIds );
		b2Array_Destroy( cluster->jointIds );
	}

	for ( int i = 0; i < B2_MAX_BORDERS; ++i )
	{
		b2PersistentBorder* border = manager->borders + i;
		b2Array_Destroy( border->contactIds );
		b2Array_Destroy( border->jointIds );
	}

	b2DestroyBitSet( &manager->dirtyBodyBitSet );
}

void b2ClusterLinkBody( b2World* world, b2Body* body )
{
	B2_ASSERT( body->clusterLocalIndex == B2_NULL_INDEX );
	B2_ASSERT( 0 <= body->clusterIndex && body->clusterIndex < B2_CLUSTER_COUNT );

	b2Cluster* cluster = world->clusterManager.clusters + body->clusterIndex;
	body->clusterLocalIndex = cluster->bodyIds.count;
	b2Array_Push( cluster->bodyIds, body->id );
}

void b2ClusterUnlinkBody( b2World* world, b2Body* body )
{
	B2_ASSERT( body->clusterLocalIndex != B2_NULL_INDEX );
	B2_ASSERT( 0 <= body->clusterIndex && body->clusterIndex < B2_CLUSTER_COUNT );

	b2Cluster* cluster = world->clusterManager.clusters + body->clusterIndex;
	int localIndex = body->clusterLocalIndex;
	B2_ASSERT( 0 <= localIndex && localIndex < cluster->bodyIds.count );
	B2_ASSERT( cluster->bodyIds.data[localIndex] == body->id );

	int movedIndex = b2Array_RemoveSwap( cluster->bodyIds, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		int movedBodyId = cluster->bodyIds.data[localIndex];
		b2Body* movedBody = b2BodyArray_Get( &world->bodies, movedBodyId );
		B2_ASSERT( movedBody->clusterLocalIndex == movedIndex );
		movedBody->clusterLocalIndex = localIndex;
	}

	body->clusterLocalIndex = B2_NULL_INDEX;
}

void b2FindAndLinkBodyCluster( b2World* world, b2Body* body )
{
	B2_ASSERT( body->clusterIndex == B2_NULL_INDEX );
	B2_ASSERT( body->clusterLocalIndex == B2_NULL_INDEX );

	b2Cluster* clusters = world->clusterManager.clusters;
	b2Vec2 center = body->center;
	float minDistSqr = b2DistanceSquared( center, clusters[0].center );
	int bestIndex = 0;

	for ( int j = 1; j < B2_CLUSTER_COUNT; ++j )
	{
		float distSqr = b2DistanceSquared( center, clusters[j].center );
		if ( distSqr < minDistSqr )
		{
			bestIndex = j;
			minDistSqr = distSqr;
		}
	}

	body->clusterIndex = (int8_t)bestIndex;
	b2ClusterLinkBody( world, body );
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
		int seedCount = b2MinInt( awakeCount, B2_CLUSTER_COUNT );
		for ( int i = 0; i < seedCount; ++i )
		{
			int bodyId = awakeSet->bodyIds.data[i];
			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
			clusters[i].center = body->center;
		}

		manager->initialized = true;

		// Bootstrap: link all existing awake bodies now that centers are seeded
		for ( int i = 0; i < awakeCount; ++i )
		{
			int bodyId = awakeSet->bodyIds.data[i];
			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
			b2FindAndLinkBodyCluster( world, body );
			b2SetBitGrow( &manager->dirtyBodyBitSet, bodyId );
		}
	}
	else
	{
		// Move dirty bodies that changed cluster during the previous step's b2FinalizeBodiesTask.
		// b2FinalizeBodiesTask updates body->clusterIndex to the new cluster and stores the old value
		// in body->previousClusterIndex. Use previousClusterIndex to unlink from the old cluster.
		b2BitSet* dirtyBits = &manager->dirtyBodyBitSet;
		uint32_t blockCount = dirtyBits->blockCount;
		uint64_t* bits = dirtyBits->bits;

		for ( uint32_t k = 0; k < blockCount; ++k )
		{
			uint64_t word = bits[k];
			while ( word != 0 )
			{
				uint32_t ctz = b2CTZ64( word );
				int bodyId = (int)( 64 * k + ctz );

				b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );

				if ( body->clusterLocalIndex != B2_NULL_INDEX )
				{
					// Body is in the old cluster's bodyIds but needs to move.
					// Temporarily swap to previousClusterIndex for unlinking.
					int8_t newCluster = body->clusterIndex;
					body->clusterIndex = body->previousClusterIndex;
					b2ClusterUnlinkBody( world, body );

					body->clusterIndex = newCluster;
					b2ClusterLinkBody( world, body );
				}

				word = word & ( word - 1 );
			}
		}
	}

	// Re-seed empty clusters
	int* bodyIds = awakeSet->bodyIds.data;
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

// Compute the cluster slot for a constraint between two bodies.
// Returns cluster index (0..15) for interior, or B2_CLUSTER_COUNT + flatBorderIndex for border.
static int b2ComputeClusterSlot( b2Body* bodyA, b2Body* bodyB )
{
	if ( bodyA->type == b2_staticBody )
	{
		return bodyB->clusterIndex;
	}

	if ( bodyB->type == b2_staticBody )
	{
		return bodyA->clusterIndex;
	}

	int clusterA = bodyA->clusterIndex;
	int clusterB = bodyB->clusterIndex;

	if ( clusterA == clusterB )
	{
		return clusterA;
	}

	int a = clusterA < clusterB ? clusterA : clusterB;
	int b = clusterA < clusterB ? clusterB : clusterA;
	return B2_CLUSTER_COUNT + b2GetBorderIndex( a, b );
}

void b2ClusterLinkContact( b2World* world, int contactId, int bodyIdA, int bodyIdB )
{
	b2ClusterManager* manager = &world->clusterManager;
	b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

	// If either non-static body hasn't been assigned to a cluster yet, skip classification.
	// The body will be marked dirty when b2ComputeClusters assigns it, and
	// b2ReclassifyDirtyConstraints will pick up this contact.
	if ( ( bodyA->type != b2_staticBody && bodyA->clusterIndex == B2_NULL_INDEX ) ||
		 ( bodyB->type != b2_staticBody && bodyB->clusterIndex == B2_NULL_INDEX ) )
	{
		return;
	}

	int slot = b2ComputeClusterSlot( bodyA, bodyB );

	if ( slot < B2_CLUSTER_COUNT )
	{
		// Interior constraint
		b2Cluster* cluster = manager->clusters + slot;
		contact->clusterSlot = (int16_t)slot;
		contact->clusterLocalIndex = cluster->contactIds.count;
		b2Array_Push( cluster->contactIds, contactId );
	}
	else
	{
		// Border constraint
		int flatIdx = slot - B2_CLUSTER_COUNT;
		b2PersistentBorder* border = manager->borders + flatIdx;
		contact->clusterSlot = (int16_t)slot;
		contact->clusterLocalIndex = border->contactIds.count;
		b2Array_Push( border->contactIds, contactId );
	}
}

void b2ClusterUnlinkContact( b2World* world, int contactId )
{
	b2ClusterManager* manager = &world->clusterManager;
	b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

	int slot = contact->clusterSlot;
	if ( slot == B2_CLUSTER_SLOT_NONE )
	{
		return;
	}

	int localIndex = contact->clusterLocalIndex;

	if ( slot < B2_CLUSTER_COUNT )
	{
		b2Cluster* cluster = manager->clusters + slot;
		B2_ASSERT( 0 <= localIndex && localIndex < cluster->contactIds.count );
		B2_ASSERT( cluster->contactIds.data[localIndex] == contactId );

		int movedIndex = b2Array_RemoveSwap( cluster->contactIds, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// Fix back-reference on the contact that was swapped in
			int movedContactId = cluster->contactIds.data[localIndex];
			b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactId );
			B2_ASSERT( movedContact->clusterLocalIndex == movedIndex );
			movedContact->clusterLocalIndex = localIndex;
		}
	}
	else
	{
		int flatIdx = slot - B2_CLUSTER_COUNT;
		b2PersistentBorder* border = manager->borders + flatIdx;
		B2_ASSERT( 0 <= localIndex && localIndex < border->contactIds.count );
		B2_ASSERT( border->contactIds.data[localIndex] == contactId );

		int movedIndex = b2Array_RemoveSwap( border->contactIds, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			int movedContactId = border->contactIds.data[localIndex];
			b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactId );
			B2_ASSERT( movedContact->clusterLocalIndex == movedIndex );
			movedContact->clusterLocalIndex = localIndex;
		}
	}

	contact->clusterSlot = B2_CLUSTER_SLOT_NONE;
	contact->clusterLocalIndex = B2_CLUSTER_SLOT_NONE;
}

void b2ClusterLinkJoint( b2World* world, int jointId, int bodyIdA, int bodyIdB )
{
	b2ClusterManager* manager = &world->clusterManager;
	b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

	// If either non-static body hasn't been assigned to a cluster yet (e.g. joint created
	// before first b2ComputeClusters), skip classification. The body will be marked dirty
	// when b2ComputeClusters assigns it, and b2ReclassifyDirtyConstraints will pick up this joint.
	if ( ( bodyA->type != b2_staticBody && bodyA->clusterIndex == B2_NULL_INDEX ) ||
		 ( bodyB->type != b2_staticBody && bodyB->clusterIndex == B2_NULL_INDEX ) )
	{
		return;
	}

	int slot = b2ComputeClusterSlot( bodyA, bodyB );

	if ( slot < B2_CLUSTER_COUNT )
	{
		b2Cluster* cluster = manager->clusters + slot;
		joint->clusterSlot = (int16_t)slot;
		joint->clusterLocalIndex = cluster->jointIds.count;
		b2Array_Push( cluster->jointIds, jointId );
	}
	else
	{
		int flatIdx = slot - B2_CLUSTER_COUNT;
		b2PersistentBorder* border = manager->borders + flatIdx;
		joint->clusterSlot = (int16_t)slot;
		joint->clusterLocalIndex = border->jointIds.count;
		b2Array_Push( border->jointIds, jointId );
	}
}

void b2ClusterUnlinkJoint( b2World* world, int jointId )
{
	b2ClusterManager* manager = &world->clusterManager;
	b2Joint* joint = b2JointArray_Get( &world->joints, jointId );

	int slot = joint->clusterSlot;
	if ( slot == B2_CLUSTER_SLOT_NONE )
	{
		return;
	}

	int localIndex = joint->clusterLocalIndex;

	if ( slot < B2_CLUSTER_COUNT )
	{
		b2Cluster* cluster = manager->clusters + slot;
		B2_ASSERT( 0 <= localIndex && localIndex < cluster->jointIds.count );
		B2_ASSERT( cluster->jointIds.data[localIndex] == jointId );

		int movedIndex = b2Array_RemoveSwap( cluster->jointIds, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			int movedJointId = cluster->jointIds.data[localIndex];
			b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedJointId );
			B2_ASSERT( movedJoint->clusterLocalIndex == movedIndex );
			movedJoint->clusterLocalIndex = localIndex;
		}
	}
	else
	{
		int flatIdx = slot - B2_CLUSTER_COUNT;
		b2PersistentBorder* border = manager->borders + flatIdx;
		B2_ASSERT( 0 <= localIndex && localIndex < border->jointIds.count );
		B2_ASSERT( border->jointIds.data[localIndex] == jointId );

		int movedIndex = b2Array_RemoveSwap( border->jointIds, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			int movedJointId = border->jointIds.data[localIndex];
			b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedJointId );
			B2_ASSERT( movedJoint->clusterLocalIndex == movedIndex );
			movedJoint->clusterLocalIndex = localIndex;
		}
	}

	joint->clusterSlot = B2_CLUSTER_SLOT_NONE;
	joint->clusterLocalIndex = B2_CLUSTER_SLOT_NONE;
}

void b2ReclassifyDirtyConstraints( b2World* world, b2StepContext* context )
{
	B2_UNUSED( context );
	b2ClusterManager* manager = &world->clusterManager;
	b2BitSet* dirtyBits = &manager->dirtyBodyBitSet;

	uint32_t blockCount = dirtyBits->blockCount;
	uint64_t* bits = dirtyBits->bits;

	for ( uint32_t k = 0; k < blockCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = b2CTZ64( word );
			int bodyId = (int)( 64 * k + ctz );

			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );

			// Walk contact linked list
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int edgeIndex = contactKey & 1;
				int contactId = contactKey >> 1;

				b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
				contactKey = contact->edges[edgeIndex].nextKey;

				// Only reclassify touching contacts (non-touching have no cluster assignment)
				if ( ( contact->flags & b2_contactTouchingFlag ) == 0 )
				{
					continue;
				}

				// Double-visit prevention: when both bodies are dirty, only process
				// from the body with the lower id.
				int otherBodyId = contact->edges[edgeIndex ^ 1].bodyId;
				if ( b2GetBit( dirtyBits, otherBodyId ) && bodyId > otherBodyId )
				{
					continue;
				}

				b2Body* bodyA = b2BodyArray_Get( &world->bodies, contact->edges[0].bodyId );
				b2Body* bodyB = b2BodyArray_Get( &world->bodies, contact->edges[1].bodyId );
				int newSlot = b2ComputeClusterSlot( bodyA, bodyB );

				if ( newSlot != contact->clusterSlot )
				{
					// Remove from old slot (if classified)
					if ( contact->clusterSlot != B2_CLUSTER_SLOT_NONE )
					{
						b2ClusterUnlinkContact( world, contactId );
					}

					// Add to new slot
					if ( newSlot < B2_CLUSTER_COUNT )
					{
						b2Cluster* cluster = manager->clusters + newSlot;
						contact->clusterSlot = (int16_t)newSlot;
						contact->clusterLocalIndex = cluster->contactIds.count;
						b2Array_Push( cluster->contactIds, contactId );
					}
					else
					{
						int flatIdx = newSlot - B2_CLUSTER_COUNT;
						b2PersistentBorder* border = manager->borders + flatIdx;
						contact->clusterSlot = (int16_t)newSlot;
						contact->clusterLocalIndex = border->contactIds.count;
						b2Array_Push( border->contactIds, contactId );
					}
				}
			}

			// Walk joint linked list
			int jointKey = body->headJointKey;
			while ( jointKey != B2_NULL_INDEX )
			{
				int edgeIndex = jointKey & 1;
				int jointId = jointKey >> 1;

				b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
				jointKey = joint->edges[edgeIndex].nextKey;

				// Double-visit prevention
				int otherBodyId = joint->edges[edgeIndex ^ 1].bodyId;
				if ( b2GetBit( dirtyBits, otherBodyId ) && bodyId > otherBodyId )
				{
					continue;
				}

				b2Body* bodyA = b2BodyArray_Get( &world->bodies, joint->edges[0].bodyId );
				b2Body* bodyB = b2BodyArray_Get( &world->bodies, joint->edges[1].bodyId );
				int newSlot = b2ComputeClusterSlot( bodyA, bodyB );

				if ( newSlot != joint->clusterSlot )
				{
					if ( joint->clusterSlot != B2_CLUSTER_SLOT_NONE )
					{
						b2ClusterUnlinkJoint( world, jointId );
					}

					if ( newSlot < B2_CLUSTER_COUNT )
					{
						b2Cluster* cluster = manager->clusters + newSlot;
						joint->clusterSlot = (int16_t)newSlot;
						joint->clusterLocalIndex = cluster->jointIds.count;
						b2Array_Push( cluster->jointIds, jointId );
					}
					else
					{
						int flatIdx = newSlot - B2_CLUSTER_COUNT;
						b2PersistentBorder* border = manager->borders + flatIdx;
						joint->clusterSlot = (int16_t)newSlot;
						joint->clusterLocalIndex = border->jointIds.count;
						b2Array_Push( border->jointIds, jointId );
					}
				}
			}

			// Clear the lowest set bit
			word = word & ( word - 1 );
		}
	}

	// Clear the dirty bitset for next step
	b2SetBitCountAndClear( dirtyBits, dirtyBits->blockCount * 64 );
}

void b2BuildSolveData( b2World* world, b2StepContext* context )
{
	b2ClusterManager* manager = &world->clusterManager;

	// Build cluster solve data from persistent arrays
	int stateIndex = 0;
	for ( int i = 0; i < B2_CLUSTER_COUNT; ++i )
	{
		b2ClusterSolveData* cd = context->clusterData + i;
		b2Cluster* cluster = manager->clusters + i;

		int cc = cluster->contactIds.count;
		int jc = cluster->jointIds.count;

		cd->contactIds = cluster->contactIds.data;
		cd->contactCount = cc;

		cd->jointIds = cluster->jointIds.data;
		cd->jointCount = jc;

		cd->contactConstraints =
			( cc > 0 ) ? b2AllocateArenaItem( &world->arena, cc * sizeof( b2ContactConstraint ), "cluster contact constraints" )
					   : NULL;

		// Per-cluster body data
		int bodyCount = cluster->bodyIds.count;
		cd->bodyCount = bodyCount;
		cd->bodyIds = cluster->bodyIds.data;
		cd->states = context->states + stateIndex;
		stateIndex += bodyCount;

		b2AtomicStoreInt( &cd->solveComplete, 0 );
	}

	// Count non-empty borders
	int borderCount = 0;
	for ( int i = 0; i < B2_MAX_BORDERS; ++i )
	{
		b2PersistentBorder* pb = manager->borders + i;
		if ( pb->contactIds.count > 0 || pb->jointIds.count > 0 )
		{
			borderCount += 1;
		}
	}

	context->borderCount = borderCount;
	context->borders = ( borderCount > 0 ) ? b2AllocateArenaItem( &world->arena, borderCount * sizeof( b2BorderConstraints ),
																  "border constraints" )
										   : NULL;

	// Build border solve data
	int borderWriteIndex = 0;
	for ( int a = 0; a < B2_CLUSTER_COUNT; ++a )
	{
		for ( int b = a + 1; b < B2_CLUSTER_COUNT; ++b )
		{
			int flatIdx = b2GetBorderIndex( a, b );
			b2PersistentBorder* pb = manager->borders + flatIdx;
			int cc = pb->contactIds.count;
			int jc = pb->jointIds.count;

			if ( cc == 0 && jc == 0 )
			{
				continue;
			}

			b2BorderConstraints* border = context->borders + borderWriteIndex;
			border->clusterA = a;
			border->clusterB = b;

			border->contactIds = pb->contactIds.data;
			border->contactCount = cc;

			border->jointIds = pb->jointIds.data;
			border->jointCount = jc;

			border->contactConstraints = ( cc > 0 ) ? b2AllocateArenaItem( &world->arena, cc * sizeof( b2ContactConstraint ),
																		   "border contact constraints" )
													: NULL;

			borderWriteIndex += 1;
		}
	}
}
