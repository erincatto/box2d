// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"

#include <string.h>

B2_ARRAY_SOURCE( b2SolverSet, b2SolverSet )

void b2DestroySolverSet( b2World* world, int setIndex )
{
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
	b2Array_Destroy( set->bodyIds );
	b2Array_Destroy( set->contactIds );
	b2Array_Destroy( set->jointIds );
	b2IslandSimArray_Destroy( &set->islandSims );
	b2FreeId( &world->solverSetIdPool, setIndex );
	*set = ( b2SolverSet ){ 0 };
	set->setIndex = B2_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
void b2WakeSolverSet( b2World* world, int setIndex )
{
	B2_ASSERT( setIndex >= b2_firstSleepingSet );
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSets, b2_disabledSet );

	b2Body* bodies = world->bodies.data;

	int bodyCount = set->bodyIds.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		int bodyId = set->bodyIds.data[i];
		b2Body* body = bodies + bodyId;
		B2_ASSERT( body->setIndex == setIndex );
		body->setIndex = b2_awakeSet;
		body->localIndex = awakeSet->bodyIds.count;

		// Mark woken body dirty so its touching contacts (which were unlinked from
		// cluster arrays during sleep) get reclassified by b2ReclassifyDirtyConstraints.
		b2SetBitGrow( &world->clusterManager.dirtyBodyBitSet, bodyId );

		// Add back to persistent cluster bodyIds
		if ( body->clusterIndex != B2_NULL_INDEX )
		{
			b2ClusterLinkBody( world, body );
		}

		// Reset sleep timer
		body->sleepTime = 0.0f;

		b2Array_Push( awakeSet->bodyIds, bodyId );

		// move non-touching contacts from disabled set to awake set
		int contactKey = body->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

			contactKey = contact->edges[edgeIndex].nextKey;

			if ( contact->setIndex != b2_disabledSet )
			{
				B2_ASSERT( contact->setIndex == b2_awakeSet || contact->setIndex == setIndex );
				continue;
			}

			int localIndex = contact->localIndex;
			B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 && contact->manifold.pointCount == 0 );
			contact->setIndex = b2_awakeSet;
			contact->localIndex = awakeSet->contactIds.count;
			b2Array_Push( awakeSet->contactIds, contact->contactId );

			// swap-remove from disabled set
			int lastIndex = disabledSet->contactIds.count - 1;
			if ( localIndex < lastIndex )
			{
				int movedId = disabledSet->contactIds.data[lastIndex];
				disabledSet->contactIds.data[localIndex] = movedId;
				b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedId );
				movedContact->localIndex = localIndex;
			}
			disabledSet->contactIds.count -= 1;
		}
	}

	// transfer touching contacts from sleeping set to awake set
	{
		int contactCount = set->contactIds.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			int contactId = set->contactIds.data[i];
			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
			B2_ASSERT( contact->flags & b2_contactTouchingFlag );
			B2_ASSERT( contact->simFlags & b2_simTouchingFlag );
			B2_ASSERT( contact->manifold.pointCount > 0 );
			B2_ASSERT( contact->setIndex == setIndex );
			contact->localIndex = awakeSet->contactIds.count;
			contact->setIndex = b2_awakeSet;
			b2Array_Push( awakeSet->contactIds, contactId );
		}
	}

	// transfer joints from sleeping set to awake set
	{
		int jointCount = set->jointIds.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			int jointId = set->jointIds.data[i];
			b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
			B2_ASSERT( joint->setIndex == setIndex );
			joint->localIndex = awakeSet->jointIds.count;
			joint->setIndex = b2_awakeSet;
			b2Array_Push( awakeSet->jointIds, jointId );
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		int islandCount = set->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set->islandSims.data + i;
			b2Island* island = b2Array_Get( world->islands, islandSrc->islandId );
			island->setIndex = b2_awakeSet;
			island->localIndex = awakeSet->islandSims.count;
			b2IslandSim* islandDst = b2IslandSimArray_Add( &awakeSet->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet( world, setIndex );
}

// Islands need to have a deterministic order because data is moved to a sleeping set according
// to island order.
void b2TrySleepIsland( b2World* world, int islandId )
{
	b2Island* island = b2Array_Get( world->islands, islandId );
	B2_ASSERT( island->setIndex == b2_awakeSet );

	// Cannot put an island to sleep while it has a pending split and more than one body.
	if ( island->constraintRemoveCount > 0 && island->bodies.count > 1 )
	{
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	int sleepSetId = b2AllocId( &world->solverSetIdPool );
	if ( sleepSetId == world->solverSets.count )
	{
		b2SolverSet set = { 0 };
		set.setIndex = B2_NULL_INDEX;
		b2SolverSetArray_Push( &world->solverSets, set );
	}

	b2SolverSet* sleepSet = b2SolverSetArray_Get( &world->solverSets, sleepSetId );
	*sleepSet = ( b2SolverSet ){ 0 };

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	B2_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islandSims.count );

	sleepSet->setIndex = sleepSetId;
	b2Array_CreateN( sleepSet->bodyIds, island->bodies.count );
	b2Array_CreateN( sleepSet->contactIds, island->contacts.count );
	b2Array_CreateN( sleepSet->jointIds, island->joints.count );

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSets, b2_disabledSet );
		for (int i = 0; i < island->bodies.count; ++i)
		{
			int bodyId = island->bodies.data[i];
			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->islandId == islandId );
			B2_ASSERT( body->islandIndex == i );

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if ( body->bodyMoveIndex != B2_NULL_INDEX )
			{
				// todo bodyMoveIndex is a bit wasteful
				// Initially bodyMoveIndex == localIndex, but these become different with
				// calls to b2RemoveBodyFromSet. A two pass approach would fix this or may a temporary buffer.
				b2BodyMoveEvent* moveEvent = b2BodyMoveEventArray_Get( &world->bodyMoveEvents, body->bodyMoveIndex );
				B2_ASSERT( moveEvent->bodyId.index1 - 1 == bodyId );
				B2_ASSERT( moveEvent->bodyId.generation == body->generation );
				moveEvent->fellAsleep = true;
				body->bodyMoveIndex = B2_NULL_INDEX;
			}

			// Remove from persistent cluster bodyIds before sleeping
			if ( body->clusterLocalIndex != B2_NULL_INDEX )
			{
				b2ClusterUnlinkBody( world, body );
			}

			int awakeBodyIndex = body->localIndex;

			// move body sim to sleep set
			int sleepBodyIndex = sleepSet->bodyIds.count;
			b2Array_Push( sleepSet->bodyIds, bodyId );

			b2RemoveBodyFromSet( awakeSet, &world->bodies, awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

				B2_ASSERT( contact->setIndex == b2_awakeSet || contact->setIndex == b2_disabledSet );
				contactKey = contact->edges[edgeIndex].nextKey;

				if ( contact->setIndex == b2_disabledSet )
				{
					// already moved to disabled set by another body in the island
					continue;
				}

				if ( ( contact->flags & b2_contactTouchingFlag ) != 0 )
				{
					// contact is touching and will be moved separately
					continue;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = b2BodyArray_Get( &world->bodies, otherBodyId );
				if ( otherBody->setIndex == b2_awakeSet )
				{
					continue;
				}

				int localIndex = contact->localIndex;
				B2_ASSERT( contact->manifold.pointCount == 0 );
				B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 );

				// move the non-touching contact to the disabled set
				contact->setIndex = b2_disabledSet;
				contact->localIndex = disabledSet->contactIds.count;
				b2Array_Push( disabledSet->contactIds, contact->contactId );

				// swap-remove from awake set
				int lastIndex = awakeSet->contactIds.count - 1;
				if ( localIndex < lastIndex )
				{
					int movedId = awakeSet->contactIds.data[lastIndex];
					awakeSet->contactIds.data[localIndex] = movedId;
					b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedId );
					movedContact->localIndex = localIndex;
				}
				awakeSet->contactIds.count -= 1;
			}
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		for ( int i = 0; i < island->contacts.count; ++i )
		{
			b2ContactLink* link = island->contacts.data + i;
			b2Contact* contact = b2ContactArray_Get( &world->contacts, link->contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );
			B2_ASSERT( contact->islandId == islandId );

			// Remove from persistent cluster arrays before sleeping
			b2ClusterUnlinkContact( world, contact->contactId );

			int localIndex = contact->localIndex;

			int sleepContactIndex = sleepSet->contactIds.count;
			b2Array_Push( sleepSet->contactIds, contact->contactId );

			// swap-remove from awake set
			int lastIndex = awakeSet->contactIds.count - 1;
			if ( localIndex < lastIndex )
			{
				int movedId = awakeSet->contactIds.data[lastIndex];
				awakeSet->contactIds.data[localIndex] = movedId;
				b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedId );
				movedContact->localIndex = localIndex;
			}
			awakeSet->contactIds.count -= 1;

			contact->setIndex = sleepSetId;
			contact->localIndex = sleepContactIndex;
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		for ( int i = 0; i < island->joints.count; ++i )
		{
			b2JointLink* link = island->joints.data + i;
			b2Joint* joint = b2JointArray_Get( &world->joints, link->jointId );
			B2_ASSERT( joint->setIndex == b2_awakeSet );
			B2_ASSERT( joint->islandId == islandId );

			// Remove from persistent cluster arrays before sleeping
			b2ClusterUnlinkJoint( world, joint->jointId );

			int localIndex = joint->localIndex;

			int sleepJointIndex = sleepSet->jointIds.count;
			b2Array_Push( sleepSet->jointIds, joint->jointId );

			// swap-remove from awake set
			int lastIndex = awakeSet->jointIds.count - 1;
			if ( localIndex < lastIndex )
			{
				int movedId = awakeSet->jointIds.data[lastIndex];
				awakeSet->jointIds.data[localIndex] = movedId;
				b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedId );
				B2_ASSERT( movedJoint->localIndex == lastIndex );
				movedJoint->localIndex = localIndex;
			}
			awakeSet->jointIds.count -= 1;

			joint->setIndex = sleepSetId;
			joint->localIndex = sleepJointIndex;
		}
	}

	// move island struct
	{
		B2_ASSERT( island->setIndex == b2_awakeSet );

		int islandIndex = island->localIndex;
		b2IslandSim* sleepIsland = b2IslandSimArray_Add( &sleepSet->islandSims );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b2IslandSimArray_RemoveSwap( &awakeSet->islandSims, islandIndex );
		if ( movedIslandIndex != B2_NULL_INDEX )
		{
			// fix index on moved element
			b2IslandSim* movedIslandSim = awakeSet->islandSims.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b2Island* movedIsland = b2Array_Get( world->islands, movedIslandId );
			B2_ASSERT( movedIsland->localIndex == movedIslandIndex );
			movedIsland->localIndex = islandIndex;
		}

		island->setIndex = sleepSetId;
		island->localIndex = 0;
	}

	if (world->splitIslandId == islandId)
	{
		world->splitIslandId = B2_NULL_INDEX;
	}

	b2ValidateSolverSets( world );
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
void b2MergeSolverSets( b2World* world, int setId1, int setId2 )
{
	B2_ASSERT( setId1 >= b2_firstSleepingSet );
	B2_ASSERT( setId2 >= b2_firstSleepingSet );
	b2SolverSet* set1 = b2SolverSetArray_Get( &world->solverSets, setId1 );
	b2SolverSet* set2 = b2SolverSetArray_Get( &world->solverSets, setId2 );

	// Move the fewest number of bodies
	if ( set1->bodyIds.count < set2->bodyIds.count )
	{
		b2SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// transfer bodies from set2 to set1
	{
		b2Body* bodies = world->bodies.data;
		int bodyCount = set2->bodyIds.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			int bodyId = set2->bodyIds.data[i];

			b2Body* body = bodies + bodyId;
			B2_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->bodyIds.count;
			b2Array_Push( set1->bodyIds, bodyId );
		}
	}

	// transfer contacts
	{
		int contactCount = set2->contactIds.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			int contactId = set2->contactIds.data[i];
			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
			B2_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contactIds.count;
			b2Array_Push( set1->contactIds, contactId );
		}
	}

	// transfer joints
	{
		int jointCount = set2->jointIds.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			int jointId = set2->jointIds.data[i];

			b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
			B2_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->jointIds.count;

			b2Array_Push( set1->jointIds, jointId );
		}
	}

	// transfer islands
	{
		int islandCount = set2->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set2->islandSims.data + i;
			int islandId = islandSrc->islandId;

			b2Island* island = b2Array_Get( world->islands, islandId );
			island->setIndex = setId1;
			island->localIndex = set1->islandSims.count;

			b2IslandSim* islandDst = b2IslandSimArray_Add( &set1->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// destroy the merged set
	b2DestroySolverSet( world, setId2 );

	b2ValidateSolverSets( world );
}

void b2TransferBody( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Body* body )
{
	if (targetSet == sourceSet)
	{
		return;
	}

	int sourceIndex = body->localIndex;

	int targetIndex = targetSet->bodyIds.count;
	b2Array_Push( targetSet->bodyIds, body->id );

	body->flags &= ~(b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact);

	// Remove body index from source solver set
	b2RemoveBodyFromSet( sourceSet, &world->bodies, sourceIndex );

	body->setIndex = targetSet->setIndex;
	body->localIndex = targetIndex;
}

void b2TransferJoint( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Joint* joint )
{
	if (targetSet == sourceSet)
	{
		return;
	}

	int sourceIndex = joint->localIndex;

	int targetIndex = targetSet->jointIds.count;
	b2Array_Push( targetSet->jointIds, joint->jointId );

	// swap-remove from source set
	int lastIndex = sourceSet->jointIds.count - 1;
	if ( sourceIndex < lastIndex )
	{
		int movedId = sourceSet->jointIds.data[lastIndex];
		sourceSet->jointIds.data[sourceIndex] = movedId;
		b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedId );
		movedJoint->localIndex = sourceIndex;
	}
	sourceSet->jointIds.count -= 1;

	joint->setIndex = targetSet->setIndex;
	joint->localIndex = targetIndex;
}
