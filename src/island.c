// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "solver_set.h"
#include "world.h"

#include <stddef.h>

B2_ARRAY_SOURCE( b2Island, b2Island );
B2_ARRAY_SOURCE( b2IslandSim, b2IslandSim );

b2Island* b2CreateIsland( b2World* world, int setIndex )
{
	B2_ASSERT( setIndex == b2_awakeSet || setIndex >= b2_firstSleepingSet );

	int islandId = b2AllocId( &world->islandIdPool );

	if ( islandId == world->islands.count )
	{
		b2Island emptyIsland = { 0 };
		b2IslandArray_Push( &world->islands, emptyIsland );
	}
	else
	{
		B2_ASSERT( world->islands.data[islandId].setIndex == B2_NULL_INDEX );
	}

	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );

	b2Island* island = b2IslandArray_Get( &world->islands, islandId );
	island->setIndex = setIndex;
	island->localIndex = set->islandSims.count;
	island->islandId = islandId;
	island->headBody = B2_NULL_INDEX;
	island->tailBody = B2_NULL_INDEX;
	island->bodyCount = 0;
	island->headContact = B2_NULL_INDEX;
	island->tailContact = B2_NULL_INDEX;
	island->contactCount = 0;
	island->headJoint = B2_NULL_INDEX;
	island->tailJoint = B2_NULL_INDEX;
	island->jointCount = 0;
	island->parentIsland = B2_NULL_INDEX;
	island->constraintRemoveCount = 0;

	b2IslandSim* islandSim = b2IslandSimArray_Add( &set->islandSims );
	islandSim->islandId = islandId;

	return island;
}

void b2DestroyIsland( b2World* world, int islandId )
{
	// assume island is empty
	b2Island* island = b2IslandArray_Get( &world->islands, islandId );
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, island->setIndex );
	int movedIndex = b2IslandSimArray_RemoveSwap( &set->islandSims, island->localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix index on moved element
		b2IslandSim* movedElement = set->islandSims.data + island->localIndex;
		int movedId = movedElement->islandId;
		b2Island* movedIsland = b2IslandArray_Get( &world->islands, movedId );
		B2_ASSERT( movedIsland->localIndex == movedIndex );
		movedIsland->localIndex = island->localIndex;
	}

	// Free island and id (preserve island revision)
	island->islandId = B2_NULL_INDEX;
	island->setIndex = B2_NULL_INDEX;
	island->localIndex = B2_NULL_INDEX;
	b2FreeId( &world->islandIdPool, islandId );
}

static void b2AddContactToIsland( b2World* world, int islandId, b2Contact* contact )
{
	B2_ASSERT( contact->islandId == B2_NULL_INDEX );
	B2_ASSERT( contact->islandPrev == B2_NULL_INDEX );
	B2_ASSERT( contact->islandNext == B2_NULL_INDEX );

	b2Island* island = b2IslandArray_Get( &world->islands, islandId );

	if ( island->headContact != B2_NULL_INDEX )
	{
		contact->islandNext = island->headContact;
		b2Contact* headContact = b2ContactArray_Get( &world->contacts, island->headContact);
		headContact->islandPrev = contact->contactId;
	}

	island->headContact = contact->contactId;
	if ( island->tailContact == B2_NULL_INDEX )
	{
		island->tailContact = island->headContact;
	}

	island->contactCount += 1;
	contact->islandId = islandId;

	b2ValidateIsland( world, islandId );
}

// Link a contact into an island.
// This performs union-find and path compression to join islands.
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
void b2LinkContact( b2World* world, b2Contact* contact )
{
	B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) != 0 && ( contact->flags & b2_contactSensorFlag ) == 0 );

	int bodyIdA = contact->edges[0].bodyId;
	int bodyIdB = contact->edges[1].bodyId;

	b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

	B2_ASSERT( bodyA->setIndex != b2_disabledSet && bodyB->setIndex != b2_disabledSet );
	B2_ASSERT( bodyA->setIndex != b2_staticSet || bodyB->setIndex != b2_staticSet );

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if ( bodyA->setIndex == b2_awakeSet && bodyB->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyB->setIndex );
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if ( bodyB->setIndex == b2_awakeSet && bodyA->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyA->setIndex );
	}

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	// Static bodies have null island indices.
	B2_ASSERT( bodyA->setIndex != b2_staticSet || islandIdA == B2_NULL_INDEX );
	B2_ASSERT( bodyB->setIndex != b2_staticSet || islandIdB == B2_NULL_INDEX );
	B2_ASSERT( islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX );

	if ( islandIdA == islandIdB )
	{
		// Contact in same island
		b2AddContactToIsland( world, islandIdA, contact );
		return;
	}

	// Union-find root of islandA
	b2Island* islandA = NULL;
	if ( islandIdA != B2_NULL_INDEX )
	{
		islandA = b2IslandArray_Get( &world->islands, islandIdA );
		int parentId = islandA->parentIsland;
		while ( parentId != B2_NULL_INDEX )
		{
			b2Island* parent = b2IslandArray_Get( &world->islands, parentId );
			if ( parent->parentIsland != B2_NULL_INDEX )
			{
				// path compression
				islandA->parentIsland = parent->parentIsland;
			}

			islandA = parent;
			islandIdA = parentId;
			parentId = islandA->parentIsland;
		}
	}

	// Union-find root of islandB
	b2Island* islandB = NULL;
	if ( islandIdB != B2_NULL_INDEX )
	{
		islandB = b2IslandArray_Get( &world->islands, islandIdB );
		int parentId = islandB->parentIsland;
		while ( islandB->parentIsland != B2_NULL_INDEX )
		{
			b2Island* parent = b2IslandArray_Get( &world->islands, parentId );
			if ( parent->parentIsland != B2_NULL_INDEX )
			{
				// path compression
				islandB->parentIsland = parent->parentIsland;
			}

			islandB = parent;
			islandIdB = parentId;
			parentId = islandB->parentIsland;
		}
	}

	B2_ASSERT( islandA != NULL || islandB != NULL );

	// Union-Find link island roots
	if ( islandA != islandB && islandA != NULL && islandB != NULL )
	{
		B2_ASSERT( islandA != islandB );
		B2_ASSERT( islandB->parentIsland == B2_NULL_INDEX );
		islandB->parentIsland = islandIdA;
	}

	if ( islandA != NULL )
	{
		b2AddContactToIsland( world, islandIdA, contact );
	}
	else
	{
		b2AddContactToIsland( world, islandIdB, contact );
	}
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
void b2UnlinkContact( b2World* world, b2Contact* contact )
{
	B2_ASSERT( ( contact->flags & b2_contactSensorFlag ) == 0 );
	B2_ASSERT( contact->islandId != B2_NULL_INDEX );

	// remove from island
	int islandId = contact->islandId;
	b2Island* island = b2IslandArray_Get( &world->islands, islandId );

	if ( contact->islandPrev != B2_NULL_INDEX )
	{
		b2Contact* prevContact = b2ContactArray_Get( &world->contacts, contact->islandPrev);
		B2_ASSERT( prevContact->islandNext == contact->contactId );
		prevContact->islandNext = contact->islandNext;
	}

	if ( contact->islandNext != B2_NULL_INDEX )
	{
		b2Contact* nextContact = b2ContactArray_Get( &world->contacts, contact->islandNext );
		B2_ASSERT( nextContact->islandPrev == contact->contactId );
		nextContact->islandPrev = contact->islandPrev;
	}

	if ( island->headContact == contact->contactId )
	{
		island->headContact = contact->islandNext;
	}

	if ( island->tailContact == contact->contactId )
	{
		island->tailContact = contact->islandPrev;
	}

	B2_ASSERT( island->contactCount > 0 );
	island->contactCount -= 1;
	island->constraintRemoveCount += 1;

	contact->islandId = B2_NULL_INDEX;
	contact->islandPrev = B2_NULL_INDEX;
	contact->islandNext = B2_NULL_INDEX;

	b2ValidateIsland( world, islandId );
}

static void b2AddJointToIsland( b2World* world, int islandId, b2Joint* joint )
{
	B2_ASSERT( joint->islandId == B2_NULL_INDEX );
	B2_ASSERT( joint->islandPrev == B2_NULL_INDEX );
	B2_ASSERT( joint->islandNext == B2_NULL_INDEX );

	b2Island* island = b2IslandArray_Get( &world->islands, islandId );

	if ( island->headJoint != B2_NULL_INDEX )
	{
		joint->islandNext = island->headJoint;
		b2Joint* headJoint = b2JointArray_Get( &world->joints, island->headJoint );
		headJoint->islandPrev = joint->jointId;
	}

	island->headJoint = joint->jointId;
	if ( island->tailJoint == B2_NULL_INDEX )
	{
		island->tailJoint = island->headJoint;
	}

	island->jointCount += 1;
	joint->islandId = islandId;

	b2ValidateIsland( world, islandId );
}

void b2LinkJoint( b2World* world, b2Joint* joint, bool mergeIslands )
{
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, joint->edges[0].bodyId );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, joint->edges[1].bodyId );

	if ( bodyA->setIndex == b2_awakeSet && bodyB->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyB->setIndex );
	}
	else if ( bodyB->setIndex == b2_awakeSet && bodyA->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyA->setIndex );
	}

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	B2_ASSERT( islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX );

	if ( islandIdA == islandIdB )
	{
		// Joint in same island
		b2AddJointToIsland( world, islandIdA, joint );
		return;
	}

	// Union-find root of islandA
	b2Island* islandA = NULL;
	if ( islandIdA != B2_NULL_INDEX )
	{
		islandA = b2IslandArray_Get( &world->islands, islandIdA );
		while ( islandA->parentIsland != B2_NULL_INDEX )
		{
			b2Island* parent = b2IslandArray_Get( &world->islands, islandA->parentIsland );
			if ( parent->parentIsland != B2_NULL_INDEX )
			{
				// path compression
				islandA->parentIsland = parent->parentIsland;
			}

			islandIdA = islandA->parentIsland;
			islandA = parent;
		}
	}

	// Union-find root of islandB
	b2Island* islandB = NULL;
	if ( islandIdB != B2_NULL_INDEX )
	{
		islandB = b2IslandArray_Get( &world->islands, islandIdB );
		while ( islandB->parentIsland != B2_NULL_INDEX )
		{
			b2Island* parent = b2IslandArray_Get( &world->islands, islandB->parentIsland );
			if ( parent->parentIsland != B2_NULL_INDEX )
			{
				// path compression
				islandB->parentIsland = parent->parentIsland;
			}

			islandIdB = islandB->parentIsland;
			islandB = parent;
		}
	}

	B2_ASSERT( islandA != NULL || islandB != NULL );

	// Union-Find link island roots
	if ( islandA != islandB && islandA != NULL && islandB != NULL )
	{
		B2_ASSERT( islandA != islandB );
		B2_ASSERT( islandB->parentIsland == B2_NULL_INDEX );
		islandB->parentIsland = islandIdA;
	}

	if ( islandA != NULL )
	{
		b2AddJointToIsland( world, islandIdA, joint );
	}
	else
	{
		b2AddJointToIsland( world, islandIdB, joint );
	}

	// Joints need to have islands merged immediately when they are created
	// to keep the island graph valid.
	// However, when a body type is being changed the merge can be deferred until
	// all joints are linked.
	if (mergeIslands)
	{
		b2MergeAwakeIslands( world );
	}
}

void b2UnlinkJoint( b2World* world, b2Joint* joint )
{
	B2_ASSERT( joint->islandId != B2_NULL_INDEX );

	// remove from island
	int islandId = joint->islandId;
	b2Island* island = b2IslandArray_Get( &world->islands, islandId );

	if ( joint->islandPrev != B2_NULL_INDEX )
	{
		b2Joint* prevJoint = b2JointArray_Get( &world->joints, joint->islandPrev );
		B2_ASSERT( prevJoint->islandNext == joint->jointId );
		prevJoint->islandNext = joint->islandNext;
	}

	if ( joint->islandNext != B2_NULL_INDEX )
	{
		b2Joint* nextJoint = b2JointArray_Get( &world->joints, joint->islandNext );
		B2_ASSERT( nextJoint->islandPrev == joint->jointId );
		nextJoint->islandPrev = joint->islandPrev;
	}

	if ( island->headJoint == joint->jointId )
	{
		island->headJoint = joint->islandNext;
	}

	if ( island->tailJoint == joint->jointId )
	{
		island->tailJoint = joint->islandPrev;
	}

	B2_ASSERT( island->jointCount > 0 );
	island->jointCount -= 1;
	island->constraintRemoveCount += 1;

	joint->islandId = B2_NULL_INDEX;
	joint->islandPrev = B2_NULL_INDEX;
	joint->islandNext = B2_NULL_INDEX;

	b2ValidateIsland( world, islandId );
}

// Merge an island into its root island.
// todo we can assume all islands are awake here
static void b2MergeIsland( b2World* world, b2Island* island )
{
	B2_ASSERT( island->parentIsland != B2_NULL_INDEX );

	int rootId = island->parentIsland;
	b2Island* rootIsland = b2IslandArray_Get( &world->islands, rootId );
	B2_ASSERT( rootIsland->parentIsland == B2_NULL_INDEX );

	// remap island indices
	int bodyId = island->headBody;
	while ( bodyId != B2_NULL_INDEX )
	{
		b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
		body->islandId = rootId;
		bodyId = body->islandNext;
	}

	int contactId = island->headContact;
	while ( contactId != B2_NULL_INDEX )
	{
		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
		contact->islandId = rootId;
		contactId = contact->islandNext;
	}

	int jointId = island->headJoint;
	while ( jointId != B2_NULL_INDEX )
	{
		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		joint->islandId = rootId;
		jointId = joint->islandNext;
	}

	// connect body lists
	B2_ASSERT( rootIsland->tailBody != B2_NULL_INDEX );
	b2Body* tailBody = b2BodyArray_Get( &world->bodies, rootIsland->tailBody );
	B2_ASSERT( tailBody->islandNext == B2_NULL_INDEX );
	tailBody->islandNext = island->headBody;

	B2_ASSERT( island->headBody != B2_NULL_INDEX );
	b2Body* headBody = b2BodyArray_Get( &world->bodies, island->headBody );
	B2_ASSERT( headBody->islandPrev == B2_NULL_INDEX );
	headBody->islandPrev = rootIsland->tailBody;

	rootIsland->tailBody = island->tailBody;
	rootIsland->bodyCount += island->bodyCount;

	// connect contact lists
	if ( rootIsland->headContact == B2_NULL_INDEX )
	{
		// Root island has no contacts
		B2_ASSERT( rootIsland->tailContact == B2_NULL_INDEX && rootIsland->contactCount == 0 );
		rootIsland->headContact = island->headContact;
		rootIsland->tailContact = island->tailContact;
		rootIsland->contactCount = island->contactCount;
	}
	else if ( island->headContact != B2_NULL_INDEX )
	{
		// Both islands have contacts
		B2_ASSERT( island->tailContact != B2_NULL_INDEX && island->contactCount > 0 );
		B2_ASSERT( rootIsland->tailContact != B2_NULL_INDEX && rootIsland->contactCount > 0 );

		b2Contact* tailContact = b2ContactArray_Get( &world->contacts, rootIsland->tailContact );
		B2_ASSERT( tailContact->islandNext == B2_NULL_INDEX );
		tailContact->islandNext = island->headContact;

		b2Contact* headContact = b2ContactArray_Get( &world->contacts, island->headContact );
		B2_ASSERT( headContact->islandPrev == B2_NULL_INDEX );
		headContact->islandPrev = rootIsland->tailContact;

		rootIsland->tailContact = island->tailContact;
		rootIsland->contactCount += island->contactCount;
	}

	if ( rootIsland->headJoint == B2_NULL_INDEX )
	{
		// Root island has no joints
		B2_ASSERT( rootIsland->tailJoint == B2_NULL_INDEX && rootIsland->jointCount == 0 );
		rootIsland->headJoint = island->headJoint;
		rootIsland->tailJoint = island->tailJoint;
		rootIsland->jointCount = island->jointCount;
	}
	else if ( island->headJoint != B2_NULL_INDEX )
	{
		// Both islands have joints
		B2_ASSERT( island->tailJoint != B2_NULL_INDEX && island->jointCount > 0 );
		B2_ASSERT( rootIsland->tailJoint != B2_NULL_INDEX && rootIsland->jointCount > 0 );

		b2Joint* tailJoint = b2JointArray_Get( &world->joints, rootIsland->tailJoint );
		B2_ASSERT( tailJoint->islandNext == B2_NULL_INDEX );
		tailJoint->islandNext = island->headJoint;

		b2Joint* headJoint = b2JointArray_Get( &world->joints, island->headJoint );
		B2_ASSERT( headJoint->islandPrev == B2_NULL_INDEX );
		headJoint->islandPrev = rootIsland->tailJoint;

		rootIsland->tailJoint = island->tailJoint;
		rootIsland->jointCount += island->jointCount;
	}

	// Track removed constraints
	rootIsland->constraintRemoveCount += island->constraintRemoveCount;

	b2ValidateIsland( world, rootId );
}

// Iterate over all awake islands and merge any that need merging
// Islands that get merged into a root island will be removed from the awake island array
// and returned to the pool.
// todo this might be faster if b2IslandSim held the connectivity data
void b2MergeAwakeIslands( b2World* world )
{
	b2TracyCZoneNC( merge_islands, "Merge Islands", b2_colorMediumTurquoise, true );

	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2IslandSim* islandSims = awakeSet->islandSims.data;
	int awakeIslandCount = awakeSet->islandSims.count;

	// Step 1: Ensure every child island points to its root island. This avoids merging a child island with
	// a parent island that has already been merged with a grand-parent island.
	for ( int i = 0; i < awakeIslandCount; ++i )
	{
		int islandId = islandSims[i].islandId;

		b2Island* island = b2IslandArray_Get( &world->islands, islandId );

		// find the root island
		int rootId = islandId;
		b2Island* rootIsland = island;
		while ( rootIsland->parentIsland != B2_NULL_INDEX )
		{
			b2Island* parent = b2IslandArray_Get( &world->islands, rootIsland->parentIsland );
			if ( parent->parentIsland != B2_NULL_INDEX )
			{
				// path compression
				rootIsland->parentIsland = parent->parentIsland;
			}

			rootId = rootIsland->parentIsland;
			rootIsland = parent;
		}

		if ( rootIsland != island )
		{
			island->parentIsland = rootId;
		}
	}

	// Step 2: merge every awake island into its parent (which must be a root island)
	// Reverse to support removal from awake array.
	for ( int i = awakeIslandCount - 1; i >= 0; --i )
	{
		int islandId = islandSims[i].islandId;
		b2Island* island = b2IslandArray_Get( &world->islands, islandId );

		if ( island->parentIsland == B2_NULL_INDEX )
		{
			continue;
		}

		b2MergeIsland( world, island );

		// this call does a remove swap from the end of the island sim array
		b2DestroyIsland( world, islandId );
	}

	b2ValidateConnectivity( world );

	b2TracyCZoneEnd( merge_islands );
}

#define B2_CONTACT_REMOVE_THRESHOLD 1

void b2SplitIsland( b2World* world, int baseId )
{
	b2Island* baseIsland = b2IslandArray_Get( &world->islands, baseId );
	int setIndex = baseIsland->setIndex;

	if ( setIndex != b2_awakeSet )
	{
		// can only split awake island
		return;
	}

	if ( baseIsland->constraintRemoveCount == 0 )
	{
		// this island doesn't need to be split
		return;
	}

	b2ValidateIsland( world, baseId );

	int bodyCount = baseIsland->bodyCount;

	b2Body* bodies = world->bodies.data;
	b2StackAllocator* alloc = &world->stackAllocator;

	// No lock is needed because I ensure the allocator is not used while this task is active.
	int* stack = b2AllocateStackItem( alloc, bodyCount * sizeof( int ), "island stack" );
	int* bodyIds = b2AllocateStackItem( alloc, bodyCount * sizeof( int ), "body ids" );

	// Build array containing all body indices from base island. These
	// serve as seed bodies for the depth first search (DFS).
	int index = 0;
	int nextBody = baseIsland->headBody;
	while ( nextBody != B2_NULL_INDEX )
	{
		bodyIds[index++] = nextBody;
		b2Body* body = bodies + nextBody;

		// Clear visitation mark
		body->isMarked = false;

		nextBody = body->islandNext;
	}
	B2_ASSERT( index == bodyCount );

	// Clear contact island flags. Only need to consider contacts
	// already in the base island.
	int nextContactId = baseIsland->headContact;
	while ( nextContactId != B2_NULL_INDEX )
	{
		b2Contact* contact = b2ContactArray_Get( &world->contacts, nextContactId );
		contact->isMarked = false;
		nextContactId = contact->islandNext;
	}

	// Clear joint island flags.
	int nextJoint = baseIsland->headJoint;
	while ( nextJoint != B2_NULL_INDEX )
	{
		b2Joint* joint = b2JointArray_Get( &world->joints, nextJoint );
		joint->isMarked = false;
		nextJoint = joint->islandNext;
	}

	// Done with the base split island.
	b2DestroyIsland( world, baseId );

	// Each island is found as a depth first search starting from a seed body
	for ( int i = 0; i < bodyCount; ++i )
	{
		int seedIndex = bodyIds[i];
		b2Body* seed = bodies + seedIndex;
		B2_ASSERT( seed->setIndex == setIndex );

		if ( seed->isMarked == true )
		{
			// The body has already been visited
			continue;
		}

		int stackCount = 0;
		stack[stackCount++] = seedIndex;
		seed->isMarked = true;

		// Create new island
		// No lock needed because only a single island can split per time step. No islands are being used during the constraint
		// solve. However, islands are touched during body finalization.
		b2Island* island = b2CreateIsland( world, setIndex );

		int islandId = island->islandId;

		// Perform a depth first search (DFS) on the constraint graph.
		while ( stackCount > 0 )
		{
			// Grab the next body off the stack and add it to the island.
			int bodyId = stack[--stackCount];
			b2Body* body = bodies + bodyId;
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->isMarked == true );

			// Add body to island
			body->islandId = islandId;
			if ( island->tailBody != B2_NULL_INDEX )
			{
				bodies[island->tailBody].islandNext = bodyId;
			}
			body->islandPrev = island->tailBody;
			body->islandNext = B2_NULL_INDEX;
			island->tailBody = bodyId;

			if ( island->headBody == B2_NULL_INDEX )
			{
				island->headBody = bodyId;
			}

			island->bodyCount += 1;

			// Search all contacts connected to this body.
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
				B2_ASSERT( contact->contactId == contactId );

				// Next key
				contactKey = contact->edges[edgeIndex].nextKey;

				// Has this contact already been added to this island?
				if ( contact->isMarked )
				{
					continue;
				}

				// Skip sensors
				if ( contact->flags & b2_contactSensorFlag )
				{
					continue;
				}

				// Is this contact enabled and touching?
				if ( ( contact->flags & b2_contactTouchingFlag ) == 0 )
				{
					continue;
				}

				contact->isMarked = true;

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = bodies + otherBodyId;

				// Maybe add other body to stack
				if ( otherBody->isMarked == false && otherBody->setIndex != b2_staticSet )
				{
					B2_ASSERT( stackCount < bodyCount );
					stack[stackCount++] = otherBodyId;
					otherBody->isMarked = true;
				}

				// Add contact to island
				contact->islandId = islandId;
				if ( island->tailContact != B2_NULL_INDEX )
				{
					b2Contact* tailContact = b2ContactArray_Get( &world->contacts, island->tailContact );
					tailContact->islandNext = contactId;
				}
				contact->islandPrev = island->tailContact;
				contact->islandNext = B2_NULL_INDEX;
				island->tailContact = contactId;

				if ( island->headContact == B2_NULL_INDEX )
				{
					island->headContact = contactId;
				}

				island->contactCount += 1;
			}

			// Search all joints connect to this body.
			int jointKey = body->headJointKey;
			while ( jointKey != B2_NULL_INDEX )
			{
				int jointId = jointKey >> 1;
				int edgeIndex = jointKey & 1;

				b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
				B2_ASSERT( joint->jointId == jointId );

				// Next key
				jointKey = joint->edges[edgeIndex].nextKey;

				// Has this joint already been added to this island?
				if ( joint->isMarked )
				{
					continue;
				}

				joint->isMarked = true;

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = joint->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = bodies + otherBodyId;

				// Don't simulate joints connected to disabled bodies.
				if ( otherBody->setIndex == b2_disabledSet )
				{
					continue;
				}

				// Maybe add other body to stack
				if ( otherBody->isMarked == false && otherBody->setIndex == b2_awakeSet )
				{
					B2_ASSERT( stackCount < bodyCount );
					stack[stackCount++] = otherBodyId;
					otherBody->isMarked = true;
				}

				// Add joint to island
				joint->islandId = islandId;
				if ( island->tailJoint != B2_NULL_INDEX )
				{
					b2Joint* tailJoint = b2JointArray_Get( &world->joints, island->tailJoint );
					tailJoint->islandNext = jointId;
				}
				joint->islandPrev = island->tailJoint;
				joint->islandNext = B2_NULL_INDEX;
				island->tailJoint = jointId;

				if ( island->headJoint == B2_NULL_INDEX )
				{
					island->headJoint = jointId;
				}

				island->jointCount += 1;
			}
		}

		b2ValidateIsland( world, islandId );
	}

	b2FreeStackItem( alloc, bodyIds );
	b2FreeStackItem( alloc, stack );
}

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses DFS and touches a lot of memory,
// so it can be quite slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
void b2SplitIslandTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( split, "Split Island", b2_colorOlive, true );

	B2_MAYBE_UNUSED( startIndex );
	B2_MAYBE_UNUSED( endIndex );
	B2_MAYBE_UNUSED( threadIndex );

	b2Timer timer = b2CreateTimer();
	b2World* world = context;

	B2_ASSERT( world->splitIslandId != B2_NULL_INDEX );

	b2SplitIsland( world, world->splitIslandId );

	world->profile.splitIslands += b2GetMilliseconds( &timer );
	b2TracyCZoneEnd( split );
}

#if B2_VALIDATE
void b2ValidateIsland( b2World* world, int islandId )
{
	b2Island* island = b2IslandArray_Get( &world->islands, islandId );
	B2_ASSERT( island->islandId == islandId );
	B2_ASSERT( island->setIndex != B2_NULL_INDEX );
	B2_ASSERT( island->headBody != B2_NULL_INDEX );

	{
		B2_ASSERT( island->tailBody != B2_NULL_INDEX );
		B2_ASSERT( island->bodyCount > 0 );
		if ( island->bodyCount > 1 )
		{
			B2_ASSERT( island->tailBody != island->headBody );
		}
		B2_ASSERT( island->bodyCount <= b2GetIdCount( &world->bodyIdPool ) );

		int count = 0;
		int bodyId = island->headBody;
		while ( bodyId != B2_NULL_INDEX )
		{
			b2Body* body = b2BodyArray_Get(&world->bodies, bodyId);
			B2_ASSERT( body->islandId == islandId );
			B2_ASSERT( body->setIndex == island->setIndex );
			count += 1;

			if ( count == island->bodyCount )
			{
				B2_ASSERT( bodyId == island->tailBody );
			}

			bodyId = body->islandNext;
		}
		B2_ASSERT( count == island->bodyCount );
	}

	if ( island->headContact != B2_NULL_INDEX )
	{
		B2_ASSERT( island->tailContact != B2_NULL_INDEX );
		B2_ASSERT( island->contactCount > 0 );
		if ( island->contactCount > 1 )
		{
			B2_ASSERT( island->tailContact != island->headContact );
		}
		B2_ASSERT( island->contactCount <= b2GetIdCount( &world->contactIdPool ) );

		int count = 0;
		int contactId = island->headContact;
		while ( contactId != B2_NULL_INDEX )
		{
			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
			B2_ASSERT( contact->setIndex == island->setIndex );
			B2_ASSERT( contact->islandId == islandId );
			count += 1;

			if ( count == island->contactCount )
			{
				B2_ASSERT( contactId == island->tailContact );
			}

			contactId = contact->islandNext;
		}
		B2_ASSERT( count == island->contactCount );
	}
	else
	{
		B2_ASSERT( island->tailContact == B2_NULL_INDEX );
		B2_ASSERT( island->contactCount == 0 );
	}

	if ( island->headJoint != B2_NULL_INDEX )
	{
		B2_ASSERT( island->tailJoint != B2_NULL_INDEX );
		B2_ASSERT( island->jointCount > 0 );
		if ( island->jointCount > 1 )
		{
			B2_ASSERT( island->tailJoint != island->headJoint );
		}
		B2_ASSERT( island->jointCount <= b2GetIdCount( &world->jointIdPool ) );

		int count = 0;
		int jointId = island->headJoint;
		while ( jointId != B2_NULL_INDEX )
		{
			b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
			B2_ASSERT( joint->setIndex == island->setIndex );
			count += 1;

			if ( count == island->jointCount )
			{
				B2_ASSERT( jointId == island->tailJoint );
			}

			jointId = joint->islandNext;
		}
		B2_ASSERT( count == island->jointCount );
	}
	else
	{
		B2_ASSERT( island->tailJoint == B2_NULL_INDEX );
		B2_ASSERT( island->jointCount == 0 );
	}
}

#else

void b2ValidateIsland( b2World* world, int islandId )
{
	B2_MAYBE_UNUSED( world );
	B2_MAYBE_UNUSED( islandId );
}
#endif
