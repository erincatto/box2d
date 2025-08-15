// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "solver_set.h"

#include <stddef.h>

B2_ARRAY_SOURCE( b2Island, b2Island )
B2_ARRAY_SOURCE( b2IslandSim, b2IslandSim )

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
	island->constraintRemoveCount = 0;

	b2IslandSim* islandSim = b2IslandSimArray_Add( &set->islandSims );
	islandSim->islandId = islandId;

	return island;
}

void b2DestroyIsland( b2World* world, int islandId )
{
	if ( world->splitIslandId == islandId )
	{
		world->splitIslandId = B2_NULL_INDEX;
	}

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

static int b2MergeIslands( b2World* world, int islandIdA, int islandIdB )
{
	if ( islandIdA == islandIdB )
	{
		return islandIdA;
	}

	if ( islandIdA == B2_NULL_INDEX )
	{
		B2_ASSERT( islandIdB != B2_NULL_INDEX );
		return islandIdB;
	}

	if ( islandIdB == B2_NULL_INDEX )
	{
		B2_ASSERT( islandIdA != B2_NULL_INDEX );
		return islandIdA;
	}

	b2Island* islandA = b2IslandArray_Get( &world->islands, islandIdA );
	b2Island* islandB = b2IslandArray_Get( &world->islands, islandIdB );

	// Keep the biggest island to reduce cache misses
	b2Island* big;
	b2Island* small;
	if ( islandA->bodyCount >= islandB->bodyCount )
	{
		big = islandA;
		small = islandB;
	}
	else
	{
		big = islandB;
		small = islandA;
	}

	int bigId = big->islandId;

	// remap island indices (cache misses)
	int bodyId = small->headBody;
	while ( bodyId != B2_NULL_INDEX )
	{
		b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
		body->islandId = bigId;
		bodyId = body->islandNext;
	}

	int contactId = small->headContact;
	while ( contactId != B2_NULL_INDEX )
	{
		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
		contact->islandId = bigId;
		contactId = contact->islandNext;
	}

	int jointId = small->headJoint;
	while ( jointId != B2_NULL_INDEX )
	{
		b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
		joint->islandId = bigId;
		jointId = joint->islandNext;
	}

	// connect body lists
	B2_ASSERT( big->tailBody != B2_NULL_INDEX );
	b2Body* tailBody = b2BodyArray_Get( &world->bodies, big->tailBody );
	B2_ASSERT( tailBody->islandNext == B2_NULL_INDEX );
	tailBody->islandNext = small->headBody;

	B2_ASSERT( small->headBody != B2_NULL_INDEX );
	b2Body* headBody = b2BodyArray_Get( &world->bodies, small->headBody );
	B2_ASSERT( headBody->islandPrev == B2_NULL_INDEX );
	headBody->islandPrev = big->tailBody;

	big->tailBody = small->tailBody;
	big->bodyCount += small->bodyCount;

	// connect contact lists
	if ( big->headContact == B2_NULL_INDEX )
	{
		// Big island has no contacts
		B2_ASSERT( big->tailContact == B2_NULL_INDEX && big->contactCount == 0 );
		big->headContact = small->headContact;
		big->tailContact = small->tailContact;
		big->contactCount = small->contactCount;
	}
	else if ( small->headContact != B2_NULL_INDEX )
	{
		// Both islands have contacts
		B2_ASSERT( small->tailContact != B2_NULL_INDEX && small->contactCount > 0 );
		B2_ASSERT( big->tailContact != B2_NULL_INDEX && big->contactCount > 0 );

		b2Contact* tailContact = b2ContactArray_Get( &world->contacts, big->tailContact );
		B2_ASSERT( tailContact->islandNext == B2_NULL_INDEX );
		tailContact->islandNext = small->headContact;

		b2Contact* headContact = b2ContactArray_Get( &world->contacts, small->headContact );
		B2_ASSERT( headContact->islandPrev == B2_NULL_INDEX );
		headContact->islandPrev = big->tailContact;

		big->tailContact = small->tailContact;
		big->contactCount += small->contactCount;
	}

	if ( big->headJoint == B2_NULL_INDEX )
	{
		// Root island has no joints
		B2_ASSERT( big->tailJoint == B2_NULL_INDEX && big->jointCount == 0 );
		big->headJoint = small->headJoint;
		big->tailJoint = small->tailJoint;
		big->jointCount = small->jointCount;
	}
	else if ( small->headJoint != B2_NULL_INDEX )
	{
		// Both islands have joints
		B2_ASSERT( small->tailJoint != B2_NULL_INDEX && small->jointCount > 0 );
		B2_ASSERT( big->tailJoint != B2_NULL_INDEX && big->jointCount > 0 );

		b2Joint* tailJoint = b2JointArray_Get( &world->joints, big->tailJoint );
		B2_ASSERT( tailJoint->islandNext == B2_NULL_INDEX );
		tailJoint->islandNext = small->headJoint;

		b2Joint* headJoint = b2JointArray_Get( &world->joints, small->headJoint );
		B2_ASSERT( headJoint->islandPrev == B2_NULL_INDEX );
		headJoint->islandPrev = big->tailJoint;

		big->tailJoint = small->tailJoint;
		big->jointCount += small->jointCount;
	}

	// Track removed constraints
	big->constraintRemoveCount += small->constraintRemoveCount;

	small->bodyCount = 0;
	small->contactCount = 0;
	small->jointCount = 0;
	small->headBody = B2_NULL_INDEX;
	small->headContact = B2_NULL_INDEX;
	small->headJoint = B2_NULL_INDEX;
	small->tailBody = B2_NULL_INDEX;
	small->tailContact = B2_NULL_INDEX;
	small->tailJoint = B2_NULL_INDEX;
	small->constraintRemoveCount = 0;

	b2DestroyIsland( world, small->islandId );

	b2ValidateIsland( world, bigId );

	return bigId;
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
		b2Contact* headContact = b2ContactArray_Get( &world->contacts, island->headContact );
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
void b2LinkContact( b2World* world, b2Contact* contact )
{
	B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) != 0 );

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

	// Merge islands. This will destroy one of the islands.
	int finalIslandId = b2MergeIslands( world, islandIdA, islandIdB );

	// Add contact to the island that survived
	b2AddContactToIsland( world, finalIslandId, contact );
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
void b2UnlinkContact( b2World* world, b2Contact* contact )
{
	B2_ASSERT( contact->islandId != B2_NULL_INDEX );

	// remove from island
	int islandId = contact->islandId;
	b2Island* island = b2IslandArray_Get( &world->islands, islandId );

	if ( contact->islandPrev != B2_NULL_INDEX )
	{
		b2Contact* prevContact = b2ContactArray_Get( &world->contacts, contact->islandPrev );
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

void b2LinkJoint( b2World* world, b2Joint* joint )
{
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, joint->edges[0].bodyId );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, joint->edges[1].bodyId );

	B2_ASSERT( bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody );

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

	// Merge islands. This will destroy one of the islands.
	int finalIslandId = b2MergeIslands( world, islandIdA, islandIdB );

	// Add joint the island that survived
	b2AddJointToIsland( world, finalIslandId, joint );
}

void b2UnlinkJoint( b2World* world, b2Joint* joint )
{
	if ( joint->islandId == B2_NULL_INDEX )
	{
		return;
	}

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

#define B2_CONTACT_REMOVE_THRESHOLD 1

// Possible optimizations:
// 1. use the body island id as the mark
// 2. start from the sleepy bodies and stop processing if a sleep body is connected to a non-sleepy body
// 3. use a sleepy flag on bodies to avoid velocity access
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
	b2ArenaAllocator* alloc = &world->arena;

	// No lock is needed because I ensure the allocator is not used while this task is active.
	int* stack = b2AllocateArenaItem( alloc, bodyCount * sizeof( int ), "island stack" );
	int* bodyIds = b2AllocateArenaItem( alloc, bodyCount * sizeof( int ), "body ids" );

	// Build array containing all body indices from base island. These
	// serve as seed bodies for the depth first search (DFS).
	int index = 0;
	int nextBody = baseIsland->headBody;
	while ( nextBody != B2_NULL_INDEX )
	{
		bodyIds[index++] = nextBody;
		b2Body* body = bodies + nextBody;

		nextBody = body->islandNext;
	}
	B2_ASSERT( index == bodyCount );

	// Each island is found as a depth first search starting from a seed body
	for ( int i = 0; i < bodyCount; ++i )
	{
		int seedIndex = bodyIds[i];
		b2Body* seed = bodies + seedIndex;
		B2_ASSERT( seed->setIndex == setIndex );

		if ( seed->islandId != baseId )
		{
			// The body has already been visited
			continue;
		}

		int stackCount = 0;
		stack[stackCount++] = seedIndex;

		// Create new island
		// No lock needed because only a single island can split per time step. No islands are being used during the constraint
		// solve. However, islands are touched during body finalization.
		b2Island* island = b2CreateIsland( world, setIndex );

		int islandId = island->islandId;
		seed->islandId = islandId;

		// Perform a depth first search (DFS) on the constraint graph.
		while ( stackCount > 0 )
		{
			// Grab the next body off the stack and add it to the island.
			int bodyId = stack[--stackCount];
			b2Body* body = bodies + bodyId;
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->islandId == islandId );

			// Add body to island
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
				if ( contact->islandId == islandId )
				{
					continue;
				}

				// Is this contact enabled and touching?
				if ( ( contact->flags & b2_contactTouchingFlag ) == 0 )
				{
					continue;
				}

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = bodies + otherBodyId;

				// Maybe add other body to stack
				if ( otherBody->islandId != islandId && otherBody->setIndex != b2_staticSet )
				{
					B2_ASSERT( stackCount < bodyCount );
					stack[stackCount++] = otherBodyId;

					// Need to update the body's island id immediately so it is not traversed again
					otherBody->islandId = islandId;
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
				if ( joint->islandId == islandId )
				{
					continue;
				}

				if ( joint->setIndex == b2_disabledSet )
				{
					continue;
				}

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = joint->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = bodies + otherBodyId;

				// Don't simulate joints connected to disabled bodies.
				if ( otherBody->setIndex == b2_disabledSet )
				{
					continue;
				}

				// At least one body must be dynamic
				if ( body->type != b2_dynamicBody && otherBody->type != b2_dynamicBody )
				{
					continue;
				}

				// Maybe add other body to stack
				if ( otherBody->islandId != islandId && otherBody->setIndex == b2_awakeSet )
				{
					B2_ASSERT( stackCount < bodyCount );
					stack[stackCount++] = otherBodyId;

					// Need to update the body's island id immediately so it is not traversed again
					otherBody->islandId = islandId;
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

	// Done with the base split island. This is delayed because the baseId is used as a marker and it
	// should not be recycled in while splitting.
	b2DestroyIsland( world, baseId );

	b2FreeArenaItem( alloc, bodyIds );
	b2FreeArenaItem( alloc, stack );
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

	B2_UNUSED( startIndex, endIndex, threadIndex );

	uint64_t ticks = b2GetTicks();
	b2World* world = context;

	B2_ASSERT( world->splitIslandId != B2_NULL_INDEX );

	b2SplitIsland( world, world->splitIslandId );

	world->profile.splitIslands += b2GetMilliseconds( ticks );
	b2TracyCZoneEnd( split );
}

#if B2_VALIDATE
void b2ValidateIsland( b2World* world, int islandId )
{
	if ( islandId == B2_NULL_INDEX )
	{
		return;
	}

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
			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
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
	B2_UNUSED( world );
	B2_UNUSED( islandId );
}
#endif
