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
		b2Array_Push( world->islands, emptyIsland );
	}
	else
	{
		B2_ASSERT( world->islands.data[islandId].setIndex == B2_NULL_INDEX );
	}

	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );

	b2Island* island = b2Array_Get( world->islands, islandId );
	island->setIndex = setIndex;
	island->localIndex = set->islandSims.count;
	island->islandId = islandId;
	b2Array_Create( island->bodies );
	b2Array_Create( island->contacts );
	b2Array_Create( island->joints );
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
	b2Island* island = b2Array_Get( world->islands, islandId );
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, island->setIndex );
	int movedIndex = b2IslandSimArray_RemoveSwap( &set->islandSims, island->localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix index on moved element
		b2IslandSim* movedElement = set->islandSims.data + island->localIndex;
		int movedId = movedElement->islandId;
		b2Island* movedIsland = b2Array_Get( world->islands, movedId );
		B2_ASSERT( movedIsland->localIndex == movedIndex );
		movedIsland->localIndex = island->localIndex;
	}

	// Free island and id (preserve island revision)
	b2Array_Destroy( island->bodies );
	b2Array_Destroy( island->contacts );
	b2Array_Destroy( island->joints );
	island->constraintRemoveCount = 0;
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

	b2Island* smallIsland;
	b2Island* bigIsland;
	{
		b2Island* islandA = b2Array_Get( world->islands, islandIdA );
		b2Island* islandB = b2Array_Get( world->islands, islandIdB );

		// Keep the biggest island to reduce cache misses
		if ( islandA->bodies.count >= islandB->bodies.count )
		{
			bigIsland = islandA;
			smallIsland = islandB;
		}
		else
		{
			bigIsland = islandB;
			smallIsland = islandA;
		}
	}

	int bigIslandId = bigIsland->islandId;
	b2Array_Reserve( bigIsland->bodies, bigIsland->bodies.count + smallIsland->bodies.count );

	// Move bodies from smaller island to larger island
	for ( int i = 0; i < smallIsland->bodies.count; ++i )
	{
		int bodyId = smallIsland->bodies.data[i];
		b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
		B2_VALIDATE( body->islandId == smallIsland->islandId );
		body->islandId = bigIslandId;
		body->islandIndex = bigIsland->bodies.count;
		b2Array_Push( bigIsland->bodies, bodyId );
	}

	// Migrate contacts from smaller island to larger island
	if ( smallIsland->contacts.count > 0 )
	{
		b2Array_Reserve( bigIsland->contacts, bigIsland->contacts.count + smallIsland->contacts.count );

		for ( int i = 0; i < smallIsland->contacts.count; ++i )
		{
			b2ContactLink* link = smallIsland->contacts.data + i;
			b2Contact* contact = b2ContactArray_Get( &world->contacts, link->contactId );
			contact->islandId = bigIslandId;
			contact->islandIndex = bigIsland->contacts.count;
			b2Array_Push( bigIsland->contacts, *link );
		}
	}

	// Migrate joints from smaller island to larger island
	if ( smallIsland->joints.count > 0 )
	{
		b2Array_Reserve( bigIsland->joints, bigIsland->joints.count + smallIsland->joints.count );

		for ( int i = 0; i < smallIsland->joints.count; ++i )
		{
			b2JointLink* link = smallIsland->joints.data + i;
			b2Joint* joint = b2JointArray_Get( &world->joints, link->jointId );
			joint->islandId = bigIslandId;
			joint->islandIndex = bigIsland->joints.count;
			b2Array_Push( bigIsland->joints, *link );
		}
	}

	// Track removed constraints
	bigIsland->constraintRemoveCount += smallIsland->constraintRemoveCount;

	b2DestroyIsland( world, smallIsland->islandId );

	b2ValidateIsland( world, bigIslandId );

	return bigIslandId;
}

static void b2AddContactToIsland( b2World* world, int islandId, b2Contact* contact )
{
	B2_ASSERT( contact->islandId == B2_NULL_INDEX );
	B2_ASSERT( contact->islandIndex == B2_NULL_INDEX );

	b2Island* island = b2Array_Get( world->islands, islandId );

	contact->islandId = islandId;
	contact->islandIndex = island->contacts.count;

	b2ContactLink link;
	link.contactId = contact->contactId;
	link.bodyIdA = contact->edges[0].bodyId;
	link.bodyIdB = contact->edges[1].bodyId;
	b2Array_Push( island->contacts, link );

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
	b2Island* island = b2Array_Get( world->islands, islandId );

	int removeIndex = contact->islandIndex;
	B2_ASSERT( 0 <= removeIndex && removeIndex < island->contacts.count );
	B2_ASSERT( island->contacts.data[removeIndex].contactId == contact->contactId );

	int movedIndex = b2Array_RemoveSwap( island->contacts, removeIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix islandIndex on the contact that was swapped into removeIndex
		b2ContactLink* movedLink = island->contacts.data + removeIndex;
		b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedLink->contactId );
		B2_ASSERT( movedContact->islandIndex == movedIndex );
		movedContact->islandIndex = removeIndex;
	}

	island->constraintRemoveCount += 1;

	contact->islandId = B2_NULL_INDEX;
	contact->islandIndex = B2_NULL_INDEX;

	b2ValidateIsland( world, islandId );
}

static void b2AddJointToIsland( b2World* world, int islandId, b2Joint* joint )
{
	B2_ASSERT( joint->islandId == B2_NULL_INDEX );
	B2_ASSERT( joint->islandIndex == B2_NULL_INDEX );

	b2Island* island = b2Array_Get( world->islands, islandId );

	joint->islandId = islandId;
	joint->islandIndex = island->joints.count;

	b2JointLink link;
	link.jointId = joint->jointId;
	link.bodyIdA = joint->edges[0].bodyId;
	link.bodyIdB = joint->edges[1].bodyId;
	b2Array_Push( island->joints, link );

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
	b2Island* island = b2Array_Get( world->islands, islandId );

	int removeIndex = joint->islandIndex;
	B2_ASSERT( 0 <= removeIndex && removeIndex < island->joints.count );
	B2_ASSERT( island->joints.data[removeIndex].jointId == joint->jointId );

	int movedIndex = b2Array_RemoveSwap( island->joints, removeIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix islandIndex on the joint that was swapped into removeIndex
		b2JointLink* movedLink = island->joints.data + removeIndex;
		b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedLink->jointId );
		B2_ASSERT( movedJoint->islandIndex == movedIndex );
		movedJoint->islandIndex = removeIndex;
	}

	island->constraintRemoveCount += 1;

	joint->islandId = B2_NULL_INDEX;
	joint->islandIndex = B2_NULL_INDEX;

	b2ValidateIsland( world, islandId );
}

// Find parent of a node. Use path halving to speed up further queries.
static inline int b2IslandFindParent( int* parents, int node )
{
	// Walk the chain of parents to find the node that is its own parent (the root)
	while ( parents[node] != node )
	{
		int grandParent = parents[parents[node]];
		parents[node] = grandParent;
		node = grandParent;
	}

	return node;
}

// Connect the components containing node1 and node2.
// Uses rank to keep tree balanced.
static inline void b2IslandUnion( int* parents, int* ranks, int node1, int node2 )
{
	int root1 = b2IslandFindParent( parents, node1 );
	int root2 = b2IslandFindParent( parents, node2 );
	if ( root1 != root2 )
	{
		if ( ranks[root1] < ranks[root2] )
		{
			parents[root1] = root2;
		}
		else if ( ranks[root1] > ranks[root2] )
		{
			parents[root2] = root1;
		}
		else
		{
			parents[root2] = root1;
			ranks[root1] += 1;
		}
	}
}

// This uses union-find.
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
void b2SplitIsland( b2World* world, int baseId )
{
	b2Island* baseIsland = b2Array_Get( world->islands, baseId );
	B2_ASSERT( baseIsland->constraintRemoveCount > 0 );
	B2_ASSERT( baseIsland->setIndex == b2_awakeSet );

	b2ValidateIsland( world, baseId );

	// Cache base island fields before b2CreateIsland, which may reallocate
	// world->islands and invalidate the baseIsland pointer.
	int baseBodyCount = baseIsland->bodies.count;
	int* baseBodyIds = baseIsland->bodies.data;
	int baseBodyCapacity = baseIsland->bodies.capacity;

	int baseContactCount = baseIsland->contacts.count;
	b2ContactLink* baseContacts = baseIsland->contacts.data;
	int baseContactCapacity = baseIsland->contacts.capacity;

	int baseJointCount = baseIsland->joints.count;
	b2JointLink* baseJoints = baseIsland->joints.data;
	int baseJointCapacity = baseIsland->joints.capacity;

	b2ArenaAllocator* alloc = &world->arena;

	// No lock is needed because I ensure the allocator is not used while this task is active.
	int* parents = b2AllocateArenaItem( alloc, baseBodyCount * sizeof( int ), "body ids" );
	int* ranks = b2AllocateArenaItem( alloc, baseBodyCount * sizeof( int ), "ranks" );
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		parents[i] = i;
		ranks[i] = 0;
	}

	b2Body* bodies = world->bodies.data;

	// Union over contacts
	for ( int i = 0; i < baseContactCount; ++i )
	{
		int bodyIdA = baseContacts[i].bodyIdA;
		int bodyIdB = baseContacts[i].bodyIdB;
		B2_VALIDATE( 0 <= bodyIdA && bodyIdA < world->bodies.count );
		B2_VALIDATE( 0 <= bodyIdB && bodyIdB < world->bodies.count );
		b2Body* bodyA = bodies + bodyIdA;
		b2Body* bodyB = bodies + bodyIdB;
		int islandIndexA = bodyA->islandIndex;
		int islandIndexB = bodyB->islandIndex;

		// Only connect non-static bodies
		if ( islandIndexA != B2_NULL_INDEX && islandIndexB != B2_NULL_INDEX )
		{
			B2_VALIDATE( 0 <= islandIndexA && islandIndexA < baseBodyCount );
			B2_VALIDATE( 0 <= islandIndexB && islandIndexB < baseBodyCount );
			b2IslandUnion( parents, ranks, islandIndexA, islandIndexB );
		}
	}

	// Union over joints
	for ( int i = 0; i < baseJointCount; ++i )
	{
		int bodyIdA = baseJoints[i].bodyIdA;
		int bodyIdB = baseJoints[i].bodyIdB;
		B2_VALIDATE( 0 <= bodyIdA && bodyIdA < world->bodies.count );
		B2_VALIDATE( 0 <= bodyIdB && bodyIdB < world->bodies.count );
		b2Body* bodyA = bodies + bodyIdA;
		b2Body* bodyB = bodies + bodyIdB;
		int islandIndexA = bodyA->islandIndex;
		int islandIndexB = bodyB->islandIndex;

		// Only connect non-static bodies
		if ( islandIndexA != B2_NULL_INDEX && islandIndexB != B2_NULL_INDEX )
		{
			B2_VALIDATE( 0 <= islandIndexA && islandIndexA < baseBodyCount );
			B2_VALIDATE( 0 <= islandIndexB && islandIndexB < baseBodyCount );
			b2IslandUnion( parents, ranks, islandIndexA, islandIndexB );
		}
	}

	// Done with ranks
	b2FreeArenaItem( alloc, ranks );
	ranks = NULL;

	// Flatten all parent indices and count connected components.
	int componentCount = 0;
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		parents[i] = b2IslandFindParent( parents, i );
		if ( parents[i] == i )
		{
			componentCount += 1;
		}
	}

	// Early return — island is still fully connected, no split needed.
	if ( componentCount == 1 )
	{
		baseIsland->constraintRemoveCount = 0;
		b2FreeArenaItem( alloc, parents );
		return;
	}

	// Detach body/contact/joint arrays from base island so b2DestroyIsland won't free them
	baseIsland->bodies.data = NULL;
	baseIsland->bodies.count = 0;
	baseIsland->bodies.capacity = 0;

	baseIsland->contacts.data = NULL;
	baseIsland->contacts.count = 0;
	baseIsland->contacts.capacity = 0;

	baseIsland->joints.data = NULL;
	baseIsland->joints.count = 0;
	baseIsland->joints.capacity = 0;

	// Null so code below doesn't accidentally use this.
	baseIsland = NULL;

	// Map from body index to new island index. Only set for root bodies.
	int* rootMap = b2AllocateArenaItem( alloc, baseBodyCount * sizeof( int ), "root map" );
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		rootMap[i] = B2_NULL_INDEX;
	}

	int* componentBodyCounts = b2AllocateArenaItem( alloc, componentCount * sizeof( int ), "component body counts" );
	int islandCount = 0;

	// Find the root body for each body and create islands as needed
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		int rootIndex = parents[i];
		if ( rootMap[rootIndex] == B2_NULL_INDEX )
		{
			rootMap[rootIndex] = islandCount;
			componentBodyCounts[islandCount] = 0;
			islandCount += 1;
		}

		componentBodyCounts[rootMap[rootIndex]] += 1;
	}

	B2_ASSERT( islandCount == componentCount );

	// Map from new island index to island id
	int* islandIds = b2AllocateArenaItem( alloc, islandCount * sizeof( int ), "island ids" );

	// Create new islands and reserve body arrays
	for ( int i = 0; i < islandCount; ++i )
	{
		// WARNING: this invalidates baseIsland pointer
		b2Island* newIsland = b2CreateIsland( world, b2_awakeSet );
		islandIds[i] = newIsland->islandId;
		b2Array_Reserve( newIsland->bodies, componentBodyCounts[i] );
	}

	// Assign bodies to new islands
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		int bodyId = baseBodyIds[i];
		int root = b2IslandFindParent( parents, i );
		int newIslandId = islandIds[rootMap[root]];

		b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
		b2Island* newIsland = b2Array_Get( world->islands, newIslandId );

		body->islandId = newIslandId;
		body->islandIndex = newIsland->bodies.count;
		b2Array_Push( newIsland->bodies, bodyId );
	}

	// Assign contacts to the island of their bodies
	for ( int i = 0; i < baseContactCount; ++i )
	{
		b2ContactLink* link = baseContacts + i;
		b2Contact* contact = b2ContactArray_Get( &world->contacts, link->contactId );

		// Static bodies don't have an island id.
		b2Body* bodyA = b2BodyArray_Get( &world->bodies, link->bodyIdA );
		b2Body* bodyB = b2BodyArray_Get( &world->bodies, link->bodyIdB );
		int targetIslandId = bodyA->islandId != B2_NULL_INDEX ? bodyA->islandId : bodyB->islandId;

		b2Island* targetIsland = b2Array_Get( world->islands, targetIslandId );
		contact->islandId = targetIslandId;
		contact->islandIndex = targetIsland->contacts.count;
		b2Array_Push( targetIsland->contacts, *link );
	}

	// Assign joints to the island of their bodies
	for ( int i = 0; i < baseJointCount; ++i )
	{
		b2JointLink* link = baseJoints + i;
		b2Joint* joint = b2JointArray_Get( &world->joints, link->jointId );

		// Static bodies don't have an island id.
		b2Body* bodyA = b2BodyArray_Get( &world->bodies, link->bodyIdA );
		b2Body* bodyB = b2BodyArray_Get( &world->bodies, link->bodyIdB );
		int targetIslandId = bodyA->islandId != B2_NULL_INDEX ? bodyA->islandId : bodyB->islandId;

		b2Island* targetIsland = b2Array_Get( world->islands, targetIslandId );
		joint->islandId = targetIslandId;
		joint->islandIndex = targetIsland->joints.count;
		b2Array_Push( targetIsland->joints, *link );
	}

	// Destroy the base island
	b2DestroyIsland( world, baseId );

	// Free the detached arrays manually
	b2Free( baseBodyIds, baseBodyCapacity * sizeof( int ) );
	b2Free( baseContacts, baseContactCapacity * sizeof( b2ContactLink ) );
	b2Free( baseJoints, baseJointCapacity * sizeof( b2JointLink ) );

	b2FreeArenaItem( alloc, islandIds );
	b2FreeArenaItem( alloc, componentBodyCounts );
	b2FreeArenaItem( alloc, rootMap );
	b2FreeArenaItem( alloc, parents );
}

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses union find and
// touches a lot of memory, so it can be slow.
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

	world->splitIslandId = B2_NULL_INDEX;
	world->profile.splitIslands += b2GetMilliseconds( ticks );
	b2TracyCZoneEnd( split );
}

#if B2_ENABLE_VALIDATION
void b2ValidateIsland( b2World* world, int islandId )
{
	if ( islandId == B2_NULL_INDEX )
	{
		return;
	}

	b2Island* island = b2Array_Get( world->islands, islandId );
	B2_ASSERT( island->islandId == islandId );
	B2_ASSERT( island->setIndex != B2_NULL_INDEX );

	{
		B2_ASSERT( island->bodies.count > 0 );
		B2_ASSERT( island->bodies.count <= b2GetIdCount( &world->bodyIdPool ) );

		for ( int i = 0; i < island->bodies.count; ++i )
		{
			b2Body* body = b2BodyArray_Get( &world->bodies, island->bodies.data[i] );
			B2_ASSERT( body->islandId == islandId );
			B2_ASSERT( body->islandIndex == i );
			B2_ASSERT( body->setIndex == island->setIndex );
		}
	}

	if ( island->contacts.count > 0 )
	{
		B2_ASSERT( island->contacts.count <= b2GetIdCount( &world->contactIdPool ) );

		for ( int i = 0; i < island->contacts.count; ++i )
		{
			b2ContactLink* link = island->contacts.data + i;
			b2Contact* contact = b2ContactArray_Get( &world->contacts, link->contactId );
			B2_ASSERT( contact->setIndex == island->setIndex );
			B2_ASSERT( contact->islandId == islandId );
			B2_ASSERT( contact->islandIndex == i );
		}
	}

	if ( island->joints.count > 0 )
	{
		B2_ASSERT( island->joints.count <= b2GetIdCount( &world->jointIdPool ) );

		for ( int i = 0; i < island->joints.count; ++i )
		{
			b2JointLink* link = island->joints.data + i;
			b2Joint* joint = b2JointArray_Get( &world->joints, link->jointId );
			B2_ASSERT( joint->setIndex == island->setIndex );
			B2_ASSERT( joint->islandId == islandId );
			B2_ASSERT( joint->islandIndex == i );
		}
	}
}

#else

void b2ValidateIsland( b2World* world, int islandId )
{
	B2_UNUSED( world );
	B2_UNUSED( islandId );
}
#endif
