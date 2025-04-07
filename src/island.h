// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
typedef struct b2World b2World;

// Deterministic solver
//
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: increment constraintRemoveCount

// Persistent island for awake bodies, joints, and contacts
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
// map from int to solver set and index
typedef struct b2Island
{
	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	int setIndex;

	// island index within set
	// may be B2_NULL_INDEX
	int localIndex;

	int islandId;

	int headBody;
	int tailBody;
	int bodyCount;

	int headContact;
	int tailContact;
	int contactCount;

	int headJoint;
	int tailJoint;
	int jointCount;

	// Union find
	// todo this could go away if islands are merged immediately with b2LinkJoint and b2LinkContact
	int parentIsland;

	// Keeps track of how many contacts have been removed from this island.
	// This is used to determine if an island is a candidate for splitting.
	int constraintRemoveCount;
} b2Island;

// This is used to move islands across solver sets
typedef struct b2IslandSim
{
	int islandId;
} b2IslandSim;

b2Island* b2CreateIsland( b2World* world, int setIndex );
void b2DestroyIsland( b2World* world, int islandId );

// Link contacts into the island graph when it starts having contact points
void b2LinkContact( b2World* world, b2Contact* contact );

// Unlink contact from the island graph when it stops having contact points
void b2UnlinkContact( b2World* world, b2Contact* contact );

// Link a joint into the island graph when it is created
void b2LinkJoint( b2World* world, b2Joint* joint, bool mergeIslands );

// Unlink a joint from the island graph when it is destroyed
void b2UnlinkJoint( b2World* world, b2Joint* joint );

void b2MergeAwakeIslands( b2World* world );

void b2SplitIsland( b2World* world, int baseId );
void b2SplitIslandTask( int startIndex, int endIndex, uint32_t threadIndex, void* context );

void b2ValidateIsland( b2World* world, int islandId );

B2_ARRAY_INLINE( b2Island, b2Island )
B2_ARRAY_INLINE( b2IslandSim, b2IslandSim )
