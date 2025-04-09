// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

typedef struct b2Body b2Body;
typedef struct b2Joint b2Joint;
typedef struct b2World b2World;

// This holds solver set data. The following sets are used:
// - static set for all static bodies (no contacts or joints)
// - active set for all active bodies with body states (no contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
typedef struct b2SolverSet
{
	// Body array. Empty for unused set.
	b2BodySimArray bodySims;

	// Body state only exists for active set
	b2BodyStateArray bodyStates;

	// This holds sleeping/disabled joints. Empty for static/active set.
	b2JointSimArray jointSims;

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	b2ContactSimArray contactSims;

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	b2IslandSimArray islandSims;

	// Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	int setIndex;
} b2SolverSet;

void b2DestroySolverSet( b2World* world, int setIndex );

void b2WakeSolverSet( b2World* world, int setIndex );
void b2TrySleepIsland( b2World* world, int islandId );

// Merge set 2 into set 1 then destroy set 2.
// Warning: any pointers into these sets will be orphaned.
void b2MergeSolverSets( b2World* world, int setIndex1, int setIndex2 );

void b2TransferBody( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Body* body );
void b2TransferJoint( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Joint* joint );

B2_ARRAY_INLINE( b2SolverSet, b2SolverSet )
