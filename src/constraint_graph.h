// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"

typedef struct b2Body b2Body;
typedef struct b2ContactSim b2ContactSim;
typedef struct b2Contact b2Contact;
typedef struct b2ContactConstraint b2ContactConstraint;
typedef struct b2ContactConstraintSIMD b2ContactConstraintSIMD;
typedef struct b2JointSim b2JointSim;
typedef struct b2Joint b2Joint;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
#define b2_overflowIndex b2_graphColorCount - 1

typedef struct b2GraphColor
{
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	b2BitSet bodySet;

	// cache friendly arrays
	b2ContactSimArray contactSims;
	b2JointSimArray jointSims;

	// transient
	union
	{
		b2ContactConstraintSIMD* simdConstraints;
		b2ContactConstraint* overflowConstraints;
	};
} b2GraphColor;

typedef struct b2ConstraintGraph
{
	// including overflow at the end
	b2GraphColor colors[b2_graphColorCount];
} b2ConstraintGraph;

void b2CreateGraph( b2ConstraintGraph* graph, int bodyCapacity );
void b2DestroyGraph( b2ConstraintGraph* graph );

void b2AddContactToGraph( b2World* world, b2ContactSim* contactSim, b2Contact* contact );
void b2RemoveContactFromGraph( b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex );

b2JointSim* b2CreateJointInGraph( b2World* world, b2Joint* joint );
void b2AddJointToGraph( b2World* world, b2JointSim* jointSim, b2Joint* joint );
void b2RemoveJointFromGraph( b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex );
