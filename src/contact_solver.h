// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"

typedef struct b2ContactSim b2ContactSim;

typedef struct b2ContactConstraintPoint
{
	b2Vec2 anchorA, anchorB;
	float baseSeparation;
	float relativeVelocity;
	float normalImpulse;
	float tangentImpulse;
	float maxNormalImpulse;
	float normalMass;
	float tangentMass;
} b2ContactConstraintPoint;

typedef struct b2ContactConstraint
{
	int indexA;
	int indexB;
	b2ContactConstraintPoint points[2];
	b2Vec2 normal;
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float tangentSpeed;
	float rollingResistance;
	float rollingMass;
	float rollingImpulse;
	b2Softness softness;
	int pointCount;
} b2ContactConstraint;

int b2GetContactConstraintSIMDByteCount( void );

// Overflow contacts don't fit into the constraint graph coloring
void b2PrepareOverflowContacts( b2StepContext* context );
void b2WarmStartOverflowContacts( b2StepContext* context );
void b2SolveOverflowContacts( b2StepContext* context, bool useBias );
void b2ApplyOverflowRestitution( b2StepContext* context );
void b2StoreOverflowImpulses( b2StepContext* context );

// Contacts that live within the constraint graph coloring
void b2PrepareContactsTask( int startIndex, int endIndex, b2StepContext* context );
void b2WarmStartContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex );
void b2SolveContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias );
void b2ApplyRestitutionTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex );
void b2StoreImpulsesTask( int startIndex, int endIndex, b2StepContext* context );
