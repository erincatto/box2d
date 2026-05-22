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
	float totalNormalImpulse;
	float normalMass;
	float tangentMass;
} b2ContactConstraintPoint;

typedef struct b2ContactConstraint
{
	// base-1, 0 for null
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

// This function allows hiding SIMD intrinsics in the source file to improve compilation performance.
int b2GetWideContactConstraintByteCount( void );

// Overflow contacts don't fit into the constraint graph coloring
void b2PrepareContacts_Overflow( b2StepContext* context );
void b2WarmStartContacts_Overflow( b2StepContext* context );
void b2SolveContacts_Overflow( b2StepContext* context, bool useBias );
void b2ApplyRestitution_Overflow( b2StepContext* context );
void b2StoreImpulses_Overflow( b2StepContext* context );

// Contacts that live within the constraint graph coloring
void b2PrepareContactsTask( b2SolverBlock block, b2StepContext* context );
void b2WarmStartContactsTask( b2SolverBlock block, b2StepContext* context );
void b2SolveContactsTask( b2SolverBlock block, b2StepContext* context, bool useBias );
void b2ApplyRestitutionTask( b2SolverBlock block, b2StepContext* context );
void b2StoreImpulsesTask( b2SolverBlock block, b2StepContext* context, int workerIndex );
