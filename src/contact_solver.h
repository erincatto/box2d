// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"
#include "x86/avx.h"

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
	b2Softness softness;
	int pointCount;
} b2ContactConstraint;

// Wide float
typedef simde__m256 b2FloatW;

// Wide vec2
typedef struct b2Vec2W
{
	b2FloatW X, Y;
} b2Vec2W;

// Wide rotation
typedef struct b2RotW
{
	b2FloatW S, C;
} b2RotW;

typedef struct b2ContactConstraintSIMD
{
	int indexA[8];
	int indexB[8];

	b2FloatW invMassA, invMassB;
	b2FloatW invIA, invIB;
	b2Vec2W normal;
	b2FloatW friction;
	b2FloatW biasRate;
	b2FloatW massScale;
	b2FloatW impulseScale;
	b2Vec2W anchorA1, anchorB1;
	b2FloatW normalMass1, tangentMass1;
	b2FloatW baseSeparation1;
	b2FloatW normalImpulse1;
	b2FloatW maxNormalImpulse1;
	b2FloatW tangentImpulse1;
	b2Vec2W anchorA2, anchorB2;
	b2FloatW baseSeparation2;
	b2FloatW normalImpulse2;
	b2FloatW maxNormalImpulse2;
	b2FloatW tangentImpulse2;
	b2FloatW normalMass2, tangentMass2;
	b2FloatW restitution;
	b2FloatW relativeVelocity1, relativeVelocity2;
} b2ContactConstraintSIMD;

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
