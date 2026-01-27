// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"

typedef struct b2Contact b2Contact;
typedef struct b2ContactSim b2ContactSim;
typedef struct b2GraphColor b2GraphColor;

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
	// Base 1 indices so that 0 means null

	// b2Contact index
	// todo currently not used
	int contactIndex;

	// b2BodyState indices
	int indexA;
	int indexB;
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
	b2ContactConstraintPoint points[2];
} b2ContactConstraint;

#if defined( B2_SIMD_AVX2 )

#include <immintrin.h>

// wide float holds 8 numbers
typedef __m256 b2FloatW;

#elif defined( B2_SIMD_NEON )

#include <arm_neon.h>

// wide float holds 4 numbers
typedef float32x4_t b2FloatW;

#elif defined( B2_SIMD_SSE2 )

#include <emmintrin.h>

// wide float holds 4 numbers
typedef __m128 b2FloatW;

#else

// scalar math
typedef struct b2FloatW
{
	float x, y, z, w;
} b2FloatW;

#endif

// Wide vec2
typedef struct b2Vec2W
{
	b2FloatW X, Y;
} b2Vec2W;

typedef struct b2ContactConstraintWide
{
	// Base 1 indices so that 0 means null

	// b2Contact index
	// Not the b2ContactSim index
	int contactIndex[B2_SIMD_WIDTH];

	// b2BodyState indices
	int indexA[B2_SIMD_WIDTH];
	int indexB[B2_SIMD_WIDTH];
	b2FloatW invMassA, invMassB;
	b2FloatW invIA, invIB;
	b2Vec2W normal;
	b2FloatW friction;
	b2FloatW tangentSpeed;
	b2FloatW rollingResistance;
	b2FloatW rollingMass;
	b2FloatW rollingImpulse;
	b2FloatW biasRate;
	b2FloatW massScale;
	b2FloatW impulseScale;
	b2Vec2W anchorA1, anchorB1;
	b2FloatW normalMass1, tangentMass1;
	b2FloatW baseSeparation1;
	b2FloatW normalImpulse1;
	b2FloatW totalNormalImpulse1;
	b2FloatW tangentImpulse1;
	b2Vec2W anchorA2, anchorB2;
	b2FloatW baseSeparation2;
	b2FloatW normalImpulse2;
	b2FloatW totalNormalImpulse2;
	b2FloatW tangentImpulse2;
	b2FloatW normalMass2, tangentMass2;
	b2FloatW restitution;
	b2FloatW relativeVelocity1, relativeVelocity2;
} b2ContactConstraintWide;

int b2GetContactConstraintSIMDByteCount( void );

// Overflow contacts don't fit into the constraint graph coloring
//void b2PrepareOverflowContacts( b2StepContext* context );
void b2WarmStartOverflowContacts( b2StepContext* context );
void b2SolveOverflowContacts( b2StepContext* context, bool useBias );
void b2ApplyOverflowRestitution( b2StepContext* context );
void b2StoreOverflowImpulses( b2StepContext* context );

// Contacts that live within the constraint graph coloring
//void b2PrepareContactsTask( int startIndex, int endIndex, b2StepContext* context );
void b2WarmStartContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex );
void b2SolveContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias );
void b2ApplyRestitutionTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex );
//void b2StoreImpulsesTask( int startIndex, int endIndex, b2StepContext* context );

void b2PrepareContact( b2StepContext* context, b2Contact* contact, bool copyImpulses );
void b2PreparePendingContacts( int startIndex, int endIndex, b2StepContext* context );
