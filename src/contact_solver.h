// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver.h"

typedef struct b2BodyState b2BodyState;
typedef struct b2World b2World;

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
	int stateIndexA;
	int stateIndexB;
	b2ContactConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	float restitution;
	float tangentSpeed;
	float rollingResistance;
	float rollingMass;
	float rollingImpulse;
	int pointCount;
} b2ContactConstraint;

void b2PrepareContactConstraints( b2StepContext* context, int* contactIds, b2ContactConstraint* constraints, int count );
void b2WarmStartContactConstraints( b2StepContext* context, b2ContactConstraint* constraints, int count );
void b2SolveContactConstraints( b2StepContext* context, b2ContactConstraint* constraints, int count, float inv_h,
								float contactSpeed, bool useBias );
void b2ApplyContactRestitution( b2StepContext* context, b2ContactConstraint* constraints, int count, float threshold );
void b2StoreContactImpulses( b2World* world, int* contactIds, b2ContactConstraint* constraints, int count );
