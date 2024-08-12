// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "solver.h"

#include "box2d/types.h"

typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
typedef struct b2JointEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b2JointEdge;

// Map from b2JointId to b2Joint in the solver sets
typedef struct b2Joint
{
	void* userData;

	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
	// B2_NULL_INDEX when slot is free
	int colorIndex;

	// joint index within set or graph color
	// B2_NULL_INDEX when slot is free
	int localIndex;

	b2JointEdge edges[2];

	int jointId;
	int islandId;
	int islandPrev;
	int islandNext;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2JointId
	int revision;

	float drawSize;

	b2JointType type;
	bool isMarked;
	bool collideConnected;

} b2Joint;

typedef struct b2DistanceJoint
{
	float length;
	float hertz;
	float dampingRatio;
	float minLength;
	float maxLength;

	float maxMotorForce;
	float motorSpeed;

	float impulse;
	float lowerImpulse;
	float upperImpulse;
	float motorImpulse;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	b2Softness distanceSoftness;
	float axialMass;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
} b2DistanceJoint;

typedef struct b2MotorJoint
{
	b2Vec2 linearOffset;
	float angularOffset;
	b2Vec2 linearImpulse;
	float angularImpulse;
	float maxForce;
	float maxTorque;
	float correctionFactor;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	b2Mat22 linearMass;
	float angularMass;
} b2MotorJoint;

typedef struct b2MouseJoint
{
	b2Vec2 targetA;
	float hertz;
	float dampingRatio;
	float maxForce;

	b2Vec2 linearImpulse;
	float angularImpulse;

	b2Softness linearSoftness;
	b2Softness angularSoftness;
	int indexB;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	b2Mat22 linearMass;
} b2MouseJoint;

typedef struct b2PrismaticJoint
{
	b2Vec2 localAxisA;
	b2Vec2 impulse;
	float springImpulse;
	float motorImpulse;
	float lowerImpulse;
	float upperImpulse;
	float hertz;
	float dampingRatio;
	float maxMotorForce;
	float motorSpeed;
	float referenceAngle;
	float lowerTranslation;
	float upperTranslation;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 axisA;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
} b2PrismaticJoint;

typedef struct b2RevoluteJoint
{
	b2Vec2 linearImpulse;
	float springImpulse;
	float motorImpulse;
	float lowerImpulse;
	float upperImpulse;
	float hertz;
	float dampingRatio;
	float maxMotorTorque;
	float motorSpeed;
	float referenceAngle;
	float lowerAngle;
	float upperAngle;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
} b2RevoluteJoint;

typedef struct b2WeldJoint
{
	float referenceAngle;
	float linearHertz;
	float linearDampingRatio;
	float angularHertz;
	float angularDampingRatio;

	b2Softness linearSoftness;
	b2Softness angularSoftness;
	b2Vec2 linearImpulse;
	float angularImpulse;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
} b2WeldJoint;

typedef struct b2WheelJoint
{
	b2Vec2 localAxisA;
	float perpImpulse;
	float motorImpulse;
	float springImpulse;
	float lowerImpulse;
	float upperImpulse;
	float maxMotorTorque;
	float motorSpeed;
	float lowerTranslation;
	float upperTranslation;
	float hertz;
	float dampingRatio;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 axisA;
	b2Vec2 deltaCenter;
	float perpMass;
	float motorMass;
	float axialMass;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
} b2WheelJoint;

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
typedef struct b2JointSim
{
	int jointId;

	int bodyIdA;
	int bodyIdB;

	b2JointType type;

	// Anchors relative to body origin
	b2Vec2 localOriginAnchorA;
	b2Vec2 localOriginAnchorB;

	float invMassA, invMassB;
	float invIA, invIB;

	union
	{
		b2DistanceJoint distanceJoint;
		b2MotorJoint motorJoint;
		b2MouseJoint mouseJoint;
		b2RevoluteJoint revoluteJoint;
		b2PrismaticJoint prismaticJoint;
		b2WeldJoint weldJoint;
		b2WheelJoint wheelJoint;
	};
} b2JointSim;

b2Joint* b2GetJoint( b2World* world, int jointId );
void b2DestroyJointInternal( b2World* world, b2Joint* joint, bool wakeBodies );

b2JointSim* b2GetJointSim( b2World* world, b2Joint* joint );
b2JointSim* b2GetJointSimCheckType( b2JointId jointId, b2JointType type );

void b2PrepareJoint( b2JointSim* joint, b2StepContext* context );
void b2WarmStartJoint( b2JointSim* joint, b2StepContext* context );
void b2SolveJoint( b2JointSim* joint, b2StepContext* context, bool useBias );

void b2PrepareOverflowJoints( b2StepContext* context );
void b2WarmStartOverflowJoints( b2StepContext* context );
void b2SolveOverflowJoints( b2StepContext* context, bool useBias );

void b2DrawJoint( b2DebugDraw* draw, b2World* world, b2Joint* joint );
