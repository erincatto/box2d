// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

#include "box2d/math_functions.h"
#include "box2d/types.h"

typedef struct b2World b2World;

// Body organizational details that are not used in the solver.
typedef struct b2Body
{
	char name[32];

	void* userData;

	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	int setIndex;

	// body sim and state index within set
	// may be B2_NULL_INDEX
	int localIndex;

	// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;

	// todo maybe move this to the body sim
	int headShapeId;
	int shapeCount;

	int headChainId;

	// [31 : jointId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;

	// All enabled dynamic and kinematic bodies are in an island.
	int islandId;

	// doubly-linked island list
	int islandPrev;
	int islandNext;

	float mass;

	// Rotational inertia about the center of mass.
	float inertia;

	float sleepThreshold;
	float sleepTime;

	// this is used to adjust the fellAsleep flag in the body move array
	int bodyMoveIndex;

	int id;

	b2BodyType type;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2BodyId
	uint16_t generation;

	bool enableSleep;
	bool fixedRotation;
	bool isSpeedCapped;
	bool isMarked;
} b2Body;

// Body State
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.

// 32 bytes
typedef struct b2BodyState
{
	b2Vec2 linearVelocity; // 8
	float angularVelocity; // 4
	int flags;			   // 4

	// Using delta position reduces round-off error far from the origin
	b2Vec2 deltaPosition; // 8

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	b2Rot deltaRotation; // 8
} b2BodyState;

// Identity body state, notice the deltaRotation is {1, 0}
static const b2BodyState b2_identityBodyState = { { 0.0f, 0.0f }, 0.0f, 0, { 0.0f, 0.0f }, { 1.0f, 0.0f } };

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
typedef struct b2BodySim
{
	// todo better to have transform in sim or in base body? Try both!
	// transform for body origin
	b2Transform transform;

	// center of mass position in world space
	b2Vec2 center;

	// previous rotation and COM for TOI
	b2Rot rotation0;
	b2Vec2 center0;

	// location of center of mass relative to the body origin
	b2Vec2 localCenter;

	b2Vec2 force;
	float torque;

	// inverse inertia
	float invMass;
	float invInertia;

	float minExtent;
	float maxExtent;
	float linearDamping;
	float angularDamping;
	float gravityScale;

	// body data can be moved around, the id is stable (used in b2BodyId)
	int bodyId;

	// This flag is used for debug draw
	bool isFast;

	bool isBullet;
	bool isSpeedCapped;
	bool allowFastRotation;
	bool enlargeAABB;
} b2BodySim;

// Get a validated body from a world using an id.
b2Body* b2GetBodyFullId( b2World* world, b2BodyId bodyId );

b2Transform b2GetBodyTransformQuick( b2World* world, b2Body* body );
b2Transform b2GetBodyTransform( b2World* world, int bodyId );

// Create a b2BodyId from a raw id.
b2BodyId b2MakeBodyId( b2World* world, int bodyId );

bool b2ShouldBodiesCollide( b2World* world, b2Body* bodyA, b2Body* bodyB );

b2BodySim* b2GetBodySim( b2World* world, b2Body* body );
b2BodyState* b2GetBodyState( b2World* world, b2Body* body );

// careful calling this because it can invalidate body, state, joint, and contact pointers
bool b2WakeBody( b2World* world, b2Body* body );

void b2UpdateBodyMassData( b2World* world, b2Body* body );

static inline b2Sweep b2MakeSweep( const b2BodySim* bodySim )
{
	b2Sweep s;
	s.c1 = bodySim->center0;
	s.c2 = bodySim->center;
	s.q1 = bodySim->rotation0;
	s.q2 = bodySim->transform.q;
	s.localCenter = bodySim->localCenter;
	return s;
}

// Define inline functions for arrays
B2_ARRAY_INLINE( b2Body, b2Body )
B2_ARRAY_INLINE( b2BodySim, b2BodySim )
B2_ARRAY_INLINE( b2BodyState, b2BodyState )
