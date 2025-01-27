// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "core.h"

#include "box2d/collision.h"
#include "box2d/types.h"

typedef struct b2Shape b2Shape;
typedef struct b2World b2World;

enum b2ContactFlags
{
	// Set when the solid shapes are touching.
	b2_contactTouchingFlag = 0x00000001,

	// Contact has a hit event
	b2_contactHitEventFlag = 0x00000002,

	// This contact wants contact events
	b2_contactEnableContactEvents = 0x00000004,
};

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
typedef struct b2ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b2ContactEdge;

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
typedef struct b2Contact
{
	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array
	// B2_NULL_INDEX for non-touching or sleeping contacts
	// B2_NULL_INDEX when slot is free
	int colorIndex;

	// contact index within set or graph color
	// B2_NULL_INDEX when slot is free
	int localIndex;

	b2ContactEdge edges[2];
	int shapeIdA;
	int shapeIdB;

	// A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
	int islandPrev;
	int islandNext;
	int islandId;

	int contactId;

	// b2ContactFlags
	uint32_t flags;

	bool isMarked;
} b2Contact;

// Shifted to be distinct from b2ContactFlags
enum b2ContactSimFlags
{
	// Set when the shapes are touching, including sensors
	b2_simTouchingFlag = 0x00010000,

	// This contact no longer has overlapping AABBs
	b2_simDisjoint = 0x00020000,

	// This contact started touching
	b2_simStartedTouching = 0x00040000,

	// This contact stopped touching
	b2_simStoppedTouching = 0x00080000,

	// This contact has a hit event
	b2_simEnableHitEvent = 0x00100000,

	// This contact wants pre-solve events
	b2_simEnablePreSolveEvents = 0x00200000,
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
typedef struct b2ContactSim
{
	int contactId;

#if B2_VALIDATE
	int bodyIdA;
	int bodyIdB;
#endif

	int bodySimIndexA;
	int bodySimIndexB;

	int shapeIdA;
	int shapeIdB;

	float invMassA;
	float invIA;

	float invMassB;
	float invIB;

	b2Manifold manifold;

	// Mixed friction and restitution
	float friction;
	float restitution;
	float rollingResistance;
	float tangentSpeed;

	// b2ContactSimFlags
	uint32_t simFlags;

	b2SimplexCache cache;
} b2ContactSim;

void b2InitializeContactRegisters( void );

void b2CreateContact( b2World* world, b2Shape* shapeA, b2Shape* shapeB );
void b2DestroyContact( b2World* world, b2Contact* contact, bool wakeBodies );

b2ContactSim* b2GetContactSim( b2World* world, b2Contact* contact );

bool b2ShouldShapesCollide( b2Filter filterA, b2Filter filterB );

bool b2UpdateContact( b2World* world, b2ContactSim* contactSim, b2Shape* shapeA, b2Transform transformA, b2Vec2 centerOffsetA,
					  b2Shape* shapeB, b2Transform transformB, b2Vec2 centerOffsetB );

b2Manifold b2ComputeManifold( b2Shape* shapeA, b2Transform transformA, b2Shape* shapeB, b2Transform transformB );

B2_ARRAY_INLINE( b2Contact, b2Contact );
B2_ARRAY_INLINE( b2ContactSim, b2ContactSim );
