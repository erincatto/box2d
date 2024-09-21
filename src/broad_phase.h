// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "table.h"

#include "box2d/collision.h"
#include "box2d/types.h"

typedef struct b2Shape b2Shape;
typedef struct b2MovePair b2MovePair;
typedef struct b2MoveResult b2MoveResult;
typedef struct b2StackAllocator b2StackAllocator;
typedef struct b2World b2World;

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
#define B2_PROXY_TYPE( KEY ) ( (b2BodyType)( ( KEY ) & 3 ) )
#define B2_PROXY_ID( KEY ) ( ( KEY ) >> 2 )
#define B2_PROXY_KEY( ID, TYPE ) ( ( ( ID ) << 2 ) | ( TYPE ) )

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
typedef struct b2BroadPhase
{
	b2DynamicTree trees[b2_bodyTypeCount];
	int proxyCount;

	// The move set and array are used to track shapes that have moved significantly
	// and need a pair query for new contacts. The array has a deterministic order.
	// todo perhaps just a move set?
	// todo implement a 32bit hash set for faster lookup
	// todo moveSet can grow quite large on the first time step and remain large
	b2HashSet moveSet;
	b2IntArray moveArray;

	// These are the results from the pair query and are used to create new contacts
	// in deterministic order.
	// todo these could be in the step context
	b2MoveResult* moveResults;
	b2MovePair* movePairs;
	int movePairCapacity;
	_Atomic int movePairIndex;

	// Tracks shape pairs that have a b2Contact
	// todo pairSet can grow quite large on the first time step and remain large
	b2HashSet pairSet;

} b2BroadPhase;

void b2CreateBroadPhase( b2BroadPhase* bp );
void b2DestroyBroadPhase( b2BroadPhase* bp );

int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType proxyType, b2AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation );
void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey );

void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb );
void b2BroadPhase_EnlargeProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb );

void b2BroadPhase_RebuildTrees( b2BroadPhase* bp );

int b2BroadPhase_GetShapeIndex( b2BroadPhase* bp, int proxyKey );

void b2UpdateBroadPhasePairs( b2World* world );
bool b2BroadPhase_TestOverlap( const b2BroadPhase* bp, int proxyKeyA, int proxyKeyB );

void b2ValidateBroadphase( const b2BroadPhase* bp );
void b2ValidateNoEnlarged( const b2BroadPhase* bp );

// This is what triggers new contact pairs to be created
// Warning: this must be called in deterministic order
static inline void b2BufferMove( b2BroadPhase* bp, int queryProxy )
{
	// Adding 1 because 0 is the sentinel
	bool alreadyAdded = b2AddKey( &bp->moveSet, queryProxy + 1 );
	if ( alreadyAdded == false )
	{
		b2IntArray_Push( &bp->moveArray, queryProxy );
	}
}
