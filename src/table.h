// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define B2_SHAPE_PAIR_KEY( K1, K2 ) K1 < K2 ? (uint64_t)K1 << 32 | (uint64_t)K2 : (uint64_t)K2 << 32 | (uint64_t)K1

typedef struct b2SetItem
{
	uint64_t key;

	// storing lower 32 bits of hash
	// this is wasteful because I just need to know if the item is occupied
	// I could require the key to be non-zero and use 0 to indicate an empty slot
	// Update: looks like I store this to make growing the table faster, however this is wasteful once
	// the table has hit the high water mark
	//uint32_t hash;
} b2SetItem;

typedef struct b2HashSet
{
	b2SetItem* items;
	uint32_t capacity;
	uint32_t count;
} b2HashSet;

b2HashSet b2CreateSet( int capacity );
void b2DestroySet( b2HashSet* set );

void b2ClearSet( b2HashSet* set );

// Returns true if key was already in set
bool b2AddKey( b2HashSet* set, uint64_t key );

// Returns true if the key was found
bool b2RemoveKey( b2HashSet* set, uint64_t key );

bool b2ContainsKey( const b2HashSet* set, uint64_t key );

int b2GetHashSetBytes( b2HashSet* set );

static inline int b2GetSetCount( b2HashSet* set )
{
	return set->count;
}

static inline int b2GetSetCapacity( b2HashSet* set )
{
	return set->capacity;
}
