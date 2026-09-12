// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "atomic.h"

#include "box2d/base.h"

#include <stdbool.h>
#include <stdint.h>

#define B2_SHAPE_PAIR_KEY( K1, K2 ) K1 < K2 ? (uint64_t)K1 << 32 | (uint64_t)K2 : (uint64_t)K2 << 32 | (uint64_t)K1

// I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
// todo try:
// https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/

// I compared with CC on https://jacksonallan.github.io/c_cpp_hash_tables_benchmark/ and got slightly better performance
// in the washer benchmark.
// I compared with verstable across 8 benchmarks and the performance was similar.

B2_FORCE_INLINE uint64_t b2KeyHash( uint64_t key )
{
	// Murmur hash
	uint64_t h = key;
	h ^= h >> 33;
	h *= 0xff51afd7ed558ccduLL;
	h ^= h >> 33;
	h *= 0xc4ceb9fe1a85ec53uLL;
	h ^= h >> 33;
	return h;
}

typedef struct b2SetItem
{
	uint64_t key;

	// storing lower 32 bits of hash
	// this is wasteful because I just need to know if the item is occupied
	// I could require the key to be non-zero and use 0 to indicate an empty slot
	// Update: looks like I store this to make growing the table faster, however this is wasteful once
	// the table has hit the high water mark
	// uint32_t hash;
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
bool b2ContainsHashedKey( const b2HashSet* set, uint64_t key, uint64_t hash );

int b2GetHashSetBytes( b2HashSet* set );

static inline int b2GetSetCount( b2HashSet* set )
{
	return set->count;
}

static inline int b2GetSetCapacity( b2HashSet* set )
{
	return set->capacity;
}

static inline void b2PrefetchHash(b2HashSet* set, uint64_t hash)
{
	uint32_t capacity = set->capacity;
	uint32_t index = (uint32_t)hash & ( capacity - 1 );
	b2Prefetch( set->items + index );
}
