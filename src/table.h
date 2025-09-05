// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdalign.h>
#include <stdbool.h>
#include <stdint.h>

#define B2_SHAPE_PAIR_KEY( K1, K2 ) K1 < K2 ? (uint64_t)K1 << 32 | (uint64_t)K2 : (uint64_t)K2 << 32 | (uint64_t)K1

#if 1

#define NAME ver_set
#define KEY_TY uint64_t
#include "verstable.h"

#pragma warning( disable : 4100 4189 4127 4774 4305 )

#include "box2d/base.h"

#define b2HashSet ver_set

static inline b2HashSet b2CreateSet( int capacity )
{
	b2HashSet set;
	vt_init( &set );
	bool success = vt_reserve( &set, capacity );
	B2_ASSERT( success );
	return set;
}

static inline void b2DestroySet( b2HashSet* set )
{
	vt_cleanup( set );
}

static inline void b2ClearSet( b2HashSet* set )
{
	vt_clear( set );
}

// Returns true if key was already in set
static inline bool b2AddKey( b2HashSet* set, uint64_t key )
{
	size_t size1 = vt_size( set );
	vt_insert( set, key );
	size_t size2 = vt_size( set );
	return size1 == size2;
}

// Returns true if the key was found
static inline bool b2RemoveKey( b2HashSet* set, uint64_t key )
{
	return vt_erase( set, key );
}

static inline bool b2ContainsKey( const b2HashSet* set, uint64_t key )
{
	ver_set_itr itr = vt_get( set, key );
	return vt_is_end( itr ) == false;
}

static inline int b2GetHashSetBytes( b2HashSet* set )
{
	return 0;
}

static inline int b2GetSetCount( b2HashSet* set )
{
	return (int)vt_size( set );
}

static inline int b2GetSetCapacity( b2HashSet* set )
{
	return (int)vt_bucket_count( set );
}
#else

typedef struct b2SetItem
{
	uint64_t key;
	uint64_t hash;
} b2SetItem;

typedef struct b2SetItem32
{
	uint32_t key;
	uint32_t hash;
} b2SetItem32;

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

#endif
