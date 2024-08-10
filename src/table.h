// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define B2_SHAPE_PAIR_KEY( K1, K2 ) K1 < K2 ? (uint64_t)K1 << 32 | (uint64_t)K2 : (uint64_t)K2 << 32 | (uint64_t)K1

typedef struct b2SetItem
{
	uint64_t key;
	uint32_t hash;
} b2SetItem;

typedef struct b2HashSet
{
	b2SetItem* items;
	uint32_t capacity;
	uint32_t count;
} b2HashSet;

b2HashSet b2CreateSet( int32_t capacity );
void b2DestroySet( b2HashSet* set );

void b2ClearSet( b2HashSet* set );

// Returns true if key was already in set
bool b2AddKey( b2HashSet* set, uint64_t key );

// Returns true if the key was found
bool b2RemoveKey( b2HashSet* set, uint64_t key );

bool b2ContainsKey( const b2HashSet* set, uint64_t key );

int b2GetHashSetBytes( b2HashSet* set );
