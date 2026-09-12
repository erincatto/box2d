// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "table.h"

#include "atomic.h"
#include "bitset.h"
#include "core.h"
#include "ctz.h"

#include <stdbool.h>
#include <string.h>

#if B2_SNOOP_TABLE_COUNTERS
b2AtomicInt b2_findCount;
b2AtomicInt b2_probeCount;
#endif

b2HashSet b2CreateSet( int capacity )
{
	b2HashSet set = { 0 };

	// Capacity must be a power of 2
	if ( capacity > 16 )
	{
		set.capacity = b2RoundUpPowerOf2( capacity );
	}
	else
	{
		set.capacity = 16;
	}

	set.count = 0;
	set.items = b2Alloc( set.capacity * sizeof( b2SetItem ) );
	memset( set.items, 0, set.capacity * sizeof( b2SetItem ) );

	return set;
}

void b2DestroySet( b2HashSet* set )
{
	b2Free( set->items, set->capacity * sizeof( b2SetItem ) );
	set->items = NULL;
	set->count = 0;
	set->capacity = 0;
}

void b2ClearSet( b2HashSet* set )
{
	set->count = 0;
	memset( set->items, 0, set->capacity * sizeof( b2SetItem ) );
}

static int b2FindSlot( const b2HashSet* set, uint64_t key, uint64_t hash )
{
#if B2_SNOOP_TABLE_COUNTERS
	b2AtomicFetchAddInt( &b2_findCount, 1 );
#endif

	uint32_t capacity = set->capacity;
	uint32_t index = (uint32_t)hash & ( capacity - 1 );
	const b2SetItem* items = set->items;
	while ( items[index].key != 0 && items[index].key != key )
	{
#if B2_SNOOP_TABLE_COUNTERS
		b2AtomicFetchAddInt( &b2_probeCount, 1 );
#endif
		index = ( index + 1 ) & ( capacity - 1 );
	}

	return index;
}

static void b2AddKeyHaveCapacity( b2HashSet* set, uint64_t key, uint64_t hash )
{
	int index = b2FindSlot( set, key, hash );
	b2SetItem* items = set->items;

	B2_ASSERT( items[index].key == 0 );
	items[index].key = key;
	set->count += 1;
}

static void b2GrowTable( b2HashSet* set )
{
	uint32_t oldCount = set->count;
	B2_UNUSED( oldCount );

	uint32_t oldCapacity = set->capacity;
	b2SetItem* oldItems = set->items;

	set->count = 0;
	// Capacity must be a power of 2
	set->capacity = 2 * oldCapacity;
	set->items = b2Alloc( set->capacity * sizeof( b2SetItem ) );
	memset( set->items, 0, set->capacity * sizeof( b2SetItem ) );

	// Transfer items into new array
	for ( uint32_t i = 0; i < oldCapacity; ++i )
	{
		b2SetItem* item = oldItems + i;
		if ( item->key == 0 )
		{
			// this item was empty
			continue;
		}

		uint64_t hash = b2KeyHash( item->key );
		b2AddKeyHaveCapacity( set, item->key, hash );
	}

	B2_ASSERT( set->count == oldCount );

	b2Free( oldItems, oldCapacity * sizeof( b2SetItem ) );
}

bool b2ContainsKey( const b2HashSet* set, uint64_t key )
{
	// key of zero is a sentinel
	B2_VALIDATE( key != 0 );
	uint64_t hash = b2KeyHash( key );
	int index = b2FindSlot( set, key, hash );
	return set->items[index].key == key;
}

bool b2ContainsHashedKey( const b2HashSet* set, uint64_t key, uint64_t hash )
{
	// key of zero is a sentinel
	B2_VALIDATE( key != 0 );
	int index = b2FindSlot( set, key, hash );
	return set->items[index].key == key;
}

int b2GetHashSetBytes( b2HashSet* set )
{
	return set->capacity * (int)sizeof( b2SetItem );
}

bool b2AddKey( b2HashSet* set, uint64_t key )
{
	// key of zero is a sentinel
	B2_ASSERT( key != 0 );

	uint64_t hash = b2KeyHash( key );
	B2_ASSERT( hash != 0 );

	int index = b2FindSlot( set, key, hash );
	if ( set->items[index].key != 0 )
	{
		// Already in set
		B2_ASSERT( set->items[index].key == key );
		return true;
	}

	if ( 2 * set->count >= set->capacity )
	{
		b2GrowTable( set );
	}

	b2AddKeyHaveCapacity( set, key, hash );
	return false;
}

// See https://en.wikipedia.org/wiki/Open_addressing
bool b2RemoveKey( b2HashSet* set, uint64_t key )
{
	uint64_t hash = b2KeyHash( key );
	int i = b2FindSlot( set, key, hash );
	b2SetItem* items = set->items;
	if ( items[i].key == 0 )
	{
		// Not in set
		return false;
	}

	// Mark item i as unoccupied
	items[i].key = 0;

	B2_ASSERT( set->count > 0 );
	set->count -= 1;

	// Attempt to fill item i
	int j = i;
	uint32_t capacity = set->capacity;
	for ( ;; )
	{
		j = ( j + 1 ) & ( capacity - 1 );
		if ( items[j].key == 0 )
		{
			break;
		}

		// k is the first item for the hash of j
		uint64_t hash_j = b2KeyHash( items[j].key );
		int k = hash_j & ( capacity - 1 );

		// determine if k lies cyclically in (i,j]
		// i <= j: | i..k..j |
		// i > j: |.k..j  i....| or |....j     i..k.|
		if ( i <= j )
		{
			if ( i < k && k <= j )
			{
				continue;
			}
		}
		else
		{
			if ( i < k || k <= j )
			{
				continue;
			}
		}

		// Move j into i
		items[i] = items[j];

		// Mark item j as unoccupied
		items[j].key = 0;

		i = j;
	}

	return true;
}

// This function is here because ctz.h is included by
// this file but not in bitset.c
int b2CountSetBits( b2BitSet* bitSet )
{
	int popCount = 0;
	uint32_t blockCount = bitSet->blockCount;
	for ( uint32_t i = 0; i < blockCount; ++i )
	{
		popCount += b2PopCount64( bitSet->bits[i] );
	}

	return popCount;
}
