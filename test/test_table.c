// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "atomic.h"
#include "core.h"
#include "ctz.h"
#include "table.h"
#include "test_macros.h"

#include "box2d/base.h"

#define SET_SPAN 317
#define ITEM_COUNT ( ( SET_SPAN * SET_SPAN - SET_SPAN ) / 2 )

static int BasicHashSetTest( void )
{
	// Test basic creation and destruction
	b2HashSet set = b2CreateSet( 16 );
	ENSURE( b2GetSetCount( &set ) == 0 );
	ENSURE( b2GetSetCapacity( &set ) == 16 );

	b2DestroySet( &set );
	ENSURE( set.items == NULL );
	ENSURE( set.count == 0 );
	ENSURE( set.capacity == 0 );

	return 0;
}

static int HashSetCapacityTest( void )
{
	// Test capacity adjustments - capacity should be power of 2
	{
		b2HashSet set = b2CreateSet( 1 );
		ENSURE( b2GetSetCapacity( &set ) == 16 ); // Minimum capacity
		b2DestroySet( &set );
	}

	{
		b2HashSet set = b2CreateSet( 15 );
		ENSURE( b2GetSetCapacity( &set ) == 16 ); // Should round up to 16
		b2DestroySet( &set );
	}

	{
		b2HashSet set = b2CreateSet( 32 );
		ENSURE( b2GetSetCapacity( &set ) == 32 ); // Should stay at 32
		b2DestroySet( &set );
	}

	{
		b2HashSet set = b2CreateSet( 33 );
		ENSURE( b2GetSetCapacity( &set ) == 64 ); // Should round up to 64
		b2DestroySet( &set );
	}

	return 0;
}

static int HashSetAddRemoveTest( void )
{
	b2HashSet set = b2CreateSet( 16 );

	// Test adding new keys
	bool found = b2AddKey( &set, 42 );
	ENSURE( found == false ); // Should be new
	ENSURE( b2GetSetCount( &set ) == 1 );

	found = b2AddKey( &set, 123 );
	ENSURE( found == false ); // Should be new
	ENSURE( b2GetSetCount( &set ) == 2 );

	// Test adding duplicate key
	found = b2AddKey( &set, 42 );
	ENSURE( found == true );			  // Should already exist
	ENSURE( b2GetSetCount( &set ) == 2 ); // Count shouldn't change

	// Test contains
	ENSURE( b2ContainsKey( &set, 42 ) == true );
	ENSURE( b2ContainsKey( &set, 123 ) == true );
	ENSURE( b2ContainsKey( &set, 999 ) == false );

	// Test removal
	bool removed = b2RemoveKey( &set, 42 );
	ENSURE( removed == true );
	ENSURE( b2GetSetCount( &set ) == 1 );
	ENSURE( b2ContainsKey( &set, 42 ) == false );
	ENSURE( b2ContainsKey( &set, 123 ) == true );

	// Test removing non-existent key
	removed = b2RemoveKey( &set, 999 );
	ENSURE( removed == false );
	ENSURE( b2GetSetCount( &set ) == 1 );

	// Test removing same key twice
	removed = b2RemoveKey( &set, 42 );
	ENSURE( removed == false );
	ENSURE( b2GetSetCount( &set ) == 1 );

	b2DestroySet( &set );
	return 0;
}

static int HashSetClearTest( void )
{
	b2HashSet set = b2CreateSet( 16 );

	// Add some keys
	b2AddKey( &set, 10 );
	b2AddKey( &set, 20 );
	b2AddKey( &set, 30 );
	ENSURE( b2GetSetCount( &set ) == 3 );

	// Clear the set
	b2ClearSet( &set );
	ENSURE( b2GetSetCount( &set ) == 0 );
	ENSURE( b2ContainsKey( &set, 10 ) == false );
	ENSURE( b2ContainsKey( &set, 20 ) == false );
	ENSURE( b2ContainsKey( &set, 30 ) == false );

	// Test that we can add keys after clearing
	b2AddKey( &set, 40 );
	ENSURE( b2GetSetCount( &set ) == 1 );
	ENSURE( b2ContainsKey( &set, 40 ) == true );

	b2DestroySet( &set );
	return 0;
}

static int HashSetGrowthTest( void )
{
	b2HashSet set = b2CreateSet( 16 );
	int initialCapacity = b2GetSetCapacity( &set );

	// Add enough keys to trigger growth (load factor is 0.5)
	// With capacity 16, growth should happen when count reaches 8
	for ( uint64_t i = 0; i < 8; ++i )
	{
		b2AddKey( &set, i + 1);
	}

	// Should have grown
	int newCapacity = b2GetSetCapacity( &set );
	ENSURE( newCapacity >= initialCapacity );
	ENSURE( b2GetSetCount( &set ) == 8 );

	// Verify all keys are still present after growth
	for ( uint64_t i = 1; i <= 8; ++i )
	{
		ENSURE( b2ContainsKey( &set, i ) == true );
	}

	b2DestroySet( &set );
	return 0;
}

static int HashSetEdgeCasesTest( void )
{
	b2HashSet set = b2CreateSet( 16 );

	// Test large key values
	uint64_t largeKey = 0xFFFFFFFFFFFFFFFFULL - 1; // Max value minus 1 (since 0 is sentinel)
	b2AddKey( &set, largeKey );
	ENSURE( b2ContainsKey( &set, largeKey ) == true );
	ENSURE( b2GetSetCount( &set ) == 1 );

	// Test keys that might cause hash collisions
	uint64_t key1 = 0x123456789ABCDEFULL;
	uint64_t key2 = 0x987654321FEDCBAULL;
	b2AddKey( &set, key1 );
	b2AddKey( &set, key2 );
	ENSURE( b2ContainsKey( &set, key1 ) == true );
	ENSURE( b2ContainsKey( &set, key2 ) == true );

	// Test pattern that could cause clustering
	for ( uint64_t i = 0x1000; i < 0x1010; ++i )
	{
		b2AddKey( &set, i );
	}

	for ( uint64_t i = 0x1000; i < 0x1010; ++i )
	{
		ENSURE( b2ContainsKey( &set, i ) == true );
	}

	b2DestroySet( &set );
	return 0;
}

static int HashSetRemovalReorganizationTest( void )
{
	b2HashSet set = b2CreateSet( 16 );

	// Add keys that might cluster together
	uint64_t keys[] = { 100, 116, 132, 148, 164 }; // These might hash to similar slots
	int keyCount = sizeof( keys ) / sizeof( keys[0] );

	for ( int i = 0; i < keyCount; ++i )
	{
		b2AddKey( &set, keys[i] );
	}

	// Verify all keys are present
	for ( int i = 0; i < keyCount; ++i )
	{
		ENSURE( b2ContainsKey( &set, keys[i] ) == true );
	}

	// Remove a key from the middle
	b2RemoveKey( &set, keys[2] );
	ENSURE( b2ContainsKey( &set, keys[2] ) == false );

	// Verify other keys are still present (tests reorganization)
	for ( int i = 0; i < keyCount; ++i )
	{
		if ( i != 2 )
		{
			ENSURE( b2ContainsKey( &set, keys[i] ) == true );
		}
	}

	b2DestroySet( &set );
	return 0;
}

#define TEST_SIZE 1000

static int HashSetStressTest( void )
{
	b2HashSet set = b2CreateSet( 32 );

	const int testSize = TEST_SIZE;
	uint64_t keys[TEST_SIZE];

	// Generate test keys
	for ( int i = 0; i < testSize; ++i )
	{
		keys[i] = (uint64_t)( i * 7 + 13 ); // Some pattern to avoid zero
	}

	// Add all keys
	for ( int i = 0; i < testSize; ++i )
	{
		bool found = b2AddKey( &set, keys[i] );
		ENSURE( found == false );
	}

	ENSURE( b2GetSetCount( &set ) == testSize );

	// Verify all keys are present
	for ( int i = 0; i < testSize; ++i )
	{
		ENSURE( b2ContainsKey( &set, keys[i] ) == true );
	}

	// Remove every other key
	int removedCount = 0;
	for ( int i = 0; i < testSize; i += 2 )
	{
		bool removed = b2RemoveKey( &set, keys[i] );
		ENSURE( removed == true );
		removedCount++;
	}

	ENSURE( b2GetSetCount( &set ) == testSize - removedCount );

	// Verify remaining keys are still present
	for ( int i = 0; i < testSize; ++i )
	{
		bool shouldBePresent = ( i % 2 == 1 );
		ENSURE( b2ContainsKey( &set, keys[i] ) == shouldBePresent );
	}

	b2DestroySet( &set );
	return 0;
}

static int HashSetShapePairKeyTest( void )
{
	b2HashSet set = b2CreateSet( 16 );

	// Test the B2_SHAPE_PAIR_KEY macro
	uint64_t key1 = B2_SHAPE_PAIR_KEY( 5, 10 );
	uint64_t key2 = B2_SHAPE_PAIR_KEY( 10, 5 ); // Should be same as key1
	ENSURE( key1 == key2 );

	b2AddKey( &set, key1 );
	ENSURE( b2ContainsKey( &set, key1 ) == true );
	ENSURE( b2ContainsKey( &set, key2 ) == true ); // Should find same key

	// Test different pairs
	uint64_t key3 = B2_SHAPE_PAIR_KEY( 1, 2 );
	uint64_t key4 = B2_SHAPE_PAIR_KEY( 2, 3 );
	ENSURE( key3 != key4 );

	b2AddKey( &set, key3 );
	b2AddKey( &set, key4 );
	ENSURE( b2GetSetCount( &set ) == 3 );

	b2DestroySet( &set );
	return 0;
}

static int HashSetBytesTest( void )
{
	b2HashSet set = b2CreateSet( 32 );

	int bytes = b2GetHashSetBytes( &set );
	int expectedBytes = 32 * (int)sizeof( b2SetItem );
	ENSURE( bytes == expectedBytes );

	// Add some items and verify bytes calculation doesn't change
	b2AddKey( &set, 100 );
	b2AddKey( &set, 200 );

	int bytesAfterAdd = b2GetHashSetBytes( &set );
	ENSURE( bytesAfterAdd == expectedBytes );

	b2DestroySet( &set );
	return 0;
}

static int HashSetTest( void )
{
	const int N = SET_SPAN;
	const uint32_t itemCount = ITEM_COUNT;
	bool removed[ITEM_COUNT] = { 0 };

	for ( int iter = 0; iter < 1; ++iter )
	{
		b2HashSet set = b2CreateSet( 16 );

		// Fill set
		for ( int i = 0; i < N; ++i )
		{
			for ( int j = i + 1; j < N; ++j )
			{
				uint64_t key = B2_SHAPE_PAIR_KEY( i, j );
				bool found = b2AddKey( &set, key );
				ENSURE( found == false );
			}
		}

		ENSURE( b2GetSetCount( &set ) == itemCount );

		// Remove a portion of the set
		int k = 0;
		uint32_t removeCount = 0;
		for ( int i = 0; i < N; ++i )
		{
			for ( int j = i + 1; j < N; ++j )
			{
				if ( j == i + 1 )
				{
					uint64_t key = B2_SHAPE_PAIR_KEY( i, j );
					int size1 = b2GetSetCount( &set );
					bool found = b2RemoveKey( &set, key );
					ENSURE( found );
					int size2 = b2GetSetCount( &set );
					ENSURE( size2 == size1 - 1 );
					removed[k++] = true;
					removeCount += 1;
				}
				else
				{
					removed[k++] = false;
				}
			}
		}

		ENSURE( b2GetSetCount( &set ) == ( itemCount - removeCount ) );

#if B2_SNOOP_TABLE_COUNTERS
		extern b2AtomicInt b2_probeCount;
		b2AtomicStoreInt( &b2_probeCount, 0 );
#endif

		// Test key search
		// ~5ns per search on an AMD 7950x
		uint64_t ticks = b2GetTicks();

		k = 0;
		for ( int i = 0; i < N; ++i )
		{
			for ( int j = i + 1; j < N; ++j )
			{
				uint64_t key = B2_SHAPE_PAIR_KEY( j, i );
				bool found = b2ContainsKey( &set, key );
				ENSURE( found || removed[k] );
				k += 1;
			}
		}

		// uint64_t ticks = b2GetTicks(&timer);
		// printf("set ticks = %llu\n", ticks);

		float ms = b2GetMilliseconds( ticks );
		printf( "set: count = %d, b2ContainsKey = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0f * ms / itemCount );

#if B2_SNOOP_TABLE_COUNTERS
		int probeCount = b2AtomicLoadInt( &b2_probeCount );
		float aveProbeCount = (float)probeCount / (float)itemCount;
		printf( "item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, probeCount, aveProbeCount );
#endif

		// Remove all keys from set
		for ( int i = 0; i < N; ++i )
		{
			for ( int j = i + 1; j < N; ++j )
			{
				uint64_t key = B2_SHAPE_PAIR_KEY( i, j );
				b2RemoveKey( &set, key );
			}
		}

		ENSURE( b2GetSetCount( &set ) == 0 );

		b2DestroySet( &set );
	}

	return 0;
}

int TableTest( void )
{
	// Test helper functions first
	int power = b2BoundingPowerOf2( 3008 );
	ENSURE( power == 12 );

	int nextPowerOf2 = b2RoundUpPowerOf2( 3008 );
	ENSURE( nextPowerOf2 == ( 1 << power ) );

	// Run all hash set tests
	RUN_SUBTEST( BasicHashSetTest );
	RUN_SUBTEST( HashSetCapacityTest );
	RUN_SUBTEST( HashSetAddRemoveTest );
	RUN_SUBTEST( HashSetClearTest );
	RUN_SUBTEST( HashSetGrowthTest );
	RUN_SUBTEST( HashSetEdgeCasesTest );
	RUN_SUBTEST( HashSetRemovalReorganizationTest );
	RUN_SUBTEST( HashSetStressTest );
	RUN_SUBTEST( HashSetShapePairKeyTest );
	RUN_SUBTEST( HashSetBytesTest );
	RUN_SUBTEST( HashSetTest );

	return 0;
}
