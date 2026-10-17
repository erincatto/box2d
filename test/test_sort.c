// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "broad_phase.h"
#include "test_macros.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_KEYS 4096

static uint64_t s_keys[MAX_KEYS];
static uint64_t s_expected[MAX_KEYS];
static uint64_t s_temp[MAX_KEYS];

static uint64_t s_state = 0x9E3779B97F4A7C15ull;

static uint64_t NextRandom( void )
{
	// xorshift64*
	s_state ^= s_state >> 12;
	s_state ^= s_state << 25;
	s_state ^= s_state >> 27;
	return s_state * 0x2545F4914F6CDD1Dull;
}

static int CompareKeys( const void* a, const void* b )
{
	uint64_t ka = *(const uint64_t*)a;
	uint64_t kb = *(const uint64_t*)b;
	return ka < kb ? -1 : ( ka > kb ? 1 : 0 );
}

static int SortAndCheck( int count )
{
	memcpy( s_expected, s_keys, count * sizeof( uint64_t ) );
	qsort( s_expected, count, sizeof( uint64_t ), CompareKeys );

	b2RadixSortKeys( s_keys, s_temp, count );

	for ( int i = 0; i < count; ++i )
	{
		ENSURE( s_keys[i] == s_expected[i] );
	}

	return 0;
}

static int TestRandomFullWidth( void )
{
	int counts[] = { 2, 3, 17, 255, 256, 257, 1000, MAX_KEYS };
	for ( int c = 0; c < (int)( sizeof( counts ) / sizeof( counts[0] ) ); ++c )
	{
		for ( int i = 0; i < counts[c]; ++i )
		{
			s_keys[i] = NextRandom();
		}

		ENSURE( SortAndCheck( counts[c] ) == 0 );
	}

	return 0;
}

static int TestShapePairKeys( void )
{
	//int count = 2000;
	int count = 8;
	for ( int i = 0; i < count; ++i )
	{
		uint64_t shapeA = NextRandom() % 300;
		uint64_t shapeB = NextRandom() % 300;
		s_keys[i] = ( shapeA << 32 ) | shapeB;
	}

	return SortAndCheck( count );
}

static int TestSingleDigit( void )
{
	// One active digit leaves the result in the temp buffer
	int count = 500;
	for ( int i = 0; i < count; ++i )
	{
		s_keys[i] = 0xABCD000000000000ull | ( NextRandom() & 0xFF );
	}

	ENSURE( SortAndCheck( count ) == 0 );

	for ( int i = 0; i < count; ++i )
	{
		s_keys[i] = 0x1234ull | ( ( NextRandom() & 0xFF ) << 56 );
	}

	return SortAndCheck( count );
}

static int TestTwoDigits( void )
{
	int count = 500;
	for ( int i = 0; i < count; ++i )
	{
		s_keys[i] = ( NextRandom() & 0xFF ) | ( ( NextRandom() & 0xFF ) << 40 );
	}

	return SortAndCheck( count );
}

static int TestAllEqual( void )
{
	int count = 100;
	for ( int i = 0; i < count; ++i )
	{
		s_keys[i] = 0x0102030405060708ull;
	}

	return SortAndCheck( count );
}

static int TestOrdered( void )
{
	int count = 1000;
	for ( int i = 0; i < count; ++i )
	{
		s_keys[i] = (uint64_t)i * 0x0101010101ull;
	}

	ENSURE( SortAndCheck( count ) == 0 );

	for ( int i = 0; i < count; ++i )
	{
		s_keys[i] = (uint64_t)( count - i ) * 0x0101010101ull;
	}

	return SortAndCheck( count );
}

static int TestExtremes( void )
{
	s_keys[0] = UINT64_MAX;
	s_keys[1] = 0;
	s_keys[2] = 1ull << 63;
	s_keys[3] = ( 1ull << 63 ) - 1;
	s_keys[4] = 0;
	s_keys[5] = UINT64_MAX;

	return SortAndCheck( 6 );
}

int SortTest( void )
{
	RUN_SUBTEST( TestRandomFullWidth );
	RUN_SUBTEST( TestShapePairKeys );
	RUN_SUBTEST( TestSingleDigit );
	RUN_SUBTEST( TestTwoDigits );
	RUN_SUBTEST( TestAllEqual );
	RUN_SUBTEST( TestOrdered );
	RUN_SUBTEST( TestExtremes );

	return 0;
}
