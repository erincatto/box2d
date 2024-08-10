// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined( _MSC_VER ) && !defined( __clang__ )
	#include <intrin.h>

// https://en.wikipedia.org/wiki/Find_first_set

static inline uint32_t b2CTZ32( uint32_t block )
{
	unsigned long index;
	_BitScanForward( &index, block );
	return index;
}

// This function doesn't need to be fast, so using the Ivy Bridge fallback.
static inline uint32_t b2CLZ32( uint32_t value )
{
	#if 1

	// Use BSR (Bit Scan Reverse) which is available on Ivy Bridge
	unsigned long index;
	if ( _BitScanReverse( &index, value ) )
	{
		// BSR gives the index of the most significant 1-bit
		// We need to invert this to get the number of leading zeros
		return 31 - index;
	}
	else
	{
		// If x is 0, BSR sets the zero flag and doesn't modify index
		// LZCNT should return 32 for an input of 0
		return 32;
	}

	#else

	return __lzcnt( value );

	#endif
}

static inline uint32_t b2CTZ64( uint64_t block )
{
	unsigned long index;

	#ifdef _WIN64
	_BitScanForward64( &index, block );
	#else
	// 32-bit fall back
	if ( (uint32_t)block != 0 )
	{
		_BitScanForward( &index, (uint32_t)block );
	}
	else
	{
		_BitScanForward( &index, (uint32_t)( block >> 32 ) );
		index += 32;
	}
	#endif

	return index;
}

#else

static inline uint32_t b2CTZ32( uint32_t block )
{
	return __builtin_ctz( block );
}

static inline uint32_t b2CLZ32( uint32_t value )
{
	return __builtin_clz( value );
}

static inline uint32_t b2CTZ64( uint64_t block )
{
	return __builtin_ctzll( block );
}

#endif

static inline bool b2IsPowerOf2( int x )
{
	return ( x & ( x - 1 ) ) == 0;
}

static inline int b2BoundingPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 32 - (int)b2CLZ32( (uint32_t)x - 1 );
}

static inline int b2RoundUpPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 1 << ( 32 - (int)b2CLZ32( (uint32_t)x - 1 ) );
}
