// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "simd.h"

#include "platform.h"

#include <stdbool.h>
#include <stdint.h>

static b2AtomicInt b2_simdWidth;

#if defined( B2_SIMD_HAS_WIDTH_8 )

#if defined( _MSC_VER )
#include <intrin.h>
#endif

// This code enables runtime dispatch to AVX2.
// Inspired by https://github.com/simdjson/simdjson/blob/master/src/internal/isadetection.h

static void b2CpuId( unsigned int leaf, unsigned int subLeaf, unsigned int registers[4] )
{
#if defined( _MSC_VER )
	int info[4];
	__cpuidex( info, (int)leaf, (int)subLeaf );
	registers[0] = (unsigned int)info[0];
	registers[1] = (unsigned int)info[1];
	registers[2] = (unsigned int)info[2];
	registers[3] = (unsigned int)info[3];
#else
	// clang cpuid.h can give the rbx swap register the same register as the leaf input, clobbering rbx
	__asm__ volatile( "cpuid"
					  : "=a"( registers[0] ), "=b"( registers[1] ), "=c"( registers[2] ), "=d"( registers[3] )
					  : "a"( leaf ), "c"( subLeaf ) );
#endif
}

static uint64_t b2GetExtendedControlRegister( void )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	return _xgetbv( 0 );
#else
	uint32_t low, high;
	__asm__ volatile( "xgetbv" : "=a"( low ), "=d"( high ) : "c"( 0 ) );
	return ( (uint64_t)high << 32 ) | low;
#endif
}

static bool b2HasAVX2( void )
{
	unsigned int registers[4];
	b2CpuId( 0, 0, registers );
	if ( registers[0] < 7 )
	{
		return false;
	}

	b2CpuId( 1, 0, registers );
	unsigned int osxsaveAndAvx = ( 1u << 27 ) | ( 1u << 28 );
	if ( ( registers[2] & osxsaveAndAvx ) != osxsaveAndAvx )
	{
		return false;
	}

	if ( ( b2GetExtendedControlRegister() & 6 ) != 6 )
	{
		return false;
	}

	b2CpuId( 7, 0, registers );
	return ( registers[1] & ( 1u << 5 ) ) != 0;
}

#endif

static int b2DetectSIMDWidth( void )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	return b2HasAVX2() ? 8 : 4;
#else
	return 4;
#endif
}

int b2GetSIMDWidth( void )
{
	int width = b2AtomicLoadInt( &b2_simdWidth );
	if ( width == 0 )
	{
		width = b2DetectSIMDWidth();
		b2AtomicStoreInt( &b2_simdWidth, width );
	}

	return width;
}

bool b2IsAVX2Available( void )
{
	return b2DetectSIMDWidth() == 8;
}

void b2SetSIMDWidth( int width )
{
	B2_ASSERT( width == 0 || width == 4 || width == 8 );
	if ( width == 8 && b2DetectSIMDWidth() != 8 )
	{
		width = 4;
	}

	b2AtomicStoreInt( &b2_simdWidth, width );
}
