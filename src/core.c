// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#if defined( B2_COMPILER_MSVC )
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#include <stdlib.h>
#else
#include <stdlib.h>
#endif

#include <stdatomic.h>
#include <string.h>

#ifdef BOX2D_PROFILE

#include <tracy/TracyC.h>
#define b2TracyCAlloc( ptr, size ) TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr ) TracyCFree( ptr )

#else

#define b2TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr )

#endif

#include "box2d/math_functions.h"

#include <stdio.h>

// This allows the user to change the length units at runtime
float b2_lengthUnitsPerMeter = 1.0f;

void b2SetLengthUnitsPerMeter( float lengthUnits )
{
	B2_ASSERT( b2IsValidFloat( lengthUnits ) && lengthUnits > 0.0f );
	b2_lengthUnitsPerMeter = lengthUnits;
}

float b2GetLengthUnitsPerMeter( void )
{
	return b2_lengthUnitsPerMeter;
}

static int b2DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );

	// return non-zero to break to debugger
	return 1;
}

b2AssertFcn* b2AssertHandler = b2DefaultAssertFcn;

void b2SetAssertFcn( b2AssertFcn* assertFcn )
{
	B2_ASSERT( assertFcn != NULL );
	b2AssertHandler = assertFcn;
}

int b2InternalAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	return b2AssertHandler( condition, fileName, lineNumber );
}

b2Version b2GetVersion( void )
{
	return ( b2Version ){ 3, 1, 0 };
}

static b2AllocFcn* b2_allocFcn = NULL;
static b2FreeFcn* b2_freeFcn = NULL;

static _Atomic int b2_byteCount;

void b2SetAllocator( b2AllocFcn* allocFcn, b2FreeFcn* freeFcn )
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

// Use 32 byte alignment for everything. Works with 256bit SIMD.
#define B2_ALIGNMENT 32

void* b2Alloc( int size )
{
	if (size == 0)
	{
		return NULL;
	}

	// This could cause some sharing issues, however Box2D rarely calls b2Alloc.
	atomic_fetch_add_explicit( &b2_byteCount, size, memory_order_relaxed );

	// Allocation must be a multiple of 32 or risk a seg fault
	// https://en.cppreference.com/w/c/memory/aligned_alloc
	int size32 = ( ( size - 1 ) | 0x1F ) + 1;

	if ( b2_allocFcn != NULL )
	{
		void* ptr = b2_allocFcn( size32, B2_ALIGNMENT );
		b2TracyCAlloc( ptr, size );

		B2_ASSERT( ptr != NULL );
		B2_ASSERT( ( (uintptr_t)ptr & 0x1F ) == 0 );

		return ptr;
	}

#ifdef B2_PLATFORM_WINDOWS
	void* ptr = _aligned_malloc( size32, B2_ALIGNMENT );
#elif defined( B2_PLATFORM_ANDROID )
	void* ptr = NULL;
	if ( posix_memalign( &ptr, B2_ALIGNMENT, size32 ) != 0 )
	{
		// allocation failed, exit the application
		exit( EXIT_FAILURE );
	}
#else
	void* ptr = aligned_alloc( B2_ALIGNMENT, size32 );
#endif

	b2TracyCAlloc( ptr, size );

	B2_ASSERT( ptr != NULL );
	B2_ASSERT( ( (uintptr_t)ptr & 0x1F ) == 0 );

	return ptr;
}

void b2Free( void* mem, int size )
{
	if ( mem == NULL )
	{
		return;
	}

	b2TracyCFree( mem );

	if ( b2_freeFcn != NULL )
	{
		b2_freeFcn( mem );
	}
	else
	{
#ifdef B2_PLATFORM_WINDOWS
		_aligned_free( mem );
#else
		free( mem );
#endif
	}

	atomic_fetch_sub_explicit( &b2_byteCount, size, memory_order_relaxed );
}

void* b2GrowAlloc( void* oldMem, int oldSize, int newSize )
{
	B2_ASSERT( newSize > oldSize );
	void* newMem = b2Alloc( newSize );
	if ( oldSize > 0 )
	{
		memcpy( newMem, oldMem, oldSize );
		b2Free( oldMem, oldSize );
	}
	return newMem;
}

int b2GetByteCount( void )
{
	return atomic_load_explicit( &b2_byteCount, memory_order_relaxed );
}
