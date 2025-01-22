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

#include <stdio.h>
#include <string.h>

#ifdef BOX2D_PROFILE

#include <tracy/TracyC.h>
#define b2TracyCAlloc( ptr, size ) TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr ) TracyCFree( ptr )

#else

#define b2TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr )

#endif

#include "atomic.h"

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

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
int b2InternalAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	return b2AssertHandler( condition, fileName, lineNumber );
}
#endif

b2Version b2GetVersion( void )
{
	return ( b2Version ){ 3, 1, 0 };
}

#if 0
#if defined( _MSC_VER )
#include <intrin.h>
#endif

void b2AtomicStoreInt( b2AtomicInt* a, int value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

int b2AtomicLoadInt( b2AtomicInt* a )
{
#if defined( _MSC_VER )
	return _InterlockedOr( (long*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

int b2AtomicFetchAddInt( b2AtomicInt* a, int increment )
{
#if defined( _MSC_VER )
	return _InterlockedExchangeAdd( (long*)&a->value, (long)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

bool b2AtomicCompareExchangeInt( b2AtomicInt* a, int expected, int desired )
{
#if defined( _MSC_VER )
	return _InterlockedCompareExchange( (long*)&a->value, (long)desired, (long)expected ) == expected;
#elif defined( __GNUC__ ) || defined( __clang__ )
	// The value written to expected is ignored
	return __atomic_compare_exchange_n( &a->value, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

void b2AtomicStoreU32( b2AtomicU32* a, uint32_t value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

uint32_t b2AtomicLoadU32( b2AtomicU32* a )
{
#if defined( _MSC_VER )
	return (uint32_t)_InterlockedOr( (long*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}
#endif

static b2AllocFcn* b2_allocFcn = NULL;
static b2FreeFcn* b2_freeFcn = NULL;

b2AtomicInt b2_byteCount;

void b2SetAllocator( b2AllocFcn* allocFcn, b2FreeFcn* freeFcn )
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

// Use 32 byte alignment for everything. Works with 256bit SIMD.
#define B2_ALIGNMENT 32

void* b2Alloc( int size )
{
	if ( size == 0 )
	{
		return NULL;
	}

	// This could cause some sharing issues, however Box2D rarely calls b2Alloc.
	b2AtomicFetchAddInt( &b2_byteCount, size );

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

	b2AtomicFetchAddInt( &b2_byteCount, -size );
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
	return b2AtomicLoadInt( &b2_byteCount );
}
