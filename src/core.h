// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/math_functions.h"

// clang-format off

#define B2_NULL_INDEX ( -1 )

// for performance comparisons
#define B2_RESTRICT restrict

#ifdef NDEBUG
	#define B2_DEBUG 0
#else
	#define B2_DEBUG 1
#endif

#if defined( BOX2D_VALIDATE ) && !defined( NDEBUG )
	#define B2_VALIDATE 1
#else
	#define B2_VALIDATE 0
#endif

// Define platform
#if defined(_WIN32) || defined(_WIN64)
	#define B2_PLATFORM_WINDOWS
#elif defined( __ANDROID__ )
	#define B2_PLATFORM_ANDROID
#elif defined( __linux__ )
	#define B2_PLATFORM_LINUX
#elif defined( __APPLE__ )
	#include <TargetConditionals.h>
	#if defined( TARGET_OS_IPHONE ) && !TARGET_OS_IPHONE
		#define B2_PLATFORM_MACOS
	#else
		#define B2_PLATFORM_IOS
	#endif
#elif defined( __EMSCRIPTEN__ )
	#define B2_PLATFORM_WASM
#else
	#define B2_PLATFORM_UNKNOWN
#endif

// Define CPU
#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( _M_IX86 )
	#define B2_CPU_X86_X64
#elif defined( __aarch64__ ) || defined( _M_ARM64 ) || defined( __arm__ ) || defined( _M_ARM )
	#define B2_CPU_ARM
#elif defined( __EMSCRIPTEN__ )
	#define B2_CPU_WASM
#else
	#define B2_CPU_UNKNOWN
#endif

// Define SIMD
#if defined( BOX2D_ENABLE_SIMD )
	#if defined( B2_CPU_X86_X64 )
		#if defined( BOX2D_AVX2 )
			#define B2_SIMD_AVX2
			#define B2_SIMD_WIDTH 8
		#else
			#define B2_SIMD_SSE2
			#define B2_SIMD_WIDTH 4
		#endif
	#elif defined( B2_CPU_ARM )
		#define B2_SIMD_NEON
		#define B2_SIMD_WIDTH 4
	#elif defined( B2_CPU_WASM )
		#define B2_CPU_WASM
		#define B2_SIMD_SSE2
		#define B2_SIMD_WIDTH 4
	#else
		#define B2_SIMD_NONE
		#define B2_SIMD_WIDTH 4
	#endif
#else
	#define B2_SIMD_NONE
	// note: I tried width of 1 and got no performance change
	#define B2_SIMD_WIDTH 4
#endif

// Define compiler
#if defined( __clang__ )
	#define B2_COMPILER_CLANG
#elif defined( __GNUC__ )
	#define B2_COMPILER_GCC
#elif defined( _MSC_VER )
	#define B2_COMPILER_MSVC
#endif

/// Tracy profiler instrumentation
/// https://github.com/wolfpld/tracy
#ifdef BOX2D_PROFILE
	#include <tracy/TracyC.h>
	#define b2TracyCZoneC( ctx, color, active ) TracyCZoneC( ctx, color, active )
	#define b2TracyCZoneNC( ctx, name, color, active ) TracyCZoneNC( ctx, name, color, active )
	#define b2TracyCZoneEnd( ctx ) TracyCZoneEnd( ctx )
#else
	#define b2TracyCZoneC( ctx, color, active )
	#define b2TracyCZoneNC( ctx, name, color, active )
	#define b2TracyCZoneEnd( ctx )
#endif

// clang-format on

// Returns the number of elements of an array
#define B2_ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

// Used to prevent the compiler from warning about unused variables
#define B2_UNUSED( ... ) (void)sizeof( ( __VA_ARGS__, 0 ) )

// Use to validate definitions. Do not take my cookie.
#define B2_SECRET_COOKIE 1152023

// Snoop counters. These should be disabled in optimized builds because they are expensive.
#define B2_SNOOP_TABLE_COUNTERS B2_DEBUG
#define B2_SNOOP_PAIR_COUNTERS B2_DEBUG
#define B2_SNOOP_TOI_COUNTERS B2_DEBUG

#define B2_CHECK_DEF( DEF ) B2_ASSERT( DEF->internalValue == B2_SECRET_COOKIE )

void* b2Alloc( int size );
#define B2_ALLOC_STRUCT( type ) b2Alloc(sizeof(type))
#define B2_ALLOC_ARRAY( count, type ) b2Alloc(count * sizeof(type))

void b2Free( void* mem, int size );
#define B2_FREE_STRUCT( mem, type ) b2Free( mem, sizeof(type));
#define B2_FREE_ARRAY( mem, count, type ) b2Free(mem, count * sizeof(type))

void* b2GrowAlloc( void* oldMem, int oldSize, int newSize );

typedef struct b2AtomicInt
{
	int value;
} b2AtomicInt;

typedef struct b2AtomicU32
{
	uint32_t value;
} b2AtomicU32;

#if 0
void b2AtomicStoreInt( b2AtomicInt* a, int value );
int b2AtomicLoadInt( b2AtomicInt* a );
int b2AtomicFetchAddInt( b2AtomicInt* a, int increment );
bool b2AtomicCompareExchangeInt( b2AtomicInt* obj, int expected, int desired );

void b2AtomicStoreU32( b2AtomicU32* a, uint32_t value );
uint32_t b2AtomicLoadU32( b2AtomicU32* a );
#endif
