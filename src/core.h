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
#define B2_MAYBE_UNUSED( x ) ( (void)( x ) )

// Use to validate definitions. Do not take my cookie.
#define B2_SECRET_COOKIE 1152023

#define b2CheckDef( DEF ) B2_ASSERT( DEF->internalValue == B2_SECRET_COOKIE )

void* b2Alloc( int size );
void b2Free( void* mem, int size );
void* b2GrowAlloc( void* oldMem, int oldSize, int newSize );
