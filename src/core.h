// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/base.h"

#define B2_NULL_INDEX ( -1 )

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
#if defined( _WIN64 )
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
	#error Unsupported platform
#endif

// Define CPU
#if defined( __x86_64__ ) || defined( _M_X64 )
	#define B2_CPU_X64
#elif defined( __aarch64__ ) || defined( _M_ARM64 )
	#define B2_CPU_ARM
#elif defined( __EMSCRIPTEN__ )
	#define B2_CPU_WASM
#else
	#error Unsupported CPU
#endif

// Define compiler
#if defined( __clang__ )
	#define B2_COMPILER_CLANG
#elif defined( __GNUC__ )
	#define B2_COMPILER_GCC
#elif defined( _MSC_VER )
	#define B2_COMPILER_MSVC
#endif

#if defined( B2_COMPILER_MSVC )
	#define B2_BREAKPOINT __debugbreak()
#elif defined( B2_PLATFORM_WASM )
	#define B2_BREAKPOINT                                                                                                        \
		do                                                                                                                       \
		{                                                                                                                        \
		}                                                                                                                        \
		while ( 0 )
#elif defined( B2_COMPILER_GCC ) || defined( B2_COMPILER_CLANG )
	#if defined( B2_CPU_X64 )
		#define B2_BREAKPOINT __asm volatile( "int $0x3" )
	#elif defined( B2_CPU_ARM )
		#define B2_BREAKPOINT __builtin_trap()
	#endif
#else
	#error Unknown platform
#endif

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
extern b2AssertFcn* b2AssertHandler;
	#define B2_ASSERT( condition )                                                                                               \
		do                                                                                                                       \
		{                                                                                                                        \
			if ( !( condition ) && b2AssertHandler( #condition, __FILE__, (int)__LINE__ ) )                                      \
				B2_BREAKPOINT;                                                                                                   \
		}                                                                                                                        \
		while ( 0 )
#else
	#define B2_ASSERT( ... ) ( (void)0 )
#endif

/// Tracy profiler instrumentation
///	https://github.com/wolfpld/tracy
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

extern float b2_lengthUnitsPerMeter;

// Used to detect bad values. Positions greater than about 16km will have precision
// problems, so 100km as a limit should be fine in all cases.
#define b2_huge ( 100000.0f * b2_lengthUnitsPerMeter )

// Maximum parallel workers. Used to size some static arrays.
#define b2_maxWorkers 64

// Maximum number of colors in the constraint graph. Constraints that cannot
//	find a color are added to the overflow set which are solved single-threaded.
#define b2_graphColorCount 12

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// @warning modifying this can have a significant impact on stability
#define b2_linearSlop ( 0.005f * b2_lengthUnitsPerMeter )

// Maximum number of simultaneous worlds that can be allocated
#define b2_maxWorlds 128

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// @warning increasing this to 0.5f * b2_pi or greater will break continuous collision.
#define b2_maxRotation ( 0.25f * b2_pi )

// @warning modifying this can have a significant impact on performance and stability
#define b2_speculativeDistance ( 4.0f * b2_linearSlop )

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment.
// This is in meters.
// @warning modifying this can have a significant impact on performance
#define b2_aabbMargin ( 0.1f * b2_lengthUnitsPerMeter )

// The time that a body must be still before it will go to sleep. In seconds.
#define b2_timeToSleep 0.5f

// Returns the number of elements of an array
#define B2_ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

// Used to prevent the compiler from warning about unused variables
#define B2_MAYBE_UNUSED( x ) ( (void)( x ) )

// Use to validate definitions. Do not take my cookie.
#define B2_SECRET_COOKIE 1152023

#define b2CheckDef( DEF ) B2_ASSERT( DEF->internalValue == B2_SECRET_COOKIE )
