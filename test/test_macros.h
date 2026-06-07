// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#define RUN_TEST( T )                                                                                                            \
	do                                                                                                                           \
	{                                                                                                                            \
		uint64_t testTicks = b2GetTicks();                                                                                           \
		int result = T();                                                                                                        \
		float s = 0.001f * b2GetMilliseconds( testTicks );                                                                                   \
		if ( result == 1 )                                                                                                       \
		{                                                                                                                        \
			printf( "test failed: " #T "\n" );                                                                                   \
			return 1;                                                                                                            \
		}                                                                                                                        \
		else                                                                                                                     \
		{                                                                                                                        \
			printf( "test passed: " #T " after %.2f s\n", s );                                                                 \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

#define RUN_SUBTEST( T )                                                                                                         \
	do                                                                                                                           \
	{                                                                                                                            \
		int result = T();                                                                                                        \
		if ( result == 1 )                                                                                                       \
		{                                                                                                                        \
			printf( "  subtest failed: " #T "\n" );                                                                              \
			return 1;                                                                                                            \
		}                                                                                                                        \
		else                                                                                                                     \
		{                                                                                                                        \
			printf( "  subtest passed: " #T "\n" );                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

#define ENSURE( C )                                                                                                              \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( C ) == false )                                                                                                    \
		{                                                                                                                        \
			printf( "condition false: " #C "\n" );                                                                               \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

#define ENSURE_SMALL( C, tol )                                                                                                   \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( C ) < -( tol ) || ( tol ) < ( C ) )                                                                               \
		{                                                                                                                        \
			printf( "condition false: abs(" #C ") < %g\n", tol );                                                                \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

/// Used to prevent the compiler from warning about unused variables
#define MAYBE_UNUSED( x ) ( (void)( x ) )

// Filter-aware test runner: skips tests that don't match the filter
#define MAYBE_RUN_TEST( T )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( filter != NULL && strcmp( filter, #T ) != 0 )                                                                       \
		{                                                                                                                        \
			printf( "test skipped: " #T "\n" );                                                                                  \
			break;                                                                                                               \
		}                                                                                                                        \
		RUN_TEST( T );                                                                                                           \
	}                                                                                                                            \
	while ( false )
