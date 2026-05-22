// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "box2d/base.h"

#include <string.h>

#if defined( _MSC_VER )
	#include <crtdbg.h>

// int MyAllocHook(int allocType, void* userData, size_t size, int blockType, long requestNumber, const unsigned char* filename,
//	int lineNumber)
//{
//	if (size == 16416)
//	{
//		size += 0;
//	}
//
//	return 1;
// }
#endif

#ifdef TRACY_ENABLE
#include <tracy/TracyC.h>
#endif

extern int BitSetTest( void );
extern int CollisionTest( void );
extern int ContainerTest( void );
extern int DeterminismTest( void );
extern int DistanceTest( void );
extern int DynamicTreeTest( void );
extern int IdTest( void );
extern int MathTest( void );
extern int ShapeTest( void );
extern int TableTest( void );
extern int ThreadTest( void );
extern int WorldTest( void );

// Filter-aware test runner: skips tests that don't match the filter
#define MAYBE_RUN_TEST( T )                                                                                                  \
	do                                                                                                                       \
	{                                                                                                                        \
		if ( filter != NULL && strcmp( filter, #T ) != 0 )                                                                   \
		{                                                                                                                    \
			printf( "test skipped: " #T "\n" );                                                                              \
			break;                                                                                                           \
		}                                                                                                                    \
		RUN_TEST( T );                                                                                                       \
	}                                                                                                                        \
	while ( false )

int main( int argc, char** argv )
{
#if defined( _MSC_VER )
	// Enable memory-leak reports
	//_CrtSetBreakAlloc(196);
	_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_WARN, _CRTDBG_FILE_STDERR );
	//_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
	//_CrtSetAllocHook(MyAllocHook);
#endif

#ifdef TRACY_ENABLE
	___tracy_startup_profiler();
#endif

	const char* filter = NULL;
	if ( argc > 1 )
	{
		filter = argv[1];
	}

	uint64_t ticks = b2GetTicks();

	printf( "Starting Box2D unit tests\n" );
	if ( filter != NULL )
	{
		printf( "Filter: %s\n", filter );
	}
	printf( "======================================\n" );

	MAYBE_RUN_TEST( TableTest );
	MAYBE_RUN_TEST( MathTest );
	MAYBE_RUN_TEST( BitSetTest );
	MAYBE_RUN_TEST( CollisionTest );
	MAYBE_RUN_TEST( ContainerTest );
	MAYBE_RUN_TEST( DeterminismTest );
	MAYBE_RUN_TEST( DistanceTest );
	MAYBE_RUN_TEST( DynamicTreeTest );
	MAYBE_RUN_TEST( IdTest );
	MAYBE_RUN_TEST( ShapeTest );
	MAYBE_RUN_TEST( ThreadTest );
	MAYBE_RUN_TEST( WorldTest );

	printf( "======================================\n" );
	printf( "All Box2D tests passed!\n" );
	
	float duration = b2GetMilliseconds( ticks );
	printf( "Test duration = %.2f s\n", 0.001f * duration );

#ifdef TRACY_ENABLE
	___tracy_shutdown_profiler();
#endif

#if defined( _MSC_VER )
	if ( _CrtDumpMemoryLeaks() )
	{
		return 1;
	}
#endif

	return 0;
}
