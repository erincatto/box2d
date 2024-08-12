// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#if defined( _WIN32 )
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

extern int BitSetTest( void );
extern int MathTest( void );
extern int CollisionTest( void );
extern int DeterminismTest( void );
extern int DistanceTest( void );
extern int WorldTest( void );
extern int ShapeTest( void );
extern int TableTest( void );

int main( void )
{
#if defined( _WIN32 )
	// Enable memory-leak reports

	// How to break at the leaking allocation, in the watch window enter this variable
	// and set it to the allocation number in {}. Do this at the first line in main.
	// {,,ucrtbased.dll}_crtBreakAlloc = <allocation number> 3970

	_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_WARN, _CRTDBG_FILE_STDERR );
	//_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
	//_CrtSetAllocHook(MyAllocHook);
	//_CrtSetBreakAlloc(196);
#endif

	printf( "Starting Box2D unit tests\n" );
	printf( "======================================\n" );

	RUN_TEST( MathTest );
	RUN_TEST( CollisionTest );
	RUN_TEST( DeterminismTest );
	RUN_TEST( DistanceTest );
	RUN_TEST( WorldTest );
	RUN_TEST( ShapeTest );
	RUN_TEST( TableTest );
	RUN_TEST( BitSetTest );

	printf( "======================================\n" );
	printf( "All Box2D tests passed!\n" );

#if defined( _WIN32 )
	if ( _CrtDumpMemoryLeaks() )
	{
		return 1;
	}
#endif

	return 0;
}
