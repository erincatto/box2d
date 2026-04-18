// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/types.h"

#include <stdio.h>

#ifdef BOX2D_PROFILE
#include <tracy/TracyC.h>
#else
#define TracyCFrameMark
#endif

#define EXPECTED_SLEEP_STEP 293
#define EXPECTED_HASH 0x2FF98AC6

static int SingleMultithreadingTest( int workerCount )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b2WorldId worldId = b2CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;
	int stepLimit = 1000;
	for ( int i = 0; i < stepLimit; ++i )
	{
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		bool done = UpdateFallingHinges( worldId, &data );

		if ( done )
		{
			break;
		}
	}

	b2DestroyWorld( worldId );

	if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
	{
		printf( "  workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

// Test multithreaded determinism.
static int MultithreadingTest( void )
{
	for ( int run = 0; run < 3; ++run )
	{
		for ( int workerCount = 1; workerCount < 16; workerCount += 2 )
		{
			int result = SingleMultithreadingTest( workerCount );
			ENSURE( result == 0 );
		}

		for ( int workerCount = 32; workerCount >= 0; workerCount -= 5 )
		{
			int result = SingleMultithreadingTest( workerCount );
			ENSURE( result == 0 );
		}
	}

	return 0;
}

// Test determinism using the built-in scheduler (no external task system).
static int BuiltInSchedulerTest( void )
{
	for ( int workerCount = 2; workerCount <= 8; workerCount += 2 )
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		worldDef.workerCount = workerCount;

		b2WorldId worldId = b2CreateWorld( &worldDef );

		FallingHingeData data = CreateFallingHinges( worldId );

		float timeStep = 1.0f / 60.0f;
		int stepLimit = 1000;
		for ( int i = 0; i < stepLimit; ++i )
		{
			int subStepCount = 4;
			b2World_Step( worldId, timeStep, subStepCount );

			bool done = UpdateFallingHinges( worldId, &data );
			if ( done )
			{
				break;
			}
		}

		b2DestroyWorld( worldId );

		if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
		{
			printf( "  built-in scheduler workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
		}

		ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
		ENSURE( data.hash == EXPECTED_HASH );

		DestroyFallingHinges( &data );
	}

	return 0;
}

// Test cross-platform determinism.
static int CrossPlatformTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;

	bool done = false;
	while ( done == false )
	{
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateFallingHinges( worldId, &data );
	}

	if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
	{
		printf( "  cross-platform sleepStep=%d hash=0x%08X\n", data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	b2DestroyWorld( worldId );

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( BuiltInSchedulerTest );
	RUN_SUBTEST( CrossPlatformTest );

	return 0;
}
