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

static int SingleMultithreadingTest( int workerCount )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b2WorldId worldId = b2CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;
	int stepLimit = 500;
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

static int SingleFallingHingeTest( int workerCount, bool sse2Fallback )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2World_EnableSSE2Fallback( worldId, sse2Fallback );

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;
	bool done = false;
	while ( done == false )
	{
		b2World_Step( worldId, timeStep, 4 );
		done = UpdateFallingHinges( worldId, &data );
	}

	b2DestroyWorld( worldId );

	if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
	{
		printf( "  sse2Fallback=%d workers=%d sleepStep=%d hash=0x%08X\n", sse2Fallback ? 1 : 0, workerCount, data.sleepStep,
				data.hash );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

static b2WorldId CreateBouncePile( int workerCount, bool sse2Fallback )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2World_EnableSSE2Fallback( worldId, sse2Fallback );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	groundShapeDef.material.tangentSpeed = 0.5f;
	b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &groundShapeDef, &segment );

	b2Polygon box = b2MakeRoundedBox( 0.4f, 0.3f, 0.05f );
	b2Circle circle = { { 0.0f, 0.0f }, 0.35f };

	for ( int i = 0; i < 300; ++i )
	{
		int column = i % 30;
		int row = i / 30;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -15.0f + 1.0f * (float)column + 0.1f * (float)( row % 2 ), 1.0f + 0.9f * (float)row };
		bodyDef.rotation = b2MakeRot( 0.3f * (float)i );
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = ( i % 3 ) == 0 ? 0.0f : 0.25f * (float)( i % 5 );
		shapeDef.material.friction = 0.2f + 0.1f * (float)( i % 7 );
		shapeDef.material.rollingResistance = ( i % 4 ) == 0 ? 0.1f : 0.0f;
		shapeDef.enableHitEvents = ( i % 2 ) == 0;

		if ( i % 3 == 1 )
		{
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
		else
		{
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	return worldId;
}

// Width 4 and the native width must produce bitwise identical simulations.
static int SIMDWidthTest( void )
{
	if ( b2IsAVX2Available() == false )
	{
		printf( "  subtest skipped: SIMDWidthTest, native SIMD width is 4\n" );
		return 0;
	}

	for ( int workerCount = 0; workerCount <= 4; workerCount += 4 )
	{
		ENSURE( SingleFallingHingeTest( workerCount, true ) == 0 );

		b2WorldId narrowId = CreateBouncePile( workerCount, true );
		b2WorldId wideId = CreateBouncePile( workerCount, false );

		float timeStep = 1.0f / 60.0f;
		for ( int i = 0; i < 300; ++i )
		{
			b2World_Step( narrowId, timeStep, 4 );
			b2World_Step( wideId, timeStep, 4 );
			ENSURE( b2World_GetStateHash( narrowId ) == b2World_GetStateHash( wideId ) );
		}

		b2DestroyWorld( narrowId );
		b2DestroyWorld( wideId );
	}

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( BuiltInSchedulerTest );
	RUN_SUBTEST( CrossPlatformTest );
	RUN_SUBTEST( SIMDWidthTest );

	return 0;
}
