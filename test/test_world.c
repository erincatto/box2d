// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "benchmarks.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <stdio.h>

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
int HelloWorld( void )
{
	// Construct a world object, which will hold and simulate the rigid bodies.
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	// Define the ground body.
	b2BodyDef groundBodyDef = b2DefaultBodyDef();
	groundBodyDef.position = b2MakePosition( (b2Vec2){ 0.0f, -10.0f } );

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b2BodyId groundId = b2CreateBody( worldId, &groundBodyDef );
	ENSURE( b2Body_IsValid( groundId ) );

	// Define the ground box shape. The extents are the half-widths of the box.
	b2Polygon groundBox = b2MakeBox( 50.0f, 10.0f );

	// Add the box shape to the ground body.
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );

	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2MakePosition( (b2Vec2){ 0.0f, 4.0f } );

	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	// Define another box shape for our dynamic body.
	b2Polygon dynamicBox = b2MakeBox( 1.0f, 1.0f );

	// Define the dynamic body shape
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.material.friction = 0.3f;

	// Add the shape to the body.
	b2CreatePolygonShape( bodyId, &shapeDef, &dynamicBox );

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	b2Vec2 position = b2ToVec2( b2Body_GetPosition( bodyId ) );
	b2Rot rotation = b2Body_GetRotation( bodyId );

	// This is our little game loop.
	for ( int i = 0; i < 90; ++i )
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b2World_Step( worldId, timeStep, subStepCount );

		// Now print the position and angle of the body.
		position = b2ToVec2( b2Body_GetPosition( bodyId ) );
		rotation = b2Body_GetRotation( bodyId );

		// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b2Rot_GetAngle(rotation));
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b2DestroyWorld( worldId );

	ENSURE( b2AbsFloat( position.x ) < 0.01f );
	ENSURE( b2AbsFloat( position.y - 1.00f ) < 0.01f );
	ENSURE( b2AbsFloat( b2Rot_GetAngle( rotation ) ) < 0.01f );

	return 0;
}

int EmptyWorld( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) == true );

	float timeStep = 1.0f / 60.0f;
	int32_t subStepCount = 1;

	for ( int32_t i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, timeStep, subStepCount );
	}

	b2DestroyWorld( worldId );

	ENSURE( b2World_IsValid( worldId ) == false );

	return 0;
}

#define BODY_COUNT 10
int DestroyAllBodiesWorld( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) == true );

	int count = 0;
	bool creating = true;

	b2BodyId bodyIds[BODY_COUNT];
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2Polygon square = b2MakeSquare( 0.5f );

	for ( int32_t i = 0; i < 2 * BODY_COUNT + 10; ++i )
	{
		if ( creating )
		{
			if ( count < BODY_COUNT )
			{
				bodyIds[count] = b2CreateBody( worldId, &bodyDef );

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2CreatePolygonShape( bodyIds[count], &shapeDef, &square );
				count += 1;
			}
			else
			{
				creating = false;
			}
		}
		else if ( count > 0 )
		{
			b2DestroyBody( bodyIds[count - 1] );
			bodyIds[count - 1] = b2_nullBodyId;
			count -= 1;
		}

		b2World_Step( worldId, 1.0f / 60.0f, 3 );
	}

	b2Counters counters = b2World_GetCounters( worldId );
	ENSURE( counters.bodyCount == 0 );

	b2DestroyWorld( worldId );

	ENSURE( b2World_IsValid( worldId ) == false );

	return 0;
}

static int TestIsValid( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	b2BodyDef bodyDef = b2DefaultBodyDef();

	b2BodyId bodyId1 = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId1 ) == true );

	b2BodyId bodyId2 = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId2 ) == true );

	b2DestroyBody( bodyId1 );
	ENSURE( b2Body_IsValid( bodyId1 ) == false );

	b2DestroyBody( bodyId2 );
	ENSURE( b2Body_IsValid( bodyId2 ) == false );

	b2DestroyWorld( worldId );

	ENSURE( b2World_IsValid( worldId ) == false );
	ENSURE( b2Body_IsValid( bodyId2 ) == false );
	ENSURE( b2Body_IsValid( bodyId1 ) == false );

	return 0;
}

#define WORLD_COUNT ( B2_MAX_WORLDS / 2 )

int TestWorldRecycle( void )
{
	_Static_assert( WORLD_COUNT > 0, "world count" );

	int count = 100;

	b2WorldId worldIds[WORLD_COUNT];

	for ( int i = 0; i < count; ++i )
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			worldIds[j] = b2CreateWorld( &worldDef );
			ENSURE( b2World_IsValid( worldIds[j] ) == true );

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2CreateBody( worldIds[j], &bodyDef );
		}

		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			float timeStep = 1.0f / 60.0f;
			int subStepCount = 1;

			for ( int k = 0; k < 10; ++k )
			{
				b2World_Step( worldIds[j], timeStep, subStepCount );
			}
		}

		for ( int j = WORLD_COUNT - 1; j >= 0; --j )
		{
			b2DestroyWorld( worldIds[j] );
			ENSURE( b2World_IsValid( worldIds[j] ) == false );
			worldIds[j] = b2_nullWorldId;
		}
	}

	return 0;
}

static bool CustomFilter( b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	ENSURE( context == NULL );
	return true;
}

static bool PreSolveStatic( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Position point, b2Vec2 normal, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)point;
	(void)normal;
	ENSURE( context == NULL );
	return false;
}

// This test is here to ensure all API functions link correctly.
int TestWorldCoverage( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	b2World_EnableSleeping( worldId, true );
	b2World_EnableSleeping( worldId, false );
	bool flag = b2World_IsSleepingEnabled( worldId );
	ENSURE( flag == false );

	b2World_EnableContinuous( worldId, false );
	b2World_EnableContinuous( worldId, true );
	flag = b2World_IsContinuousEnabled( worldId );
	ENSURE( flag == true );

	b2World_SetRestitutionThreshold( worldId, 0.0f );
	b2World_SetRestitutionThreshold( worldId, 2.0f );
	float value = b2World_GetRestitutionThreshold( worldId );
	ENSURE( value == 2.0f );

	b2World_SetHitEventThreshold( worldId, 0.0f );
	b2World_SetHitEventThreshold( worldId, 100.0f );
	value = b2World_GetHitEventThreshold( worldId );
	ENSURE( value == 100.0f );

	b2World_SetCustomFilterCallback( worldId, CustomFilter, NULL );
	b2World_SetPreSolveCallback( worldId, PreSolveStatic, NULL );

	b2Vec2 g = { 1.0f, 2.0f };
	b2World_SetGravity( worldId, g );
	b2Vec2 v = b2World_GetGravity( worldId );
	ENSURE( v.x == g.x );
	ENSURE( v.y == g.y );

	b2ExplosionDef explosionDef = b2DefaultExplosionDef();
	b2World_Explode( worldId, &explosionDef );

	b2World_SetContactTuning( worldId, 10.0f, 2.0f, 4.0f );

	b2World_SetMaximumLinearSpeed( worldId, 10.0f );
	value = b2World_GetMaximumLinearSpeed( worldId );
	ENSURE( value == 10.0f );

	b2World_EnableWarmStarting( worldId, true );
	flag = b2World_IsWarmStartingEnabled( worldId );
	ENSURE( flag == true );

	int count = b2World_GetAwakeBodyCount( worldId );
	ENSURE( count == 0 );

	b2World_SetUserData( worldId, &value );
	void* userData = b2World_GetUserData( worldId );
	ENSURE( userData == &value );

	b2World_Step( worldId, 1.0f, 1 );

	b2DestroyWorld( worldId );

	return 0;
}

static int TestSensor( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Wall from x = 1 to x = 2
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_staticBody;
	bodyDef.position.x = 1.5f;
	bodyDef.position.y = 11.0f;
	b2BodyId wallId = b2CreateBody( worldId, &bodyDef );
	b2Polygon box = b2MakeBox( 0.5f, 10.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b2CreatePolygonShape( wallId, &shapeDef, &box );

	// Bullet fired towards the wall
	bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = b2MakePosition( (b2Vec2){ 7.39814f, 4.0f } );
	bodyDef.linearVelocity = (b2Vec2){ -20.0f, 0.0f };
	b2BodyId bulletId = b2CreateBody( worldId, &bodyDef );
	shapeDef = b2DefaultShapeDef();
	shapeDef.isSensor = true;
	shapeDef.enableSensorEvents = true;
	b2Circle circle = { { 0.0f, 0.0f }, 0.1f };
	b2CreateCircleShape( bulletId, &shapeDef, &circle );

	int beginCount = 0;
	int endCount = 0;

	while ( true )
	{
		float timeStep = 1.0f / 60.0f;
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount );

		b2Vec2 bulletPos = b2ToVec2( b2Body_GetPosition( bulletId ) );
		// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

		b2SensorEvents events = b2World_GetSensorEvents( worldId );

		if ( events.beginCount > 0 )
		{
			beginCount += 1;
		}

		if ( events.endCount > 0 )
		{
			endCount += 1;
		}

		if ( bulletPos.x < -1.0f )
		{
			break;
		}
	}

	b2DestroyWorld( worldId );

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 1 );

	return 0;
}

static int TestSetWorkerCount( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );
	ENSURE( b2World_GetWorkerCount( worldId ) == 1 );

	CreateJunkyard( worldId );
	StepJunkyard( worldId, 1 );

	b2World_SetWorkerCount( worldId, 4 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 2 );

	b2World_SetWorkerCount( worldId, 4 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 3 );

	b2World_SetWorkerCount( worldId, 0 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 4 );

	b2World_SetWorkerCount( worldId, -5 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 5 );

	b2World_SetWorkerCount( worldId, B2_MAX_WORKERS + 10 );
	ENSURE( b2World_GetWorkerCount( worldId ) == B2_MAX_WORKERS );

	StepJunkyard( worldId, 2 );

	b2DestroyWorld( worldId );

	return 0;
}

static int ChainSegmentShapeTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	b2ChainSegment cs = { 0 };
	cs.ghost1 = (b2Vec2){ 2.0f, 0.0f };
	cs.segment.point1 = (b2Vec2){ 1.0f, 0.0f };
	cs.segment.point2 = (b2Vec2){ -1.0f, 0.0f };
	cs.ghost2 = (b2Vec2){ -2.0f, 0.0f };
	cs.chainId = 99;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2ShapeId orphanShape = b2CreateChainSegmentShape( groundId, &shapeDef, &cs );
	ENSURE( B2_IS_NON_NULL( orphanShape ) );

	ENSURE( b2Shape_GetType( orphanShape ) == b2_chainSegmentShape );

	b2ChainId parentChain = b2Shape_GetParentChain( orphanShape );
	ENSURE( B2_IS_NULL( parentChain ) );

	b2ChainSegment got = b2Shape_GetChainSegment( orphanShape );
	ENSURE_SMALL( got.ghost1.x - cs.ghost1.x, 1e-5f );
	ENSURE_SMALL( got.ghost1.y - cs.ghost1.y, 1e-5f );
	ENSURE_SMALL( got.segment.point1.x - cs.segment.point1.x, 1e-5f );
	ENSURE_SMALL( got.segment.point1.y - cs.segment.point1.y, 1e-5f );
	ENSURE_SMALL( got.segment.point2.x - cs.segment.point2.x, 1e-5f );
	ENSURE_SMALL( got.segment.point2.y - cs.segment.point2.y, 1e-5f );
	ENSURE_SMALL( got.ghost2.x - cs.ghost2.x, 1e-5f );
	ENSURE_SMALL( got.ghost2.y - cs.ghost2.y, 1e-5f );
	ENSURE( got.chainId == B2_NULL_INDEX );

	b2BodyDef dynamicDef = b2DefaultBodyDef();
	dynamicDef.type = b2_dynamicBody;
	dynamicDef.position = b2MakePosition( (b2Vec2){ 0.0f, 2.0f } );
	b2BodyId circleBodyId = b2CreateBody( worldId, &dynamicDef );
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2ShapeDef circleShapeDef = b2DefaultShapeDef();
	b2CreateCircleShape( circleBodyId, &circleShapeDef, &circle );

	for ( int i = 0; i < 120; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b2Vec2 circlePos = b2ToVec2( b2Body_GetPosition( circleBodyId ) );
	ENSURE( circlePos.y > 0.0f );

	b2ChainSegment cs2 = { 0 };
	cs2.ghost1 = (b2Vec2){ 3.0f, 0.0f };
	cs2.segment.point1 = (b2Vec2){ 2.0f, 0.0f };
	cs2.segment.point2 = (b2Vec2){ -2.0f, 0.0f };
	cs2.ghost2 = (b2Vec2){ -3.0f, 0.0f };
	cs2.chainId = B2_NULL_INDEX;

	b2Shape_SetChainSegment( orphanShape, &cs2 );

	b2ChainSegment got2 = b2Shape_GetChainSegment( orphanShape );
	ENSURE_SMALL( got2.segment.point1.x - cs2.segment.point1.x, 1e-5f );
	ENSURE_SMALL( got2.segment.point1.y - cs2.segment.point1.y, 1e-5f );
	ENSURE_SMALL( got2.segment.point2.x - cs2.segment.point2.x, 1e-5f );
	ENSURE_SMALL( got2.segment.point2.y - cs2.segment.point2.y, 1e-5f );
	ENSURE_SMALL( got2.ghost1.x - cs2.ghost1.x, 1e-5f );
	ENSURE_SMALL( got2.ghost1.y - cs2.ghost1.y, 1e-5f );
	ENSURE_SMALL( got2.ghost2.x - cs2.ghost2.x, 1e-5f );
	ENSURE_SMALL( got2.ghost2.y - cs2.ghost2.y, 1e-5f );
	ENSURE( got2.chainId == B2_NULL_INDEX );

	b2ChainId parentChain2 = b2Shape_GetParentChain( orphanShape );
	ENSURE( B2_IS_NULL( parentChain2 ) );

	b2BodyId convBody = b2CreateBody( worldId, &bodyDef );
	b2Circle convCircle = { { 0.0f, 0.0f }, 0.25f };
	b2ShapeId convShape = b2CreateCircleShape( convBody, &shapeDef, &convCircle );
	ENSURE( b2Shape_GetType( convShape ) == b2_circleShape );

	b2Shape_SetChainSegment( convShape, &cs2 );

	ENSURE( b2Shape_GetType( convShape ) == b2_chainSegmentShape );
	b2ChainSegment got3 = b2Shape_GetChainSegment( convShape );
	ENSURE_SMALL( got3.ghost1.x - cs2.ghost1.x, 1e-5f );
	ENSURE_SMALL( got3.ghost1.y - cs2.ghost1.y, 1e-5f );
	ENSURE_SMALL( got3.ghost2.x - cs2.ghost2.x, 1e-5f );
	ENSURE_SMALL( got3.ghost2.y - cs2.ghost2.y, 1e-5f );
	ENSURE( got3.chainId == B2_NULL_INDEX );

	b2ChainId parentChain3 = b2Shape_GetParentChain( convShape );
	ENSURE( B2_IS_NULL( parentChain3 ) );

	b2DestroyShape( orphanShape, true );
	b2DestroyWorld( worldId );

	return 0;
}

static int DeferredMassFlagSyncTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.updateBodyMass = false;

	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2CreateCircleShape( bodyId, &shapeDef, &circle );

	b2Body_ApplyMassFromShapes( bodyId );

	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	b2DestroyWorld( worldId );
	return 0;
}

static int EnableSleepFlagSyncTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.enableSleep = false;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	ENSURE( b2Body_IsSleepEnabled( bodyId ) == false );

	b2Body_EnableSleep( bodyId, true );
	ENSURE( b2Body_IsSleepEnabled( bodyId ) == true );

	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	b2DestroyWorld( worldId );
	return 0;
}

static int EnableContactRecyclingTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	// Default is enabled
	b2BodyId bodyA = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyA ) == true );

	b2Body_EnableContactRecycling( bodyA, false );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyA ) == false );

	b2Body_EnableContactRecycling( bodyA, true );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyA ) == true );

	// Per-def opt-out at creation
	bodyDef.enableContactRecycling = false;
	b2BodyId bodyB = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyB ) == false );

	// Stepping after toggling must not trip the flag-sync validator
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	b2DestroyWorld( worldId );
	return 0;
}

static int SetBulletDriftTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = false;
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		ENSURE( b2Body_IsBullet( bodyId ) == false );

		b2Body_SetBullet( bodyId, true );
		ENSURE( b2Body_IsBullet( bodyId ) == true );

		b2MotionLocks locks = { 0 };
		locks.linearX = true;
		b2Body_SetMotionLocks( bodyId, locks );

		ENSURE( b2Body_IsBullet( bodyId ) == true );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		ENSURE( b2Body_IsBullet( bodyId ) == true );

		b2Body_SetBullet( bodyId, false );
		ENSURE( b2Body_IsBullet( bodyId ) == false );

		b2MotionLocks locks = { 0 };
		locks.linearX = true;
		b2Body_SetMotionLocks( bodyId, locks );

		ENSURE( b2Body_IsBullet( bodyId ) == false );
	}

	b2DestroyWorld( worldId );
	return 0;
}

int WorldTest( void )
{
	RUN_SUBTEST( HelloWorld );
	RUN_SUBTEST( EmptyWorld );
	RUN_SUBTEST( DestroyAllBodiesWorld );
	RUN_SUBTEST( TestIsValid );
	RUN_SUBTEST( TestWorldRecycle );
	RUN_SUBTEST( TestWorldCoverage );
	RUN_SUBTEST( TestSensor );
	RUN_SUBTEST( TestSetWorkerCount );
	RUN_SUBTEST( ChainSegmentShapeTest );
	RUN_SUBTEST( SetBulletDriftTest );
	RUN_SUBTEST( DeferredMassFlagSyncTest );
	RUN_SUBTEST( EnableSleepFlagSyncTest );
	RUN_SUBTEST( EnableContactRecyclingTest );

	return 0;
}
