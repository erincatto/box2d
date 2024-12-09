// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constants.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
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
	worldDef.gravity = ( b2Vec2 ){ 0.0f, -10.0f };

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	// Define the ground body.
	b2BodyDef groundBodyDef = b2DefaultBodyDef();
	groundBodyDef.position = ( b2Vec2 ){ 0.0f, -10.0f };

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
	bodyDef.position = ( b2Vec2 ){ 0.0f, 4.0f };

	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	// Define another box shape for our dynamic body.
	b2Polygon dynamicBox = b2MakeBox( 1.0f, 1.0f );

	// Define the dynamic body shape
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.friction = 0.3f;

	// Add the shape to the body.
	b2CreatePolygonShape( bodyId, &shapeDef, &dynamicBox );

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	b2Vec2 position = b2Body_GetPosition( bodyId );
	b2Rot rotation = b2Body_GetRotation( bodyId );

	// This is our little game loop.
	for ( int i = 0; i < 90; ++i )
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b2World_Step( worldId, timeStep, subStepCount );

		// Now print the position and angle of the body.
		position = b2Body_GetPosition( bodyId );
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

int TestForAmy( void )
{
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		b2WorldId world_id = b2CreateWorld( &worldDef );

		b2BodyDef body_def = b2DefaultBodyDef();

		body_def.type = b2_staticBody;
		body_def.position = ( b2Vec2 ){ 0., 0. };
		b2BodyId body_id = b2CreateBody( world_id, &body_def );
		b2Polygon polygon = b2MakeBox( 1., 1. );
		b2ShapeDef shape_def = b2DefaultShapeDef();
		b2CreatePolygonShape( body_id, &shape_def, &polygon );

		b2BodyDef simulon_body_def = b2DefaultBodyDef();

		simulon_body_def.position = ( b2Vec2 ){ 0., -7.5 };
		simulon_body_def.type = b2_dynamicBody;

		b2BodyId simulon_body_id = b2CreateBody( world_id, &simulon_body_def );
		b2Circle ball = { { 0.0, 0.35 }, 0.5 };

		b2ShapeDef simulon_shape_def = b2DefaultShapeDef();
		b2CreateCircleShape( simulon_body_id, &simulon_shape_def, &ball );

		b2Polygon the_box = b2MakeRoundedBox( 0.1, 0.1, 0.01 );
		b2CreatePolygonShape( simulon_body_id, &simulon_shape_def, &the_box );
		b2BodyDef head_body_def = b2DefaultBodyDef();
		head_body_def.position = ( b2Vec2 ){ 0., 6. };
		head_body_def.type = b2_dynamicBody;
		b2BodyId head_body_id = b2CreateBody( world_id, &head_body_def );
		b2RevoluteJointDef joint_def5 = b2DefaultRevoluteJointDef();
		joint_def5.bodyIdA = simulon_body_id;
		joint_def5.bodyIdB = head_body_id;
		joint_def5.localAnchorA = ( b2Vec2 ){ 0.0, 0.8 };
		joint_def5.localAnchorB = ( b2Vec2 ){ 0.0, -0.17 / 2.0 };

		b2JointId revolute_joint_id = b2CreateRevoluteJoint( world_id, &joint_def5 );
		b2DistanceJointDef joint_def6 = b2DefaultDistanceJointDef();
		joint_def6.bodyIdA = simulon_body_id;
		joint_def6.bodyIdB = head_body_id;
		joint_def6.localAnchorA = ( b2Vec2 ){ 0.0, 1.7 };
		joint_def6.localAnchorB = ( b2Vec2 ){ 0.0, 0.8 };
		joint_def6.length = 0.005;
		joint_def6.hertz = 1.;
		b2CreateDistanceJoint( world_id, &joint_def6 );

		b2DestroyBody( simulon_body_id );

		b2World_Step( world_id, 1. / 60., 4 );

		b2DestroyWorld( world_id );
	}

	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		b2WorldId world_id = b2CreateWorld( &worldDef );

		b2BodyDef ground_body_def = b2DefaultBodyDef();
		ground_body_def.type = b2_staticBody;
		b2BodyId ground_body_id = b2CreateBody( world_id, &ground_body_def );

		b2BodyDef box_body_def = b2DefaultBodyDef();
		box_body_def.type = b2_dynamicBody;
		box_body_def.position = ( b2Vec2 ){ 0.0, 0.0 };
		b2BodyId box_body_id = b2CreateBody( world_id, &box_body_def );
		b2Polygon polygon = b2MakeBox( 1., 1. );
		b2ShapeDef shape_def = b2DefaultShapeDef();
		b2ShapeId box_shape = b2CreatePolygonShape( box_body_id, &shape_def, &polygon );

		b2DistanceJointDef distance_joint_def = b2DefaultDistanceJointDef();
		distance_joint_def.hertz = 1.;
		distance_joint_def.dampingRatio = 0.1;
		distance_joint_def.bodyIdA = ground_body_id;
		distance_joint_def.bodyIdB = box_body_id;
		distance_joint_def.minLength = 0.005;
		distance_joint_def.enableSpring = true;
		distance_joint_def.enableLimit = false;
		distance_joint_def.collideConnected = false;
		distance_joint_def.length = 0.005;
		b2Body_SetTransform( ground_body_id, ( b2Vec2 ){ 0.0, 0.0 }, ( b2Rot ){ 1., 0. } );
		distance_joint_def.localAnchorA = ( b2Vec2 ){ 0.0, 0.0 };
		distance_joint_def.localAnchorB = ( b2Vec2 ){ 0.0, 0.0 };
		b2JointId distance_joint_id = b2CreateDistanceJoint( world_id, &distance_joint_def );

		b2Body_SetType( box_body_id, b2_staticBody );
		b2World_Step( world_id, 1. / 60., 4 );

		b2DestroyJoint( distance_joint_id );

		b2DestroyWorld( world_id );
	}

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

int WorldTest( void )
{
	RUN_SUBTEST( TestForAmy );
	RUN_SUBTEST( HelloWorld );
	RUN_SUBTEST( EmptyWorld );
	RUN_SUBTEST( DestroyAllBodiesWorld );
	RUN_SUBTEST( TestIsValid );
	RUN_SUBTEST( TestWorldRecycle );

	return 0;
}
