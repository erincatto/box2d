// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>

static const char* s_recPath = "recording_test.b2rec";
static const char* s_savedPath = "test_recording_saved.b2rec";

int RecordingTest( void )
{
	// Record a session

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	worldDef.workerCount = 1;
	worldDef.recordingPath = s_recPath;

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	// Static ground body with a circle shape
	b2BodyDef groundDef = b2DefaultBodyDef();
	groundDef.position = (b2Vec2){ 0.0f, -10.0f };
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	ENSURE( b2Body_IsValid( groundId ) );

	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2Circle groundCircle = { { 0.0f, 0.0f }, 10.0f };
	b2ShapeId groundShapeId = b2CreateCircleShape( groundId, &groundShapeDef, &groundCircle );
	ENSURE( b2Shape_IsValid( groundShapeId ) );

	// Dynamic body with a circle shape
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Vec2){ 0.0f, 4.0f };
	bodyDef.name = "testBody";
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId ) );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2ShapeId shapeId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
	ENSURE( b2Shape_IsValid( shapeId ) );

	// Exercise the recorded mutators
	b2Body_SetTransform( bodyId, (b2Vec2){ 1.0f, 5.0f }, b2Rot_identity );
	b2Body_SetLinearVelocity( bodyId, (b2Vec2){ 0.5f, 0.0f } );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, timeStep, subStepCount );
	}

	// Save a mid-recording snapshot, must replay deterministically too
	b2World_SaveRecording( worldId, s_savedPath );

	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );

	// Verify both files are non-empty
	{
		FILE* fp = fopen( s_recPath, "rb" );
		ENSURE( fp != NULL );
		fseek( fp, 0, SEEK_END );
		long sz = ftell( fp );
		fclose( fp );
		ENSURE( sz > 0 );
	}
	{
		FILE* fp = fopen( s_savedPath, "rb" );
		ENSURE( fp != NULL );
		fseek( fp, 0, SEEK_END );
		long sz = ftell( fp );
		fclose( fp );
		ENSURE( sz > 0 );
	}

	// Replay with recorded worker count
	bool ok1 = b2ReplayFile( s_recPath, 0 );
	ENSURE( ok1 );

	// Replay with a different worker count to prove cross-thread determinism
	bool ok4 = b2ReplayFile( s_recPath, 4 );
	ENSURE( ok4 );

	// The saved snapshot must replay deterministically at both worker counts
	bool okSaved0 = b2ReplayFile( s_savedPath, 0 );
	ENSURE( okSaved0 );

	bool okSaved4 = b2ReplayFile( s_savedPath, 4 );
	ENSURE( okSaved4 );

	remove( s_recPath );
	remove( s_savedPath );
	return 0;
}
