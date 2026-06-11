// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "body.h"
#include "physics_world.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

// Read a body center of mass relative to a base position, in double precision mode. The public
// getters demote to float (~1 m resolution at 1e7), so the precision check reaches into the body
// sim. In float mode this is the same demoted vector.
static b2Vec2 BodyRelativeCenter( b2WorldId worldId, b2BodyId bodyId, b2Vec2 baseVec )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* sim = b2GetBodySim( world, body );
	return b2PositionDelta( sim->center, b2MakePosition( baseVec ) );
}

#define PYRAMID_BODY_COUNT 9

typedef struct PyramidResult
{
	int sleepStep;
	b2Vec2 rel[PYRAMID_BODY_COUNT];
} PyramidResult;

// Build a stepped pyramid of unit boxes on a base position, settle it, and report the sleep frame
// and the settled centers relative to the base. Integer offsets keep every float bodyDef.position
// exact even at 1e7 (the public position API is still float), so the origin and large world runs
// start from an identical relative configuration.
static PyramidResult RunPyramid( b2Vec2 baseVec )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = baseVec;
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		// Ground top surface at baseY + 0.5
		b2Polygon box = b2MakeBox( 10.0f, 0.5f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	// Each box is 1 m, centers on integer offsets. Row 0 rests on the ground, rows stack directly
	// above so the configuration is stable and sleeps quickly.
	static const b2Vec2 offsets[PYRAMID_BODY_COUNT] = {
		{ -2.0f, 1.0f }, { -1.0f, 1.0f }, { 0.0f, 1.0f }, { 1.0f, 1.0f }, { 2.0f, 1.0f },
		{ -1.0f, 2.0f }, { 0.0f, 2.0f }, { 1.0f, 2.0f },
		{ 0.0f, 3.0f },
	};

	b2BodyId bodyIds[PYRAMID_BODY_COUNT];
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = ( b2Vec2 ){ baseVec.x + offsets[i].x, baseVec.y + offsets[i].y };
		bodyIds[i] = b2CreateBody( worldId, &bodyDef );
		b2CreatePolygonShape( bodyIds[i], &shapeDef, &box );
	}

	PyramidResult result = { .sleepStep = -1 };

	for ( int step = 0; step < 250; ++step )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );

		if ( result.sleepStep < 0 )
		{
			b2BodyEvents events = b2World_GetBodyEvents( worldId );
			if ( events.moveCount == 0 && b2World_GetAwakeBodyCount( worldId ) == 0 )
			{
				result.sleepStep = step;
			}
		}
	}

	for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
	{
		result.rel[i] = BodyRelativeCenter( worldId, bodyIds[i], baseVec );
	}

	b2DestroyWorld( worldId );
	return result;
}

// Far from the origin the contact boundary is differenced in double, so a settling pyramid must
// follow the same relative trajectory as one at the origin and sleep on the same frame. A float
// build cannot resolve the body positions at 1e7, which is the whole point of large world mode.
static int LargeWorldPyramidTest( void )
{
	PyramidResult origin = RunPyramid( ( b2Vec2 ){ 0.0f, 0.0f } );
	ENSURE( origin.sleepStep > 0 );

#if defined( BOX2D_DOUBLE_PRECISION )
	PyramidResult large = RunPyramid( ( b2Vec2 ){ 1.0e7f, 0.0f } );

	ENSURE( large.sleepStep == origin.sleepStep );
	for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
	{
		ENSURE_SMALL( large.rel[i].x - origin.rel[i].x, 1e-3f );
		ENSURE_SMALL( large.rel[i].y - origin.rel[i].y, 1e-3f );
	}
#endif

	return 0;
}

// Fire a bullet at a thin wall and report where it ends up relative to the base. With continuous
// collision the bullet stops at the near face, without it the bullet tunnels far past the wall.
static float RunBullet( b2Vec2 baseVec )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Thin tall wall centered on the base
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_staticBody;
	bodyDef.position = baseVec;
	b2BodyId wallId = b2CreateBody( worldId, &bodyDef );
	b2Polygon wall = b2MakeBox( 0.05f, 5.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( wallId, &shapeDef, &wall );

	// Bullet fired at the wall from the far side. At 200 m/s it crosses ~3.3 m per step, so without
	// continuous collision it passes the 0.1 m wall in the first step.
	bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = ( b2Vec2 ){ baseVec.x + 10.0f, baseVec.y };
	bodyDef.linearVelocity = ( b2Vec2 ){ -200.0f, 0.0f };
	b2BodyId bulletId = b2CreateBody( worldId, &bodyDef );
	b2Circle circle = { { 0.0f, 0.0f }, 0.1f };
	shapeDef = b2DefaultShapeDef();
	b2CreateCircleShape( bulletId, &shapeDef, &circle );

	for ( int step = 0; step < 30; ++step )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float relX = BodyRelativeCenter( worldId, bulletId, baseVec ).x;
	b2DestroyWorld( worldId );
	return relX;
}

// The swept query box is rounded back to world float, which at 1e7 has ~1 m resolution, the most
// likely place to drop the hit. The re-centered TOI must still catch the wall with no tunneling.
static int LargeWorldBulletTest( void )
{
	// At the origin the bullet must stop at the near face in both precision modes. Wall face at
	// 0.05 plus the 0.1 bullet radius puts the rest position near 0.15.
	float relX = RunBullet( ( b2Vec2 ){ 0.0f, 0.0f } );
	ENSURE( relX > 0.0f && relX < 0.5f );

#if defined( BOX2D_DOUBLE_PRECISION )
	relX = RunBullet( ( b2Vec2 ){ 1.0e7f, 0.0f } );
	ENSURE( relX > 0.0f && relX < 0.5f );
#endif

	return 0;
}

int LargeWorldTest( void )
{
	RUN_SUBTEST( LargeWorldPyramidTest );
	RUN_SUBTEST( LargeWorldBulletTest );

	return 0;
}
