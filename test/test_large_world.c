// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "physics_world.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

// Read a body center of mass relative to a base position, in double precision mode. The public
// getters demote to float (~1 m resolution at 1e7), so the precision check reaches into the body
// sim. In float mode this is the same demoted vector.
static b2Vec2 BodyRelativeCenter( b2WorldId worldId, b2BodyId bodyId, b2Pos baseVec )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* sim = b2GetBodySim( world, body );
	return b2SubPos( sim->center, baseVec );
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
static PyramidResult RunPyramid( b2Pos baseVec )
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
		{ -1.0f, 2.0f }, { 0.0f, 2.0f },  { 1.0f, 2.0f }, { 0.0f, 3.0f },
	};

	b2BodyId bodyIds[PYRAMID_BODY_COUNT];
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	for ( int i = 0; i < PYRAMID_BODY_COUNT; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ baseVec.x + offsets[i].x, baseVec.y + offsets[i].y };
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
	PyramidResult origin = RunPyramid( (b2Pos){ 0.0f, 0.0f } );
	ENSURE( origin.sleepStep > 0 );

#if defined( BOX2D_DOUBLE_PRECISION )
	PyramidResult large = RunPyramid( (b2Pos){ 1.0e7f, 0.0f } );

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
static float RunBullet( b2Pos baseVec )
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
	bodyDef.position = (b2Pos){ baseVec.x + 10.0f, baseVec.y };
	bodyDef.linearVelocity = (b2Vec2){ -200.0f, 0.0f };
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
	float relX = RunBullet( (b2Pos){ 0.0f, 0.0f } );
	ENSURE( relX > 0.0f && relX < 0.5f );

#if defined( BOX2D_DOUBLE_PRECISION )
	relX = RunBullet( (b2Pos){ 1.0e7f, 0.0f } );
	ENSURE( relX > 0.0f && relX < 0.5f );
#endif

	return 0;
}

// Cast a ray at a unit box on the base and report the hit point relative to the base. The ray
// origin is differenced against the body position in double, so the hit must land on the box face
// regardless of how far the base is from the origin.
static b2Vec2 RunRayCast( b2Pos baseVec )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = baseVec;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	// Ray from 5 m left of the box, traveling 10 m right. Hits the left face at base + {-0.5, 0}.
	b2Pos origin = (b2Pos){ baseVec.x - 5.0f, baseVec.y };
	b2Vec2 translation = { 10.0f, 0.0f };
	b2RayResult result = b2World_CastRayClosest( worldId, origin, translation, b2DefaultQueryFilter() );

	// A miss leaves the point at the origin, which the caller's position check rejects
	b2Vec2 rel = b2SubPos( result.point, baseVec );
	b2DestroyWorld( worldId );
	return rel;
}

// A float ray cast at 1e7 would resolve the hit only to the ~1 m coordinate ULP. The double origin
// plus per-shape re-centering keeps the analytic hit point accurate far from the origin.
static int LargeWorldRayCastTest( void )
{
	b2Vec2 rel = RunRayCast( (b2Pos){ 0.0f, 0.0f } );
	ENSURE_SMALL( rel.x + 0.5f, 1e-4f );
	ENSURE_SMALL( rel.y, 1e-4f );

#if defined( BOX2D_DOUBLE_PRECISION )
	rel = RunRayCast( (b2Pos){ 1.0e7f, 0.0f } );
	ENSURE_SMALL( rel.x + 0.5f, 1e-4f );
	ENSURE_SMALL( rel.y, 1e-4f );
#endif

	return 0;
}

typedef struct OriginQueryData
{
	int overlapCount;
	b2Pos castPoint;
	float castFraction;
	float moverFraction;
	int planeCount;
	bool insidePoint;
	b2Pos shapeRayPoint;
	bool shapeRayHit;
} OriginQueryData;

static bool OriginOverlapFcn( b2ShapeId id, void* ctx )
{
	(void)id;
	OriginQueryData* data = ctx;
	data->overlapCount += 1;
	return true;
}

static float OriginCastFcn( b2ShapeId id, b2Pos point, b2Vec2 normal, float fraction, void* ctx )
{
	(void)id;
	(void)normal;
	OriginQueryData* data = ctx;
	data->castPoint = point;
	data->castFraction = fraction;
	return fraction;
}

static bool OriginPlaneFcn( b2ShapeId id, const b2PlaneResult* plane, void* ctx )
{
	(void)id;
	(void)plane;
	OriginQueryData* data = ctx;
	data->planeCount += 1;
	return true;
}

// Issue every origin taking query against a unit box on the base, with all geometry relative to
// the base. The results must match an origin zero run, which is what makes the origin plumbing
// (tree lift, per shape re-centering, output compose) non vacuous far from the origin.
static OriginQueryData RunOriginQueries( b2Pos base )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = base;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2ShapeId shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );

	b2QueryFilter filter = b2DefaultQueryFilter();
	OriginQueryData data = { 0 };
	data.castFraction = 1.0f;

	// Overlap a small circle centered on the box
	b2Vec2 center = { 0.0f, 0.0f };
	b2ShapeProxy overlapProxy = b2MakeProxy( &center, 1, 0.1f );
	b2World_OverlapShape( worldId, base, &overlapProxy, filter, OriginOverlapFcn, &data );

	// Cast a small circle at the left face. Center stops at -0.6, hit point on the face at -0.5.
	b2Vec2 start = { -5.0f, 0.0f };
	b2ShapeProxy castProxy = b2MakeProxy( &start, 1, 0.1f );
	b2World_CastShape( worldId, base, &castProxy, (b2Vec2){ 10.0f, 0.0f }, filter, OriginCastFcn, &data );

	// Mover cast at the box
	b2Capsule mover = { { -5.0f, -0.2f }, { -5.0f, 0.2f }, 0.3f };
	data.moverFraction = b2World_CastMover( worldId, base, &mover, (b2Vec2){ 10.0f, 0.0f }, filter );

	// Mover overlapping the box gathers planes
	b2Capsule touching = { { -0.9f, -0.2f }, { -0.9f, 0.2f }, 0.5f };
	b2World_CollideMover( worldId, base, &touching, filter, OriginPlaneFcn, &data );

	// Shape level queries at the base
	data.insidePoint = b2Shape_TestPoint( shapeId, base );

	b2WorldCastOutput rayOutput =
		b2Shape_RayCast( shapeId, b2OffsetPos( base, (b2Vec2){ -5.0f, 0.0f } ), (b2Vec2){ 10.0f, 0.0f } );
	data.shapeRayHit = rayOutput.hit;
	data.shapeRayPoint = rayOutput.point;

	b2DestroyWorld( worldId );
	return data;
}

static int LargeWorldOriginQueryTest( void )
{
	OriginQueryData origin = RunOriginQueries( b2Pos_zero );
	ENSURE( origin.overlapCount == 1 );
	ENSURE( origin.castFraction < 1.0f );
	ENSURE( origin.moverFraction < 1.0f );
	ENSURE( origin.planeCount >= 1 );
	ENSURE( origin.insidePoint == true );
	ENSURE( origin.shapeRayHit == true );

	b2Vec2 castRel = b2SubPos( origin.castPoint, b2Pos_zero );
	ENSURE_SMALL( castRel.x + 0.5f, 1e-3f );
	b2Vec2 rayRel = b2SubPos( origin.shapeRayPoint, b2Pos_zero );
	ENSURE_SMALL( rayRel.x + 0.5f, 1e-3f );

#if defined( BOX2D_DOUBLE_PRECISION )
	// The same relative queries far from the origin must reproduce the origin run. A float query
	// at 1e7 could not resolve the faces below the coordinate ULP.
	b2Pos base = { 1.0e7f, 0.0f };
	OriginQueryData large = RunOriginQueries( base );
	ENSURE( large.overlapCount == origin.overlapCount );
	ENSURE( large.planeCount == origin.planeCount );
	ENSURE( large.insidePoint == origin.insidePoint );
	ENSURE( large.shapeRayHit == origin.shapeRayHit );
	ENSURE_SMALL( large.castFraction - origin.castFraction, 1e-4f );
	ENSURE_SMALL( large.moverFraction - origin.moverFraction, 1e-4f );

	b2Vec2 castRelLarge = b2SubPos( large.castPoint, base );
	ENSURE_SMALL( castRelLarge.x - castRel.x, 1e-3f );
	ENSURE_SMALL( castRelLarge.y - castRel.y, 1e-3f );

	b2Vec2 rayRelLarge = b2SubPos( large.shapeRayPoint, base );
	ENSURE_SMALL( rayRelLarge.x - rayRel.x, 1e-3f );
	ENSURE_SMALL( rayRelLarge.y - rayRel.y, 1e-3f );
#endif

	return 0;
}

// A recording made far from the origin must replay with no divergence. The recorded world positions
// ride the double precision wire format added for large world mode; demoting them to float would snap
// a replayed body off the recorded path and the per step state hash would diverge. Float build: the
// positions are float either way, so this is a plain recording round trip.
static int LargeWorldRecordingTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// At 1e7 the double precision wire format is what keeps the replay on the recorded path. The
	// float build has no large world range (the broad phase asserts |coord| < B2_HUGE = 1e5), so it
	// records the same scene at the origin: a plain recording round trip.
#if defined( BOX2D_DOUBLE_PRECISION )
	b2Pos base = { 1.0e7f, 0.0f };
#else
	b2Pos base = { 0.0f, 0.0f };
#endif

	// Record from before any body exists so world positions ride the op stream (CreateBody,
	// SetTransform), exercising the recorded position wire format rather than the seed snapshot.
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, rec );

	b2BodyDef groundDef = b2DefaultBodyDef();
	groundDef.position = base;
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2Polygon groundBox = b2MakeBox( 20.0f, 0.5f );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );

	// A box settling on the ground exercises the contact solver far from the origin.
	b2BodyDef stackDef = b2DefaultBodyDef();
	stackDef.type = b2_dynamicBody;
	stackDef.position = (b2Pos){ base.x, base.y + 1.0f };
	b2BodyId stackId = b2CreateBody( worldId, &stackDef );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef boxDef = b2DefaultShapeDef();
	boxDef.density = 1.0f;
	b2CreatePolygonShape( stackId, &boxDef, &box );

	// A free body sliding along x so its center accrues sub-meter detail that float cannot hold at
	// 1e7. Its evolved double transform is fed back through SetTransform mid recording, so the op
	// stream carries a position no float could round trip.
	b2BodyDef sliderDef = b2DefaultBodyDef();
	sliderDef.type = b2_dynamicBody;
	sliderDef.gravityScale = 0.0f;
	sliderDef.position = (b2Pos){ base.x, base.y + 5.0f };
	sliderDef.linearVelocity = (b2Vec2){ 3.0f, 0.0f };
	b2BodyId sliderId = b2CreateBody( worldId, &sliderDef );
	b2CreatePolygonShape( sliderId, &boxDef, &box );

	for ( int step = 0; step < 30; ++step )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b2WorldTransform sliderXf = b2Body_GetTransform( sliderId );
	b2Body_SetTransform( sliderId, sliderXf.p, sliderXf.q );

	for ( int step = 0; step < 30; ++step )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );

	const uint8_t* data = b2Recording_GetData( rec );
	int size = b2Recording_GetSize( rec );
	ENSURE( size > 0 );

	// Replay at the recorded worker count and a different one. A demoted op stream position would
	// diverge the state hash; the double precision wire format reproduces the run exactly.
	ENSURE( b2ValidateReplay( data, size, 0 ) );
	ENSURE( b2ValidateReplay( data, size, 4 ) );

	b2DestroyRecording( rec );
	return 0;
}

int LargeWorldTest( void )
{
	RUN_SUBTEST( LargeWorldPyramidTest );
	RUN_SUBTEST( LargeWorldBulletTest );
	RUN_SUBTEST( LargeWorldRayCastTest );
	RUN_SUBTEST( LargeWorldOriginQueryTest );
	RUN_SUBTEST( LargeWorldRecordingTest );

	return 0;
}
