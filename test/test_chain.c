// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"

#include <stdint.h>

// The point array is the collidable surface and the ghosts sit outside it, so segment i spans
// points i and i + 1 and there is one index space for everything: segmentCount is pointCount
// for a loop and pointCount - 1 otherwise, and material slot i is segment i.

static const b2Vec2 loopTriangle[3] = {
	{ -2.0f, 0.0f },
	{ 2.0f, 0.0f },
	{ 0.0f, 3.0f },
};

static const b2Vec2 loopSquare[4] = {
	{ -2.0f, 0.0f },
	{ 2.0f, 0.0f },
	{ 2.0f, 4.0f },
	{ -2.0f, 4.0f },
};

static const b2Vec2 openLine[6] = {
	{ 0.0f, 0.0f }, { 2.0f, 1.0f }, { 4.0f, 1.0f }, { 6.0f, 0.0f }, { 8.0f, 0.0f }, { 10.0f, 1.0f },
};

static const b2Vec2 openPair[2] = {
	{ 0.0f, 0.0f },
	{ 4.0f, 0.0f },
};

static const b2Vec2 ghostBegin = { -2.0f, -1.0f };
static const b2Vec2 ghostEnd = { 12.0f, 2.0f };

static int assertCount;

static int CountingAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	MAYBE_UNUSED( condition );
	MAYBE_UNUSED( fileName );
	MAYBE_UNUSED( lineNumber );
	assertCount += 1;

	// Keep going so the subtest can report rather than break to the debugger
	return 0;
}

static void BeginCountingAsserts( void )
{
	assertCount = 0;
	b2SetAssertCallback( CountingAssertFcn );
}

// Always zero in a build without asserts, so a check on this is a debug signal only
static int EndCountingAsserts( void )
{
	b2SetAssertCallback( TestAssertFcn );
	return assertCount;
}

// Tag each material so a segment can be traced back to the slot it came from
static b2ChainId CreateTaggedChain( b2BodyId bodyId, const b2Vec2* points, int pointCount, int materialCount, bool isLoop )
{
	b2SurfaceMaterial materials[8];
	for ( int i = 0; i < materialCount; ++i )
	{
		materials[i] = b2DefaultSurfaceMaterial();
		materials[i].userMaterialId = (uint64_t)( i + 1 );
	}

	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.points = points;
	chainDef.pointCount = pointCount;
	chainDef.materials = materials;
	chainDef.materialCount = materialCount;
	chainDef.isLoop = isLoop;
	chainDef.ghost1 = ghostBegin;
	chainDef.ghost2 = ghostEnd;

	return b2CreateChain( bodyId, &chainDef );
}

static void ReadSegmentTags( b2ChainId chainId, uint64_t* tags, int count )
{
	b2ShapeId segments[8];
	b2Chain_GetSegments( chainId, segments, count );
	for ( int i = 0; i < count; ++i )
	{
		tags[i] = b2Shape_GetSurfaceMaterial( segments[i] ).userMaterialId;
	}
}

// Points are cloned into body local space with no transform applied, so an exact compare is
// right here and a tolerance would only hide a wiring mistake
static bool SamePoint( b2Vec2 a, b2Vec2 b )
{
	return a.x == b.x && a.y == b.y;
}

// A loop closes, so every point leads a segment. An open chain leaves the last point trailing.
static int SegmentCountTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	int triangle = b2Chain_GetSegmentCount( CreateTaggedChain( bodyId, loopTriangle, 3, 1, true ) );
	int square = b2Chain_GetSegmentCount( CreateTaggedChain( bodyId, loopSquare, 4, 1, true ) );
	int pair = b2Chain_GetSegmentCount( CreateTaggedChain( bodyId, openPair, 2, 1, false ) );
	int line = b2Chain_GetSegmentCount( CreateTaggedChain( bodyId, openLine, 6, 1, false ) );

	b2DestroyWorld( worldId );

	ENSURE( triangle == 3 );
	ENSURE( square == 4 );
	ENSURE( pair == 1 );
	ENSURE( line == 5 );

	return 0;
}

// Segment i spans points i and i + 1 and takes its ghosts from the neighbors. A loop wraps for
// both ends. An open chain takes the def ghosts at the two ends and neighbors everywhere else.
static int SegmentGeometryTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ChainId loopId = CreateTaggedChain( bodyId, loopSquare, 4, 1, true );
	b2ChainId openId = CreateTaggedChain( bodyId, openLine, 6, 1, false );

	b2ShapeId loopSegments[4];
	b2ShapeId openSegments[5];
	int loopCount = b2Chain_GetSegments( loopId, loopSegments, 4 );
	int openCount = b2Chain_GetSegments( openId, openSegments, 5 );

	b2ChainSegment loop[4];
	b2ChainSegment open[5];
	for ( int i = 0; i < 4; ++i )
	{
		loop[i] = b2Shape_GetChainSegment( loopSegments[i] );
	}
	for ( int i = 0; i < 5; ++i )
	{
		open[i] = b2Shape_GetChainSegment( openSegments[i] );
	}

	b2DestroyWorld( worldId );

	ENSURE( loopCount == 4 );
	ENSURE( openCount == 5 );

	// ENSURE stringizes into a format string, so keep a percent out of the condition
	for ( int i = 0; i < 4; ++i )
	{
		int next = ( i + 1 ) % 4;
		int after = ( i + 2 ) % 4;
		int prev = ( i + 3 ) % 4;
		ENSURE( SamePoint( loop[i].segment.point1, loopSquare[i] ) );
		ENSURE( SamePoint( loop[i].segment.point2, loopSquare[next] ) );
		ENSURE( SamePoint( loop[i].ghost1, loopSquare[prev] ) );
		ENSURE( SamePoint( loop[i].ghost2, loopSquare[after] ) );
	}

	for ( int i = 0; i < 5; ++i )
	{
		ENSURE( SamePoint( open[i].segment.point1, openLine[i] ) );
		ENSURE( SamePoint( open[i].segment.point2, openLine[i + 1] ) );
	}

	// The def ghosts extend the chain past its ends without consuming a point or a segment
	ENSURE( SamePoint( open[0].ghost1, ghostBegin ) );
	ENSURE( SamePoint( open[4].ghost2, ghostEnd ) );

	for ( int i = 1; i < 5; ++i )
	{
		ENSURE( SamePoint( open[i].ghost1, openLine[i - 1] ) );
	}
	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( SamePoint( open[i].ghost2, openLine[i + 2] ) );
	}

	return 0;
}

static int LoopMaterialTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2ChainId chainId = CreateTaggedChain( bodyId, loopSquare, 4, 4, true );

	uint64_t created[4];
	ReadSegmentTags( chainId, created, 4 );

	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	material.userMaterialId = 99;

	BeginCountingAsserts();
	b2Chain_SetSurfaceMaterial( chainId, &material, 1 );
	int assertsTripped = EndCountingAsserts();

	uint64_t updated[4];
	uint64_t reported[4];
	ReadSegmentTags( chainId, updated, 4 );
	for ( int i = 0; i < 4; ++i )
	{
		reported[i] = b2Chain_GetSurfaceMaterial( chainId, i ).userMaterialId;
	}

	b2DestroyWorld( worldId );

	ENSURE( assertsTripped == 0 );

	for ( int i = 0; i < 4; ++i )
	{
		uint64_t expected = i == 1 ? 99 : (uint64_t)( i + 1 );
		ENSURE( created[i] == (uint64_t)( i + 1 ) );
		ENSURE( updated[i] == expected );

		// The getter and the segment shape cannot disagree, there is one copy
		ENSURE( reported[i] == expected );
	}

	return 0;
}

// No ghost offset anywhere. Slot 0 and the last slot are ordinary segments, which is the part
// that used to be inert on an open chain.
static int OpenMaterialTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2ChainId chainId = CreateTaggedChain( bodyId, openLine, 6, 5, false );

	uint64_t created[5];
	ReadSegmentTags( chainId, created, 5 );

	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	material.userMaterialId = 99;

	BeginCountingAsserts();
	b2Chain_SetSurfaceMaterial( chainId, &material, 0 );
	material.userMaterialId = 98;
	b2Chain_SetSurfaceMaterial( chainId, &material, 4 );
	int assertsTripped = EndCountingAsserts();

	uint64_t updated[5];
	uint64_t reported[5];
	ReadSegmentTags( chainId, updated, 5 );
	for ( int i = 0; i < 5; ++i )
	{
		reported[i] = b2Chain_GetSurfaceMaterial( chainId, i ).userMaterialId;
	}

	b2DestroyWorld( worldId );

	ENSURE( assertsTripped == 0 );

	for ( int i = 0; i < 5; ++i )
	{
		uint64_t expected = (uint64_t)( i + 1 );
		if ( i == 0 )
		{
			expected = 99;
		}
		else if ( i == 4 )
		{
			expected = 98;
		}

		ENSURE( created[i] == (uint64_t)( i + 1 ) );
		ENSURE( updated[i] == expected );
		ENSURE( reported[i] == expected );
	}

	return 0;
}

// One material at creation covers the chain and does not lock it that way. A broadcast and a
// per segment set both work afterwards.
static int SingleMaterialTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2ChainId chainId = CreateTaggedChain( bodyId, openLine, 6, 1, false );

	uint64_t created[5];
	ReadSegmentTags( chainId, created, 5 );

	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	material.userMaterialId = 99;

	BeginCountingAsserts();
	b2Chain_SetAllSurfaceMaterials( chainId, &material );
	int assertsTripped = EndCountingAsserts();

	uint64_t broadcast[5];
	ReadSegmentTags( chainId, broadcast, 5 );

	// One material at creation must not narrow the segment index, which is what the old count
	// bound would have rejected here
	material.userMaterialId = 98;
	BeginCountingAsserts();
	b2Chain_SetSurfaceMaterial( chainId, &material, 2 );
	assertsTripped += EndCountingAsserts();

	uint64_t single[5];
	uint64_t reported[5];
	ReadSegmentTags( chainId, single, 5 );
	for ( int i = 0; i < 5; ++i )
	{
		reported[i] = b2Chain_GetSurfaceMaterial( chainId, i ).userMaterialId;
	}

	b2DestroyWorld( worldId );

	ENSURE( assertsTripped == 0 );

	for ( int i = 0; i < 5; ++i )
	{
		uint64_t expected = i == 2 ? 98 : 99;
		ENSURE( created[i] == 1 );
		ENSURE( broadcast[i] == 99 );
		ENSURE( single[i] == expected );
		ENSURE( reported[i] == expected );
	}

	return 0;
}

// A ghost left on its endpoint means no neighbor, so both ends must act as convex corners: a
// circle past either cap gets a contact, and a box corner just past either end gets the diagonal
// vertex normal instead of being snapped up onto the segment normal as if the surface continued.
static void CollideEnds( b2ChainSegment segment, int* circleCount, float* boxNormalX )
{
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2Polygon box = b2MakeSquare( 0.25f );

	b2Transform xf1 = { { 4.3f, 0.2f }, b2Rot_identity };
	b2Transform xf2 = { { -0.3f, 0.2f }, b2Rot_identity };
	circleCount[0] = b2CollideChainSegmentAndCircle( &segment, &circle, xf1 ).pointCount;
	circleCount[1] = b2CollideChainSegmentAndCircle( &segment, &circle, xf2 ).pointCount;

	b2SimplexCache cache = { 0 };
	xf1.p = (b2Vec2){ 4.26f, 0.26f };
	b2LocalManifold m1 = b2CollideChainSegmentAndPolygon( &segment, &box, xf1, &cache );

	cache = (b2SimplexCache){ 0 };
	xf2.p = (b2Vec2){ -0.26f, 0.26f };
	b2LocalManifold m2 = b2CollideChainSegmentAndPolygon( &segment, &box, xf2, &cache );

	boxNormalX[0] = m1.pointCount > 0 ? m1.normal.x : 0.0f;
	boxNormalX[1] = m2.pointCount > 0 ? m2.normal.x : 0.0f;

}

static int DegenerateGhostTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	// Right to left so the collision side faces up
	b2Vec2 p1 = { 4.0f, 0.0f };
	b2Vec2 p2 = { 0.0f, 0.0f };
	b2ChainSegment degenerate = { p1, { p1, p2 }, p2, B2_NULL_INDEX };

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2ShapeId createdId = b2CreateChainSegmentShape( bodyId, &shapeDef, &degenerate );

	b2Segment plain = { { 0.0f, -1.0f }, { 4.0f, -1.0f } };
	b2ShapeId setId = b2CreateSegmentShape( bodyId, &shapeDef, &plain );
	b2Shape_SetChainSegment( setId, &degenerate );

	b2ChainSegment created = b2Shape_GetChainSegment( createdId );
	b2ChainSegment set = b2Shape_GetChainSegment( setId );

	b2DestroyWorld( worldId );

	ENSURE( SamePoint( created.segment.point1, p1 ) );
	ENSURE( SamePoint( created.segment.point2, p2 ) );
	ENSURE( SamePoint( created.ghost1, set.ghost1 ) );
	ENSURE( SamePoint( created.ghost2, set.ghost2 ) );
	ENSURE( b2Distance( created.ghost1, p1 ) > B2_LINEAR_SLOP );
	ENSURE( b2Distance( created.ghost2, p2 ) > B2_LINEAR_SLOP );

	int circleCount[2];
	float boxNormalX[2];
	CollideEnds( created, circleCount, boxNormalX );

	ENSURE( circleCount[0] == 1 );
	ENSURE( circleCount[1] == 1 );
	ENSURE( boxNormalX[0] > 0.5f );
	ENSURE( boxNormalX[1] < -0.5f );

	return 0;
}

typedef struct MoverPlanes
{
	int count;
	b2Vec2 normal;
} MoverPlanes;

static bool GatherPlane( b2ShapeId shapeId, const b2PlaneResult* plane, void* context )
{
	MAYBE_UNUSED( shapeId );
	MoverPlanes* planes = context;
	planes->count += 1;
	planes->normal = plane->plane.normal;
	return true;
}

static MoverPlanes CollideMoverAt( b2WorldId worldId, float y )
{
	b2Capsule mover = { { 1.5f, y }, { 2.5f, y }, 0.3f };
	MoverPlanes planes = { 0 };
	b2World_CollideMover( worldId, b2Pos_zero, &mover, b2DefaultQueryFilter(), GatherPlane, &planes );
	return planes;
}

// The mover must see a chain segment from the front only, matching contacts and the mover cast.
// A plain segment beside it stays two-sided.
static int MoverOneSidedTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	// Right to left so the collision side faces up
	b2ChainSegment chainSegment = { { 5.0f, 0.0f }, { { 4.0f, 0.0f }, { 0.0f, 0.0f } }, { -1.0f, 0.0f }, B2_NULL_INDEX };
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2CreateChainSegmentShape( bodyId, &shapeDef, &chainSegment );

	MoverPlanes front = CollideMoverAt( worldId, 0.2f );
	MoverPlanes behind = CollideMoverAt( worldId, -0.2f );

	b2Segment segment = { { 4.0f, 0.0f }, { 0.0f, 0.0f } };
	b2BodyDef plainDef = b2DefaultBodyDef();
	plainDef.position = (b2Pos){ 0.0f, 10.0f };
	b2BodyId plainId = b2CreateBody( worldId, &plainDef );
	b2CreateSegmentShape( plainId, &shapeDef, &segment );

	MoverPlanes plainBehind = CollideMoverAt( worldId, 9.8f );

	b2DestroyWorld( worldId );

	ENSURE( front.count == 1 );
	ENSURE( front.normal.y > 0.99f );
	ENSURE( behind.count == 0 );
	ENSURE( plainBehind.count == 1 );
	ENSURE( plainBehind.normal.y < -0.99f );

	return 0;
}

// Each subtest probes a different part of the index space, so report all of them instead of
// stopping at the first failure the way RUN_SUBTEST does
static int RunChainSubtest( const char* name, int ( *subtest )( void ) )
{
	int result = subtest();
	printf( "  subtest %s: %s\n", result == 1 ? "failed" : "passed", name );
	return result;
}

int ChainTest( void )
{
	int result = 0;

	result |= RunChainSubtest( "SegmentCountTest", SegmentCountTest );
	result |= RunChainSubtest( "SegmentGeometryTest", SegmentGeometryTest );
	result |= RunChainSubtest( "LoopMaterialTest", LoopMaterialTest );
	result |= RunChainSubtest( "OpenMaterialTest", OpenMaterialTest );
	result |= RunChainSubtest( "SingleMaterialTest", SingleMaterialTest );
	result |= RunChainSubtest( "DegenerateGhostTest", DegenerateGhostTest );
	result |= RunChainSubtest( "MoverOneSidedTest", MoverOneSidedTest );

	return result;
}
