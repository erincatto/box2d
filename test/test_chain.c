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

// Matches the library default, restored so a later assert still breaks
static int BreakingAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	fprintf( stderr, "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stderr );
	return 1;
}

static void BeginCountingAsserts( void )
{
	assertCount = 0;
	b2SetAssertFcn( CountingAssertFcn );
}

// Always zero in a build without asserts, so a check on this is a debug signal only
static int EndCountingAsserts( void )
{
	b2SetAssertFcn( BreakingAssertFcn );
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
	chainDef.ghostBegin = ghostBegin;
	chainDef.ghostEnd = ghostEnd;

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

	material.userMaterialId = 98;
	b2Chain_SetSurfaceMaterial( chainId, &material, 2 );

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

	return result;
}
