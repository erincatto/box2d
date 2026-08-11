// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

static int SegmentDistanceTest( void )
{
	b2Vec2 p1 = { -1.0f, -1.0f };
	b2Vec2 q1 = { -1.0f, 1.0f };
	b2Vec2 p2 = { 2.0f, 0.0f };
	b2Vec2 q2 = { 1.0f, 0.0f };

	b2SegmentDistanceResult result = b2SegmentDistance( p1, q1, p2, q2 );

	ENSURE_SMALL( result.fraction1 - 0.5f, FLT_EPSILON );
	ENSURE_SMALL( result.fraction2 - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( result.closest1.x + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( result.closest1.y, FLT_EPSILON );
	ENSURE_SMALL( result.closest2.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( result.closest2.y, FLT_EPSILON );
	ENSURE_SMALL( result.distanceSquared - 4.0f, FLT_EPSILON );

	return 0;
}

static int ShapeDistanceTest( void )
{
	b2Vec2 vas[] = { ( b2Vec2 ){ -1.0f, -1.0f }, ( b2Vec2 ){ 1.0f, -1.0f }, ( b2Vec2 ){ 1.0f, 1.0f }, ( b2Vec2 ){ -1.0f, 1.0f } };

	b2Vec2 vbs[] = {
		( b2Vec2 ){ 2.0f, -1.0f },
		( b2Vec2 ){ 2.0f, 1.0f },
	};

	b2DistanceInput input;
	input.proxyA = b2MakeProxy( vas, ARRAY_COUNT( vas ), 0.0f );
	input.proxyB = b2MakeProxy( vbs, ARRAY_COUNT( vbs ), 0.0f );
	input.transform = b2Transform_identity;
	input.useRadii = false;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance(&input,  &cache, NULL, 0 );

	ENSURE_SMALL( output.distance - 1.0f, FLT_EPSILON );

	return 0;
}

static int ShapeCastTest( void )
{
	b2Vec2 vas[] = { ( b2Vec2 ){ -1.0f, -1.0f }, ( b2Vec2 ){ 1.0f, -1.0f }, ( b2Vec2 ){ 1.0f, 1.0f }, ( b2Vec2 ){ -1.0f, 1.0f } };

	b2Vec2 vbs[] = {
		( b2Vec2 ){ 2.0f, -1.0f },
		( b2Vec2 ){ 2.0f, 1.0f },
	};

	b2ShapeCastPairInput input = { 0 };
	input.proxyA = b2MakeProxy( vas, ARRAY_COUNT( vas ), 0.0f );
	input.proxyB = b2MakeProxy( vbs, ARRAY_COUNT( vbs ), 0.0f );
	input.transform = b2Transform_identity;
	input.translationB = ( b2Vec2 ){ -2.0f, 0.0f };
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b2CastOutput output = b2ShapeCast( &input );

	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

	return 0;
}

// A thin plank and a point a linear slop off the middle of its short end face. The closest feature
// is that end face, so GJK finishes on a two point simplex spanning it.
//
// b2SolveSimplex2 hands back a search direction of magnitude 2 * distance * edge^2, which the
// caller weighs against an absolute epsilon. That test is cubic in length, so it gives out on
// short edges: below an edge of about 3.5 mm at default length units it declares the shapes
// overlapped and returns a zero distance with no normal, though they are plainly apart.
//
// Measured cutoff matches 2 * distance * edge^2 < FLT_EPSILON to three digits, and it tracks the
// contact edge alone. Widening the plank from 1 m to 100 m changes nothing.
static int ShapeDistanceShortEdgeTest( void )
{
	float halfThickness = 0.0016f;

	b2Vec2 corners[] = {
		( b2Vec2 ){ -0.5f, -halfThickness },
		( b2Vec2 ){ 0.5f, -halfThickness },
		( b2Vec2 ){ 0.5f, halfThickness },
		( b2Vec2 ){ -0.5f, halfThickness },
	};

	b2Vec2 point = { 0.5f + B2_LINEAR_SLOP, 0.0f };

	b2DistanceInput input = { 0 };
	input.proxyA = b2MakeProxy( corners, ARRAY_COUNT( corners ), 0.0f );
	input.proxyB = b2MakeProxy( &point, 1, 0.0f );
	input.transform = b2Transform_identity;
	input.useRadii = false;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, NULL, 0 );

	ENSURE( output.distance > 0.0f );
	ENSURE( b2IsNormalized( output.normal ) );
	ENSURE_SMALL( output.distance - B2_LINEAR_SLOP, 1e-6f );

	return 0;
}

// The same false overlap seen through b2ShapeCast, which is how it was reported.
//
// Conservative advancement stops the cores a target apart and target is at least a linear slop, so
// the query can never legitimately answer overlap after the first iteration. Here iteration 1 does,
// and b2ShapeCast has no fallback: it trips
// B2_ASSERT( distanceOutput.distance > 0.0f && b2IsNormalized( distanceOutput.normal ) ).
//
// Enable once b2ShapeDistance stops reporting the false overlap. The assert aborts the run, so
// leaving it on would take the rest of the suite with it.
#define B2_SHORT_EDGE_CAST_REPRO 0
#if B2_SHORT_EDGE_CAST_REPRO
static int ShapeCastShortEdgeTest( void )
{
	float halfThickness = 0.0002f;

	b2Vec2 corners[] = {
		( b2Vec2 ){ -0.5f, -halfThickness },
		( b2Vec2 ){ 0.5f, -halfThickness },
		( b2Vec2 ){ 0.5f, halfThickness },
		( b2Vec2 ){ -0.5f, halfThickness },
	};

	// falls past the end of the plank and drifts into it, so the second query lands on the end face
	b2Vec2 start = { 0.506f, 0.0035f };

	b2ShapeCastPairInput input = { 0 };
	input.proxyA = b2MakeProxy( corners, ARRAY_COUNT( corners ), 0.0f );
	input.proxyB = b2MakeProxy( &start, 1, 0.0f );
	input.transform = b2Transform_identity;
	input.translationB = ( b2Vec2 ){ -0.01f, -0.5f };
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b2CastOutput output = b2ShapeCast( &input );

	ENSURE( output.hit == false || b2IsNormalized( output.normal ) );

	return 0;
}
#endif

static int TimeOfImpactTest( void )
{
	b2Vec2 vas[] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };

	b2Vec2 vbs[] = {
		{ 2.0f, -1.0f },
		{ 2.0f, 1.0f },
	};

	b2TOIInput input;
	input.proxyA = b2MakeProxy( vas, ARRAY_COUNT( vas ), 0.0f );
	input.proxyB = b2MakeProxy( vbs, ARRAY_COUNT( vbs ), 0.0f );
	input.sweepA = ( b2Sweep ){ b2Vec2_zero, b2Vec2_zero, b2Vec2_zero, b2Rot_identity, b2Rot_identity };
	input.sweepB = ( b2Sweep ){ b2Vec2_zero, b2Vec2_zero, ( b2Vec2 ){ -2.0f, 0.0f }, b2Rot_identity, b2Rot_identity };
	input.maxFraction = 1.0f;

	b2TOIOutput output = b2TimeOfImpact( &input );

	ENSURE( output.state == b2_toiStateHit );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

	return 0;
}

int DistanceTest( void )
{
	RUN_SUBTEST( SegmentDistanceTest );
	RUN_SUBTEST( ShapeDistanceTest );
	RUN_SUBTEST( ShapeCastTest );
	RUN_SUBTEST( ShapeDistanceShortEdgeTest );
#if B2_SHORT_EDGE_CAST_REPRO
	RUN_SUBTEST( ShapeCastShortEdgeTest );
#endif
	RUN_SUBTEST( TimeOfImpactTest );

	return 0;
}
