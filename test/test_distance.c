// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
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
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
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

	b2ShapeCastPairInput input;
	input.proxyA = b2MakeProxy( vas, ARRAY_COUNT( vas ), 0.0f );
	input.proxyB = b2MakeProxy( vbs, ARRAY_COUNT( vbs ), 0.0f );
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.translationB = ( b2Vec2 ){ -2.0f, 0.0f };
	input.maxFraction = 1.0f;

	b2CastOutput output = b2ShapeCast( &input );

	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

	return 0;
}

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
	RUN_SUBTEST( TimeOfImpactTest );

	return 0;
}
