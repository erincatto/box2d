// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/math_functions.h"

#include <float.h>
#include <stdio.h>

// 0.0023 degrees
#define ATAN_TOL 0.00004f

int MathTest( void )
{
	for ( float t = -10.0f; t < 10.0f; t += 0.01f )
	{
		float angle = b2_pi * t;
		b2Rot r = b2MakeRot( angle );
		float c = cosf( angle );
		float s = sinf( angle );

		// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
		// printf( "%g %g\n", r.c - c, r.s - s );
		ENSURE_SMALL( r.c - c, 0.002f );
		ENSURE_SMALL( r.s - s, 0.002f );

		float xn = b2UnwindLargeAngle( angle );
		float a = b2Atan2( s, c );
		ENSURE( b2IsValid( a ) );

		float diff = b2AbsFloat( a - xn );

		// The two results can be off by 360 degrees (-pi and pi)
		if ( diff > b2_pi )
		{
			diff -= 2.0f * b2_pi;
		}

		// The approximate atan2 is quite accurate
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	for ( float y = -1.0f; y <= 1.0f; y += 0.01f )
	{
		for ( float x = -1.0f; x <= 1.0f; x += 0.01f )
		{
			float a1 = b2Atan2( y, x );
			float a2 = atan2f( y, x );
			float diff = b2AbsFloat( a1 - a2 );
			ENSURE( b2IsValid( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}
	}

	{
		float a1 = b2Atan2( 1.0f, 0.0f );
		float a2 = atan2f( 1.0f, 0.0f );
		float diff = b2AbsFloat( a1 - a2 );
		ENSURE( b2IsValid( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b2Atan2( -1.0f, 0.0f );
		float a2 = atan2f( -1.0f, 0.0f );
		float diff = b2AbsFloat( a1 - a2 );
		ENSURE( b2IsValid( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b2Atan2( 0.0f, 1.0f );
		float a2 = atan2f( 0.0f, 1.0f );
		float diff = b2AbsFloat( a1 - a2 );
		ENSURE( b2IsValid( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b2Atan2( 0.0f, -1.0f );
		float a2 = atan2f( 0.0f, -1.0f );
		float diff = b2AbsFloat( a1 - a2 );
		ENSURE( b2IsValid( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b2Atan2( 0.0f, 0.0f );
		float a2 = atan2f( 0.0f, 0.0f );
		float diff = b2AbsFloat( a1 - a2 );
		ENSURE( b2IsValid( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	b2Vec2 zero = b2Vec2_zero;
	b2Vec2 one = { 1.0f, 1.0f };
	b2Vec2 two = { 2.0f, 2.0f };

	b2Vec2 v = b2Add( one, two );
	ENSURE( v.x == 3.0f && v.y == 3.0f );

	v = b2Sub( zero, two );
	ENSURE( v.x == -2.0f && v.y == -2.0f );

	v = b2Add( two, two );
	ENSURE( v.x != 5.0f && v.y != 5.0f );

	b2Transform transform1 = { { -2.0f, 3.0f }, b2MakeRot( 1.0f ) };
	b2Transform transform2 = { { 1.0f, 0.0f }, b2MakeRot( -2.0f ) };

	b2Transform transform = b2MulTransforms( transform2, transform1 );

	v = b2TransformPoint( transform2, b2TransformPoint( transform1, two ) );

	b2Vec2 u = b2TransformPoint( transform, two );

	ENSURE_SMALL( u.x - v.x, 10.0f * FLT_EPSILON );
	ENSURE_SMALL( u.y - v.y, 10.0f * FLT_EPSILON );

	v = b2TransformPoint( transform1, two );
	v = b2InvTransformPoint( transform1, v );

	ENSURE_SMALL( v.x - two.x, 8.0f * FLT_EPSILON );
	ENSURE_SMALL( v.y - two.y, 8.0f * FLT_EPSILON );

	return 0;
}
