// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

#include "core.h"

#include <float.h>

bool b2IsValid( float a )
{
	if ( isnan( a ) )
	{
		return false;
	}

	if ( isinf( a ) )
	{
		return false;
	}

	return true;
}

bool b2Vec2_IsValid( b2Vec2 v )
{
	if ( isnan( v.x ) || isnan( v.y ) )
	{
		return false;
	}

	if ( isinf( v.x ) || isinf( v.y ) )
	{
		return false;
	}

	return true;
}

bool b2Rot_IsValid( b2Rot q )
{
	if ( isnan( q.s ) || isnan( q.c ) )
	{
		return false;
	}

	if ( isinf( q.s ) || isinf( q.c ) )
	{
		return false;
	}

	return b2IsNormalized( q );
}

// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
float b2Atan2( float y, float x )
{
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0.0f && y == 0.0f)
	{
		return 0.0f;
	}

	float ax = b2AbsFloat( x );
	float ay = b2AbsFloat( y );
	float mx = b2MaxFloat( ay, ax );
	float mn = b2MinFloat( ay, ax );
	float a = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	float s = a * a;
	float c = s * a;
	float q = s * s;
	float r = 0.024840285f * q + 0.18681418f;
	float t = -0.094097948f * q - 0.33213072f;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = 1.57079637f - r;
	}

	if ( x < 0 )
	{
		r = 3.14159274f - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return r;
}

// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b2CosSin b2ComputeCosSin( float angle )
{
	// return ( b2CosSin ){ cosf( angle ), sinf( angle ) };

	float x = b2UnwindLargeAngle( angle );
	float pi2 = b2_pi * b2_pi;

	b2Rot q;

	// cosine needs angle in [-pi/2, pi/2]
	if ( x < -0.5f * b2_pi )
	{
		float y = x + b2_pi;
		float y2 = y * y;
		q.c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else if ( x > 0.5f * b2_pi )
	{
		float y = x - b2_pi;
		float y2 = y * y;
		q.c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else
	{
		float y2 = x * x;
		q.c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	if ( x < 0.0f )
	{
		float y = x + b2_pi;
		q.s = -16.0f * y * ( b2_pi - y ) / ( 5.0f * pi2 - 4.0f * y * ( b2_pi - y ) );
	}
	else
	{
		q.s = 16.0f * x * ( b2_pi - x ) / ( 5.0f * pi2 - 4.0f * x * ( b2_pi - x ) );
	}

	q = b2NormalizeRot( q );
	return ( b2CosSin ){ q.c, q.s };
}
