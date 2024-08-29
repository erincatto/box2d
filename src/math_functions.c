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

// https://mazzo.li/posts/vectorized-atan2.html
static inline float b2Atan( float x )
{
	float a1 = 0.99997726f;
	float a3 = -0.33262347f;
	float a5 = 0.19354346f;
	float a7 = -0.11643287f;
	float a9 = 0.05265332f;
	float a11 = -0.01172120f;

	float x2 = x * x;
	return x * ( a1 + x2 * ( a3 + x2 * ( a5 + x2 * ( a7 + x2 * ( a9 + x2 * a11 ) ) ) ) );
}

// I tested atan2f and got different results on Apple Clang (Arm) than MSVC (x64).
float b2Atan2( float y, float x )
{
	float pi = b2_pi;
	float halfPi = 0.5f * b2_pi;

	bool swap = b2AbsFloat( x ) < b2AbsFloat( y );
	float atanInput = ( swap ? x : y ) / ( swap ? y : x );

	// Approximate atan
	float res = b2Atan( atanInput );

	// If swapped, adjust atan output
	res = swap ? ( atanInput >= 0.0f ? halfPi : -halfPi ) - res : res;
	// Adjust quadrants
	if ( x >= 0.0f && y >= 0.0f )
	{
	} // 1st quadrant
	else if ( x < 0.0f && y >= 0.0f )
	{
		res = pi + res;
	} // 2nd quadrant
	else if ( x < 0.0f && y < 0.0f )
	{
		res = -pi + res;
	} // 3rd quadrant
	else if ( x >= 0.0f && y < 0.0f )
	{
	} // 4th quadrant

	return res;
}

// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b2Rot b2MakeRot(float angle)
{
	// return ( b2Rot ){ cosf( angle ), sinf( angle ) };

	float x = b2UnwindLargeAngle( angle );
	float pi2 = b2_pi * b2_pi;

	b2Rot q;

	// cosine needs angle in [-pi/2, pi/2]
	if (x < -0.5f * b2_pi)
	{
		float y = x + b2_pi;
		float y2 = y * y;
		q.c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else if (x > 0.5f * b2_pi)
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
	if (x < 0.0f)
	{
		float y = x + b2_pi;
		q.s = -16.0f * y * ( b2_pi - y ) / ( 5.0f * pi2 - 4.0f * y * ( b2_pi - y ) );
	}
	else
	{
		q.s = 16.0f * x * ( b2_pi - x ) / ( 5.0f * pi2 - 4.0f * x * ( b2_pi - x ) );
	}

	q = b2NormalizeRot( q );
	return q;
}
