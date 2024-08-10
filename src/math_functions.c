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

b2Vec2 b2Normalize( b2Vec2 v )
{
	float length = b2Length( v );
	if ( length < FLT_EPSILON )
	{
		return b2Vec2_zero;
	}

	float invLength = 1.0f / length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

b2Vec2 b2NormalizeChecked( b2Vec2 v )
{
	float length = b2Length( v );
	if ( length < FLT_EPSILON )
	{
		B2_ASSERT( false );
		return b2Vec2_zero;
	}

	float invLength = 1.0f / length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

b2Vec2 b2GetLengthAndNormalize( float* length, b2Vec2 v )
{
	*length = b2Length( v );
	if ( *length < FLT_EPSILON )
	{
		return b2Vec2_zero;
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}
