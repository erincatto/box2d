// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"

#include "box2d/math_functions.h"

#include <float.h>

bool b2IsValidAABB( b2AABB a )
{
	b2Vec2 d = b2Sub( a.upperBound, a.lowerBound );
	bool valid = d.x >= 0.0f && d.y >= 0.0f;
	valid = valid && b2IsValidVec2( a.lowerBound ) && b2IsValidVec2( a.upperBound );
	return valid;
}

// From Real-time Collision Detection, p179.
b2CastOutput b2AABB_RayCast( b2AABB a, b2Vec2 p1, b2Vec2 p2 )
{
	// Radius not handled
	b2CastOutput output = { 0 };

	float tMin = -FLT_MAX;
	float tMax = FLT_MAX;

	b2Vec2 p = p1;
	b2Vec2 d = b2Sub( p2, p1 );
	b2Vec2 absD = b2Abs( d );

	b2Vec2 normal = b2Vec2_zero;

	// x-coordinate
	if ( absD.x < FLT_EPSILON )
	{
		// parallel
		if ( p.x < a.lowerBound.x || a.upperBound.x < p.x )
		{
			return output;
		}
	}
	else
	{
		float inv_d = 1.0f / d.x;
		float t1 = ( a.lowerBound.x - p.x ) * inv_d;
		float t2 = ( a.upperBound.x - p.x ) * inv_d;

		// Sign of the normal vector.
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		// Push the min up
		if ( t1 > tMin )
		{
			normal.y = 0.0f;
			normal.x = s;
			tMin = t1;
		}

		// Pull the max down
		tMax = b2MinFloat( tMax, t2 );

		if ( tMin > tMax )
		{
			return output;
		}
	}

	// y-coordinate
	if ( absD.y < FLT_EPSILON )
	{
		// parallel
		if ( p.y < a.lowerBound.y || a.upperBound.y < p.y )
		{
			return output;
		}
	}
	else
	{
		float inv_d = 1.0f / d.y;
		float t1 = ( a.lowerBound.y - p.y ) * inv_d;
		float t2 = ( a.upperBound.y - p.y ) * inv_d;

		// Sign of the normal vector.
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		// Push the min up
		if ( t1 > tMin )
		{
			normal.x = 0.0f;
			normal.y = s;
			tMin = t1;
		}

		// Pull the max down
		tMax = b2MinFloat( tMax, t2 );

		if ( tMin > tMax )
		{
			return output;
		}
	}

	// Does the ray start inside the box?
	if ( tMin < 0.0f )
	{
		return output;
	}

	// Does the ray intersect beyond the segment length?
	if ( 1.0f < tMin )
	{
		return output;
	}

	// Intersection.
	output.fraction = tMin;
	output.normal = normal;
	output.point = b2Lerp( p1, p2, tMin );
	output.hit = true;
	return output;
}
