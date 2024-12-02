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

	float tmin = -FLT_MAX;
	float tmax = FLT_MAX;

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
		if ( t1 > tmin )
		{
			normal.y = 0.0f;
			normal.x = s;
			tmin = t1;
		}

		// Pull the max down
		tmax = b2MinFloat( tmax, t2 );

		if ( tmin > tmax )
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
		if ( t1 > tmin )
		{
			normal.x = 0.0f;
			normal.y = s;
			tmin = t1;
		}

		// Pull the max down
		tmax = b2MinFloat( tmax, t2 );

		if ( tmin > tmax )
		{
			return output;
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ( tmin < 0.0f || 1.0f < tmin )
	{
		return output;
	}

	// Intersection.
	output.fraction = tmin;
	output.normal = normal;
	output.point = b2Lerp( p1, p2, tmin );
	output.hit = true;
	return output;
}

#if 0
bool b2TestOverlap(	const b2Shape* shapeA, int32_t indexA,
					const b2Shape* shapeB, int32_t indexB,
					b2Transform xfA, b2Transform xfB)
{
	b2DistanceInput input;
	input->proxyA.Set(shapeA, indexA);
	input->proxyB.Set(shapeB, indexB);
	input->transformA = xfA;
	input->transformB = xfB;
	input->useRadii = true;

	b2DistanceCache cache;
	cache.count = 0;

	b2DistanceOutput output;

	b2Distance(&output, &cache, &input);

	return output.distance < 10.0f * b2_epsilon;
}
#endif
