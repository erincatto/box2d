// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

// Ray cast an AABB
b2CastOutput b2AABB_RayCast( b2AABB a, b2Vec2 p1, b2Vec2 p2 );

// Get surface area of an AABB (the perimeter length)
static inline float b2Perimeter( b2AABB a )
{
	float wx = a.upperBound.x - a.lowerBound.x;
	float wy = a.upperBound.y - a.lowerBound.y;
	return 2.0f * ( wx + wy );
}

/// Enlarge a to contain b
/// @return true if the AABB grew
static inline bool b2EnlargeAABB( b2AABB* a, b2AABB b )
{
	bool changed = false;
	if ( b.lowerBound.x < a->lowerBound.x )
	{
		a->lowerBound.x = b.lowerBound.x;
		changed = true;
	}

	if ( b.lowerBound.y < a->lowerBound.y )
	{
		a->lowerBound.y = b.lowerBound.y;
		changed = true;
	}

	if ( a->upperBound.x < b.upperBound.x )
	{
		a->upperBound.x = b.upperBound.x;
		changed = true;
	}

	if ( a->upperBound.y < b.upperBound.y )
	{
		a->upperBound.y = b.upperBound.y;
		changed = true;
	}

	return changed;
}

// Translate a relative AABB into world space, rounding outward so the float box always contains
// the true box far from the origin where a float coordinate cannot resolve the shape extent.
// Float ULP at 1e8 dwarfs the AABB margin, plain truncation could clip a shape out of its own
// box. The broadphase pair order rides on the deterministic directed rounding.
static inline b2AABB b2OffsetAABB( b2AABB box, b2Pos origin )
{
	b2AABB result;
	result.lowerBound.x = b2RoundDownFloat( origin.x + (double)box.lowerBound.x );
	result.lowerBound.y = b2RoundDownFloat( origin.y + (double)box.lowerBound.y );
	result.upperBound.x = b2RoundUpFloat( origin.x + (double)box.upperBound.x );
	result.upperBound.y = b2RoundUpFloat( origin.y + (double)box.upperBound.y );
	return result;
}
