// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "test_macros.h"

#include "box2d/math_functions.h"

static int AABBTest( void )
{
	b2AABB a;
	a.lowerBound = (b2Vec2){ -1.0f, -1.0f };
	a.upperBound = (b2Vec2){ -2.0f, -2.0f };

	ENSURE( b2IsValidAABB( a ) == false );

	a.upperBound = (b2Vec2){ 1.0f, 1.0f };
	ENSURE( b2IsValidAABB( a ) == true );

	b2AABB b = { { 2.0f, 2.0f }, { 4.0f, 4.0f } };
	ENSURE( b2AABB_Overlaps( a, b ) == false );
	ENSURE( b2AABB_Contains( a, b ) == false );

	return 0;
}

static int AABBRayCastTest( void )
{
	// Test AABB centered at origin with bounds [-1, -1] to [1, 1]
	b2AABB aabb = { { -1.0f, -1.0f }, { 1.0f, 1.0f } };

	// Test 1: Ray hits AABB from left side
	{
		b2Vec2 p1 = { -3.0f, 0.0f };
		b2Vec2 p2 = { 3.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.point.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y, FLT_EPSILON );
	}

	// Test 2: Ray hits AABB from right side
	{
		b2Vec2 p1 = { 3.0f, 0.0f };
		b2Vec2 p2 = { -3.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.point.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y, FLT_EPSILON );
	}

	// Test 3: Ray hits AABB from bottom
	{
		b2Vec2 p1 = { 0.0f, -3.0f };
		b2Vec2 p2 = { 0.0f, 3.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.x, FLT_EPSILON );
		ENSURE_SMALL( output.point.y + 1.0f, FLT_EPSILON );
	}

	// Test 4: Ray hits AABB from top
	{
		b2Vec2 p1 = { 0.0f, 3.0f };
		b2Vec2 p2 = { 0.0f, -3.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.x, FLT_EPSILON );
		ENSURE_SMALL( output.point.y - 1.0f, FLT_EPSILON );
	}

	// Test 5: Ray misses AABB completely (parallel to x-axis)
	{
		b2Vec2 p1 = { -3.0f, 2.0f };
		b2Vec2 p2 = { 3.0f, 2.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == false );
	}

	// Test 6: Ray misses AABB completely (parallel to y-axis)
	{
		b2Vec2 p1 = { 2.0f, -3.0f };
		b2Vec2 p2 = { 2.0f, 3.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == false );
	}

	// Test 7: Ray starts inside AABB
	{
		b2Vec2 p1 = { 0.0f, 0.0f };
		b2Vec2 p2 = { 2.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == false );
	}

	// Test 8: Ray hits corner of AABB (diagonal ray)
	{
		b2Vec2 p1 = { -2.0f, -2.0f };
		b2Vec2 p2 = { 2.0f, 2.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 0.25f, FLT_EPSILON );
		// Normal should be either (-1, 0) or (0, -1) depending on which edge is hit first
		ENSURE( ( output.normal.x == -1.0f && output.normal.y == 0.0f ) ||
				( output.normal.x == 0.0f && output.normal.y == -1.0f ) );
	}

	// Test 9: Ray parallel to AABB edge but outside
	{
		b2Vec2 p1 = { -2.0f, 1.5f };
		b2Vec2 p2 = { 2.0f, 1.5f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == false );
	}

	// Test 10: Ray parallel to AABB edge and exactly on boundary
	{
		b2Vec2 p1 = { -2.0f, 1.0f };
		b2Vec2 p2 = { 2.0f, 1.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 0.25f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
	}

	// Test 11: Very short ray that doesn't reach AABB
	{
		b2Vec2 p1 = { -3.0f, 0.0f };
		b2Vec2 p2 = { -2.5f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == false );
	}

	// Test 12: Zero-length ray (degenerate case)
	{
		b2Vec2 p1 = { 0.0f, 0.0f };
		b2Vec2 p2 = { 0.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == false );
	}

	// Test 13: Ray hits AABB at exact boundary condition (t = 1.0)
	{
		b2Vec2 p1 = { -2.0f, 0.0f };
		b2Vec2 p2 = { -1.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
	}

	// Test 14: Different AABB position (not centered at origin)
	{
		b2AABB offsetAABB = { { 2.0f, 3.0f }, { 4.0f, 5.0f } };
		b2Vec2 p1 = { 0.0f, 4.0f };
		b2Vec2 p2 = { 6.0f, 4.0f };
		b2CastOutput output = b2AABB_RayCast( offsetAABB, p1, p2 );

		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.point.x - 2.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y - 4.0f, FLT_EPSILON );
	}

	return 0;
}

int CollisionTest( void )
{
	RUN_SUBTEST( AABBTest );
	RUN_SUBTEST( AABBRayCastTest );

	return 0;
}
