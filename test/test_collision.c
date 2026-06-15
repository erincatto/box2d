// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "shape.h"
#include "test_macros.h"

#include "box2d/collision.h"
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

// The narrow phase differences the two world positions in double then works in frame A, so a
// manifold far from the origin must match the same manifold at the origin. Float loses this past
// ~1e7 m where the ULP grows larger than the overlap, which is the whole point of large world mode.
static int LargeWorldManifoldTest( void )
{
	b2Polygon boxA = b2MakeBox( 0.5f, 0.5f );
	b2Polygon boxB = b2MakeBox( 0.5f, 0.5f );

	// Centers 0.9 apart so the boxes overlap by 0.1 along x
	b2Vec2 sep = { 0.9f, 0.0f };

	b2WorldTransform xfAo = b2WorldTransform_identity;
	b2WorldTransform xfBo = { b2OffsetPos( b2Pos_zero, sep ), b2Rot_identity };
	b2Manifold mOrigin = b2CollidePolygons( &boxA, xfAo, &boxB, xfBo );

	ENSURE( mOrigin.pointCount == 2 );
	ENSURE_SMALL( mOrigin.points[0].separation + 0.1f, 0.01f );
	ENSURE_SMALL( mOrigin.points[1].separation + 0.1f, 0.01f );

#if defined( BOX2D_DOUBLE_PRECISION )
	// Same relative configuration shifted far from the origin. In double the manifold is
	// preserved to float precision, in float it would collapse since the offset is below the ULP.
	b2Pos base = b2OffsetPos( b2Pos_zero, ( b2Vec2 ){ 1.0e7f, 1.0e7f } );
	b2WorldTransform xfAl = { base, b2Rot_identity };
	b2WorldTransform xfBl = { b2OffsetPos( base, sep ), b2Rot_identity };
	b2Manifold mLarge = b2CollidePolygons( &boxA, xfAl, &boxB, xfBl );

	ENSURE( mLarge.pointCount == mOrigin.pointCount );
	ENSURE_SMALL( mLarge.normal.x - mOrigin.normal.x, 1e-4f );
	ENSURE_SMALL( mLarge.normal.y - mOrigin.normal.y, 1e-4f );
	for ( int i = 0; i < mLarge.pointCount; ++i )
	{
		ENSURE_SMALL( mLarge.points[i].separation - mOrigin.points[i].separation, 1e-4f );
		ENSURE_SMALL( mLarge.points[i].anchorA.x - mOrigin.points[i].anchorA.x, 1e-4f );
		ENSURE_SMALL( mLarge.points[i].anchorA.y - mOrigin.points[i].anchorA.y, 1e-4f );
		ENSURE_SMALL( mLarge.points[i].anchorB.x - mOrigin.points[i].anchorB.x, 1e-4f );
		ENSURE_SMALL( mLarge.points[i].anchorB.y - mOrigin.points[i].anchorB.y, 1e-4f );
	}
#endif

	return 0;
}

// Broad-phase AABBs are built in double and narrowed to float with directed outward rounding, so a
// shape and its speculative margin stay inside their box far from the origin. A float build would
// round the extent away into the ULP (~1 m at 1e7) and clip the shape out of its own box.
static int LargeWorldAABBTest( void )
{
	// Rounded box: 0.5 half extents plus 0.1 radius, so the tight extent is 0.6 each way
	b2Polygon box = b2MakeRoundedBox( 0.5f, 0.5f, 0.1f );

	b2AABB aabbOrigin = b2ComputePolygonAABB( &box, b2WorldTransform_identity );
	ENSURE_SMALL( aabbOrigin.lowerBound.x + 0.6f, FLT_EPSILON );
	ENSURE_SMALL( aabbOrigin.lowerBound.y + 0.6f, FLT_EPSILON );
	ENSURE_SMALL( aabbOrigin.upperBound.x - 0.6f, FLT_EPSILON );
	ENSURE_SMALL( aabbOrigin.upperBound.y - 0.6f, FLT_EPSILON );

#if defined( BOX2D_DOUBLE_PRECISION )
	double d = 1.0e7;
	b2WorldTransform xfLarge = { { d, d }, b2Rot_identity };

	// Tight world AABB still contains the 0.6 m extent
	b2AABB tight = b2ComputePolygonAABB( &box, xfLarge );
	ENSURE( (double)tight.lowerBound.x <= d - 0.6 );
	ENSURE( (double)tight.lowerBound.y <= d - 0.6 );
	ENSURE( (double)tight.upperBound.x >= d + 0.6 );
	ENSURE( (double)tight.upperBound.y >= d + 0.6 );

	// The fat helper folds the extra into the double step before the single outward rounding, so a
	// margin smaller than a float ULP at this range survives instead of becoming a no-op subtract.
	float extra = 0.05f;
	b2Shape shape = { 0 };
	shape.type = b2_polygonShape;
	shape.polygon = box;
	b2AABB fat = b2ComputeFatShapeAABB( &shape, xfLarge, extra );
	ENSURE( (double)fat.lowerBound.x <= d - 0.6 - (double)extra );
	ENSURE( (double)fat.lowerBound.y <= d - 0.6 - (double)extra );
	ENSURE( (double)fat.upperBound.x >= d + 0.6 + (double)extra );
	ENSURE( (double)fat.upperBound.y >= d + 0.6 + (double)extra );
#endif

	return 0;
}

int CollisionTest( void )
{
	RUN_SUBTEST( AABBTest );
	RUN_SUBTEST( AABBRayCastTest );
	RUN_SUBTEST( LargeWorldManifoldTest );
	RUN_SUBTEST( LargeWorldAABBTest );

	return 0;
}
