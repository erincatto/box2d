// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>

static b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
static b2Circle circle = { { 1.0f, 0.0f }, 1.0f };
static b2Polygon box;
static b2Segment segment = { { 0.0f, 1.0f }, { 0.0f, -1.0f } };

#define N 4

static int ShapeMassTest( void )
{
	{
		b2MassData md = b2ComputeCircleMass( &circle, 1.0f );
		ENSURE_SMALL( md.mass - B2_PI, FLT_EPSILON );
		ENSURE( md.center.x == 1.0f && md.center.y == 0.0f );
		ENSURE_SMALL( md.rotationalInertia - 1.5f * B2_PI, FLT_EPSILON );
	}

	{
		float radius = capsule.radius;
		float length = b2Distance( capsule.center1, capsule.center2 );

		b2MassData md = b2ComputeCapsuleMass( &capsule, 1.0f );

		// Box that full contains capsule
		b2Polygon r = b2MakeBox( radius, radius + 0.5f * length );
		b2MassData mdr = b2ComputePolygonMass( &r, 1.0f );

		// Approximate capsule using convex hull
		b2Vec2 points[2 * N];
		float d = B2_PI / ( N - 1.0f );
		float angle = -0.5f * B2_PI;
		for ( int i = 0; i < N; ++i )
		{
			points[i].x = 1.0f + radius * cosf( angle );
			points[i].y = radius * sinf( angle );
			angle += d;
		}

		angle = 0.5f * B2_PI;
		for ( int i = N; i < 2 * N; ++i )
		{
			points[i].x = -1.0f + radius * cosf( angle );
			points[i].y = radius * sinf( angle );
			angle += d;
		}

		b2Hull hull = b2ComputeHull( points, 2 * N );
		b2Polygon ac = b2MakePolygon( &hull, 0.0f );
		b2MassData ma = b2ComputePolygonMass( &ac, 1.0f );

		ENSURE( ma.mass < md.mass && md.mass < mdr.mass );
		ENSURE( ma.rotationalInertia < md.rotationalInertia && md.rotationalInertia < mdr.rotationalInertia );
	}

	{
		b2MassData md = b2ComputePolygonMass( &box, 1.0f );
		ENSURE_SMALL( md.mass - 4.0f, FLT_EPSILON );
		ENSURE_SMALL( md.center.x, FLT_EPSILON );
		ENSURE_SMALL( md.center.y, FLT_EPSILON );
		ENSURE_SMALL( md.rotationalInertia - 8.0f / 3.0f, 2.0f * FLT_EPSILON );
	}

	return 0;
}

static int ShapeAABBTest( void )
{
	{
		b2AABB b = b2ComputeCircleAABB( &circle, b2Transform_identity );
		ENSURE_SMALL( b.lowerBound.x, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.x - 2.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
	}

	{
		b2AABB b = b2ComputePolygonAABB( &box, b2Transform_identity );
		ENSURE_SMALL( b.lowerBound.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
	}

	{
		b2AABB b = b2ComputeSegmentAABB( &segment, b2Transform_identity );
		ENSURE_SMALL( b.lowerBound.x, FLT_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.x, FLT_EPSILON );
		ENSURE_SMALL( b.upperBound.y - 1.0f, FLT_EPSILON );
	}

	return 0;
}

static int PointInShapeTest( void )
{
	b2Vec2 p1 = { 0.5f, 0.5f };
	b2Vec2 p2 = { 4.0f, -4.0f };

	{
		bool hit;
		hit = b2PointInCircle( p1, &circle );
		ENSURE( hit == true );
		hit = b2PointInCircle( p2, &circle );
		ENSURE( hit == false );
	}

	{
		bool hit;
		hit = b2PointInPolygon( p1, &box );
		ENSURE( hit == true );
		hit = b2PointInPolygon( p2, &box );
		ENSURE( hit == false );
	}

	return 0;
}

static int RayCastShapeTest( void )
{
	b2RayCastInput input = { { -4.0f, 0.0f }, { 8.0f, 0.0f }, 1.0f };

	{
		b2CastOutput output = b2RayCastCircle( &input, &circle );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 0.5f, FLT_EPSILON );
	}

	{
		b2CastOutput output = b2RayCastPolygon( &input, &box );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 3.0f / 8.0f, FLT_EPSILON );
	}

	{
		b2CastOutput output = b2RayCastSegment( &input, &segment, true );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.fraction - 0.5f, FLT_EPSILON );
	}

	return 0;
}

int ShapeTest( void )
{
	box = b2MakeBox( 1.0f, 1.0f );

	RUN_SUBTEST( ShapeMassTest );
	RUN_SUBTEST( ShapeAABBTest );
	RUN_SUBTEST( PointInShapeTest );
	RUN_SUBTEST( RayCastShapeTest );

	return 0;
}
