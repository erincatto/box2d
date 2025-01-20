// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constants.h"
#include "core.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <stddef.h>

#define B2_MAKE_ID( A, B ) ( (uint8_t)( A ) << 8 | (uint8_t)( B ) )

static b2Polygon b2MakeCapsule( b2Vec2 p1, b2Vec2 p2, float radius )
{
	b2Polygon shape = { 0 };
	shape.vertices[0] = p1;
	shape.vertices[1] = p2;
	shape.centroid = b2Lerp( p1, p2, 0.5f );

	b2Vec2 d = b2Sub( p2, p1 );
	B2_ASSERT( b2LengthSquared( d ) > FLT_EPSILON );
	b2Vec2 axis = b2Normalize( d );
	b2Vec2 normal = b2RightPerp( axis );

	shape.normals[0] = normal;
	shape.normals[1] = b2Neg( normal );
	shape.count = 2;
	shape.radius = radius;

	return shape;
}

// point = qA * localAnchorA + pA
// localAnchorB = qBc * (point - pB)
// anchorB = point - pB = qA * localAnchorA + pA - pB
//         = anchorA + (pA - pB)
b2Manifold b2CollideCircles( const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB )
{
	b2Manifold manifold = { 0 };

	b2Transform xf = b2InvMulTransforms( xfA, xfB );

	b2Vec2 pointA = circleA->center;
	b2Vec2 pointB = b2TransformPoint( xf, circleB->center );

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize( &distance, b2Sub( pointB, pointA ) );

	float radiusA = circleA->radius;
	float radiusB = circleB->radius;

	float separation = distance - radiusA - radiusB;
	if ( separation > B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd( pointA, radiusA, normal );
	b2Vec2 cB = b2MulAdd( pointB, -radiusB, normal );
	b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

	manifold.normal = b2RotateVector( xfA.q, normal );
	b2ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b2RotateVector( xfA.q, contactPointA );
	mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
	mp->point = b2Add( mp->anchorA, xfA.p );
	mp->separation = separation;
	mp->id = 0;
	manifold.pointCount = 1;
	return manifold;
}

/// Compute the collision manifold between a capsule and circle
b2Manifold b2CollideCapsuleAndCircle( const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB )
{
	b2Manifold manifold = { 0 };

	b2Transform xf = b2InvMulTransforms( xfA, xfB );

	// Compute circle position in the frame of the capsule.
	b2Vec2 pB = b2TransformPoint( xf, circleB->center );

	// Compute closest point
	b2Vec2 p1 = capsuleA->center1;
	b2Vec2 p2 = capsuleA->center2;

	b2Vec2 e = b2Sub( p2, p1 );

	// dot(p - pA, e) = 0
	// dot(p - (p1 + s1 * e), e) = 0
	// s1 = dot(p - p1, e)
	b2Vec2 pA;
	float s1 = b2Dot( b2Sub( pB, p1 ), e );
	float s2 = b2Dot( b2Sub( p2, pB ), e );
	if ( s1 < 0.0f )
	{
		// p1 region
		pA = p1;
	}
	else if ( s2 < 0.0f )
	{
		// p2 region
		pA = p2;
	}
	else
	{
		// circle colliding with segment interior
		float s = s1 / b2Dot( e, e );
		pA = b2MulAdd( p1, s, e );
	}

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize( &distance, b2Sub( pB, pA ) );

	float radiusA = capsuleA->radius;
	float radiusB = circleB->radius;
	float separation = distance - radiusA - radiusB;
	if ( separation > B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd( pA, radiusA, normal );
	b2Vec2 cB = b2MulAdd( pB, -radiusB, normal );
	b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

	manifold.normal = b2RotateVector( xfA.q, normal );
	b2ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b2RotateVector( xfA.q, contactPointA );
	mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
	mp->point = b2Add( xfA.p, mp->anchorA );
	mp->separation = separation;
	mp->id = 0;
	manifold.pointCount = 1;
	return manifold;
}

b2Manifold b2CollidePolygonAndCircle( const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB )
{
	b2Manifold manifold = { 0 };
	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;

	b2Transform xf = b2InvMulTransforms( xfA, xfB );

	// Compute circle position in the frame of the polygon.
	b2Vec2 center = b2TransformPoint( xf, circleB->center );
	float radiusA = polygonA->radius;
	float radiusB = circleB->radius;
	float radius = radiusA + radiusB;

	// Find the min separating edge.
	int normalIndex = 0;
	float separation = -FLT_MAX;
	int vertexCount = polygonA->count;
	const b2Vec2* vertices = polygonA->vertices;
	const b2Vec2* normals = polygonA->normals;

	for ( int i = 0; i < vertexCount; ++i )
	{
		float s = b2Dot( normals[i], b2Sub( center, vertices[i] ) );
		if ( s > separation )
		{
			separation = s;
			normalIndex = i;
		}
	}

	if ( separation > radius + speculativeDistance )
	{
		return manifold;
	}

	// Vertices of the reference edge.
	int vertIndex1 = normalIndex;
	int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	b2Vec2 v1 = vertices[vertIndex1];
	b2Vec2 v2 = vertices[vertIndex2];

	// Compute barycentric coordinates
	float u1 = b2Dot( b2Sub( center, v1 ), b2Sub( v2, v1 ) );
	float u2 = b2Dot( b2Sub( center, v2 ), b2Sub( v1, v2 ) );

	if ( u1 < 0.0f && separation > FLT_EPSILON )
	{
		// Circle center is closest to v1 and safely outside the polygon
		b2Vec2 normal = b2Normalize( b2Sub( center, v1 ) );
		separation = b2Dot( b2Sub( center, v1 ), normal );
		if ( separation > radius + speculativeDistance )
		{
			return manifold;
		}

		b2Vec2 cA = b2MulAdd( v1, radiusA, normal );
		b2Vec2 cB = b2MulSub( center, radiusB, normal );
		b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

		manifold.normal = b2RotateVector( xfA.q, normal );
		b2ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b2RotateVector( xfA.q, contactPointA );
		mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		mp->point = b2Add( xfA.p, mp->anchorA );
		mp->separation = b2Dot( b2Sub( cB, cA ), normal );
		mp->id = 0;
		manifold.pointCount = 1;
	}
	else if ( u2 < 0.0f && separation > FLT_EPSILON )
	{
		// Circle center is closest to v2 and safely outside the polygon
		b2Vec2 normal = b2Normalize( b2Sub( center, v2 ) );
		separation = b2Dot( b2Sub( center, v2 ), normal );
		if ( separation > radius + speculativeDistance )
		{
			return manifold;
		}

		b2Vec2 cA = b2MulAdd( v2, radiusA, normal );
		b2Vec2 cB = b2MulSub( center, radiusB, normal );
		b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

		manifold.normal = b2RotateVector( xfA.q, normal );
		b2ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b2RotateVector( xfA.q, contactPointA );
		mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		mp->point = b2Add( xfA.p, mp->anchorA );
		mp->separation = b2Dot( b2Sub( cB, cA ), normal );
		mp->id = 0;
		manifold.pointCount = 1;
	}
	else
	{
		// Circle center is between v1 and v2. Center may be inside polygon
		b2Vec2 normal = normals[normalIndex];
		manifold.normal = b2RotateVector( xfA.q, normal );

		// cA is the projection of the circle center onto to the reference edge
		b2Vec2 cA = b2MulAdd( center, radiusA - b2Dot( b2Sub( center, v1 ), normal ), normal );

		// cB is the deepest point on the circle with respect to the reference edge
		b2Vec2 cB = b2MulSub( center, radiusB, normal );

		b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

		// The contact point is the midpoint in world space
		b2ManifoldPoint* mp = manifold.points + 0;
		mp->anchorA = b2RotateVector( xfA.q, contactPointA );
		mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		mp->point = b2Add( xfA.p, mp->anchorA );
		mp->separation = separation - radius;
		mp->id = 0;
		manifold.pointCount = 1;
	}

	return manifold;
}

// Follows Ericson 5.1.9 Closest Points of Two Line Segments
// Adds some logic to support clipping to get two contact points
b2Manifold b2CollideCapsules( const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB )
{
	b2Vec2 origin = capsuleA->center1;

	// Shift polyA to origin
	// pw = q * pb + p
	// pw = q * (pbs + origin) + p
	// pw = q * pbs + (p + q * origin)
	b2Transform sfA = { b2Add( xfA.p, b2RotateVector( xfA.q, origin ) ), xfA.q };
	b2Transform xf = b2InvMulTransforms( sfA, xfB );

	b2Vec2 p1 = b2Vec2_zero;
	b2Vec2 q1 = b2Sub( capsuleA->center2, origin );

	b2Vec2 p2 = b2TransformPoint( xf, capsuleB->center1 );
	b2Vec2 q2 = b2TransformPoint( xf, capsuleB->center2 );

	b2Vec2 d1 = b2Sub( q1, p1 );
	b2Vec2 d2 = b2Sub( q2, p2 );

	float dd1 = b2Dot( d1, d1 );
	float dd2 = b2Dot( d2, d2 );

	const float epsSqr = FLT_EPSILON * FLT_EPSILON;
	B2_ASSERT( dd1 > epsSqr && dd2 > epsSqr );

	b2Vec2 r = b2Sub( p1, p2 );
	float rd1 = b2Dot( r, d1 );
	float rd2 = b2Dot( r, d2 );

	float d12 = b2Dot( d1, d2 );

	float denom = dd1 * dd2 - d12 * d12;

	// Fraction on segment 1
	float f1 = 0.0f;
	if ( denom != 0.0f )
	{
		// not parallel
		f1 = b2ClampFloat( ( d12 * rd2 - rd1 * dd2 ) / denom, 0.0f, 1.0f );
	}

	// Compute point on segment 2 closest to p1 + f1 * d1
	float f2 = ( d12 * f1 + rd2 ) / dd2;

	// Clamping of segment 2 requires a do over on segment 1
	if ( f2 < 0.0f )
	{
		f2 = 0.0f;
		f1 = b2ClampFloat( -rd1 / dd1, 0.0f, 1.0f );
	}
	else if ( f2 > 1.0f )
	{
		f2 = 1.0f;
		f1 = b2ClampFloat( ( d12 - rd1 ) / dd1, 0.0f, 1.0f );
	}

	b2Vec2 closest1 = b2MulAdd( p1, f1, d1 );
	b2Vec2 closest2 = b2MulAdd( p2, f2, d2 );
	float distanceSquared = b2DistanceSquared( closest1, closest2 );

	b2Manifold manifold = { 0 };
	float radiusA = capsuleA->radius;
	float radiusB = capsuleB->radius;
	float radius = radiusA + radiusB;
	float maxDistance = radius + B2_SPECULATIVE_DISTANCE;

	if ( distanceSquared > maxDistance * maxDistance )
	{
		return manifold;
	}

	float distance = sqrtf( distanceSquared );

	float length1, length2;
	b2Vec2 u1 = b2GetLengthAndNormalize( &length1, d1 );
	b2Vec2 u2 = b2GetLengthAndNormalize( &length2, d2 );

	// Does segment B project outside segment A?
	float fp2 = b2Dot( b2Sub( p2, p1 ), u1 );
	float fq2 = b2Dot( b2Sub( q2, p1 ), u1 );
	bool outsideA = ( fp2 <= 0.0f && fq2 <= 0.0f ) || ( fp2 >= length1 && fq2 >= length1 );

	// Does segment A project outside segment B?
	float fp1 = b2Dot( b2Sub( p1, p2 ), u2 );
	float fq1 = b2Dot( b2Sub( q1, p2 ), u2 );
	bool outsideB = ( fp1 <= 0.0f && fq1 <= 0.0f ) || ( fp1 >= length2 && fq1 >= length2 );

	if ( outsideA == false && outsideB == false )
	{
		// attempt to clip
		// this may yield contact points with excessive separation
		// in that case the algorithm falls back to single point collision

		// find reference edge using SAT
		b2Vec2 normalA;
		float separationA;

		{
			normalA = b2LeftPerp( u1 );
			float ss1 = b2Dot( b2Sub( p2, p1 ), normalA );
			float ss2 = b2Dot( b2Sub( q2, p1 ), normalA );
			float s1p = ss1 < ss2 ? ss1 : ss2;
			float s1n = -ss1 < -ss2 ? -ss1 : -ss2;

			if ( s1p > s1n )
			{
				separationA = s1p;
			}
			else
			{
				separationA = s1n;
				normalA = b2Neg( normalA );
			}
		}

		b2Vec2 normalB;
		float separationB;
		{
			normalB = b2LeftPerp( u2 );
			float ss1 = b2Dot( b2Sub( p1, p2 ), normalB );
			float ss2 = b2Dot( b2Sub( q1, p2 ), normalB );
			float s1p = ss1 < ss2 ? ss1 : ss2;
			float s1n = -ss1 < -ss2 ? -ss1 : -ss2;

			if ( s1p > s1n )
			{
				separationB = s1p;
			}
			else
			{
				separationB = s1n;
				normalB = b2Neg( normalB );
			}
		}

		if ( separationA >= separationB )
		{
			manifold.normal = normalA;

			b2Vec2 cp = p2;
			b2Vec2 cq = q2;

			// clip to p1
			if ( fp2 < 0.0f && fq2 > 0.0f )
			{
				cp = b2Lerp( p2, q2, ( 0.0f - fp2 ) / ( fq2 - fp2 ) );
			}
			else if ( fq2 < 0.0f && fp2 > 0.0f )
			{
				cq = b2Lerp( q2, p2, ( 0.0f - fq2 ) / ( fp2 - fq2 ) );
			}

			// clip to q1
			if ( fp2 > length1 && fq2 < length1 )
			{
				cp = b2Lerp( p2, q2, ( fp2 - length1 ) / ( fp2 - fq2 ) );
			}
			else if ( fq2 > length1 && fp2 < length1 )
			{
				cq = b2Lerp( q2, p2, ( fq2 - length1 ) / ( fq2 - fp2 ) );
			}

			float sp = b2Dot( b2Sub( cp, p1 ), normalA );
			float sq = b2Dot( b2Sub( cq, p1 ), normalA );

			if ( sp <= distance + B2_LINEAR_SLOP || sq <= distance + B2_LINEAR_SLOP )
			{
				b2ManifoldPoint* mp;
				mp = manifold.points + 0;
				mp->anchorA = b2MulAdd( cp, 0.5f * ( radiusA - radiusB - sp ), normalA );
				mp->separation = sp - radius;
				mp->id = B2_MAKE_ID( 0, 0 );

				mp = manifold.points + 1;
				mp->anchorA = b2MulAdd( cq, 0.5f * ( radiusA - radiusB - sq ), normalA );
				mp->separation = sq - radius;
				mp->id = B2_MAKE_ID( 0, 1 );
				manifold.pointCount = 2;
			}
		}
		else
		{
			// normal always points from A to B
			manifold.normal = b2Neg( normalB );

			b2Vec2 cp = p1;
			b2Vec2 cq = q1;

			// clip to p2
			if ( fp1 < 0.0f && fq1 > 0.0f )
			{
				cp = b2Lerp( p1, q1, ( 0.0f - fp1 ) / ( fq1 - fp1 ) );
			}
			else if ( fq1 < 0.0f && fp1 > 0.0f )
			{
				cq = b2Lerp( q1, p1, ( 0.0f - fq1 ) / ( fp1 - fq1 ) );
			}

			// clip to q2
			if ( fp1 > length2 && fq1 < length2 )
			{
				cp = b2Lerp( p1, q1, ( fp1 - length2 ) / ( fp1 - fq1 ) );
			}
			else if ( fq1 > length2 && fp1 < length2 )
			{
				cq = b2Lerp( q1, p1, ( fq1 - length2 ) / ( fq1 - fp1 ) );
			}

			float sp = b2Dot( b2Sub( cp, p2 ), normalB );
			float sq = b2Dot( b2Sub( cq, p2 ), normalB );

			if ( sp <= distance + B2_LINEAR_SLOP || sq <= distance + B2_LINEAR_SLOP )
			{
				b2ManifoldPoint* mp;
				mp = manifold.points + 0;
				mp->anchorA = b2MulAdd( cp, 0.5f * ( radiusB - radiusA - sp ), normalB );
				mp->separation = sp - radius;
				mp->id = B2_MAKE_ID( 0, 0 );
				mp = manifold.points + 1;
				mp->anchorA = b2MulAdd( cq, 0.5f * ( radiusB - radiusA - sq ), normalB );
				mp->separation = sq - radius;
				mp->id = B2_MAKE_ID( 1, 0 );
				manifold.pointCount = 2;
			}
		}
	}

	if ( manifold.pointCount == 0 )
	{
		// single point collision
		b2Vec2 normal = b2Sub( closest2, closest1 );
		if ( b2Dot( normal, normal ) > epsSqr )
		{
			normal = b2Normalize( normal );
		}
		else
		{
			normal = b2LeftPerp( u1 );
		}

		b2Vec2 c1 = b2MulAdd( closest1, radiusA, normal );
		b2Vec2 c2 = b2MulAdd( closest2, -radiusB, normal );

		int i1 = f1 == 0.0f ? 0 : 1;
		int i2 = f2 == 0.0f ? 0 : 1;

		manifold.normal = normal;
		manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
		manifold.points[0].separation = sqrtf( distanceSquared ) - radius;
		manifold.points[0].id = B2_MAKE_ID( i1, i2 );
		manifold.pointCount = 1;
	}

	// Convert manifold to world space
	manifold.normal = b2RotateVector( xfA.q, manifold.normal );
	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		b2ManifoldPoint* mp = manifold.points + i;

		// anchor points relative to shape origin in world space
		mp->anchorA = b2RotateVector( xfA.q, b2Add( mp->anchorA, origin ) );
		mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		mp->point = b2Add( xfA.p, mp->anchorA );
	}

	return manifold;
}

b2Manifold b2CollideSegmentAndCapsule( const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB )
{
	b2Capsule capsuleA = { segmentA->point1, segmentA->point2, 0.0f };
	return b2CollideCapsules( &capsuleA, xfA, capsuleB, xfB );
}

b2Manifold b2CollidePolygonAndCapsule( const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB )
{
	b2Polygon polyB = b2MakeCapsule( capsuleB->center1, capsuleB->center2, capsuleB->radius );
	return b2CollidePolygons( polygonA, xfA, &polyB, xfB );
}

// Polygon clipper used to compute contact points when there are potentially two contact points.
static b2Manifold b2ClipPolygons( const b2Polygon* polyA, const b2Polygon* polyB, int edgeA, int edgeB, bool flip )
{
	b2Manifold manifold = { 0 };

	// reference polygon
	const b2Polygon* poly1;
	int i11, i12;

	// incident polygon
	const b2Polygon* poly2;
	int i21, i22;

	if ( flip )
	{
		poly1 = polyB;
		poly2 = polyA;
		i11 = edgeB;
		i12 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
		i21 = edgeA;
		i22 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		i11 = edgeA;
		i12 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
		i21 = edgeB;
		i22 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
	}

	b2Vec2 normal = poly1->normals[i11];

	// Reference edge vertices
	b2Vec2 v11 = poly1->vertices[i11];
	b2Vec2 v12 = poly1->vertices[i12];

	// Incident edge vertices
	b2Vec2 v21 = poly2->vertices[i21];
	b2Vec2 v22 = poly2->vertices[i22];

	b2Vec2 tangent = b2CrossSV( 1.0f, normal );

	float lower1 = 0.0f;
	float upper1 = b2Dot( b2Sub( v12, v11 ), tangent );

	// Incident edge points opposite of tangent due to CCW winding
	float upper2 = b2Dot( b2Sub( v21, v11 ), tangent );
	float lower2 = b2Dot( b2Sub( v22, v11 ), tangent );

	// This check can fail slightly due to mismatch with GJK code.
	// Perhaps fall back to a single point here? Otherwise we get two coincident points.
	// if (upper2 < lower1 || upper1 < lower2)
	//{
	//	// numeric failure
	//	B2_ASSERT(false);
	//	return manifold;
	//}

	b2Vec2 vLower;
	if ( lower2 < lower1 && upper2 - lower2 > FLT_EPSILON )
	{
		vLower = b2Lerp( v22, v21, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vLower = v22;
	}

	b2Vec2 vUpper;
	if ( upper2 > upper1 && upper2 - lower2 > FLT_EPSILON )
	{
		vUpper = b2Lerp( v22, v21, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vUpper = v21;
	}

	// todo vLower can be very close to vUpper, reduce to one point?

	float separationLower = b2Dot( b2Sub( vLower, v11 ), normal );
	float separationUpper = b2Dot( b2Sub( vUpper, v11 ), normal );

	float r1 = poly1->radius;
	float r2 = poly2->radius;

	// Put contact points at midpoint, accounting for radii
	vLower = b2MulAdd( vLower, 0.5f * ( r1 - r2 - separationLower ), normal );
	vUpper = b2MulAdd( vUpper, 0.5f * ( r1 - r2 - separationUpper ), normal );

	float radius = r1 + r2;

	if ( flip == false )
	{
		manifold.normal = normal;
		b2ManifoldPoint* cp = manifold.points + 0;

		{
			cp->anchorA = vLower;
			cp->separation = separationLower - radius;
			cp->id = B2_MAKE_ID( i11, i22 );
			manifold.pointCount += 1;
			cp += 1;
		}

		{
			cp->anchorA = vUpper;
			cp->separation = separationUpper - radius;
			cp->id = B2_MAKE_ID( i12, i21 );
			manifold.pointCount += 1;
		}
	}
	else
	{
		manifold.normal = b2Neg( normal );
		b2ManifoldPoint* cp = manifold.points + 0;

		{
			cp->anchorA = vUpper;
			cp->separation = separationUpper - radius;
			cp->id = B2_MAKE_ID( i21, i12 );
			manifold.pointCount += 1;
			cp += 1;
		}

		{
			cp->anchorA = vLower;
			cp->separation = separationLower - radius;
			cp->id = B2_MAKE_ID( i22, i11 );
			manifold.pointCount += 1;
		}
	}

	return manifold;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static float b2FindMaxSeparation( int* edgeIndex, const b2Polygon* poly1, const b2Polygon* poly2 )
{
	int count1 = poly1->count;
	int count2 = poly2->count;
	const b2Vec2* n1s = poly1->normals;
	const b2Vec2* v1s = poly1->vertices;
	const b2Vec2* v2s = poly2->vertices;

	int bestIndex = 0;
	float maxSeparation = -FLT_MAX;
	for ( int i = 0; i < count1; ++i )
	{
		// Get poly1 normal in frame2.
		b2Vec2 n = n1s[i];
		b2Vec2 v1 = v1s[i];

		// Find the deepest point for normal i.
		float si = FLT_MAX;
		for ( int j = 0; j < count2; ++j )
		{
			float sij = b2Dot( n, b2Sub( v2s[j], v1 ) );
			if ( sij < si )
			{
				si = sij;
			}
		}

		if ( si > maxSeparation )
		{
			maxSeparation = si;
			bestIndex = i;
		}
	}

	*edgeIndex = bestIndex;
	return maxSeparation;
}

// Due to speculation, every polygon is rounded
// Algorithm:
//
// compute edge separation using the separating axis test (SAT)
// if (separation > speculation_distance)
//   return
// find reference and incident edge
// if separation >= 0.1f * B2_LINEAR_SLOP
//   compute closest points between reference and incident edge
//   if vertices are closest
//      single vertex-vertex contact
//   else
//      clip edges
//   end
// else
//   clip edges
// end

b2Manifold b2CollidePolygons( const b2Polygon* polygonA, b2Transform xfA, const b2Polygon* polygonB, b2Transform xfB )
{
	b2Vec2 origin = polygonA->vertices[0];

	// Shift polyA to origin
	// pw = q * pb + p
	// pw = q * (pbs + origin) + p
	// pw = q * pbs + (p + q * origin)
	b2Transform sfA = { b2Add( xfA.p, b2RotateVector( xfA.q, origin ) ), xfA.q };
	b2Transform xf = b2InvMulTransforms( sfA, xfB );

	b2Polygon localPolyA;
	localPolyA.count = polygonA->count;
	localPolyA.radius = polygonA->radius;
	localPolyA.vertices[0] = b2Vec2_zero;
	localPolyA.normals[0] = polygonA->normals[0];
	for ( int i = 1; i < localPolyA.count; ++i )
	{
		localPolyA.vertices[i] = b2Sub( polygonA->vertices[i], origin );
		localPolyA.normals[i] = polygonA->normals[i];
	}

	// Put polyB in polyA's frame to reduce round-off error
	b2Polygon localPolyB;
	localPolyB.count = polygonB->count;
	localPolyB.radius = polygonB->radius;
	for ( int i = 0; i < localPolyB.count; ++i )
	{
		localPolyB.vertices[i] = b2TransformPoint( xf, polygonB->vertices[i] );
		localPolyB.normals[i] = b2RotateVector( xf.q, polygonB->normals[i] );
	}

	int edgeA = 0;
	float separationA = b2FindMaxSeparation( &edgeA, &localPolyA, &localPolyB );

	int edgeB = 0;
	float separationB = b2FindMaxSeparation( &edgeB, &localPolyB, &localPolyA );

	float radius = localPolyA.radius + localPolyB.radius;

	if ( separationA > B2_SPECULATIVE_DISTANCE + radius || separationB > B2_SPECULATIVE_DISTANCE + radius )
	{
		return ( b2Manifold ){ 0 };
	}

	// Find incident edge
	bool flip;
	if ( separationA >= separationB )
	{
		flip = false;

		b2Vec2 searchDirection = localPolyA.normals[edgeA];

		// Find the incident edge on polyB
		int count = localPolyB.count;
		const b2Vec2* normals = localPolyB.normals;
		edgeB = 0;
		float minDot = FLT_MAX;
		for ( int i = 0; i < count; ++i )
		{
			float dot = b2Dot( searchDirection, normals[i] );
			if ( dot < minDot )
			{
				minDot = dot;
				edgeB = i;
			}
		}
	}
	else
	{
		flip = true;

		b2Vec2 searchDirection = localPolyB.normals[edgeB];

		// Find the incident edge on polyA
		int count = localPolyA.count;
		const b2Vec2* normals = localPolyA.normals;
		edgeA = 0;
		float minDot = FLT_MAX;
		for ( int i = 0; i < count; ++i )
		{
			float dot = b2Dot( searchDirection, normals[i] );
			if ( dot < minDot )
			{
				minDot = dot;
				edgeA = i;
			}
		}
	}

	b2Manifold manifold = { 0 };

	// Using slop here to ensure vertex-vertex normal vectors can be safely normalized
	// todo this means edge clipping needs to handle slightly non-overlapping edges.
	if ( separationA > 0.1f * B2_LINEAR_SLOP || separationB > 0.1f * B2_LINEAR_SLOP )
	{
		// Polygons are disjoint. Find closest points between reference edge and incident edge
		// Reference edge on polygon A
		int i11 = edgeA;
		int i12 = edgeA + 1 < localPolyA.count ? edgeA + 1 : 0;
		int i21 = edgeB;
		int i22 = edgeB + 1 < localPolyB.count ? edgeB + 1 : 0;

		b2Vec2 v11 = localPolyA.vertices[i11];
		b2Vec2 v12 = localPolyA.vertices[i12];
		b2Vec2 v21 = localPolyB.vertices[i21];
		b2Vec2 v22 = localPolyB.vertices[i22];

		b2SegmentDistanceResult result = b2SegmentDistance( v11, v12, v21, v22 );

		if ( result.fraction1 == 0.0f && result.fraction2 == 0.0f )
		{
			// v11 - v21
			b2Vec2 normal = b2Sub( v21, v11 );
			B2_ASSERT( result.distanceSquared > 0.0f );
			float distance = sqrtf( result.distanceSquared );
			if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			{
				return manifold;
			}
			float invDistance = 1.0f / distance;
			normal.x *= invDistance;
			normal.y *= invDistance;

			b2Vec2 c1 = b2MulAdd( v11, localPolyA.radius, normal );
			b2Vec2 c2 = b2MulAdd( v21, -localPolyB.radius, normal );

			manifold.normal = normal;
			manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			manifold.points[0].separation = distance - radius;
			manifold.points[0].id = B2_MAKE_ID( i11, i21 );
			manifold.pointCount = 1;
		}
		else if ( result.fraction1 == 0.0f && result.fraction2 == 1.0f )
		{
			// v11 - v22
			b2Vec2 normal = b2Sub( v22, v11 );
			B2_ASSERT( result.distanceSquared > 0.0f );
			float distance = sqrtf( result.distanceSquared );
			if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			{
				return manifold;
			}
			float invDistance = 1.0f / distance;
			normal.x *= invDistance;
			normal.y *= invDistance;

			b2Vec2 c1 = b2MulAdd( v11, localPolyA.radius, normal );
			b2Vec2 c2 = b2MulAdd( v22, -localPolyB.radius, normal );

			manifold.normal = normal;
			manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			manifold.points[0].separation = distance - radius;
			manifold.points[0].id = B2_MAKE_ID( i11, i22 );
			manifold.pointCount = 1;
		}
		else if ( result.fraction1 == 1.0f && result.fraction2 == 0.0f )
		{
			// v12 - v21
			b2Vec2 normal = b2Sub( v21, v12 );
			B2_ASSERT( result.distanceSquared > 0.0f );
			float distance = sqrtf( result.distanceSquared );
			if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			{
				return manifold;
			}
			float invDistance = 1.0f / distance;
			normal.x *= invDistance;
			normal.y *= invDistance;

			b2Vec2 c1 = b2MulAdd( v12, localPolyA.radius, normal );
			b2Vec2 c2 = b2MulAdd( v21, -localPolyB.radius, normal );

			manifold.normal = normal;
			manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			manifold.points[0].separation = distance - radius;
			manifold.points[0].id = B2_MAKE_ID( i12, i21 );
			manifold.pointCount = 1;
		}
		else if ( result.fraction1 == 1.0f && result.fraction2 == 1.0f )
		{
			// v12 - v22
			b2Vec2 normal = b2Sub( v22, v12 );
			B2_ASSERT( result.distanceSquared > 0.0f );
			float distance = sqrtf( result.distanceSquared );
			if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			{
				return manifold;
			}
			float invDistance = 1.0f / distance;
			normal.x *= invDistance;
			normal.y *= invDistance;

			b2Vec2 c1 = b2MulAdd( v12, localPolyA.radius, normal );
			b2Vec2 c2 = b2MulAdd( v22, -localPolyB.radius, normal );

			manifold.normal = normal;
			manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			manifold.points[0].separation = distance - radius;
			manifold.points[0].id = B2_MAKE_ID( i12, i22 );
			manifold.pointCount = 1;
		}
		else
		{
			// Edge region
			manifold = b2ClipPolygons( &localPolyA, &localPolyB, edgeA, edgeB, flip );
		}
	}
	else
	{
		// Polygons overlap
		manifold = b2ClipPolygons( &localPolyA, &localPolyB, edgeA, edgeB, flip );
	}

	// Convert manifold to world space
	if ( manifold.pointCount > 0 )
	{
		manifold.normal = b2RotateVector( xfA.q, manifold.normal );
		for ( int i = 0; i < manifold.pointCount; ++i )
		{
			b2ManifoldPoint* mp = manifold.points + i;

			// anchor points relative to shape origin in world space
			mp->anchorA = b2RotateVector( xfA.q, b2Add( mp->anchorA, origin ) );
			mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
			mp->point = b2Add( xfA.p, mp->anchorA );
		}
	}

	return manifold;
}

b2Manifold b2CollideSegmentAndCircle( const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB )
{
	b2Capsule capsuleA = { segmentA->point1, segmentA->point2, 0.0f };
	return b2CollideCapsuleAndCircle( &capsuleA, xfA, circleB, xfB );
}

b2Manifold b2CollideSegmentAndPolygon( const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB, b2Transform xfB )
{
	b2Polygon polygonA = b2MakeCapsule( segmentA->point1, segmentA->point2, 0.0f );
	return b2CollidePolygons( &polygonA, xfA, polygonB, xfB );
}

b2Manifold b2CollideChainSegmentAndCircle( const b2ChainSegment* segmentA, b2Transform xfA, const b2Circle* circleB,
										   b2Transform xfB )
{
	b2Manifold manifold = { 0 };

	b2Transform xf = b2InvMulTransforms( xfA, xfB );

	// Compute circle in frame of segment
	b2Vec2 pB = b2TransformPoint( xf, circleB->center );

	b2Vec2 p1 = segmentA->segment.point1;
	b2Vec2 p2 = segmentA->segment.point2;
	b2Vec2 e = b2Sub( p2, p1 );

	// Normal points to the right
	float offset = b2Dot( b2RightPerp( e ), b2Sub( pB, p1 ) );
	if ( offset < 0.0f )
	{
		// collision is one-sided
		return manifold;
	}

	// Barycentric coordinates
	float u = b2Dot( e, b2Sub( p2, pB ) );
	float v = b2Dot( e, b2Sub( pB, p1 ) );

	b2Vec2 pA;

	if ( v <= 0.0f )
	{
		// Behind point1?
		// Is pB in the Voronoi region of the previous edge?
		b2Vec2 prevEdge = b2Sub( p1, segmentA->ghost1 );
		float uPrev = b2Dot( prevEdge, b2Sub( pB, p1 ) );
		if ( uPrev <= 0.0f )
		{
			return manifold;
		}

		pA = p1;
	}
	else if ( u <= 0.0f )
	{
		// Ahead of point2?
		b2Vec2 nextEdge = b2Sub( segmentA->ghost2, p2 );
		float vNext = b2Dot( nextEdge, b2Sub( pB, p2 ) );

		// Is pB in the Voronoi region of the next edge?
		if ( vNext > 0.0f )
		{
			return manifold;
		}

		pA = p2;
	}
	else
	{
		float ee = b2Dot( e, e );
		pA = ( b2Vec2 ){ u * p1.x + v * p2.x, u * p1.y + v * p2.y };
		pA = ee > 0.0f ? b2MulSV( 1.0f / ee, pA ) : p1;
	}

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize( &distance, b2Sub( pB, pA ) );

	float radius = circleB->radius;
	float separation = distance - radius;
	if ( separation > B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 cA = pA;
	b2Vec2 cB = b2MulAdd( pB, -radius, normal );
	b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

	manifold.normal = b2RotateVector( xfA.q, normal );

	b2ManifoldPoint* mp = manifold.points + 0;
	mp->anchorA = b2RotateVector( xfA.q, contactPointA );
	mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
	mp->point = b2Add( xfA.p, mp->anchorA );
	mp->separation = separation;
	mp->id = 0;
	manifold.pointCount = 1;
	return manifold;
}

b2Manifold b2CollideChainSegmentAndCapsule( const b2ChainSegment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
											b2Transform xfB, b2SimplexCache* cache )
{
	b2Polygon polyB = b2MakeCapsule( capsuleB->center1, capsuleB->center2, capsuleB->radius );
	return b2CollideChainSegmentAndPolygon( segmentA, xfA, &polyB, xfB, cache );
}

static b2Manifold b2ClipSegments( b2Vec2 a1, b2Vec2 a2, b2Vec2 b1, b2Vec2 b2, b2Vec2 normal, float ra, float rb, uint16_t id1,
								  uint16_t id2 )
{
	b2Manifold manifold = { 0 };

	b2Vec2 tangent = b2LeftPerp( normal );

	// Barycentric coordinates of each point relative to a1 along tangent
	float lower1 = 0.0f;
	float upper1 = b2Dot( b2Sub( a2, a1 ), tangent );

	// Incident edge points opposite of tangent due to CCW winding
	float upper2 = b2Dot( b2Sub( b1, a1 ), tangent );
	float lower2 = b2Dot( b2Sub( b2, a1 ), tangent );

	// Do segments overlap?
	if ( upper2 < lower1 || upper1 < lower2 )
	{
		return manifold;
	}

	b2Vec2 vLower;
	if ( lower2 < lower1 && upper2 - lower2 > FLT_EPSILON )
	{
		vLower = b2Lerp( b2, b1, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vLower = b2;
	}

	b2Vec2 vUpper;
	if ( upper2 > upper1 && upper2 - lower2 > FLT_EPSILON )
	{
		vUpper = b2Lerp( b2, b1, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vUpper = b1;
	}

	// todo vLower can be very close to vUpper, reduce to one point?

	float separationLower = b2Dot( b2Sub( vLower, a1 ), normal );
	float separationUpper = b2Dot( b2Sub( vUpper, a1 ), normal );

	// Put contact points at midpoint, accounting for radii
	vLower = b2MulAdd( vLower, 0.5f * ( ra - rb - separationLower ), normal );
	vUpper = b2MulAdd( vUpper, 0.5f * ( ra - rb - separationUpper ), normal );

	float radius = ra + rb;

	manifold.normal = normal;
	{
		b2ManifoldPoint* cp = manifold.points + 0;
		cp->anchorA = vLower;
		cp->separation = separationLower - radius;
		cp->id = id1;
	}

	{
		b2ManifoldPoint* cp = manifold.points + 1;
		cp->anchorA = vUpper;
		cp->separation = separationUpper - radius;
		cp->id = id2;
	}

	manifold.pointCount = 2;

	return manifold;
}

enum b2NormalType
{
	// This means the normal points in a direction that is non-smooth relative to a convex vertex and should be skipped
	b2_normalSkip,

	// This means the normal points in a direction that is smooth relative to a convex vertex and should be used for collision
	b2_normalAdmit,

	// This means the normal is in a region of a concave vertex and should be snapped to the segment normal
	b2_normalSnap
};

struct b2ChainSegmentParams
{
	b2Vec2 edge1;
	b2Vec2 normal0;
	b2Vec2 normal2;
	bool convex1;
	bool convex2;
};

// Evaluate Gauss map
// See https://box2d.org/posts/2020/06/ghost-collisions/
static enum b2NormalType b2ClassifyNormal( struct b2ChainSegmentParams params, b2Vec2 normal )
{
	const float sinTol = 0.01f;

	if ( b2Dot( normal, params.edge1 ) <= 0.0f )
	{
		// Normal points towards the segment tail
		if ( params.convex1 )
		{
			if ( b2Cross( normal, params.normal0 ) > sinTol )
			{
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		}
		else
		{
			return b2_normalSnap;
		}
	}
	else
	{
		// Normal points towards segment head
		if ( params.convex2 )
		{
			if ( b2Cross( params.normal2, normal ) > sinTol )
			{
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		}
		else
		{
			return b2_normalSnap;
		}
	}
}

b2Manifold b2CollideChainSegmentAndPolygon( const b2ChainSegment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
											b2Transform xfB, b2SimplexCache* cache )
{
	b2Manifold manifold = { 0 };

	b2Transform xf = b2InvMulTransforms( xfA, xfB );

	b2Vec2 centroidB = b2TransformPoint( xf, polygonB->centroid );
	float radiusB = polygonB->radius;

	b2Vec2 p1 = segmentA->segment.point1;
	b2Vec2 p2 = segmentA->segment.point2;

	b2Vec2 edge1 = b2Normalize( b2Sub( p2, p1 ) );

	struct b2ChainSegmentParams smoothParams = { 0 };
	smoothParams.edge1 = edge1;

	const float convexTol = 0.01f;
	b2Vec2 edge0 = b2Normalize( b2Sub( p1, segmentA->ghost1 ) );
	smoothParams.normal0 = b2RightPerp( edge0 );
	smoothParams.convex1 = b2Cross( edge0, edge1 ) >= convexTol;

	b2Vec2 edge2 = b2Normalize( b2Sub( segmentA->ghost2, p2 ) );
	smoothParams.normal2 = b2RightPerp( edge2 );
	smoothParams.convex2 = b2Cross( edge1, edge2 ) >= convexTol;

	// Normal points to the right
	b2Vec2 normal1 = b2RightPerp( edge1 );
	bool behind1 = b2Dot( normal1, b2Sub( centroidB, p1 ) ) < 0.0f;
	bool behind0 = true;
	bool behind2 = true;
	if ( smoothParams.convex1 )
	{
		behind0 = b2Dot( smoothParams.normal0, b2Sub( centroidB, p1 ) ) < 0.0f;
	}

	if ( smoothParams.convex2 )
	{
		behind2 = b2Dot( smoothParams.normal2, b2Sub( centroidB, p2 ) ) < 0.0f;
	}

	if ( behind1 && behind0 && behind2 )
	{
		// one-sided collision
		return manifold;
	}

	// Get polygonB in frameA
	int count = polygonB->count;
	b2Vec2 vertices[B2_MAX_POLYGON_VERTICES];
	b2Vec2 normals[B2_MAX_POLYGON_VERTICES];
	for ( int i = 0; i < count; ++i )
	{
		vertices[i] = b2TransformPoint( xf, polygonB->vertices[i] );
		normals[i] = b2RotateVector( xf.q, polygonB->normals[i] );
	}

	// Distance doesn't work correctly with partial polygons
	b2DistanceInput input;
	input.proxyA = b2MakeProxy( &segmentA->segment.point1, 2, 0.0f );
	input.proxyB = b2MakeProxy( vertices, count, 0.0f );
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.useRadii = false;

	b2DistanceOutput output = b2ShapeDistance( cache, &input, NULL, 0 );

	if ( output.distance > radiusB + B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	// Snap concave normals for partial polygon
	b2Vec2 n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
	b2Vec2 n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;

	// Index of incident vertex on polygon
	int incidentIndex = -1;
	int incidentNormal = -1;

	if ( behind1 == false && output.distance > 0.1f * B2_LINEAR_SLOP )
	{
		// The closest features may be two vertices or an edge and a vertex even when there should
		// be face contact

		if ( cache->count == 1 )
		{
			// vertex-vertex collision
			b2Vec2 pA = output.pointA;
			b2Vec2 pB = output.pointB;

			b2Vec2 normal = b2Normalize( b2Sub( pB, pA ) );

			enum b2NormalType type = b2ClassifyNormal( smoothParams, normal );
			if ( type == b2_normalSkip )
			{
				return manifold;
			}

			if ( type == b2_normalAdmit )
			{
				manifold.normal = b2RotateVector( xfA.q, normal );
				b2ManifoldPoint* cp = manifold.points + 0;
				cp->anchorA = b2RotateVector( xfA.q, pA );
				cp->anchorB = b2Add( cp->anchorA, b2Sub( xfA.p, xfB.p ) );
				cp->point = b2Add( xfA.p, cp->anchorA );
				cp->separation = output.distance - radiusB;
				cp->id = B2_MAKE_ID( cache->indexA[0], cache->indexB[0] );
				manifold.pointCount = 1;
				return manifold;
			}

			// fall through b2_normalSnap
			incidentIndex = cache->indexB[0];
		}
		else
		{
			// vertex-edge collision
			B2_ASSERT( cache->count == 2 );

			int ia1 = cache->indexA[0];
			int ia2 = cache->indexA[1];
			int ib1 = cache->indexB[0];
			int ib2 = cache->indexB[1];

			if ( ia1 == ia2 )
			{
				// 1 point on A, expect 2 points on B
				B2_ASSERT( ib1 != ib2 );

				// Find polygon normal most aligned with vector between closest points.
				// This effectively sorts ib1 and ib2
				b2Vec2 normalB = b2Sub( output.pointA, output.pointB );
				float dot1 = b2Dot( normalB, normals[ib1] );
				float dot2 = b2Dot( normalB, normals[ib2] );
				int ib = dot1 > dot2 ? ib1 : ib2;

				// Use accurate normal
				normalB = normals[ib];

				enum b2NormalType type = b2ClassifyNormal( smoothParams, b2Neg( normalB ) );
				if ( type == b2_normalSkip )
				{
					return manifold;
				}

				if ( type == b2_normalAdmit )
				{
					// Get polygon edge associated with normal
					ib1 = ib;
					ib2 = ib < count - 1 ? ib + 1 : 0;

					b2Vec2 b1 = vertices[ib1];
					b2Vec2 b2 = vertices[ib2];

					// Find incident segment vertex
					dot1 = b2Dot( normalB, b2Sub( p1, b1 ) );
					dot2 = b2Dot( normalB, b2Sub( p2, b1 ) );

					if ( dot1 < dot2 )
					{
						if ( b2Dot( n0, normalB ) < b2Dot( normal1, normalB ) )
						{
							// Neighbor is incident
							return manifold;
						}
					}
					else
					{
						if ( b2Dot( n2, normalB ) < b2Dot( normal1, normalB ) )
						{
							// Neighbor is incident
							return manifold;
						}
					}

					manifold =
						b2ClipSegments( b1, b2, p1, p2, normalB, radiusB, 0.0f, B2_MAKE_ID( ib1, 1 ), B2_MAKE_ID( ib2, 0 ) );
					manifold.normal = b2RotateVector( xfA.q, b2Neg( normalB ) );
					manifold.points[0].anchorA = b2RotateVector( xfA.q, manifold.points[0].anchorA );
					manifold.points[1].anchorA = b2RotateVector( xfA.q, manifold.points[1].anchorA );
					b2Vec2 pAB = b2Sub( xfA.p, xfB.p );
					manifold.points[0].anchorB = b2Add( manifold.points[0].anchorA, pAB );
					manifold.points[1].anchorB = b2Add( manifold.points[1].anchorA, pAB );
					manifold.points[0].point = b2Add( xfA.p, manifold.points[0].anchorA );
					manifold.points[1].point = b2Add( xfA.p, manifold.points[1].anchorA );
					return manifold;
				}

				// fall through b2_normalSnap
				incidentNormal = ib;
			}
			else
			{
				// Get index of incident polygonB vertex
				float dot1 = b2Dot( normal1, b2Sub( vertices[ib1], p1 ) );
				float dot2 = b2Dot( normal1, b2Sub( vertices[ib2], p2 ) );
				incidentIndex = dot1 < dot2 ? ib1 : ib2;
			}
		}
	}
	else
	{
		// SAT edge normal
		float edgeSeparation = FLT_MAX;

		for ( int i = 0; i < count; ++i )
		{
			float s = b2Dot( normal1, b2Sub( vertices[i], p1 ) );
			if ( s < edgeSeparation )
			{
				edgeSeparation = s;
				incidentIndex = i;
			}
		}

		// Check convex neighbor for edge separation
		if ( smoothParams.convex1 )
		{
			float s0 = FLT_MAX;

			for ( int i = 0; i < count; ++i )
			{
				float s = b2Dot( smoothParams.normal0, b2Sub( vertices[i], p1 ) );
				if ( s < s0 )
				{
					s0 = s;
				}
			}

			if ( s0 > edgeSeparation )
			{
				edgeSeparation = s0;

				// Indicate neighbor owns edge separation
				incidentIndex = -1;
			}
		}

		// Check convex neighbor for edge separation
		if ( smoothParams.convex2 )
		{
			float s2 = FLT_MAX;

			for ( int i = 0; i < count; ++i )
			{
				float s = b2Dot( smoothParams.normal2, b2Sub( vertices[i], p2 ) );
				if ( s < s2 )
				{
					s2 = s;
				}
			}

			if ( s2 > edgeSeparation )
			{
				edgeSeparation = s2;

				// Indicate neighbor owns edge separation
				incidentIndex = -1;
			}
		}

		// SAT polygon normals
		float polygonSeparation = -FLT_MAX;
		int referenceIndex = -1;

		for ( int i = 0; i < count; ++i )
		{
			b2Vec2 n = normals[i];

			enum b2NormalType type = b2ClassifyNormal( smoothParams, b2Neg( n ) );
			if ( type != b2_normalAdmit )
			{
				continue;
			}

			// Check the infinite sides of the partial polygon
			// if ((smoothParams.convex1 && b2Cross(n0, n) > 0.0f) || (smoothParams.convex2 && b2Cross(n, n2) > 0.0f))
			//{
			//	continue;
			//}

			b2Vec2 p = vertices[i];
			float s = b2MinFloat( b2Dot( n, b2Sub( p2, p ) ), b2Dot( n, b2Sub( p1, p ) ) );

			if ( s > polygonSeparation )
			{
				polygonSeparation = s;
				referenceIndex = i;
			}
		}

		if ( polygonSeparation > edgeSeparation )
		{
			int ia1 = referenceIndex;
			int ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
			b2Vec2 a1 = vertices[ia1];
			b2Vec2 a2 = vertices[ia2];

			b2Vec2 n = normals[ia1];

			float dot1 = b2Dot( n, b2Sub( p1, a1 ) );
			float dot2 = b2Dot( n, b2Sub( p2, a1 ) );

			if ( dot1 < dot2 )
			{
				if ( b2Dot( n0, n ) < b2Dot( normal1, n ) )
				{
					// Neighbor is incident
					return manifold;
				}
			}
			else
			{
				if ( b2Dot( n2, n ) < b2Dot( normal1, n ) )
				{
					// Neighbor is incident
					return manifold;
				}
			}

			manifold = b2ClipSegments( a1, a2, p1, p2, normals[ia1], radiusB, 0.0f, B2_MAKE_ID( ia1, 1 ), B2_MAKE_ID( ia2, 0 ) );
			manifold.normal = b2RotateVector( xfA.q, b2Neg( normals[ia1] ) );
			manifold.points[0].anchorA = b2RotateVector( xfA.q, manifold.points[0].anchorA );
			manifold.points[1].anchorA = b2RotateVector( xfA.q, manifold.points[1].anchorA );
			b2Vec2 pAB = b2Sub( xfA.p, xfB.p );
			manifold.points[0].anchorB = b2Add( manifold.points[0].anchorA, pAB );
			manifold.points[1].anchorB = b2Add( manifold.points[1].anchorA, pAB );
			manifold.points[0].point = b2Add( xfA.p, manifold.points[0].anchorA );
			manifold.points[1].point = b2Add( xfA.p, manifold.points[1].anchorA );
			return manifold;
		}

		if ( incidentIndex == -1 )
		{
			// neighboring segment is the separating axis
			return manifold;
		}

		// fall through segment normal axis
	}

	B2_ASSERT( incidentNormal != -1 || incidentIndex != -1 );

	// Segment normal

	// Find incident polygon normal: normal adjacent to deepest vertex that is most anti-parallel to segment normal
	b2Vec2 b1, b2;
	int ib1, ib2;

	if ( incidentNormal != -1 )
	{
		ib1 = incidentNormal;
		ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
		b1 = vertices[ib1];
		b2 = vertices[ib2];
	}
	else
	{
		int i2 = incidentIndex;
		int i1 = i2 > 0 ? i2 - 1 : count - 1;
		float d1 = b2Dot( normal1, normals[i1] );
		float d2 = b2Dot( normal1, normals[i2] );
		if ( d1 < d2 )
		{
			ib1 = i1, ib2 = i2;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
		else
		{
			ib1 = i2, ib2 = i2 < count - 1 ? i2 + 1 : 0;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
	}

	manifold = b2ClipSegments( p1, p2, b1, b2, normal1, 0.0f, radiusB, B2_MAKE_ID( 0, ib2 ), B2_MAKE_ID( 1, ib1 ) );
	manifold.normal = b2RotateVector( xfA.q, manifold.normal );
	manifold.points[0].anchorA = b2RotateVector( xfA.q, manifold.points[0].anchorA );
	manifold.points[1].anchorA = b2RotateVector( xfA.q, manifold.points[1].anchorA );
	b2Vec2 pAB = b2Sub( xfA.p, xfB.p );
	manifold.points[0].anchorB = b2Add( manifold.points[0].anchorA, pAB );
	manifold.points[1].anchorB = b2Add( manifold.points[1].anchorA, pAB );
	manifold.points[0].point = b2Add( xfA.p, manifold.points[0].anchorA );
	manifold.points[1].point = b2Add( xfA.p, manifold.points[1].anchorA );

	return manifold;
}
