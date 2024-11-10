// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "core.h"
#include "shape.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <stddef.h>

_Static_assert( b2_maxPolygonVertices > 2, "must be 3 or more" );

bool b2IsValidRay( const b2RayCastInput* input )
{
	bool isValid = b2Vec2_IsValid( input->origin ) && b2Vec2_IsValid( input->translation ) && b2Float_IsValid( input->maxFraction ) &&
				   0.0f <= input->maxFraction && input->maxFraction < b2_huge;
	return isValid;
}

static b2Vec2 b2ComputePolygonCentroid( const b2Vec2* vertices, int32_t count )
{
	b2Vec2 center = { 0.0f, 0.0f };
	float area = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 origin = vertices[0];

	const float inv3 = 1.0f / 3.0f;

	for ( int32_t i = 1; i < count - 1; ++i )
	{
		// Triangle edges
		b2Vec2 e1 = b2Sub( vertices[i], origin );
		b2Vec2 e2 = b2Sub( vertices[i + 1], origin );
		float a = 0.5f * b2Cross( e1, e2 );

		// Area weighted centroid
		center = b2MulAdd( center, a * inv3, b2Add( e1, e2 ) );
		area += a;
	}

	B2_ASSERT( area > FLT_EPSILON );
	float invArea = 1.0f / area;
	center.x *= invArea;
	center.y *= invArea;

	// Restore offset
	center = b2Add( origin, center );

	return center;
}

b2Polygon b2MakePolygon( const b2Hull* hull, float radius )
{
	B2_ASSERT( b2ValidateHull( hull ) );

	if ( hull->count < 3 )
	{
		// Handle a bad hull when assertions are disabled
		return b2MakeSquare( 0.5f );
	}

	b2Polygon shape = { 0 };
	shape.count = hull->count;
	shape.radius = radius;

	// Copy vertices
	for ( int32_t i = 0; i < shape.count; ++i )
	{
		shape.vertices[i] = hull->points[i];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for ( int32_t i = 0; i < shape.count; ++i )
	{
		int32_t i1 = i;
		int32_t i2 = i + 1 < shape.count ? i + 1 : 0;
		b2Vec2 edge = b2Sub( shape.vertices[i2], shape.vertices[i1] );
		B2_ASSERT( b2Dot( edge, edge ) > FLT_EPSILON * FLT_EPSILON );
		shape.normals[i] = b2Normalize( b2CrossVS( edge, 1.0f ) );
	}

	shape.centroid = b2ComputePolygonCentroid( shape.vertices, shape.count );

	return shape;
}

b2Polygon b2MakeOffsetPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation )
{
	return b2MakeOffsetRoundedPolygon( hull, position, rotation, 0.0f );
}

b2Polygon b2MakeOffsetRoundedPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation, float radius )
{
	B2_ASSERT( b2ValidateHull( hull ) );

	if ( hull->count < 3 )
	{
		// Handle a bad hull when assertions are disabled
		return b2MakeSquare( 0.5f );
	}

	b2Transform transform = { position, rotation };

	b2Polygon shape = { 0 };
	shape.count = hull->count;
	shape.radius = radius;

	// Copy vertices
	for ( int32_t i = 0; i < shape.count; ++i )
	{
		shape.vertices[i] = b2TransformPoint( transform, hull->points[i] );
	}

	// Compute normals. Ensure the edges have non-zero length.
	for ( int32_t i = 0; i < shape.count; ++i )
	{
		int32_t i1 = i;
		int32_t i2 = i + 1 < shape.count ? i + 1 : 0;
		b2Vec2 edge = b2Sub( shape.vertices[i2], shape.vertices[i1] );
		B2_ASSERT( b2Dot( edge, edge ) > FLT_EPSILON * FLT_EPSILON );
		shape.normals[i] = b2Normalize( b2CrossVS( edge, 1.0f ) );
	}

	shape.centroid = b2ComputePolygonCentroid( shape.vertices, shape.count );

	return shape;
}

b2Polygon b2MakeSquare( float h )
{
	return b2MakeBox( h, h );
}

b2Polygon b2MakeBox( float hx, float hy )
{
	B2_ASSERT( b2Float_IsValid( hx ) && hx > 0.0f );
	B2_ASSERT( b2Float_IsValid( hy ) && hy > 0.0f );

	b2Polygon shape = { 0 };
	shape.count = 4;
	shape.vertices[0] = ( b2Vec2 ){ -hx, -hy };
	shape.vertices[1] = ( b2Vec2 ){ hx, -hy };
	shape.vertices[2] = ( b2Vec2 ){ hx, hy };
	shape.vertices[3] = ( b2Vec2 ){ -hx, hy };
	shape.normals[0] = ( b2Vec2 ){ 0.0f, -1.0f };
	shape.normals[1] = ( b2Vec2 ){ 1.0f, 0.0f };
	shape.normals[2] = ( b2Vec2 ){ 0.0f, 1.0f };
	shape.normals[3] = ( b2Vec2 ){ -1.0f, 0.0f };
	shape.radius = 0.0f;
	shape.centroid = b2Vec2_zero;
	return shape;
}

b2Polygon b2MakeRoundedBox( float hx, float hy, float radius )
{
	B2_ASSERT( b2Float_IsValid( radius ) && radius >= 0.0f );
	b2Polygon shape = b2MakeBox( hx, hy );
	shape.radius = radius;
	return shape;
}

b2Polygon b2MakeOffsetBox( float hx, float hy, b2Vec2 center, b2Rot rotation )
{
	b2Transform xf = { center, rotation };

	b2Polygon shape = { 0 };
	shape.count = 4;
	shape.vertices[0] = b2TransformPoint( xf, ( b2Vec2 ){ -hx, -hy } );
	shape.vertices[1] = b2TransformPoint( xf, ( b2Vec2 ){ hx, -hy } );
	shape.vertices[2] = b2TransformPoint( xf, ( b2Vec2 ){ hx, hy } );
	shape.vertices[3] = b2TransformPoint( xf, ( b2Vec2 ){ -hx, hy } );
	shape.normals[0] = b2RotateVector( xf.q, ( b2Vec2 ){ 0.0f, -1.0f } );
	shape.normals[1] = b2RotateVector( xf.q, ( b2Vec2 ){ 1.0f, 0.0f } );
	shape.normals[2] = b2RotateVector( xf.q, ( b2Vec2 ){ 0.0f, 1.0f } );
	shape.normals[3] = b2RotateVector( xf.q, ( b2Vec2 ){ -1.0f, 0.0f } );
	shape.radius = 0.0f;
	shape.centroid = xf.p;
	return shape;
}

b2Polygon b2MakeOffsetRoundedBox( float hx, float hy, b2Vec2 center, b2Rot rotation, float radius )
{
	B2_ASSERT( b2Float_IsValid( radius ) && radius >= 0.0f );
	b2Transform xf = { center, rotation };

	b2Polygon shape = { 0 };
	shape.count = 4;
	shape.vertices[0] = b2TransformPoint( xf, ( b2Vec2 ){ -hx, -hy } );
	shape.vertices[1] = b2TransformPoint( xf, ( b2Vec2 ){ hx, -hy } );
	shape.vertices[2] = b2TransformPoint( xf, ( b2Vec2 ){ hx, hy } );
	shape.vertices[3] = b2TransformPoint( xf, ( b2Vec2 ){ -hx, hy } );
	shape.normals[0] = b2RotateVector( xf.q, ( b2Vec2 ){ 0.0f, -1.0f } );
	shape.normals[1] = b2RotateVector( xf.q, ( b2Vec2 ){ 1.0f, 0.0f } );
	shape.normals[2] = b2RotateVector( xf.q, ( b2Vec2 ){ 0.0f, 1.0f } );
	shape.normals[3] = b2RotateVector( xf.q, ( b2Vec2 ){ -1.0f, 0.0f } );
	shape.radius = radius;
	shape.centroid = xf.p;
	return shape;
}

b2Polygon b2TransformPolygon( b2Transform transform, const b2Polygon* polygon )
{
	b2Polygon p = *polygon;

	for ( int i = 0; i < p.count; ++i )
	{
		p.vertices[i] = b2TransformPoint( transform, p.vertices[i] );
		p.normals[i] = b2RotateVector( transform.q, p.normals[i] );
	}

	p.centroid = b2TransformPoint( transform, p.centroid );

	return p;
}

b2MassData b2ComputeCircleMass( const b2Circle* shape, float density )
{
	float rr = shape->radius * shape->radius;

	b2MassData massData;
	massData.mass = density * b2_pi * rr;
	massData.center = shape->center;

	// inertia about the local origin
	massData.rotationalInertia = massData.mass * ( 0.5f * rr + b2Dot( shape->center, shape->center ) );

	return massData;
}

b2MassData b2ComputeCapsuleMass( const b2Capsule* shape, float density )
{
	float radius = shape->radius;
	float rr = radius * radius;
	b2Vec2 p1 = shape->center1;
	b2Vec2 p2 = shape->center2;
	float length = b2Length( b2Sub( p2, p1 ) );
	float ll = length * length;

	float circleMass = density * ( b2_pi * radius * radius );
	float boxMass = density * ( 2.0f * radius * length );

	b2MassData massData;
	massData.mass = circleMass + boxMass;
	massData.center.x = 0.5f * ( p1.x + p2.x );
	massData.center.y = 0.5f * ( p1.y + p2.y );

	// two offset half circles, both halves add up to full circle and each half is offset by half length
	// semi-circle centroid = 4 r / 3 pi
	// Need to apply parallel-axis theorem twice:
	// 1. shift semi-circle centroid to origin
	// 2. shift semi-circle to box end
	// m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
	// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
	// I verified this formula by computing the convex hull of a 128 vertex capsule

	// half circle centroid
	float lc = 4.0f * radius / ( 3.0f * b2_pi );

	// half length of rectangular portion of capsule
	float h = 0.5f * length;

	float circleInertia = circleMass * ( 0.5f * rr + h * h + 2.0f * h * lc );
	float boxInertia = boxMass * ( 4.0f * rr + ll ) / 12.0f;
	massData.rotationalInertia = circleInertia + boxInertia;

	// inertia about the local origin
	massData.rotationalInertia += massData.mass * b2Dot( massData.center, massData.center );

	return massData;
}

b2MassData b2ComputePolygonMass( const b2Polygon* shape, float density )
{
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	B2_ASSERT( shape->count > 0 );

	if ( shape->count == 1 )
	{
		b2Circle circle;
		circle.center = shape->vertices[0];
		circle.radius = shape->radius;
		return b2ComputeCircleMass( &circle, density );
	}

	if ( shape->count == 2 )
	{
		b2Capsule capsule;
		capsule.center1 = shape->vertices[0];
		capsule.center2 = shape->vertices[1];
		capsule.radius = shape->radius;
		return b2ComputeCapsuleMass( &capsule, density );
	}

	b2Vec2 vertices[b2_maxPolygonVertices] = { 0 };
	int32_t count = shape->count;
	float radius = shape->radius;

	if ( radius > 0.0f )
	{
		// Approximate mass of rounded polygons by pushing out the vertices.
		float sqrt2 = 1.412f;
		for ( int32_t i = 0; i < count; ++i )
		{
			int32_t j = i == 0 ? count - 1 : i - 1;
			b2Vec2 n1 = shape->normals[j];
			b2Vec2 n2 = shape->normals[i];

			b2Vec2 mid = b2Normalize( b2Add( n1, n2 ) );
			vertices[i] = b2MulAdd( shape->vertices[i], sqrt2 * radius, mid );
		}
	}
	else
	{
		for ( int32_t i = 0; i < count; ++i )
		{
			vertices[i] = shape->vertices[i];
		}
	}

	b2Vec2 center = { 0.0f, 0.0f };
	float area = 0.0f;
	float rotationalInertia = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 r = vertices[0];

	const float inv3 = 1.0f / 3.0f;

	for ( int32_t i = 1; i < count - 1; ++i )
	{
		// Triangle edges
		b2Vec2 e1 = b2Sub( vertices[i], r );
		b2Vec2 e2 = b2Sub( vertices[i + 1], r );

		float D = b2Cross( e1, e2 );

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid, r at origin
		center = b2MulAdd( center, triangleArea * inv3, b2Add( e1, e2 ) );

		float ex1 = e1.x, ey1 = e1.y;
		float ex2 = e2.x, ey2 = e2.y;

		float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
		float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

		rotationalInertia += ( 0.25f * inv3 * D ) * ( intx2 + inty2 );
	}

	b2MassData massData;

	// Total mass
	massData.mass = density * area;

	// Center of mass, shift back from origin at r
	B2_ASSERT( area > FLT_EPSILON );
	float invArea = 1.0f / area;
	center.x *= invArea;
	center.y *= invArea;
	massData.center = b2Add( r, center );

	// Inertia tensor relative to the local origin (point s).
	massData.rotationalInertia = density * rotationalInertia;

	// Shift to center of mass then to original body origin.
	massData.rotationalInertia += massData.mass * ( b2Dot( massData.center, massData.center ) - b2Dot( center, center ) );

	return massData;
}

b2AABB b2ComputeCircleAABB( const b2Circle* shape, b2Transform xf )
{
	b2Vec2 p = b2TransformPoint( xf, shape->center );
	float r = shape->radius;

	b2AABB aabb = { { p.x - r, p.y - r }, { p.x + r, p.y + r } };
	return aabb;
}

b2AABB b2ComputeCapsuleAABB( const b2Capsule* shape, b2Transform xf )
{
	b2Vec2 v1 = b2TransformPoint( xf, shape->center1 );
	b2Vec2 v2 = b2TransformPoint( xf, shape->center2 );

	b2Vec2 r = { shape->radius, shape->radius };
	b2Vec2 lower = b2Sub( b2Min( v1, v2 ), r );
	b2Vec2 upper = b2Add( b2Max( v1, v2 ), r );

	b2AABB aabb = { lower, upper };
	return aabb;
}

b2AABB b2ComputePolygonAABB( const b2Polygon* shape, b2Transform xf )
{
	B2_ASSERT( shape->count > 0 );
	b2Vec2 lower = b2TransformPoint( xf, shape->vertices[0] );
	b2Vec2 upper = lower;

	for ( int32_t i = 1; i < shape->count; ++i )
	{
		b2Vec2 v = b2TransformPoint( xf, shape->vertices[i] );
		lower = b2Min( lower, v );
		upper = b2Max( upper, v );
	}

	b2Vec2 r = { shape->radius, shape->radius };
	lower = b2Sub( lower, r );
	upper = b2Add( upper, r );

	b2AABB aabb = { lower, upper };
	return aabb;
}

b2AABB b2ComputeSegmentAABB( const b2Segment* shape, b2Transform xf )
{
	b2Vec2 v1 = b2TransformPoint( xf, shape->point1 );
	b2Vec2 v2 = b2TransformPoint( xf, shape->point2 );

	b2Vec2 lower = b2Min( v1, v2 );
	b2Vec2 upper = b2Max( v1, v2 );

	b2AABB aabb = { lower, upper };
	return aabb;
}

bool b2PointInCircle( b2Vec2 point, const b2Circle* shape )
{
	b2Vec2 center = shape->center;
	return b2DistanceSquared( point, center ) <= shape->radius * shape->radius;
}

bool b2PointInCapsule( b2Vec2 point, const b2Capsule* shape )
{
	float rr = shape->radius * shape->radius;
	b2Vec2 p1 = shape->center1;
	b2Vec2 p2 = shape->center2;

	b2Vec2 d = b2Sub( p2, p1 );
	float dd = b2Dot( d, d );
	if ( dd == 0.0f )
	{
		// Capsule is really a circle
		return b2DistanceSquared( point, p1 ) <= rr;
	}

	// Get closest point on capsule segment
	// c = p1 + t * d
	// dot(point - c, d) = 0
	// dot(point - p1 - t * d, d) = 0
	// t = dot(point - p1, d) / dot(d, d)
	float t = b2Dot( b2Sub( point, p1 ), d ) / dd;
	t = b2ClampFloat( t, 0.0f, 1.0f );
	b2Vec2 c = b2MulAdd( p1, t, d );

	// Is query point within radius around closest point?
	return b2DistanceSquared( point, c ) <= rr;
}

bool b2PointInPolygon( b2Vec2 point, const b2Polygon* shape )
{
	b2DistanceInput input = { 0 };
	input.proxyA = b2MakeProxy( shape->vertices, shape->count, 0.0f );
	input.proxyB = b2MakeProxy( &point, 1, 0.0f );
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.useRadii = false;

	b2DistanceCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	return output.distance <= shape->radius;
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
b2CastOutput b2RayCastCircle( const b2RayCastInput* input, const b2Circle* shape )
{
	B2_ASSERT( b2IsValidRay( input ) );

	b2Vec2 p = shape->center;

	b2CastOutput output = { 0 };

	// Shift ray so circle center is the origin
	b2Vec2 s = b2Sub( input->origin, p );
	float length;
	b2Vec2 d = b2GetLengthAndNormalize( &length, input->translation );
	if ( length == 0.0f )
	{
		// zero length ray
		return output;
	}

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	float t = -b2Dot( s, d );

	// c is the closest point on the line to the origin
	b2Vec2 c = b2MulAdd( s, t, d );

	float cc = b2Dot( c, c );
	float r = shape->radius;
	float rr = r * r;

	if ( cc > rr )
	{
		// closest point is outside the circle
		return output;
	}

	// Pythagoras
	float h = sqrtf( rr - cc );

	float fraction = t - h;

	if ( fraction < 0.0f || input->maxFraction * length < fraction )
	{
		// outside the range of the ray segment
		return output;
	}

	b2Vec2 hitPoint = b2MulAdd( s, fraction, d );

	output.fraction = fraction / length;
	output.normal = b2Normalize( hitPoint );
	output.point = b2MulAdd( p, shape->radius, output.normal );
	output.hit = true;

	return output;
}

b2CastOutput b2RayCastCapsule( const b2RayCastInput* input, const b2Capsule* shape )
{
	B2_ASSERT( b2IsValidRay( input ) );

	b2CastOutput output = { 0 };

	b2Vec2 v1 = shape->center1;
	b2Vec2 v2 = shape->center2;

	b2Vec2 e = b2Sub( v2, v1 );

	float capsuleLength;
	b2Vec2 a = b2GetLengthAndNormalize( &capsuleLength, e );

	if ( capsuleLength < FLT_EPSILON )
	{
		// Capsule is really a circle
		b2Circle circle = { v1, shape->radius };
		return b2RayCastCircle( input, &circle );
	}

	b2Vec2 p1 = input->origin;
	b2Vec2 d = input->translation;

	// Ray from capsule start to ray start
	b2Vec2 q = b2Sub( p1, v1 );
	float qa = b2Dot( q, a );

	// Vector to ray start that is perpendicular to capsule axis
	b2Vec2 qp = b2MulAdd( q, -qa, a );

	float radius = shape->radius;

	// Does the ray start within the infinite length capsule?
	if ( b2Dot( qp, qp ) < radius * radius )
	{
		if ( qa < 0.0f )
		{
			// start point behind capsule segment
			b2Circle circle = { v1, shape->radius };
			return b2RayCastCircle( input, &circle );
		}

		if ( qa > 1.0f )
		{
			// start point ahead of capsule segment
			b2Circle circle = { v2, shape->radius };
			return b2RayCastCircle( input, &circle );
		}

		// ray starts inside capsule -> no hit
		return output;
	}

	// Perpendicular to capsule axis, pointing right
	b2Vec2 n = { a.y, -a.x };

	float rayLength;
	b2Vec2 u = b2GetLengthAndNormalize( &rayLength, d );

	// Intersect ray with infinite length capsule
	// v1 + radius * n + s1 * a = p1 + s2 * u
	// v1 - radius * n + s1 * a = p1 + s2 * u

	// s1 * a - s2 * u = b
	// b = q - radius * ap
	// or
	// b = q + radius * ap

	// Cramer's rule [a -u]
	float den = -a.x * u.y + u.x * a.y;
	if ( -FLT_EPSILON < den && den < FLT_EPSILON )
	{
		// Ray is parallel to capsule and outside infinite length capsule
		return output;
	}

	b2Vec2 b1 = b2MulSub( q, radius, n );
	b2Vec2 b2 = b2MulAdd( q, radius, n );

	float invDen = 1.0f / den;

	// Cramer's rule [a b1]
	float s21 = ( a.x * b1.y - b1.x * a.y ) * invDen;

	// Cramer's rule [a b2]
	float s22 = ( a.x * b2.y - b2.x * a.y ) * invDen;

	float s2;
	b2Vec2 b;
	if ( s21 < s22 )
	{
		s2 = s21;
		b = b1;
	}
	else
	{
		s2 = s22;
		b = b2;
		n = b2Neg( n );
	}

	if ( s2 < 0.0f || input->maxFraction * rayLength < s2 )
	{
		return output;
	}

	// Cramer's rule [b -u]
	float s1 = ( -b.x * u.y + u.x * b.y ) * invDen;

	if ( s1 < 0.0f )
	{
		// ray passes behind capsule segment
		b2Circle circle = { v1, shape->radius };
		return b2RayCastCircle( input, &circle );
	}
	else if ( capsuleLength < s1 )
	{
		// ray passes ahead of capsule segment
		b2Circle circle = { v2, shape->radius };
		return b2RayCastCircle( input, &circle );
	}
	else
	{
		// ray hits capsule side
		output.fraction = s2 / rayLength;
		output.point = b2Add( b2Lerp( v1, v2, s1 / capsuleLength ), b2MulSV( shape->radius, n ) );
		output.normal = n;
		output.hit = true;
		return output;
	}
}

// Ray vs line segment
b2CastOutput b2RayCastSegment( const b2RayCastInput* input, const b2Segment* shape, bool oneSided )
{
	if ( oneSided )
	{
		// Skip left-side collision
		float offset = b2Cross( b2Sub( input->origin, shape->point1 ), b2Sub( shape->point2, shape->point1 ) );
		if ( offset < 0.0f )
		{
			b2CastOutput output = { 0 };
			return output;
		}
	}

	// Put the ray into the edge's frame of reference.
	b2Vec2 p1 = input->origin;
	b2Vec2 d = input->translation;

	b2Vec2 v1 = shape->point1;
	b2Vec2 v2 = shape->point2;
	b2Vec2 e = b2Sub( v2, v1 );

	b2CastOutput output = { 0 };

	float length;
	b2Vec2 eUnit = b2GetLengthAndNormalize( &length, e );
	if ( length == 0.0f )
	{
		return output;
	}

	// Normal points to the right, looking from v1 towards v2
	b2Vec2 normal = b2RightPerp( eUnit );

	// Intersect ray with infinite segment using normal
	// Similar to intersecting a ray with an infinite plane
	// p = p1 + t * d
	// dot(normal, p - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	float numerator = b2Dot( normal, b2Sub( v1, p1 ) );
	float denominator = b2Dot( normal, d );

	if ( denominator == 0.0f )
	{
		// parallel
		return output;
	}

	float t = numerator / denominator;
	if ( t < 0.0f || input->maxFraction < t )
	{
		// out of ray range
		return output;
	}

	// Intersection point on infinite segment
	b2Vec2 p = b2MulAdd( p1, t, d );

	// Compute position of p along segment
	// p = v1 + s * e
	// s = dot(p - v1, e) / dot(e, e)

	float s = b2Dot( b2Sub( p, v1 ), eUnit );
	if ( s < 0.0f || length < s )
	{
		// out of segment range
		return output;
	}

	if ( numerator > 0.0f )
	{
		normal = b2Neg( normal );
	}

	output.fraction = t;
	output.point = b2MulAdd( p1, t, d );
	output.normal = normal;
	output.hit = true;

	return output;
}

b2CastOutput b2RayCastPolygon( const b2RayCastInput* input, const b2Polygon* shape )
{
	B2_ASSERT( b2IsValidRay( input ) );

	if ( shape->radius == 0.0f )
	{
		// Put the ray into the polygon's frame of reference.
		b2Vec2 p1 = input->origin;
		b2Vec2 d = input->translation;

		float lower = 0.0f, upper = input->maxFraction;

		int32_t index = -1;

		b2CastOutput output = { 0 };

		for ( int32_t i = 0; i < shape->count; ++i )
		{
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			float numerator = b2Dot( shape->normals[i], b2Sub( shape->vertices[i], p1 ) );
			float denominator = b2Dot( shape->normals[i], d );

			if ( denominator == 0.0f )
			{
				if ( numerator < 0.0f )
				{
					return output;
				}
			}
			else
			{
				// Note: we want this predicate without division:
				// lower < numerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < numerator / denominator <==> denominator * lower > numerator.
				if ( denominator < 0.0f && numerator < lower * denominator )
				{
					// Increase lower.
					// The segment enters this half-space.
					lower = numerator / denominator;
					index = i;
				}
				else if ( denominator > 0.0f && numerator < upper * denominator )
				{
					// Decrease upper.
					// The segment exits this half-space.
					upper = numerator / denominator;
				}
			}

			// The use of epsilon here causes the B2_ASSERT on lower to trip
			// in some cases. Apparently the use of epsilon was to make edge
			// shapes work, but now those are handled separately.
			// if (upper < lower - b2_epsilon)
			if ( upper < lower )
			{
				return output;
			}
		}

		B2_ASSERT( 0.0f <= lower && lower <= input->maxFraction );

		if ( index >= 0 )
		{
			output.fraction = lower;
			output.normal = shape->normals[index];
			output.point = b2MulAdd( p1, lower, d );
			output.hit = true;
		}

		return output;
	}

	// TODO_ERIN this is not working for ray vs box (zero radii)
	b2ShapeCastPairInput castInput;
	castInput.proxyA = b2MakeProxy( shape->vertices, shape->count, shape->radius );
	castInput.proxyB = b2MakeProxy( &input->origin, 1, 0.0f );
	castInput.transformA = b2Transform_identity;
	castInput.transformB = b2Transform_identity;
	castInput.translationB = input->translation;
	castInput.maxFraction = input->maxFraction;
	return b2ShapeCast( &castInput );
}

b2CastOutput b2ShapeCastCircle( const b2ShapeCastInput* input, const b2Circle* shape )
{
	b2ShapeCastPairInput pairInput;
	pairInput.proxyA = b2MakeProxy( &shape->center, 1, shape->radius );
	pairInput.proxyB = b2MakeProxy( input->points, input->count, input->radius );
	pairInput.transformA = b2Transform_identity;
	pairInput.transformB = b2Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;

	b2CastOutput output = b2ShapeCast( &pairInput );
	return output;
}

b2CastOutput b2ShapeCastCapsule( const b2ShapeCastInput* input, const b2Capsule* shape )
{
	b2ShapeCastPairInput pairInput;
	pairInput.proxyA = b2MakeProxy( &shape->center1, 2, shape->radius );
	pairInput.proxyB = b2MakeProxy( input->points, input->count, input->radius );
	pairInput.transformA = b2Transform_identity;
	pairInput.transformB = b2Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;

	b2CastOutput output = b2ShapeCast( &pairInput );
	return output;
}

b2CastOutput b2ShapeCastSegment( const b2ShapeCastInput* input, const b2Segment* shape )
{
	b2ShapeCastPairInput pairInput;
	pairInput.proxyA = b2MakeProxy( &shape->point1, 2, 0.0f );
	pairInput.proxyB = b2MakeProxy( input->points, input->count, input->radius );
	pairInput.transformA = b2Transform_identity;
	pairInput.transformB = b2Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;

	b2CastOutput output = b2ShapeCast( &pairInput );
	return output;
}

b2CastOutput b2ShapeCastPolygon( const b2ShapeCastInput* input, const b2Polygon* shape )
{
	b2ShapeCastPairInput pairInput;
	pairInput.proxyA = b2MakeProxy( shape->vertices, shape->count, shape->radius );
	pairInput.proxyB = b2MakeProxy( input->points, input->count, input->radius );
	pairInput.transformA = b2Transform_identity;
	pairInput.transformB = b2Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;

	b2CastOutput output = b2ShapeCast( &pairInput );
	return output;
}
