// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>

/**
 * @defgroup math Math
 * @brief Vector math types and functions
 * @{
 */

/// 2D vector
/// This can be used to represent a point or free vector
typedef struct b2Vec2
{
	/// coordinates
	float x, y;
} b2Vec2;

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
typedef struct b2CosSin
{
	/// cosine and sine
	float cosine;
	float sine;
} b2CosSin;

/// 2D rotation
/// This is similar to using a complex number for rotation
typedef struct b2Rot
{
	/// cosine and sine
	float c, s;
} b2Rot;

/// A 2D rigid transform
typedef struct b2Transform
{
	b2Vec2 p;
	b2Rot q;
} b2Transform;

/// A 2-by-2 Matrix
typedef struct b2Mat22
{
	/// columns
	b2Vec2 cx, cy;
} b2Mat22;

/// Axis-aligned bounding box
typedef struct b2AABB
{
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
} b2AABB;

/// separation = dot(normal, point) - offset
typedef struct b2Plane
{
	b2Vec2 normal;
	float offset;
} b2Plane;

/**@}*/

/**
 * @addtogroup math
 * @{
 */

/// https://en.wikipedia.org/wiki/Pi
#define B2_PI 3.14159265359f

static const b2Vec2 b2Vec2_zero = { 0.0f, 0.0f };
static const b2Rot b2Rot_identity = { 1.0f, 0.0f };
static const b2Transform b2Transform_identity = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };
static const b2Mat22 b2Mat22_zero = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };

/// Is this a valid number? Not NaN or infinity.
B2_API bool b2IsValidFloat( float a );

/// Is this a valid vector? Not NaN or infinity.
B2_API bool b2IsValidVec2( b2Vec2 v );

/// Is this a valid rotation? Not NaN or infinity. Is normalized.
B2_API bool b2IsValidRotation( b2Rot q );

/// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
B2_API bool b2IsValidAABB( b2AABB aabb );

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
B2_API bool b2IsValidPlane( b2Plane a );

/// @return the minimum of two integers
B2_INLINE int b2MinInt( int a, int b )
{
	return a < b ? a : b;
}

/// @return the maximum of two integers
B2_INLINE int b2MaxInt( int a, int b )
{
	return a > b ? a : b;
}

/// @return the absolute value of an integer
B2_INLINE int b2AbsInt( int a )
{
	return a < 0 ? -a : a;
}

/// @return an integer clamped between a lower and upper bound
B2_INLINE int b2ClampInt( int a, int lower, int upper )
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// @return the minimum of two floats
B2_INLINE float b2MinFloat( float a, float b )
{
	return a < b ? a : b;
}

/// @return the maximum of two floats
B2_INLINE float b2MaxFloat( float a, float b )
{
	return a > b ? a : b;
}

/// @return the absolute value of a float
B2_INLINE float b2AbsFloat( float a )
{
	return a < 0 ? -a : a;
}

/// @return a float clamped between a lower and upper bound
B2_INLINE float b2ClampFloat( float a, float lower, float upper )
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees
B2_API float b2Atan2( float y, float x );

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
B2_API b2CosSin b2ComputeCosSin( float radians );

/// Vector dot product
B2_INLINE float b2Dot( b2Vec2 a, b2Vec2 b )
{
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
B2_INLINE float b2Cross( b2Vec2 a, b2Vec2 b )
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
B2_INLINE b2Vec2 b2CrossVS( b2Vec2 v, float s )
{
	return B2_LITERAL( b2Vec2 ){ s * v.y, -s * v.x };
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
B2_INLINE b2Vec2 b2CrossSV( float s, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ -s * v.y, s * v.x };
}

/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
B2_INLINE b2Vec2 b2LeftPerp( b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ -v.y, v.x };
}

/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
B2_INLINE b2Vec2 b2RightPerp( b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ v.y, -v.x };
}

/// Vector addition
B2_INLINE b2Vec2 b2Add( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x + b.x, a.y + b.y };
}

/// Vector subtraction
B2_INLINE b2Vec2 b2Sub( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x - b.x, a.y - b.y };
}

/// Vector negation
B2_INLINE b2Vec2 b2Neg( b2Vec2 a )
{
	return B2_LITERAL( b2Vec2 ){ -a.x, -a.y };
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
B2_INLINE b2Vec2 b2Lerp( b2Vec2 a, b2Vec2 b, float t )
{
	return B2_LITERAL( b2Vec2 ){ ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y };
}

/// Component-wise multiplication
B2_INLINE b2Vec2 b2Mul( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x * b.x, a.y * b.y };
}

/// Multiply a scalar and vector
B2_INLINE b2Vec2 b2MulSV( float s, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ s * v.x, s * v.y };
}

/// a + s * b
B2_INLINE b2Vec2 b2MulAdd( b2Vec2 a, float s, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x + s * b.x, a.y + s * b.y };
}

/// a - s * b
B2_INLINE b2Vec2 b2MulSub( b2Vec2 a, float s, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x - s * b.x, a.y - s * b.y };
}

/// Component-wise absolute vector
B2_INLINE b2Vec2 b2Abs( b2Vec2 a )
{
	b2Vec2 b;
	b.x = b2AbsFloat( a.x );
	b.y = b2AbsFloat( a.y );
	return b;
}

/// Component-wise minimum vector
B2_INLINE b2Vec2 b2Min( b2Vec2 a, b2Vec2 b )
{
	b2Vec2 c;
	c.x = b2MinFloat( a.x, b.x );
	c.y = b2MinFloat( a.y, b.y );
	return c;
}

/// Component-wise maximum vector
B2_INLINE b2Vec2 b2Max( b2Vec2 a, b2Vec2 b )
{
	b2Vec2 c;
	c.x = b2MaxFloat( a.x, b.x );
	c.y = b2MaxFloat( a.y, b.y );
	return c;
}

/// Component-wise clamp vector v into the range [a, b]
B2_INLINE b2Vec2 b2Clamp( b2Vec2 v, b2Vec2 a, b2Vec2 b )
{
	b2Vec2 c;
	c.x = b2ClampFloat( v.x, a.x, b.x );
	c.y = b2ClampFloat( v.y, a.y, b.y );
	return c;
}

/// Get the length of this vector (the norm)
B2_INLINE float b2Length( b2Vec2 v )
{
	return sqrtf( v.x * v.x + v.y * v.y );
}

/// Get the distance between two points
B2_INLINE float b2Distance( b2Vec2 a, b2Vec2 b )
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrtf( dx * dx + dy * dy );
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
/// todo MSVC is not inlining this function in several places per warning 4710
B2_INLINE b2Vec2 b2Normalize( b2Vec2 v )
{
	float length = sqrtf( v.x * v.x + v.y * v.y );
	if ( length < FLT_EPSILON )
	{
		return B2_LITERAL( b2Vec2 ){ 0.0f, 0.0f };
	}

	float invLength = 1.0f / length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

/// Determines if the provided vector is normalized (norm(a) == 1).
B2_INLINE bool b2IsNormalized( b2Vec2 a )
{
	float aa = b2Dot( a, a );
	return b2AbsFloat( 1.0f - aa ) < 100.0f * FLT_EPSILON;
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
/// outputs the length.
B2_INLINE b2Vec2 b2GetLengthAndNormalize( float* length, b2Vec2 v )
{
	*length = sqrtf( v.x * v.x + v.y * v.y );
	if ( *length < FLT_EPSILON )
	{
		return B2_LITERAL( b2Vec2 ){ 0.0f, 0.0f };
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

/// Normalize rotation
B2_INLINE b2Rot b2NormalizeRot( b2Rot q )
{
	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaAngle the angular displacement in radians
B2_INLINE b2Rot b2IntegrateRotation( b2Rot q1, float deltaAngle )
{
	// dc/dt = -omega * sin(t)
	// ds/dt = omega * cos(t)
	// c2 = c1 - omega * h * s1
	// s2 = s1 + omega * h * c1
	b2Rot q2 = { q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c };
	float mag = sqrtf( q2.s * q2.s + q2.c * q2.c );
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = { q2.c * invMag, q2.s * invMag };
	return qn;
}

/// Get the length squared of this vector
B2_INLINE float b2LengthSquared( b2Vec2 v )
{
	return v.x * v.x + v.y * v.y;
}

/// Get the distance squared between points
B2_INLINE float b2DistanceSquared( b2Vec2 a, b2Vec2 b )
{
	b2Vec2 c = { b.x - a.x, b.y - a.y };
	return c.x * c.x + c.y * c.y;
}

/// Make a rotation using an angle in radians
B2_INLINE b2Rot b2MakeRot( float radians )
{
	b2CosSin cs = b2ComputeCosSin( radians );
	return B2_LITERAL( b2Rot ){ cs.cosine, cs.sine };
}

/// Compute the rotation between two unit vectors
B2_API b2Rot b2ComputeRotationBetweenUnitVectors( b2Vec2 v1, b2Vec2 v2 );

/// Is this rotation normalized?
B2_INLINE bool b2IsNormalizedRot( b2Rot q )
{
	// larger tolerance due to failure on mingw 32-bit
	float qq = q.s * q.s + q.c * q.c;
	return 1.0f - 0.0006f < qq && qq < 1.0f + 0.0006f;
}

/// Normalized linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
///	https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
B2_INLINE b2Rot b2NLerp( b2Rot q1, b2Rot q2, float t )
{
	float omt = 1.0f - t;
	b2Rot q = {
		omt * q1.c + t * q2.c,
		omt * q1.s + t * q2.s,
	};

	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// Compute the angular velocity necessary to rotate between two rotations over a give time
/// @param q1 initial rotation
/// @param q2 final rotation
/// @param inv_h inverse time step
B2_INLINE float b2ComputeAngularVelocity( b2Rot q1, b2Rot q2, float inv_h )
{
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1

	// omega * h * s1 = c1 - c2
	// omega * h * c1 = s2 - s1
	// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
	// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
	// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
	float omega = inv_h * ( q2.s * q1.c - q2.c * q1.s );
	return omega;
}

/// Get the angle in radians in the range [-pi, pi]
B2_INLINE float b2Rot_GetAngle( b2Rot q )
{
	return b2Atan2( q.s, q.c );
}

/// Get the x-axis
B2_INLINE b2Vec2 b2Rot_GetXAxis( b2Rot q )
{
	b2Vec2 v = { q.c, q.s };
	return v;
}

/// Get the y-axis
B2_INLINE b2Vec2 b2Rot_GetYAxis( b2Rot q )
{
	b2Vec2 v = { -q.s, q.c };
	return v;
}

/// Multiply two rotations: q * r
B2_INLINE b2Rot b2MulRot( b2Rot q, b2Rot r )
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s(q + r) = qs * rc + qc * rs
	// c(q + r) = qc * rc - qs * rs
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
B2_INLINE b2Rot b2InvMulRot( b2Rot q, b2Rot r )
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s(q - r) = qc * rs - qs * rc
	// c(q - r) = qc * rc + qs * rs
	b2Rot qr;
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

/// relative angle between b and a (rot_b * inv(rot_a))
B2_INLINE float b2RelativeAngle( b2Rot b, b2Rot a )
{
	// sin(b - a) = bs * ac - bc * as
	// cos(b - a) = bc * ac + bs * as
	float s = b.s * a.c - b.c * a.s;
	float c = b.c * a.c + b.s * a.s;
	return b2Atan2( s, c );
}

/// Convert any angle into the range [-pi, pi]
B2_INLINE float b2UnwindAngle( float radians )
{
	// Assuming this is deterministic
	return remainderf( radians, 2.0f * B2_PI );
}

/// Rotate a vector
B2_INLINE b2Vec2 b2RotateVector( b2Rot q, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y };
}

/// Inverse rotate a vector
B2_INLINE b2Vec2 b2InvRotateVector( b2Rot q, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y };
}

/// Transform a point (e.g. local space to world space)
B2_INLINE b2Vec2 b2TransformPoint( b2Transform t, const b2Vec2 p )
{
	float x = ( t.q.c * p.x - t.q.s * p.y ) + t.p.x;
	float y = ( t.q.s * p.x + t.q.c * p.y ) + t.p.y;

	return B2_LITERAL( b2Vec2 ){ x, y };
}

/// Inverse transform a point (e.g. world space to local space)
B2_INLINE b2Vec2 b2InvTransformPoint( b2Transform t, const b2Vec2 p )
{
	float vx = p.x - t.p.x;
	float vy = p.y - t.p.y;
	return B2_LITERAL( b2Vec2 ){ t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy };
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame.
/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
B2_INLINE b2Transform b2MulTransforms( b2Transform A, b2Transform B )
{
	b2Transform C;
	C.q = b2MulRot( A.q, B.q );
	C.p = b2Add( b2RotateVector( A.q, B.p ), A.p );
	return C;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
B2_INLINE b2Transform b2InvMulTransforms( b2Transform A, b2Transform B )
{
	b2Transform C;
	C.q = b2InvMulRot( A.q, B.q );
	C.p = b2InvRotateVector( A.q, b2Sub( B.p, A.p ) );
	return C;
}

/// Multiply a 2-by-2 matrix times a 2D vector
B2_INLINE b2Vec2 b2MulMV( b2Mat22 A, b2Vec2 v )
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

/// Get the inverse of a 2-by-2 matrix
B2_INLINE b2Mat22 b2GetInverse22( b2Mat22 A )
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}

	b2Mat22 B = {
		{ det * d, -det * c },
		{ -det * b, det * a },
	};
	return B;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_INLINE b2Vec2 b2Solve22( b2Mat22 A, b2Vec2 b )
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}
	b2Vec2 x = { det * ( a22 * b.x - a12 * b.y ), det * ( a11 * b.y - a21 * b.x ) };
	return x;
}

/// Does a fully contain b
B2_INLINE bool b2AABB_Contains( b2AABB a, b2AABB b )
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Get the center of the AABB.
B2_INLINE b2Vec2 b2AABB_Center( b2AABB a )
{
	b2Vec2 b = { 0.5f * ( a.lowerBound.x + a.upperBound.x ), 0.5f * ( a.lowerBound.y + a.upperBound.y ) };
	return b;
}

/// Get the extents of the AABB (half-widths).
B2_INLINE b2Vec2 b2AABB_Extents( b2AABB a )
{
	b2Vec2 b = { 0.5f * ( a.upperBound.x - a.lowerBound.x ), 0.5f * ( a.upperBound.y - a.lowerBound.y ) };
	return b;
}

/// Union of two AABBs
B2_INLINE b2AABB b2AABB_Union( b2AABB a, b2AABB b )
{
	b2AABB c;
	c.lowerBound.x = b2MinFloat( a.lowerBound.x, b.lowerBound.x );
	c.lowerBound.y = b2MinFloat( a.lowerBound.y, b.lowerBound.y );
	c.upperBound.x = b2MaxFloat( a.upperBound.x, b.upperBound.x );
	c.upperBound.y = b2MaxFloat( a.upperBound.y, b.upperBound.y );
	return c;
}

/// Do a and b overlap
B2_INLINE bool b2AABB_Overlaps( b2AABB a, b2AABB b )
{
	return !( b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x ||
			  a.lowerBound.y > b.upperBound.y );
}

/// Compute the bounding box of an array of circles
B2_INLINE b2AABB b2MakeAABB( const b2Vec2* points, int count, float radius )
{
	B2_ASSERT( count > 0 );
	b2AABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = b2Min( a.lowerBound, points[i] );
		a.upperBound = b2Max( a.upperBound, points[i] );
	}

	b2Vec2 r = { radius, radius };
	a.lowerBound = b2Sub( a.lowerBound, r );
	a.upperBound = b2Add( a.upperBound, r );

	return a;
}

/// Signed separation of a point from a plane
B2_INLINE float b2PlaneSeparation( b2Plane plane, b2Vec2 point )
{
	return b2Dot( plane.normal, point ) - plane.offset;
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += timeStep * newVelocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
B2_INLINE float b2SpringDamper( float hertz, float dampingRatio, float position, float velocity, float timeStep )
{
	float omega = 2.0f * B2_PI * hertz;
	float omegaH = omega * timeStep;
	return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
}

/// Box2D bases all length units on meters, but you may need different units for your game.
/// You can set this value to use different units. This should be done at application startup
/// and only modified once. Default value is 1.
/// For example, if your game uses pixels for units you can use pixels for all length values
/// sent to Box2D. There should be no extra cost. However, Box2D has some internal tolerances
/// and thresholds that have been tuned for meters. By calling this function, Box2D is able
/// to adjust those tolerances and thresholds to improve accuracy.
/// A good rule of thumb is to pass the height of your player character to this function. So
/// if your player character is 32 pixels high, then pass 32 to this function. Then you may
/// confidently use pixels for all the length values sent to Box2D. All length values returned
/// from Box2D will also be pixels because Box2D does not do any scaling internally.
/// However, you are now on the hook for coming up with good values for gravity, density, and
/// forces.
/// @warning This must be modified before any calls to Box2D
B2_API void b2SetLengthUnitsPerMeter( float lengthUnits );

/// Get the current length units per meter.
B2_API float b2GetLengthUnitsPerMeter( void );

/**@}*/

/**
 * @defgroup math_cpp C++ Math
 * @brief Math operator overloads for C++
 *
 * See math_functions.h for details.
 * @{
 */

#ifdef __cplusplus

/// Unary add one vector to another
inline void operator+=( b2Vec2& a, b2Vec2 b )
{
	a.x += b.x;
	a.y += b.y;
}

/// Unary subtract one vector from another
inline void operator-=( b2Vec2& a, b2Vec2 b )
{
	a.x -= b.x;
	a.y -= b.y;
}

/// Unary multiply a vector by a scalar
inline void operator*=( b2Vec2& a, float b )
{
	a.x *= b;
	a.y *= b;
}

/// Unary negate a vector
inline b2Vec2 operator-( b2Vec2 a )
{
	return { -a.x, -a.y };
}

/// Binary vector addition
inline b2Vec2 operator+( b2Vec2 a, b2Vec2 b )
{
	return { a.x + b.x, a.y + b.y };
}

/// Binary vector subtraction
inline b2Vec2 operator-( b2Vec2 a, b2Vec2 b )
{
	return { a.x - b.x, a.y - b.y };
}

/// Binary scalar and vector multiplication
inline b2Vec2 operator*( float a, b2Vec2 b )
{
	return { a * b.x, a * b.y };
}

/// Binary scalar and vector multiplication
inline b2Vec2 operator*( b2Vec2 a, float b )
{
	return { a.x * b, a.y * b };
}

/// Binary vector equality
inline bool operator==( b2Vec2 a, b2Vec2 b )
{
	return a.x == b.x && a.y == b.y;
}

/// Binary vector inequality
inline bool operator!=( b2Vec2 a, b2Vec2 b )
{
	return a.x != b.x || a.y != b.y;
}

#endif

/**@}*/
