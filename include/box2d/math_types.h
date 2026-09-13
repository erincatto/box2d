// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"

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

#if defined( BOX2D_DOUBLE_PRECISION )

/// A world position. Double precision in large world mode so coordinates stay accurate far
/// from the origin.
typedef struct b2Pos
{
	double x, y;
} b2Pos;

/// A world transform with double precision translation and float rotation. Rotation is frame
/// local and never needs the extra range, the same split as Jolt's DMat44.
typedef struct b2WorldTransform
{
	b2Pos p;
	b2Rot q;
} b2WorldTransform;

#else

/// Alias in single precision.
typedef b2Vec2 b2Pos;

/// Alias in single precision.
typedef b2Transform b2WorldTransform;

#endif

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

/// https://en.wikipedia.org/wiki/Pi
#define B2_PI 3.14159265359f

static const b2Vec2 b2Vec2_zero = { 0.0f, 0.0f };
static const b2Rot b2Rot_identity = { 1.0f, 0.0f };
static const b2Transform b2Transform_identity = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };
static const b2Mat22 b2Mat22_zero = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };

// Initializers valid in both modes: 0.0f promotes to double, the identity rotation is float
static const b2Pos b2Pos_zero = { 0.0f, 0.0f };
static const b2WorldTransform b2WorldTransform_identity = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };

/**@}*/
