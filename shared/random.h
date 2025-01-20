// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#define RAND_LIMIT 32767
#define RAND_SEED 12345

// Global seed for simple random number generator.

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t g_seed;
b2Polygon RandomPolygon( float extent );

#ifdef __cplusplus
}
#endif

// Simple random number generator. Using this instead of rand() for cross platform determinism.
B2_INLINE int RandomInt()
{
	// XorShift32 algorithm
	uint32_t x = g_seed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g_seed = x;

	// Map the 32-bit value to the range 0 to RAND_LIMIT
	return (int)( x % ( RAND_LIMIT + 1 ) );
}

// Random integer in range [lo, hi]
B2_INLINE float RandomIntRange( int lo, int hi )
{
	return lo + RandomInt() % ( hi - lo + 1 );
}

// Random number in range [-1,1]
B2_INLINE float RandomFloat()
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

// Random floating point number in range [lo, hi]
B2_INLINE float RandomFloatRange( float lo, float hi )
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = ( hi - lo ) * r + lo;
	return r;
}

// Random vector with coordinates in range [lo, hi]
B2_INLINE b2Vec2 RandomVec2( float lo, float hi )
{
	b2Vec2 v;
	v.x = RandomFloatRange( lo, hi );
	v.y = RandomFloatRange( lo, hi );
	return v;
}
