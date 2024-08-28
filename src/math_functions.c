// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

#include "core.h"

#include <float.h>

//#if defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
//#include <emmintrin.h>
//#else
//#include <arm_neon.h>
//#endif

bool b2IsValid( float a )
{
	if ( isnan( a ) )
	{
		return false;
	}

	if ( isinf( a ) )
	{
		return false;
	}

	return true;
}

bool b2Vec2_IsValid( b2Vec2 v )
{
	if ( isnan( v.x ) || isnan( v.y ) )
	{
		return false;
	}

	if ( isinf( v.x ) || isinf( v.y ) )
	{
		return false;
	}

	return true;
}

bool b2Rot_IsValid( b2Rot q )
{
	if ( isnan( q.s ) || isnan( q.c ) )
	{
		return false;
	}

	if ( isinf( q.s ) || isinf( q.c ) )
	{
		return false;
	}

	return b2IsNormalized( q );
}

#if 0
float b2Sqrt(float x)
{
#if 1
	return sqrtf( x );
#else
#if defined( B2_SIMD_AVX2 ) || defined( B2_SIMD_SSE2 )
	return _mm_cvtss_f32(_mm_sqrt_ss( _mm_set1_ps(x) ));
#elif defined( B2_SIMD_NEON )
	float32x4_t v = vdupq_n_f32( x );
	return vgetq_lane_f32( vsqrtq_f32( v ), 0 );
#else
	return sqrtf( x );
#endif
#endif
}

float b2Length( b2Vec2 v )
{
	return b2Sqrt( v.x * v.x + v.y * v.y );
}

float b2Distance( b2Vec2 a, b2Vec2 b )
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return b2Sqrt( dx * dx + dy * dy );
}

b2Vec2 b2Normalize( b2Vec2 v )
{
	float length = b2Sqrt( v.x * v.x + v.y * v.y );
	if ( length < FLT_EPSILON )
	{
		return b2Vec2_zero;
	}

	float invLength = 1.0f / length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

b2Vec2 b2NormalizeChecked( b2Vec2 v )
{
	float length = b2Length( v );
	if ( length < FLT_EPSILON )
	{
		B2_ASSERT( false );
		return b2Vec2_zero;
	}

	float invLength = 1.0f / length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

b2Vec2 b2GetLengthAndNormalize( float* length, b2Vec2 v )
{
	*length = b2Length( v );
	if ( *length < FLT_EPSILON )
	{
		return b2Vec2_zero;
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

b2Rot b2NormalizeRot( b2Rot q )
{
	float mag = b2Sqrt( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

b2Rot b2IntegrateRotation( b2Rot q1, float deltaAngle )
{
	// dc/dt = -omega * sin(t)
	// ds/dt = omega * cos(t)
	// c2 = c1 - omega * h * s1
	// s2 = s1 + omega * h * c1
	b2Rot q2 = { q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c };
	float mag = b2Sqrt( q2.s * q2.s + q2.c * q2.c );
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = { q2.c * invMag, q2.s * invMag };
	return qn;
}
#endif


#if 0
// From
//-------------------------------------------------------------------------------------
// DirectXMathVector.inl -- SIMD C++ Math library
//
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.
//
// http://go.microsoft.com/fwlink/?LinkID=615560
//-------------------------------------------------------------------------------------

static inline b2FloatW b2ZeroW()
{
#if defined( B2_CPU_ARM )
	return vdupq_n_f32( 0.0f );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_setzero_ps();
#endif
}

static inline b2FloatW b2SplatW(float a)
{
#if defined( B2_CPU_ARM )
	return vdupq_n_f32( a );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_set1_ps(a);
#endif
}

static inline b2FloatW b2SplatIntW(uint32_t a)
{
#if defined( B2_CPU_ARM )
	return vreinterpretq_f32_u32(vdupq_n_u32( a ));
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_castsi128_ps(_mm_set1_epi32(a));
#endif
}

static inline float b2GetX(b2FloatW a )
{
#if defined( B2_CPU_ARM )
	return vgetq_lane_f32( a, 0 );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_cvtss_f32( a );
#endif
}

static inline b2FloatW b2TrueIntW()
{
#if defined( B2_CPU_ARM )
	return vreinterpretq_f32_s32( vdupq_n_s32( -1 ) );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	__m128i v = _mm_set1_epi32( -1 );
	return _mm_castsi128_ps( v );
#endif
}

static inline b2FloatW b2IsInfiniteW( b2FloatW a )
{
#if defined( B2_CPU_ARM )

	uint32x4_t absMask = vdupq_n_u32( 0x7FFFFFFF );
	uint32x4_t infinity = vdupq_n_u32( 0x7F800000 );
	uint32x4_t temp = vandq_u32( vreinterpretq_u32_f32( a ), absMask );
	temp = vceqq_f32( vreinterpretq_f32_u32( temp ), infinity );
	return vreinterpretq_f32_u32( temp );

#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )

	b2FloatW absMask = _mm_castsi128_ps( _mm_set1_epi32( 0x7FFFFFFF ) );
	b2FloatW infinity = _mm_castsi128_ps( _mm_set1_epi32( 0x7F800000 ) );
	b2FloatW temp = _mm_and_ps( a, absMask );
	temp = _mm_cmpeq_ps( temp, infinity );
	return temp;

#endif
}

static inline b2FloatW b2EqualsW( b2FloatW a, b2FloatW b )
{
#if defined( B2_CPU_ARM )
	return vreinterpretq_f32_u32( vceqq_f32( a, b ) );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_cmpeq_ps( a, b );
#endif
}

static inline b2FloatW b2EqualsIntW( b2FloatW a, b2FloatW b )
{
#if defined( B2_CPU_ARM )
	return vreinterpretq_f32_u32( vceqq_s32( vreinterpretq_s32_f32( a ), vreinterpretq_s32_f32( b ) ) );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	__m128i result = _mm_cmpeq_epi32( _mm_castps_si128( a ), _mm_castps_si128( b ) );
	return _mm_castsi128_ps( result );
#endif
}

static inline b2FloatW b2AndW( b2FloatW a, b2FloatW b )
{
#if defined( B2_CPU_ARM )
	return vreinterpretq_f32_u32( vandq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_and_ps( a, b );
#endif
}

static inline b2FloatW b2OrW( b2FloatW a, b2FloatW b )
{
#if defined( B2_CPU_ARM )
	return vreinterpretq_f32_u32( vorq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_or_ps( a, b );
#endif
}

static inline b2FloatW b2SelectW( b2FloatW a, b2FloatW b, b2FloatW mask)
{
#if defined( B2_CPU_ARM )
	return vbslq_f32( vreinterpretq_u32_f32( mask ), b, a );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	b2FloatW temp1 = _mm_andnot_ps( mask, a );
	b2FloatW temp2 = _mm_and_ps( b, mask );
	return _mm_or_ps( temp1, temp2 );
#endif
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
#if defined( B2_CPU_ARM )
	return vaddq_f32( a, b );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_add_ps( a, b );
#endif
}

static inline b2FloatW b2DivW( b2FloatW a, b2FloatW b )
{
#if defined( B2_CPU_ARM )
	return vdivq_f32( a, b );
#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )
	return _mm_div_ps( a, b );
#endif
}

static inline b2FloatW b2ATanW( b2FloatW V )
{
#if defined( B2_CPU_ARM )
	float32x4_t absV = vabsq_f32( V );
	float32x4_t invV = XMVectorReciprocal( V );
	uint32x4_t comp = vcgtq_f32( V, g_XMOne );
	float32x4_t sign = vbslq_f32( comp, g_XMOne, g_XMNegativeOne );
	comp = vcleq_f32( absV, g_XMOne );
	sign = vbslq_f32( comp, g_XMZero, sign );
	float32x4_t x = vbslq_f32( comp, V, invV );

	float32x4_t x2 = vmulq_f32( x, x );

	// Compute polynomial approximation
	const XMVECTOR TC1 = g_XMATanCoefficients1;
	XMVECTOR vConstants = vdupq_lane_f32( vget_high_f32( TC1 ), 0 );
	XMVECTOR Result = vmlaq_lane_f32( vConstants, x2, vget_high_f32( TC1 ), 1 );

	vConstants = vdupq_lane_f32( vget_low_f32( TC1 ), 1 );
	Result = vmlaq_f32( vConstants, Result, x2 );

	vConstants = vdupq_lane_f32( vget_low_f32( TC1 ), 0 );
	Result = vmlaq_f32( vConstants, Result, x2 );

	const XMVECTOR TC0 = g_XMATanCoefficients0;
	vConstants = vdupq_lane_f32( vget_high_f32( TC0 ), 1 );
	Result = vmlaq_f32( vConstants, Result, x2 );

	vConstants = vdupq_lane_f32( vget_high_f32( TC0 ), 0 );
	Result = vmlaq_f32( vConstants, Result, x2 );

	vConstants = vdupq_lane_f32( vget_low_f32( TC0 ), 1 );
	Result = vmlaq_f32( vConstants, Result, x2 );

	vConstants = vdupq_lane_f32( vget_low_f32( TC0 ), 0 );
	Result = vmlaq_f32( vConstants, Result, x2 );

	Result = vmlaq_f32( g_XMOne, Result, x2 );
	Result = vmulq_f32( Result, x );

	float32x4_t result1 = vmulq_f32( sign, g_XMHalfPi );
	result1 = vsubq_f32( result1, Result );

	comp = vceqq_f32( sign, g_XMZero );
	Result = vbslq_f32( comp, Result, result1 );
	return Result;

#elif defined( B2_CPU_X64 ) || defined( B2_CPU_WASM )

	__m128 absV = XMVectorAbs( V );
	__m128 invV = _mm_div_ps( g_XMOne, V );
	__m128 comp = _mm_cmpgt_ps( V, g_XMOne );
	__m128 select0 = _mm_and_ps( comp, g_XMOne );
	__m128 select1 = _mm_andnot_ps( comp, g_XMNegativeOne );
	__m128 sign = _mm_or_ps( select0, select1 );
	comp = _mm_cmple_ps( absV, g_XMOne );
	select0 = _mm_and_ps( comp, g_XMZero );
	select1 = _mm_andnot_ps( comp, sign );
	sign = _mm_or_ps( select0, select1 );
	select0 = _mm_and_ps( comp, V );
	select1 = _mm_andnot_ps( comp, invV );
	__m128 x = _mm_or_ps( select0, select1 );

	__m128 x2 = _mm_mul_ps( x, x );

	// Compute polynomial approximation
	const XMVECTOR TC1 = g_XMATanCoefficients1;
	__m128 vConstantsB = XM_PERMUTE_PS( TC1, _MM_SHUFFLE( 3, 3, 3, 3 ) );
	__m128 vConstants = XM_PERMUTE_PS( TC1, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	__m128 Result = XM_FMADD_PS( vConstantsB, x2, vConstants );

	vConstants = XM_PERMUTE_PS( TC1, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	Result = XM_FMADD_PS( Result, x2, vConstants );

	vConstants = XM_PERMUTE_PS( TC1, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	Result = XM_FMADD_PS( Result, x2, vConstants );

	const XMVECTOR TC0 = g_XMATanCoefficients0;
	vConstants = XM_PERMUTE_PS( TC0, _MM_SHUFFLE( 3, 3, 3, 3 ) );
	Result = XM_FMADD_PS( Result, x2, vConstants );

	vConstants = XM_PERMUTE_PS( TC0, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	Result = XM_FMADD_PS( Result, x2, vConstants );

	vConstants = XM_PERMUTE_PS( TC0, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	Result = XM_FMADD_PS( Result, x2, vConstants );

	vConstants = XM_PERMUTE_PS( TC0, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	Result = XM_FMADD_PS( Result, x2, vConstants );

	Result = XM_FMADD_PS( Result, x2, g_XMOne );

	Result = _mm_mul_ps( Result, x );
	__m128 result1 = _mm_mul_ps( sign, g_XMHalfPi );
	result1 = _mm_sub_ps( result1, Result );

	comp = _mm_cmpeq_ps( sign, g_XMZero );
	select0 = _mm_and_ps( comp, Result );
	select1 = _mm_andnot_ps( comp, result1 );
	Result = _mm_or_ps( select0, select1 );
	return Result;
#endif
}

float b2ATan2(float y, float x)
{
	b2FloatW zero = b2ZeroW();
	b2FloatW atanResultValid = b2TrueIntW();

	b2FloatW pi = b2SplatW( b2_pi );
	b2FloatW piOverTwo = b2SplatW( 0.5f * b2_pi );
	b2FloatW piOverFour = b2SplatW( 0.25f * b2_pi );
	b2FloatW threePiOverFour = b2SplatW( 0.75f * b2_pi );

	b2FloatW yv = b2SplatW( y );
	b2FloatW xv = b2SplatW( x );
	b2FloatW yEqualsZero = b2EqualsW( yv, zero );
	b2FloatW xEqualsZero = b2EqualsW( xv, zero );
	b2FloatW signMask = b2SplatIntW( 0x8000000 );
	b2FloatW xIsPositive = b2AndW(xv, signMask );
	xIsPositive = b2EqualsIntW(xIsPositive, zero  );

	b2FloatW yEqualsInfinity = b2IsInfiniteW( yv );
	b2FloatW xEqualsInfinity = b2IsInfiniteW( xv );
	
	b2FloatW ySign = b2AndW( yv, signMask );
	pi = b2OrW( pi , ySign );
	piOverTwo = b2OrW( piOverTwo , ySign );
	piOverFour = b2OrW( piOverFour , ySign );
	threePiOverFour = b2OrW( threePiOverFour, ySign );

	b2FloatW r1 = b2SelectW( pi, ySign, xIsPositive );
	b2FloatW r2 = b2SelectW( atanResultValid, piOverTwo, xEqualsZero );
	b2FloatW r3 = b2SelectW( r2, r1, yEqualsZero );
	b2FloatW r4 = b2SelectW( threePiOverFour, piOverFour, xIsPositive );
	b2FloatW r5 = b2SelectW( piOverTwo, r4, xEqualsInfinity );
	b2FloatW result = b2SelectW( r3, r5, yEqualsInfinity );
	atanResultValid = b2EqualsIntW(  result, atanResultValid );

	b2FloatW v = b2DivW( yv, xv );
	b2FloatW r0 = b2AtanW( v );

	r1 = b2SelectW( pi, signMask, xIsPositive );
	r2 = b2AddW( r0, r1 );

	b2FloatW finalResult = b2SelectW( result, r1, atanResultValid );
	return b2GetX( finalResult );
}
#else

// https://mazzo.li/posts/vectorized-atan2.html
static inline float b2Atan( float x )
{
	float a1 = 0.99997726f;
	float a3 = -0.33262347f;
	float a5 = 0.19354346f;
	float a7 = -0.11643287f;
	float a9 = 0.05265332f;
	float a11 = -0.01172120f;

	float x2 = x * x;
	return x * ( a1 + x2 * ( a3 + x2 * ( a5 + x2 * ( a7 + x2 * ( a9 + x2 * a11 ) ) ) ) );
}

float b2Atan2( float y, float x )
{
	float pi = b2_pi;
	float halfPi = 0.5f * b2_pi;

	bool swap = b2AbsFloat( x ) < b2AbsFloat( y );
	float atanInput = ( swap ? x : y ) / ( swap ? y : x );

	// Approximate atan
	float res = b2Atan( atanInput );

	// If swapped, adjust atan output
	res = swap ? ( atanInput >= 0.0f ? halfPi : -halfPi ) - res : res;
	// Adjust quadrants
	if ( x >= 0.0f && y >= 0.0f )
	{
	} // 1st quadrant
	else if ( x < 0.0f && y >= 0.0f )
	{
		res = pi + res;
	} // 2nd quadrant
	else if ( x < 0.0f && y < 0.0f )
	{
		res = -pi + res;
	} // 3rd quadrant
	else if ( x >= 0.0f && y < 0.0f )
	{
	} // 4th quadrant

	return res;
}

// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b2Rot b2MakeRot(float angle)
{
#if 1
	return ( b2Rot ){ cosf( angle ), sinf( angle ) };
#else
	float x = b2UnwindLargeAngle( angle );
	float pi2 = b2_pi * b2_pi;

	b2Rot q;

	// cosine needs angle in [-pi/2, pi/2]
	if (x < -0.5f * b2_pi)
	{
		float y = x + b2_pi;
		float y2 = y * y;
		q.c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else if (x > 0.5f * b2_pi)
	{
		float y = x - b2_pi;
		float y2 = y * y;
		q.c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else
	{
		float y2 = x * x;
		q.c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	if (x < 0.0f)
	{
		float y = x + b2_pi;
		q.s = -16.0f * y * ( b2_pi - y ) / ( 5.0f * pi2 - 4.0f * y * ( b2_pi - y ) );
	}
	else
	{
		q.s = 16.0f * x * ( b2_pi - x ) / ( 5.0f * pi2 - 4.0f * x * ( b2_pi - x ) );
	}

	q = b2NormalizeRot( q );
	return q;
#endif
}
#endif
