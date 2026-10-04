// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#if defined( B2_SIMD_NEON )

#include <arm_neon.h>

typedef float32x4_t b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return vld1q_f32( &aabb->lowerBound.x );
}

B2_FORCE_INLINE bool b2OverlapNode( b2AABBV av, const b2TreeNode* node )
{
	// [lower.x lower.y upper.x upper.y]
	float32x4_t bv = vld1q_f32( &node->aabb.lowerBound.x );

	// [alx aly blx bly]
	float32x4_t t1 = vcombine_f32( vget_low_f32( av ), vget_low_f32( bv ) );

	// [bux buy aux auy]
	float32x4_t t2 = vcombine_f32( vget_high_f32( bv ), vget_high_f32( av ) );

	return vminvq_u32( vcleq_f32( t1, t2 ) ) != 0;
}

B2_FORCE_INLINE bool b2OverlapV( const b2AABB* a, const b2AABB* b )
{
	// [lower.x lower.y upper.x upper.y]
	float32x4_t av = vld1q_f32( &a->lowerBound.x );
	float32x4_t bv = vld1q_f32( &b->lowerBound.x );

	// [alx aly blx bly]
	float32x4_t t1 = vcombine_f32( vget_low_f32( av ), vget_low_f32( bv ) );

	// [bux buy aux auy]
	float32x4_t t2 = vcombine_f32( vget_high_f32( bv ), vget_high_f32( av ) );

	return vminvq_u32( vcleq_f32( t1, t2 ) ) != 0;
}

B2_FORCE_INLINE b2AABB b2UnionV( b2AABB a, b2AABB b )
{
	float32x4_t b1 = vld1q_f32( &a.lowerBound.x );
	float32x4_t b2 = vld1q_f32( &b.lowerBound.x );
	float32x4_t lower = vminq_f32( b1, b2 );
	float32x4_t upper = vmaxq_f32( b1, b2 );
	b2AABB result;
	vst1q_f32( &result.lowerBound.x, vcombine_f32( vget_low_f32( lower ), vget_high_f32( upper ) ) );
	return result;
}

B2_FORCE_INLINE b2AABBV b2UnionPairV( const b2TreeNode* pair )
{
	float32x4_t b1 = vld1q_f32( &pair[0].aabb.lowerBound.x );
	float32x4_t b2 = vld1q_f32( &pair[1].aabb.lowerBound.x );
	return vcombine_f32( vget_low_f32( vminq_f32( b1, b2 ) ), vget_high_f32( vmaxq_f32( b1, b2 ) ) );
}

B2_FORCE_INLINE void b2StoreAABBV( b2AABB* aabb, b2AABBV value, bool condition )
{
	uint32x4_t mask = vdupq_n_u32( condition ? 0xFFFFFFFFu : 0u );
	float32x4_t old = vld1q_f32( &aabb->lowerBound.x );
	vst1q_f32( &aabb->lowerBound.x, vbslq_f32( mask, value, old ) );
}

B2_FORCE_INLINE int b2LaneMaskV( uint32x4_t condition )
{
	static const uint32_t laneBits[4] = { 1, 2, 4, 8 };
	return (int)vaddvq_u32( vandq_u32( condition, vld1q_u32( laneBits ) ) );
}

B2_FORCE_INLINE int b2OverlapPairMask( const b2TreeNode* pairA, const b2TreeNode* pairB )
{
	float32x4_t a0 = vld1q_f32( &pairA[0].aabb.lowerBound.x );
	float32x4_t a1 = vld1q_f32( &pairA[1].aabb.lowerBound.x );
	float32x4_t b0 = vld1q_f32( &pairB[0].aabb.lowerBound.x );
	float32x4_t b1 = vld1q_f32( &pairB[1].aabb.lowerBound.x );

	float32x4_t bLower = vzip1q_f32( b0, b1 );
	float32x4_t bUpper = vzip2q_f32( b0, b1 );
	float32x4_t bLx = vcombine_f32( vget_low_f32( bLower ), vget_low_f32( bLower ) );
	float32x4_t bLy = vcombine_f32( vget_high_f32( bLower ), vget_high_f32( bLower ) );
	float32x4_t bUx = vcombine_f32( vget_low_f32( bUpper ), vget_low_f32( bUpper ) );
	float32x4_t bUy = vcombine_f32( vget_high_f32( bUpper ), vget_high_f32( bUpper ) );

	float32x4_t aLx = vcombine_f32( vdup_laneq_f32( a0, 0 ), vdup_laneq_f32( a1, 0 ) );
	float32x4_t aLy = vcombine_f32( vdup_laneq_f32( a0, 1 ), vdup_laneq_f32( a1, 1 ) );
	float32x4_t aUx = vcombine_f32( vdup_laneq_f32( a0, 2 ), vdup_laneq_f32( a1, 2 ) );
	float32x4_t aUy = vcombine_f32( vdup_laneq_f32( a0, 3 ), vdup_laneq_f32( a1, 3 ) );

	uint32x4_t x = vandq_u32( vcleq_f32( aLx, bUx ), vcleq_f32( bLx, aUx ) );
	uint32x4_t y = vandq_u32( vcleq_f32( aLy, bUy ), vcleq_f32( bLy, aUy ) );
	return b2LaneMaskV( vandq_u32( x, y ) );
}

B2_FORCE_INLINE int b2OverlapNodePairMask( b2AABBV av, const b2TreeNode* pair )
{
	float32x4_t b0 = vld1q_f32( &pair[0].aabb.lowerBound.x );
	float32x4_t b1 = vld1q_f32( &pair[1].aabb.lowerBound.x );

	float32x4_t bLower = vzip1q_f32( b0, b1 );
	float32x4_t bUpper = vzip2q_f32( b0, b1 );
	float32x4_t aLower = vzip1q_f32( av, av );
	float32x4_t aUpper = vzip2q_f32( av, av );

	int mask = b2LaneMaskV( vandq_u32( vcleq_f32( aLower, bUpper ), vcleq_f32( bLower, aUpper ) ) );
	return mask & ( mask >> 2 ) & 3;
}

#elif defined( B2_SIMD_SSE2 ) || defined( B2_SIMD_AVX2 )

#include <emmintrin.h>

typedef __m128 b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return _mm_loadu_ps( &aabb->lowerBound.x );
}

// Passing the tree node rather than the AABB avoids a stack copy.
// todo confirm assembly
B2_FORCE_INLINE bool b2OverlapNode( b2AABBV av, const b2TreeNode* node )
{
	// Unaligned load
	// [lower.x lower.y upper.x upper.y]
	__m128 bv = _mm_loadu_ps( &node->aabb.lowerBound.x );

	// [alx aly blx bly]
	__m128 t1 = _mm_movelh_ps( av, bv );

	// [bux buy aux auy]
	__m128 t2 = _mm_movehl_ps( av, bv );

	return _mm_movemask_ps( _mm_cmple_ps( t1, t2 ) ) == 0xF;
}

B2_FORCE_INLINE bool b2OverlapV( const b2AABB* a, const b2AABB* b )
{
	// Unaligned load
	// [lower.x lower.y upper.x upper.y]
	__m128 av = _mm_loadu_ps( &a->lowerBound.x );
	__m128 bv = _mm_loadu_ps( &b->lowerBound.x );

	// [alx aly blx bly]
	__m128 t1 = _mm_movelh_ps( av, bv );

	// [bux buy aux auy]
	__m128 t2 = _mm_movehl_ps( av, bv );

	return _mm_movemask_ps( _mm_cmple_ps( t1, t2 ) ) == 0xF;
}

B2_FORCE_INLINE b2AABB b2UnionV( b2AABB a, b2AABB b )
{
	// Unaligned load
	// [lower.x lower.y upper.x upper.y]
	__m128 b1 = _mm_loadu_ps( &a.lowerBound.x );
	__m128 b2 = _mm_loadu_ps( &b.lowerBound.x );
	__m128 lower = _mm_min_ps( b1, b2 );
	__m128 upper = _mm_max_ps( b1, b2 );
	__m128 c = _mm_shuffle_ps( lower, upper, _MM_SHUFFLE( 3, 2, 1, 0 ) );
	b2AABB result = { 0 };
	_mm_storeu_ps( &result.lowerBound.x, c );
	return result;
}

B2_FORCE_INLINE b2AABBV b2UnionPairV( const b2TreeNode* pair )
{
	__m128 b1 = _mm_loadu_ps( &pair[0].aabb.lowerBound.x );
	__m128 b2 = _mm_loadu_ps( &pair[1].aabb.lowerBound.x );
	__m128 lower = _mm_min_ps( b1, b2 );
	__m128 upper = _mm_max_ps( b1, b2 );
	return _mm_shuffle_ps( lower, upper, _MM_SHUFFLE( 3, 2, 1, 0 ) );
}

// Conditionally store the value. This is optimized for tree refitting.
B2_FORCE_INLINE void b2StoreAABBV( b2AABB* aabb, b2AABBV value, bool condition )
{
	__m128 mask = _mm_castsi128_ps( _mm_set1_epi32( condition ? -1 : 0 ) );
	__m128 old = _mm_loadu_ps( &aabb->lowerBound.x );

	// blend
	_mm_storeu_ps( &aabb->lowerBound.x, _mm_or_ps( _mm_and_ps( mask, value ), _mm_andnot_ps( mask, old ) ) );
}

B2_FORCE_INLINE int b2OverlapPairMask( const b2TreeNode* pairA, const b2TreeNode* pairB )
{
	__m128 a0 = _mm_loadu_ps( &pairA[0].aabb.lowerBound.x );
	__m128 a1 = _mm_loadu_ps( &pairA[1].aabb.lowerBound.x );
	__m128 b0 = _mm_loadu_ps( &pairB[0].aabb.lowerBound.x );
	__m128 b1 = _mm_loadu_ps( &pairB[1].aabb.lowerBound.x );

	__m128 bLower = _mm_unpacklo_ps( b0, b1 );
	__m128 bUpper = _mm_unpackhi_ps( b0, b1 );
	__m128 bLx = _mm_movelh_ps( bLower, bLower );
	__m128 bLy = _mm_movehl_ps( bLower, bLower );
	__m128 bUx = _mm_movelh_ps( bUpper, bUpper );
	__m128 bUy = _mm_movehl_ps( bUpper, bUpper );

	__m128 aLx = _mm_shuffle_ps( a0, a1, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	__m128 aLy = _mm_shuffle_ps( a0, a1, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	__m128 aUx = _mm_shuffle_ps( a0, a1, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	__m128 aUy = _mm_shuffle_ps( a0, a1, _MM_SHUFFLE( 3, 3, 3, 3 ) );

	__m128 x = _mm_and_ps( _mm_cmple_ps( aLx, bUx ), _mm_cmple_ps( bLx, aUx ) );
	__m128 y = _mm_and_ps( _mm_cmple_ps( aLy, bUy ), _mm_cmple_ps( bLy, aUy ) );
	return _mm_movemask_ps( _mm_and_ps( x, y ) );
}

B2_FORCE_INLINE int b2OverlapNodePairMask( b2AABBV av, const b2TreeNode* pair )
{
	__m128 b0 = _mm_loadu_ps( &pair[0].aabb.lowerBound.x );
	__m128 b1 = _mm_loadu_ps( &pair[1].aabb.lowerBound.x );

	__m128 bLower = _mm_unpacklo_ps( b0, b1 );
	__m128 bUpper = _mm_unpackhi_ps( b0, b1 );
	__m128 aLower = _mm_unpacklo_ps( av, av );
	__m128 aUpper = _mm_unpackhi_ps( av, av );

	int mask = _mm_movemask_ps( _mm_and_ps( _mm_cmple_ps( aLower, bUpper ), _mm_cmple_ps( bLower, aUpper ) ) );
	return mask & ( mask >> 2 ) & 3;
}

#else

typedef b2AABB b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return *aabb;
}

B2_FORCE_INLINE bool b2OverlapNode( b2AABBV av, const b2TreeNode* node )
{
	const b2AABB* bv = &node->aabb;
	return av.lowerBound.x <= bv->upperBound.x && av.lowerBound.y <= bv->upperBound.y && bv->lowerBound.x <= av.upperBound.x &&
		   bv->lowerBound.y <= av.upperBound.y;
}

B2_FORCE_INLINE bool b2OverlapV( const b2AABB* a, const b2AABB* b )
{
	return a->lowerBound.x <= b->upperBound.x && a->lowerBound.y <= b->upperBound.y && b->lowerBound.x <= a->upperBound.x &&
		   b->lowerBound.y <= a->upperBound.y;
}

B2_FORCE_INLINE b2AABB b2UnionV( b2AABB a, b2AABB b )
{
	return b2AABB_Union( a, b );
}

B2_FORCE_INLINE b2AABBV b2UnionPairV( const b2TreeNode* pair )
{
	return b2AABB_Union( pair[0].aabb, pair[1].aabb );
}

B2_FORCE_INLINE void b2StoreAABBV( b2AABB* aabb, b2AABBV value, bool condition )
{
	if ( condition )
	{
		*aabb = value;
	}
}

B2_FORCE_INLINE int b2OverlapPairMask( const b2TreeNode* pairA, const b2TreeNode* pairB )
{
	int mask = 0;
	mask |= b2OverlapV( &pairA[0].aabb, &pairB[0].aabb ) ? 1 : 0;
	mask |= b2OverlapV( &pairA[0].aabb, &pairB[1].aabb ) ? 2 : 0;
	mask |= b2OverlapV( &pairA[1].aabb, &pairB[0].aabb ) ? 4 : 0;
	mask |= b2OverlapV( &pairA[1].aabb, &pairB[1].aabb ) ? 8 : 0;
	return mask;
}

B2_FORCE_INLINE int b2OverlapNodePairMask( b2AABBV av, const b2TreeNode* pair )
{
	int mask = 0;
	mask |= b2OverlapNode( av, pair + 0 ) ? 1 : 0;
	mask |= b2OverlapNode( av, pair + 1 ) ? 2 : 0;
	return mask;
}

#endif

int b2GetSIMDWidth( void );
void b2SetSIMDWidth( int width );
