// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box2d/collision.h"

#define B2_TREE_STACK_SIZE 512

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body

#define B2_MOVED_NODE ( 1u << 30 )
#define B2_LEAF_NODE ( 1u << 31 )
#define B2_NODE_INDEX_MASK ( 0xFFFFFFFFu & ~( B2_MOVED_NODE | B2_LEAF_NODE ) )

// Used to indicate empty nodes. It also passes as a leaf so that internal node
// processing will naturally skip it.
#define B2_EMPTY_NODE ( B2_NODE_INDEX_MASK | B2_LEAF_NODE )

#define B2_ROOT_NODE 0

B2_FORCE_INLINE bool b2IsLeaf( const b2TreeNode* node )
{
	return ( node->flagIndex & B2_LEAF_NODE ) == B2_LEAF_NODE;
}

B2_FORCE_INLINE bool b2IsNodeMoved( const b2TreeNode* node )
{
	return ( node->flagIndex & B2_MOVED_NODE ) == B2_MOVED_NODE;
}

B2_FORCE_INLINE bool b2IsEmptyNode( const b2TreeNode* node )
{
	return node->flagIndex == B2_EMPTY_NODE;
}

B2_FORCE_INLINE int b2GetLeftChild( const b2TreeNode* node )
{
	return (int)( node->flagIndex & B2_NODE_INDEX_MASK );
}

B2_FORCE_INLINE int b2GetProxyId( const b2TreeNode* node )
{
	return (int)( node->flagIndex & B2_NODE_INDEX_MASK );
}

B2_FORCE_INLINE int b2GetRootPair( const b2TreeNode* nodes )
{
	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	return b2IsLeaf( root ) ? B2_ROOT_NODE : b2GetLeftChild( root );
}

B2_FORCE_INLINE b2TreeNode b2MakeEmptyNode( void )
{
	return (b2TreeNode){
		.aabb =
			{
				.lowerBound = { .x = INFINITY, .y = INFINITY },
				.upperBound = { .x = -INFINITY, .y = -INFINITY },
			},
		.flagIndex = B2_EMPTY_NODE,
		.leafCount = 0,
	};
}

static inline bool b2HasTreeMoved( const b2DynamicTree* tree )
{
	return b2IsNodeMoved( tree->nodes + B2_ROOT_NODE );
}

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

#if defined( __aarch64__ ) || defined( _M_ARM64 )
	return vminvq_u32( vcleq_f32( t1, t2 ) ) != 0;
#else
	uint32x4_t mask = vcleq_f32( t1, t2 );
	uint32x2_t pair = vpmin_u32( vget_low_u32( mask ), vget_high_u32( mask ) );
	return vget_lane_u32( vpmin_u32( pair, pair ), 0 ) != 0;
#endif
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

#endif

void b2DynamicTree_MarkProxyMovedSerial( b2DynamicTree* tree, int proxyId );
void b2DynamicTree_MarkProxyMoved( b2DynamicTree* tree, int proxyId, b2AABB aabb );
void b2DynamicTree_ClearMoved( b2DynamicTree* tree );
int b2DynamicTree_GatherMovedProxies( const b2DynamicTree* tree, int* proxyIds );
void b2DynamicTree_Refit( b2DynamicTree* tree );
