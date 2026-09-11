// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box2d/collision.h"

#define B2_TREE_STACK_SIZE 512
#define B2_NODE_SENTINEL ( UINT32_MAX & ~B2_MOVED_NODE )
#define B2_MOVED_NODE ( 1u << 30 )
#define B2_LEAF_NODE ( 1u << 31 )
#define B2_NODE_INDEX_MASK ( 0xFFFFFFFFu & ~( B2_MOVED_NODE | B2_LEAF_NODE ) )
#define B2_ROOT_NODE 0

#define B2_SLOT_BIT 0x00000001u
#define B2_ALLOCATED_BIT 0x00000002u
#define B2_REFIT_BIT 0x00000004u

B2_FORCE_INLINE bool b2IsLeaf( const b2TreeChild* child )
{
	return ( child->flagIndex & B2_LEAF_NODE ) == B2_LEAF_NODE;
}

B2_FORCE_INLINE bool b2IsChildMoved( const b2TreeChild* child )
{
	return ( child->flagIndex & B2_MOVED_NODE ) == B2_MOVED_NODE;
}

B2_FORCE_INLINE uint32_t b2GetChildIndex( const b2TreeChild* child )
{
	return child->flagIndex & B2_NODE_INDEX_MASK;
}

B2_FORCE_INLINE int b2GetChildSlot( const b2TreeLink* link )
{
	return link->flags & B2_SLOT_BIT;
}

B2_FORCE_INLINE bool b2IsAllocated( const b2TreeLink* link )
{
	return (link->flags & B2_ALLOCATED_BIT) == B2_ALLOCATED_BIT;
}

B2_FORCE_INLINE b2TreeChild b2MakeEmptyChild( void )
{
	return (b2TreeChild){
		.aabb = { .lowerBound = { .x = INFINITY, .y = INFINITY }, .upperBound = { .x = -INFINITY, .y = -INFINITY } },
		.flagIndex = B2_NODE_SENTINEL,
		.leafCount = 0,
	};
}

static inline bool b2HasTreeMoved( const b2DynamicTree* tree )
{
	const b2TreeNode* root = tree->nodes + B2_ROOT_NODE;
	return b2IsChildMoved( root->children + 0 ) || b2IsChildMoved( root->children + 1 );
}

// The query box is loaded into a register once and tested against each record in place. Passing
// the record's box by value copies it to the stack as two 8 byte moves that a 16 byte load cannot
// forward from, which is a stall on every node.
#if defined( B2_SIMD_NEON )

#include <arm_neon.h>

typedef float32x4_t b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return vld1q_f32( &aabb->lowerBound.x );
}

B2_FORCE_INLINE bool b2OverlapChild( b2AABBV av, const b2TreeChild* child )
{
	// [lower.x lower.y upper.x upper.y]
	float32x4_t bv = vld1q_f32( &child->aabb.lowerBound.x );

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

#elif defined( B2_SIMD_SSE2 ) || defined( B2_SIMD_AVX2 )

#include <xmmintrin.h>

typedef __m128 b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return _mm_loadu_ps( &aabb->lowerBound.x );
}

B2_FORCE_INLINE bool b2OverlapChild( b2AABBV av, const b2TreeChild* child )
{
	// Unaligned load
	// [lower.x lower.y upper.x upper.y]
	__m128 bv = _mm_loadu_ps( &child->aabb.lowerBound.x );

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

#else

typedef b2AABB b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return *aabb;
}

// Same compares as the SIMD paths, no subtraction, so an inverted empty box fails and the
// scalar build traverses exactly what the SIMD builds do
B2_FORCE_INLINE bool b2OverlapChild( b2AABBV av, const b2TreeChild* child )
{
	const b2AABB* bv = &child->aabb;
	return av.lowerBound.x <= bv->upperBound.x && av.lowerBound.y <= bv->upperBound.y &&
		   bv->lowerBound.x <= av.upperBound.x && bv->lowerBound.y <= av.upperBound.y;
}

B2_FORCE_INLINE b2AABB b2UnionV( b2AABB a, b2AABB b )
{
	return b2AABB_Union( a, b );
}

#endif

void b2DynamicTree_MarkEnlargedFlag( b2DynamicTree* tree, int proxyId );
void b2DynamicTree_MarkEnlarged( b2DynamicTree* tree, int proxyId, b2AABB aabb );
void b2DynamicTree_ClearEnlarged( b2DynamicTree* tree );
void b2DynamicTree_Refit( b2DynamicTree* tree );
