// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

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

#include <xmmintrin.h>
//B2_FORCE_INLINE bool b2OverlapsV( b2AABB a, b2AABB b )
//{
//	// Unaligned load
//	// [lower.x lower.y upper.x upper.y]
//	__m128 av = _mm_loadu_ps( &a.lowerBound.x );
//	__m128 bv = _mm_loadu_ps( &b.lowerBound.x );
//
//	// [alx aly blx bly]
//	__m128 t1 = _mm_movelh_ps( av, bv );
//
//	// [bux buy aux auy]
//	__m128 t2 = _mm_movehl_ps( av, bv );
//
//	__m128 cmp = _mm_cmple_ps( t1, t2 );
//
//	int m = _mm_movemask_ps( cmp );
//	return m == 0xF;
//}

B2_FORCE_INLINE bool b2OverlapChild(__m128 av, const b2TreeChild* child)
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

void b2DynamicTree_MarkEnlargedFlag( b2DynamicTree* tree, int proxyId );
void b2DynamicTree_MarkEnlarged( b2DynamicTree* tree, int proxyId, b2AABB aabb );
void b2DynamicTree_ClearEnlarged( b2DynamicTree* tree );
void b2DynamicTree_Refit( b2DynamicTree* tree );
