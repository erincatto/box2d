// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/math_functions.h"

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
		.height = 0,
	};
}

static inline bool b2HasTreeMoved( const b2DynamicTree* tree )
{
	return b2IsNodeMoved( tree->nodes + B2_ROOT_NODE );
}

static inline bool b2NeedsRebuild( const b2DynamicTree* tree )
{
	return b2IsNodeMoved( tree->nodes + B2_ROOT_NODE ) || tree->dfsOrdered == false;
}

int b2CreateTreeProxyInternal( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, uint64_t userData,
									   bool markMoved );
void b2DynamicTree_MoveProxyInternal( b2DynamicTree* tree, int proxyId, b2AABB aabb, bool markMoved );
void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb );

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild );

void b2DynamicTree_MarkProxyMovedSerial( b2DynamicTree* tree, int proxyId );
void b2DynamicTree_MarkProxyMoved( b2DynamicTree* tree, int proxyId, b2AABB aabb );
void b2DynamicTree_ClearMoved( b2DynamicTree* tree );
int b2DynamicTree_GatherMovedProxies( const b2DynamicTree* tree, int* proxyIds );
void b2DynamicTree_Refit( b2DynamicTree* tree );

void b2DynamicTree_ValidateNoMoved( const b2DynamicTree* tree );
