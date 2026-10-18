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

/// A node in the dynamic tree. Siblings sit together at an even index so two nodes fit in a 64 byte
/// cache line. The root is at index zero and index one is always empty.
typedef struct b2TreeNode
{
	/// The node bounding box
	b2AABB aabb;

	/// In 3D this space is used by the AABB z components.
	uint64_t padding;

	/// - bit 31 : 1 for leaf node
	/// - bit 30 : 1 for moved flag
	/// - bits 0-29 : index of the sibling pair node or the proxy id for a leaf
	uint32_t flagIndex;

	union
	{
		/// The height of an internal node. A leaf has zero height.
		int32_t height;

		/// The shape index for a leaf. Truncated from proxy user data.
		int32_t shapeIndex;
	};

} b2TreeNode;

/// Separate storage for tree leaves.
typedef struct b2TreeProxy
{
	/// User data is an index instead of void* because it is used internally as a shape index.
	uint64_t userData;

	/// Category bits for collision filtering.
	uint64_t categoryBits;

	/// The leaf node. B2_NULL_INDEX for a free proxy.
	int32_t node;

	/// Next free proxy.
	int32_t next;

} b2TreeProxy;

struct b2DynamicTree
{
	/// Array of nodes. The root is at index zero and index 1 is empty. Otherwise siblings are
	/// paired at even indices. Has holes for free node pairs.
	b2TreeNode* nodes;

	/// Parent index per node. The free list is interwoven.
	int32_t* parents;

	/// Proxy data split from node array as cold data.
	b2TreeProxy* proxies;

	/// Every allocated node has a lower index than this.
	int32_t nodeEnd;

	/// The allocated node space
	int32_t nodeCapacity;

	/// Free pairs below nodeEnd
	int32_t pairFreeList;

	/// Number of proxies created
	int32_t proxyCount;

	/// The allocated proxy space
	int32_t proxyCapacity;

	/// Proxy free list
	int32_t proxyFreeList;

	/// Array of nodes for rebuild.
	b2TreeNode* swapNodes;

	/// Leaf indices for rebuild
	int32_t* leafIndices;

	/// Leaves for the rebuild. May represent a proxy or a retained subtree.
	b2TreeNode* leafNodes;

	/// Leaf bounding boxes for rebuild
	b2AABB* leafBoxes;

	/// Leaf bounding box centers for rebuild
	b2Vec2* leafCenters;

	/// Bins for sorting during rebuild
	int32_t* binIndices;

	/// Allocated space for rebuilding
	int32_t rebuildCapacity;

	/// Rebuild orders the nodes so the children follow parents. Cache friendly for queries and
	/// refitting. The order can be disrupted by proxy creation.
	bool dfsOrdered;

};

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

void b2CreateDynamicTreeInternal( b2DynamicTree* tree, int proxyCapacity );
void b2DestroyDynamicTreeInternal( b2DynamicTree* tree );

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
