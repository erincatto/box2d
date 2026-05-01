// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "core.h"
#include "dynamic_tree.h"
#include "parallel_for.h"
#include "physics_world.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <string.h>

#define B2_TREE_STACK_SIZE 1024

enum b2TreeNodeFlags
{
	b2_allocatedNode = 0x0001,
	b2_enlargedNode = 0x0002,
	b2_leafNode = 0x0004,
};

// A node in the dynamic tree.
// todo externalize this to visualize internal nodes and speed up FindPairs
typedef struct b2TreeNode
{
	// The node bounding box
	b2AABB aabb; // 16

	// Category bits for collision filtering
	uint64_t categoryBits; // 8

	union
	{
		// Children (internal node)
		struct
		{
			int32_t child1, child2;
		} children;

		/// User data (leaf node)
		uint64_t userData;
	}; // 8

	union
	{
		/// The node parent index (allocated node)
		int32_t parent;

		/// The node freelist next index (free node)
		int32_t next;
	}; // 4

	uint16_t height; // 2
	uint16_t flags;	 // 2
} b2TreeNode;

static b2TreeNode b2_defaultTreeNode = {
	.aabb = { { 0.0f, 0.0f }, { 0.0f, 0.0f } },
	.categoryBits = B2_DEFAULT_CATEGORY_BITS,
	.children =
		{
			.child1 = B2_NULL_INDEX,
			.child2 = B2_NULL_INDEX,
		},
	.parent = B2_NULL_INDEX,
	.height = 0,
	.flags = b2_allocatedNode,
};

static bool b2IsLeaf( const b2TreeNode* node )
{
	return node->flags & b2_leafNode;
}

static bool b2IsAllocated( const b2TreeNode* node )
{
	return node->flags & b2_allocatedNode;
}

static uint16_t b2MaxUInt16( uint16_t a, uint16_t b )
{
	return a > b ? a : b;
}

b2DynamicTree b2DynamicTree_Create( int proxyCapacity )
{
	int capacity = b2MaxInt( proxyCapacity, 16 );

	b2DynamicTree tree;

	// memset needed for deterministic serialization
	memset( &tree, 0, sizeof( b2DynamicTree ) );

	tree.root = B2_NULL_INDEX;

	// maximum node count for a full binary tree is 2 * leafCount - 1
	tree.nodeCapacity = 2 * capacity - 1;
	tree.nodeCount = 0;
	tree.nodes = (b2TreeNode*)b2Alloc( tree.nodeCapacity * sizeof( b2TreeNode ) );

	// todo eliminate this memset
	memset( tree.nodes, 0, tree.nodeCapacity * sizeof( b2TreeNode ) );

	// Build a linked list for the free list.
	// todo use a bump allocation scheme to avoid this work
	for ( int i = 0; i < tree.nodeCapacity - 1; ++i )
	{
		tree.nodes[i].next = i + 1;
	}

	tree.nodes[tree.nodeCapacity - 1].next = B2_NULL_INDEX;
	tree.freeList = 0;

	tree.proxyCount = 0;

	tree.leafIndices = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
	tree.leafIndicesAlt = NULL;
	tree.leafCentersAlt = NULL;
	tree.binIndices = NULL;
	tree.rebuildCapacity = 0;

	return tree;
}

void b2DynamicTree_Destroy( b2DynamicTree* tree )
{
	b2Free( tree->nodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
	b2Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b2AABB ) );
	b2Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b2Vec2 ) );
	b2Free( tree->leafIndicesAlt, tree->rebuildCapacity * sizeof( int32_t ) );
	b2Free( tree->leafCentersAlt, tree->rebuildCapacity * sizeof( b2Vec2 ) );
	b2Free( tree->binIndices, tree->rebuildCapacity * sizeof( int32_t ) );

	memset( tree, 0, sizeof( b2DynamicTree ) );
}

// Allocate a node from the pool. Grow the pool if necessary.
static int b2AllocateNode( b2DynamicTree* tree )
{
	// Expand the node pool as needed.
	if ( tree->freeList == B2_NULL_INDEX )
	{
		B2_ASSERT( tree->nodeCount == tree->nodeCapacity );

		// The free list is empty. Rebuild a bigger pool.
		b2TreeNode* oldNodes = tree->nodes;
		int oldCapacity = tree->nodeCapacity;
		tree->nodeCapacity += oldCapacity >> 1;
		tree->nodes = (b2TreeNode*)b2Alloc( tree->nodeCapacity * sizeof( b2TreeNode ) );
		B2_ASSERT( oldNodes != NULL );
		memcpy( tree->nodes, oldNodes, tree->nodeCount * sizeof( b2TreeNode ) );

		// todo eliminate this memset
		memset( tree->nodes + tree->nodeCount, 0, ( tree->nodeCapacity - tree->nodeCount ) * sizeof( b2TreeNode ) );

		b2Free( oldNodes, oldCapacity * sizeof( b2TreeNode ) );

		// Build a linked list for the free list. The parent pointer becomes the "next" pointer.
		// todo avoid building freelist using bump allocator
		for ( int i = tree->nodeCount; i < tree->nodeCapacity - 1; ++i )
		{
			tree->nodes[i].next = i + 1;
		}

		tree->nodes[tree->nodeCapacity - 1].next = B2_NULL_INDEX;
		tree->freeList = tree->nodeCount;
	}

	// Peel a node off the free list.
	int nodeIndex = tree->freeList;
	b2TreeNode* node = tree->nodes + nodeIndex;
	tree->freeList = node->next;
	*node = b2_defaultTreeNode;
	++tree->nodeCount;
	return nodeIndex;
}

// Return a node to the pool.
static void b2FreeNode( b2DynamicTree* tree, int nodeId )
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	B2_ASSERT( 0 < tree->nodeCount );
	tree->nodes[nodeId].next = tree->freeList;
	tree->nodes[nodeId].flags = 0;
	tree->freeList = nodeId;
	--tree->nodeCount;
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
static int b2FindBestSibling( const b2DynamicTree* tree, b2AABB boxD )
{
	b2Vec2 centerD = b2AABB_Center( boxD );
	float areaD = b2Perimeter( boxD );

	const b2TreeNode* nodes = tree->nodes;
	int rootIndex = tree->root;

	b2AABB rootBox = nodes[rootIndex].aabb;

	// Area of current node
	float areaBase = b2Perimeter( rootBox );

	// Area of inflated node
	float directCost = b2Perimeter( b2AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;

	int bestSibling = rootIndex;
	float bestCost = directCost;

	// Descend the tree from root, following a single greedy path.
	int index = rootIndex;
	while ( nodes[index].height > 0 )
	{
		int child1 = nodes[index].children.child1;
		int child2 = nodes[index].children.child2;

		// Cost of creating a new parent for this node and the new leaf
		float cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( cost < bestCost )
		{
			bestSibling = index;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		bool leaf1 = nodes[child1].height == 0;
		bool leaf2 = nodes[child2].height == 0;

		// Cost of descending into child 1
		float lowerCost1 = FLT_MAX;
		b2AABB box1 = nodes[child1].aabb;
		float directCost1 = b2Perimeter( b2AABB_Union( box1, boxD ) );
		float area1 = 0.0f;
		if ( leaf1 )
		{
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			float cost1 = directCost1 + inheritedCost;

			// Need this here due to while condition above
			if ( cost1 < bestCost )
			{
				bestSibling = child1;
				bestCost = cost1;
			}
		}
		else
		{
			// Child 1 is an internal node
			area1 = b2Perimeter( box1 );

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + b2MinFloat( areaD - area1, 0.0f );
		}

		// Cost of descending into child 2
		float lowerCost2 = FLT_MAX;
		b2AABB box2 = nodes[child2].aabb;
		float directCost2 = b2Perimeter( b2AABB_Union( box2, boxD ) );
		float area2 = 0.0f;
		if ( leaf2 )
		{
			float cost2 = directCost2 + inheritedCost;

			if ( cost2 < bestCost )
			{
				bestSibling = child2;
				bestCost = cost2;
			}
		}
		else
		{
			area2 = b2Perimeter( box2 );
			lowerCost2 = inheritedCost + directCost2 + b2MinFloat( areaD - area2, 0.0f );
		}

		if ( leaf1 && leaf2 )
		{
			break;
		}

		// Can the cost possibly be decreased?
		if ( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
		{
			break;
		}

		if ( lowerCost1 == lowerCost2 && leaf1 == false )
		{
			B2_ASSERT( lowerCost1 < FLT_MAX );
			B2_ASSERT( lowerCost2 < FLT_MAX );

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			b2Vec2 d1 = b2Sub( b2AABB_Center( box1 ), centerD );
			b2Vec2 d2 = b2Sub( b2AABB_Center( box2 ), centerD );
			lowerCost1 = b2LengthSquared( d1 );
			lowerCost2 = b2LengthSquared( d2 );
		}

		// Descend
		if ( lowerCost1 < lowerCost2 && leaf1 == false )
		{
			index = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			index = child2;
			areaBase = area2;
			directCost = directCost2;
		}

		B2_ASSERT( nodes[index].height > 0 );
	}

	return bestSibling;
}

enum b2RotateType
{
	b2_rotateNone,
	b2_rotateBF,
	b2_rotateBG,
	b2_rotateCD,
	b2_rotateCE
};

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
static void b2RotateNodes( b2DynamicTree* tree, int iA )
{
	B2_ASSERT( iA != B2_NULL_INDEX );

	b2TreeNode* nodes = tree->nodes;

	b2TreeNode* A = nodes + iA;
	if ( A->height < 2 )
	{
		return;
	}

	int iB = A->children.child1;
	int iC = A->children.child2;
	B2_ASSERT( 0 <= iB && iB < tree->nodeCapacity );
	B2_ASSERT( 0 <= iC && iC < tree->nodeCapacity );

	b2TreeNode* B = nodes + iB;
	b2TreeNode* C = nodes + iC;

	if ( B->height == 0 )
	{
		// B is a leaf and C is internal
		B2_ASSERT( C->height > 0 );

		int iF = C->children.child1;
		int iG = C->children.child2;
		b2TreeNode* F = nodes + iF;
		b2TreeNode* G = nodes + iG;
		B2_ASSERT( 0 <= iF && iF < tree->nodeCapacity );
		B2_ASSERT( 0 <= iG && iG < tree->nodeCapacity );

		// Base cost
		float costBase = b2Perimeter( C->aabb );

		// Cost of swapping B and F
		b2AABB aabbBG = b2AABB_Union( B->aabb, G->aabb );
		float costBF = b2Perimeter( aabbBG );

		// Cost of swapping B and G
		b2AABB aabbBF = b2AABB_Union( B->aabb, F->aabb );
		float costBG = b2Perimeter( aabbBF );

		if ( costBase < costBF && costBase < costBG )
		{
			// Rotation does not improve cost
			return;
		}

		if ( costBF < costBG )
		{
			// Swap B and F
			A->children.child1 = iF;
			C->children.child1 = iB;

			B->parent = iC;
			F->parent = iA;

			C->aabb = aabbBG;

			C->height = 1 + b2MaxUInt16( B->height, G->height );
			A->height = 1 + b2MaxUInt16( C->height, F->height );
			C->categoryBits = B->categoryBits | G->categoryBits;
			A->categoryBits = C->categoryBits | F->categoryBits;
			C->flags |= ( B->flags | G->flags ) & b2_enlargedNode;
			A->flags |= ( C->flags | F->flags ) & b2_enlargedNode;
		}
		else
		{
			// Swap B and G
			A->children.child1 = iG;
			C->children.child2 = iB;

			B->parent = iC;
			G->parent = iA;

			C->aabb = aabbBF;

			C->height = 1 + b2MaxUInt16( B->height, F->height );
			A->height = 1 + b2MaxUInt16( C->height, G->height );
			C->categoryBits = B->categoryBits | F->categoryBits;
			A->categoryBits = C->categoryBits | G->categoryBits;
			C->flags |= ( B->flags | F->flags ) & b2_enlargedNode;
			A->flags |= ( C->flags | G->flags ) & b2_enlargedNode;
		}
	}
	else if ( C->height == 0 )
	{
		// C is a leaf and B is internal
		B2_ASSERT( B->height > 0 );

		int iD = B->children.child1;
		int iE = B->children.child2;
		b2TreeNode* D = nodes + iD;
		b2TreeNode* E = nodes + iE;
		B2_ASSERT( 0 <= iD && iD < tree->nodeCapacity );
		B2_ASSERT( 0 <= iE && iE < tree->nodeCapacity );

		// Base cost
		float costBase = b2Perimeter( B->aabb );

		// Cost of swapping C and D
		b2AABB aabbCE = b2AABB_Union( C->aabb, E->aabb );
		float costCD = b2Perimeter( aabbCE );

		// Cost of swapping C and E
		b2AABB aabbCD = b2AABB_Union( C->aabb, D->aabb );
		float costCE = b2Perimeter( aabbCD );

		if ( costBase < costCD && costBase < costCE )
		{
			// Rotation does not improve cost
			return;
		}

		if ( costCD < costCE )
		{
			// Swap C and D
			A->children.child2 = iD;
			B->children.child1 = iC;

			C->parent = iB;
			D->parent = iA;

			B->aabb = aabbCE;

			B->height = 1 + b2MaxUInt16( C->height, E->height );
			A->height = 1 + b2MaxUInt16( B->height, D->height );
			B->categoryBits = C->categoryBits | E->categoryBits;
			A->categoryBits = B->categoryBits | D->categoryBits;
			B->flags |= ( C->flags | E->flags ) & b2_enlargedNode;
			A->flags |= ( B->flags | D->flags ) & b2_enlargedNode;
		}
		else
		{
			// Swap C and E
			A->children.child2 = iE;
			B->children.child2 = iC;

			C->parent = iB;
			E->parent = iA;

			B->aabb = aabbCD;
			B->height = 1 + b2MaxUInt16( C->height, D->height );
			A->height = 1 + b2MaxUInt16( B->height, E->height );
			B->categoryBits = C->categoryBits | D->categoryBits;
			A->categoryBits = B->categoryBits | E->categoryBits;
			B->flags |= ( C->flags | D->flags ) & b2_enlargedNode;
			A->flags |= ( B->flags | E->flags ) & b2_enlargedNode;
		}
	}
	else
	{
		int iD = B->children.child1;
		int iE = B->children.child2;
		int iF = C->children.child1;
		int iG = C->children.child2;

		B2_ASSERT( 0 <= iD && iD < tree->nodeCapacity );
		B2_ASSERT( 0 <= iE && iE < tree->nodeCapacity );
		B2_ASSERT( 0 <= iF && iF < tree->nodeCapacity );
		B2_ASSERT( 0 <= iG && iG < tree->nodeCapacity );

		b2TreeNode* D = nodes + iD;
		b2TreeNode* E = nodes + iE;
		b2TreeNode* F = nodes + iF;
		b2TreeNode* G = nodes + iG;

		// Base cost
		float areaB = b2Perimeter( B->aabb );
		float areaC = b2Perimeter( C->aabb );
		float costBase = areaB + areaC;
		enum b2RotateType bestRotation = b2_rotateNone;
		float bestCost = costBase;

		// Cost of swapping B and F
		b2AABB aabbBG = b2AABB_Union( B->aabb, G->aabb );
		float costBF = areaB + b2Perimeter( aabbBG );
		if ( costBF < bestCost )
		{
			bestRotation = b2_rotateBF;
			bestCost = costBF;
		}

		// Cost of swapping B and G
		b2AABB aabbBF = b2AABB_Union( B->aabb, F->aabb );
		float costBG = areaB + b2Perimeter( aabbBF );
		if ( costBG < bestCost )
		{
			bestRotation = b2_rotateBG;
			bestCost = costBG;
		}

		// Cost of swapping C and D
		b2AABB aabbCE = b2AABB_Union( C->aabb, E->aabb );
		float costCD = areaC + b2Perimeter( aabbCE );
		if ( costCD < bestCost )
		{
			bestRotation = b2_rotateCD;
			bestCost = costCD;
		}

		// Cost of swapping C and E
		b2AABB aabbCD = b2AABB_Union( C->aabb, D->aabb );
		float costCE = areaC + b2Perimeter( aabbCD );
		if ( costCE < bestCost )
		{
			bestRotation = b2_rotateCE;
			// bestCost = costCE;
		}

		switch ( bestRotation )
		{
			case b2_rotateNone:
				break;

			case b2_rotateBF:
				A->children.child1 = iF;
				C->children.child1 = iB;

				B->parent = iC;
				F->parent = iA;

				C->aabb = aabbBG;
				C->height = 1 + b2MaxUInt16( B->height, G->height );
				A->height = 1 + b2MaxUInt16( C->height, F->height );
				C->categoryBits = B->categoryBits | G->categoryBits;
				A->categoryBits = C->categoryBits | F->categoryBits;
				C->flags |= ( B->flags | G->flags ) & b2_enlargedNode;
				A->flags |= ( C->flags | F->flags ) & b2_enlargedNode;
				break;

			case b2_rotateBG:
				A->children.child1 = iG;
				C->children.child2 = iB;

				B->parent = iC;
				G->parent = iA;

				C->aabb = aabbBF;
				C->height = 1 + b2MaxUInt16( B->height, F->height );
				A->height = 1 + b2MaxUInt16( C->height, G->height );
				C->categoryBits = B->categoryBits | F->categoryBits;
				A->categoryBits = C->categoryBits | G->categoryBits;
				C->flags |= ( B->flags | F->flags ) & b2_enlargedNode;
				A->flags |= ( C->flags | G->flags ) & b2_enlargedNode;
				break;

			case b2_rotateCD:
				A->children.child2 = iD;
				B->children.child1 = iC;

				C->parent = iB;
				D->parent = iA;

				B->aabb = aabbCE;
				B->height = 1 + b2MaxUInt16( C->height, E->height );
				A->height = 1 + b2MaxUInt16( B->height, D->height );
				B->categoryBits = C->categoryBits | E->categoryBits;
				A->categoryBits = B->categoryBits | D->categoryBits;
				B->flags |= ( C->flags | E->flags ) & b2_enlargedNode;
				A->flags |= ( B->flags | D->flags ) & b2_enlargedNode;
				break;

			case b2_rotateCE:
				A->children.child2 = iE;
				B->children.child2 = iC;

				C->parent = iB;
				E->parent = iA;

				B->aabb = aabbCD;
				B->height = 1 + b2MaxUInt16( C->height, D->height );
				A->height = 1 + b2MaxUInt16( B->height, E->height );
				B->categoryBits = C->categoryBits | D->categoryBits;
				A->categoryBits = B->categoryBits | E->categoryBits;
				B->flags |= ( C->flags | D->flags ) & b2_enlargedNode;
				A->flags |= ( B->flags | E->flags ) & b2_enlargedNode;
				break;

			default:
				B2_ASSERT( false );
				break;
		}
	}
}

static void b2InsertLeaf( b2DynamicTree* tree, int leaf, bool shouldRotate )
{
	if ( tree->root == B2_NULL_INDEX )
	{
		tree->root = leaf;
		tree->nodes[tree->root].parent = B2_NULL_INDEX;
		return;
	}

	// Stage 1: find the best sibling for this node
	b2AABB leafAABB = tree->nodes[leaf].aabb;
	int sibling = b2FindBestSibling( tree, leafAABB );

	// Stage 2: create a new parent for the leaf and sibling
	int oldParent = tree->nodes[sibling].parent;
	int newParent = b2AllocateNode( tree );

	// Warning: node pointer can change after allocation
	b2TreeNode* nodes = tree->nodes;
	nodes[newParent].parent = oldParent;
	nodes[newParent].userData = UINT64_MAX;
	nodes[newParent].aabb = b2AABB_Union( leafAABB, nodes[sibling].aabb );
	nodes[newParent].categoryBits = nodes[leaf].categoryBits | nodes[sibling].categoryBits;
	nodes[newParent].height = nodes[sibling].height + 1;
	nodes[newParent].children.child1 = sibling;
	nodes[newParent].children.child2 = leaf;
	nodes[sibling].parent = newParent;
	nodes[leaf].parent = newParent;

	// Fix grandparent links
	if ( oldParent != B2_NULL_INDEX )
	{
		// The sibling was not the root
		if ( nodes[oldParent].children.child1 == sibling )
		{
			nodes[oldParent].children.child1 = newParent;
		}
		else
		{
			B2_ASSERT( nodes[oldParent].children.child2 == sibling );
			nodes[oldParent].children.child2 = newParent;
		}
	}
	else
	{
		// The sibling was the root
		tree->root = newParent;
	}

	// Stage 3: walk back up the tree fixing heights and AABBs
	int index = nodes[leaf].parent;
	while ( index != B2_NULL_INDEX )
	{
		int child1 = nodes[index].children.child1;
		int child2 = nodes[index].children.child2;

		B2_ASSERT( child1 != B2_NULL_INDEX );
		B2_ASSERT( child2 != B2_NULL_INDEX );

		nodes[index].aabb = b2AABB_Union( nodes[child1].aabb, nodes[child2].aabb );
		nodes[index].categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;
		nodes[index].height = 1 + b2MaxUInt16( nodes[child1].height, nodes[child2].height );
		nodes[index].flags |= ( nodes[child1].flags | nodes[child2].flags ) & b2_enlargedNode;

		if ( shouldRotate )
		{
			b2RotateNodes( tree, index );
		}

		index = nodes[index].parent;
	}
}

static void b2RemoveLeaf( b2DynamicTree* tree, int leaf )
{
	if ( leaf == tree->root )
	{
		tree->root = B2_NULL_INDEX;
		return;
	}

	b2TreeNode* nodes = tree->nodes;

	int parent = nodes[leaf].parent;
	int grandParent = nodes[parent].parent;
	int sibling;
	if ( nodes[parent].children.child1 == leaf )
	{
		sibling = nodes[parent].children.child2;
	}
	else
	{
		sibling = nodes[parent].children.child1;
	}

	if ( grandParent != B2_NULL_INDEX )
	{
		// Destroy parent and connect sibling to grandParent.
		if ( nodes[grandParent].children.child1 == parent )
		{
			nodes[grandParent].children.child1 = sibling;
		}
		else
		{
			nodes[grandParent].children.child2 = sibling;
		}
		nodes[sibling].parent = grandParent;
		b2FreeNode( tree, parent );

		// Adjust ancestor bounds.
		int index = grandParent;
		while ( index != B2_NULL_INDEX )
		{
			b2TreeNode* node = nodes + index;
			b2TreeNode* child1 = nodes + node->children.child1;
			b2TreeNode* child2 = nodes + node->children.child2;

			// Fast union using SSE
			//__m128 aabb1 = _mm_load_ps(&child1->aabb.lowerBound.x);
			//__m128 aabb2 = _mm_load_ps(&child2->aabb.lowerBound.x);
			//__m128 lower = _mm_min_ps(aabb1, aabb2);
			//__m128 upper = _mm_max_ps(aabb1, aabb2);
			//__m128 aabb = _mm_shuffle_ps(lower, upper, _MM_SHUFFLE(3, 2, 1, 0));
			//_mm_store_ps(&node->aabb.lowerBound.x, aabb);

			node->aabb = b2AABB_Union( child1->aabb, child2->aabb );
			node->categoryBits = child1->categoryBits | child2->categoryBits;
			node->height = 1 + b2MaxUInt16( child1->height, child2->height );

			index = node->parent;
		}
	}
	else
	{
		tree->root = sibling;
		tree->nodes[sibling].parent = B2_NULL_INDEX;
		b2FreeNode( tree, parent );
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
int b2DynamicTree_CreateProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, uint64_t userData )
{
	B2_ASSERT( -B2_HUGE < aabb.lowerBound.x && aabb.lowerBound.x < B2_HUGE );
	B2_ASSERT( -B2_HUGE < aabb.lowerBound.y && aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( -B2_HUGE < aabb.upperBound.x && aabb.upperBound.x < B2_HUGE );
	B2_ASSERT( -B2_HUGE < aabb.upperBound.y && aabb.upperBound.y < B2_HUGE );

	int proxyId = b2AllocateNode( tree );
	b2TreeNode* node = tree->nodes + proxyId;

	node->aabb = aabb;
	node->userData = userData;
	node->categoryBits = categoryBits;
	node->height = 0;
	node->flags = b2_allocatedNode | b2_leafNode;

	bool shouldRotate = true;
	b2InsertLeaf( tree, proxyId, shouldRotate );

	tree->proxyCount += 1;

	return proxyId;
}

void b2DynamicTree_DestroyProxy( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B2_ASSERT( b2IsLeaf( tree->nodes + proxyId ) );

	b2RemoveLeaf( tree, proxyId );
	b2FreeNode( tree, proxyId );

	B2_ASSERT( tree->proxyCount > 0 );
	tree->proxyCount -= 1;
}

int b2DynamicTree_GetProxyCount( const b2DynamicTree* tree )
{
	return tree->proxyCount;
}

void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );
	B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B2_ASSERT( b2IsLeaf( tree->nodes + proxyId ) );

	b2RemoveLeaf( tree, proxyId );

	tree->nodes[proxyId].aabb = aabb;

	bool shouldRotate = false;
	b2InsertLeaf( tree, proxyId, shouldRotate );
}

void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	b2TreeNode* nodes = tree->nodes;

	B2_VALIDATE( b2IsValidAABB( aabb ) );
	B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B2_ASSERT( b2IsLeaf( tree->nodes + proxyId ) );

	// Caller must ensure this
	B2_VALIDATE( b2AABB_Contains( nodes[proxyId].aabb, aabb ) == false );

	nodes[proxyId].aabb = aabb;

	int parentIndex = nodes[proxyId].parent;
	while ( parentIndex != B2_NULL_INDEX )
	{
		bool changed = b2EnlargeAABB( &nodes[parentIndex].aabb, aabb );
		nodes[parentIndex].flags |= b2_enlargedNode;
		parentIndex = nodes[parentIndex].parent;

		if ( changed == false )
		{
			break;
		}
	}

	while ( parentIndex != B2_NULL_INDEX )
	{
		if ( nodes[parentIndex].flags & b2_enlargedNode )
		{
			// early out because this ancestor was previously ascended and marked as enlarged
			break;
		}

		nodes[parentIndex].flags |= b2_enlargedNode;
		parentIndex = nodes[parentIndex].parent;
	}
}

void b2DynamicTree_SetCategoryBits( b2DynamicTree* tree, int proxyId, uint64_t categoryBits )
{
	b2TreeNode* nodes = tree->nodes;

	B2_ASSERT( nodes[proxyId].children.child1 == B2_NULL_INDEX );
	B2_ASSERT( nodes[proxyId].children.child2 == B2_NULL_INDEX );
	B2_ASSERT( ( nodes[proxyId].flags & b2_leafNode ) == b2_leafNode );

	nodes[proxyId].categoryBits = categoryBits;

	// Fix up category bits in ancestor internal nodes
	int nodeIndex = nodes[proxyId].parent;
	while ( nodeIndex != B2_NULL_INDEX )
	{
		b2TreeNode* node = nodes + nodeIndex;
		int child1 = node->children.child1;
		B2_ASSERT( child1 != B2_NULL_INDEX );
		int child2 = node->children.child2;
		B2_ASSERT( child2 != B2_NULL_INDEX );
		node->categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;

		nodeIndex = node->parent;
	}
}

uint64_t b2DynamicTree_GetCategoryBits( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	return tree->nodes[proxyId].categoryBits;
}

int b2DynamicTree_GetHeight( const b2DynamicTree* tree )
{
	if ( tree->root == B2_NULL_INDEX )
	{
		return 0;
	}

	return tree->nodes[tree->root].height;
}

float b2DynamicTree_GetAreaRatio( const b2DynamicTree* tree )
{
	if ( tree->root == B2_NULL_INDEX )
	{
		return 0.0f;
	}

	const b2TreeNode* root = tree->nodes + tree->root;
	float rootArea = b2Perimeter( root->aabb );

	float totalArea = 0.0f;
	for ( int i = 0; i < tree->nodeCapacity; ++i )
	{
		const b2TreeNode* node = tree->nodes + i;
		if ( b2IsAllocated( node ) == false || b2IsLeaf( node ) || i == tree->root )
		{
			continue;
		}

		totalArea += b2Perimeter( node->aabb );
	}

	return totalArea / rootArea;
}

b2AABB b2DynamicTree_GetRootBounds( const b2DynamicTree* tree )
{
	if ( tree->root != B2_NULL_INDEX )
	{
		return tree->nodes[tree->root].aabb;
	}

	b2AABB empty = { b2Vec2_zero, b2Vec2_zero };
	return empty;
}

#if B2_ENABLE_VALIDATION
// Compute the height of a sub-tree.
static int b2ComputeHeight( const b2DynamicTree* tree, int nodeId )
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	b2TreeNode* node = tree->nodes + nodeId;

	if ( b2IsLeaf( node ) )
	{
		return 0;
	}

	int height1 = b2ComputeHeight( tree, node->children.child1 );
	int height2 = b2ComputeHeight( tree, node->children.child2 );
	return 1 + b2MaxInt( height1, height2 );
}

static void b2ValidateStructure( const b2DynamicTree* tree, int index )
{
	if ( index == B2_NULL_INDEX )
	{
		return;
	}

	if ( index == tree->root )
	{
		B2_ASSERT( tree->nodes[index].parent == B2_NULL_INDEX );
	}

	const b2TreeNode* node = tree->nodes + index;

	B2_ASSERT( node->flags == 0 || ( node->flags & b2_allocatedNode ) != 0 );

	if ( b2IsLeaf( node ) )
	{
		B2_ASSERT( node->height == 0 );
		return;
	}

	int child1 = node->children.child1;
	int child2 = node->children.child2;

	B2_ASSERT( 0 <= child1 && child1 < tree->nodeCapacity );
	B2_ASSERT( 0 <= child2 && child2 < tree->nodeCapacity );

	B2_ASSERT( tree->nodes[child1].parent == index );
	B2_ASSERT( tree->nodes[child2].parent == index );

	if ( ( tree->nodes[child1].flags | tree->nodes[child2].flags ) & b2_enlargedNode )
	{
		B2_ASSERT( node->flags & b2_enlargedNode );
	}

	b2ValidateStructure( tree, child1 );
	b2ValidateStructure( tree, child2 );
}

static void b2ValidateMetrics( const b2DynamicTree* tree, int index )
{
	if ( index == B2_NULL_INDEX )
	{
		return;
	}

	const b2TreeNode* node = tree->nodes + index;

	if ( b2IsLeaf( node ) )
	{
		B2_ASSERT( node->height == 0 );
		return;
	}

	int child1 = node->children.child1;
	int child2 = node->children.child2;

	B2_ASSERT( 0 <= child1 && child1 < tree->nodeCapacity );
	B2_ASSERT( 0 <= child2 && child2 < tree->nodeCapacity );

	int height1 = tree->nodes[child1].height;
	int height2 = tree->nodes[child2].height;
	int height = 1 + b2MaxInt( height1, height2 );
	B2_ASSERT( node->height == height );

	// b2AABB aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);

	B2_ASSERT( b2AABB_Contains( node->aabb, tree->nodes[child1].aabb ) );
	B2_ASSERT( b2AABB_Contains( node->aabb, tree->nodes[child2].aabb ) );

	// B2_ASSERT(aabb.lowerBound.x == node->aabb.lowerBound.x);
	// B2_ASSERT(aabb.lowerBound.y == node->aabb.lowerBound.y);
	// B2_ASSERT(aabb.upperBound.x == node->aabb.upperBound.x);
	// B2_ASSERT(aabb.upperBound.y == node->aabb.upperBound.y);

	uint64_t categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
	B2_ASSERT( node->categoryBits == categoryBits );

	b2ValidateMetrics( tree, child1 );
	b2ValidateMetrics( tree, child2 );
}
#endif

void b2DynamicTree_Validate( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION
	if ( tree->root == B2_NULL_INDEX )
	{
		return;
	}

	b2ValidateStructure( tree, tree->root );
	b2ValidateMetrics( tree, tree->root );

	int freeCount = 0;
	int freeIndex = tree->freeList;
	while ( freeIndex != B2_NULL_INDEX )
	{
		B2_ASSERT( 0 <= freeIndex && freeIndex < tree->nodeCapacity );
		freeIndex = tree->nodes[freeIndex].next;
		++freeCount;
	}

	int height = b2DynamicTree_GetHeight( tree );
	int computedHeight = b2ComputeHeight( tree, tree->root );
	B2_ASSERT( height == computedHeight );

	B2_ASSERT( tree->nodeCount + freeCount == tree->nodeCapacity );
#else
	B2_UNUSED( tree );
#endif
}

void b2DynamicTree_ValidateNoEnlarged( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION == 1
	int capacity = tree->nodeCapacity;
	const b2TreeNode* nodes = tree->nodes;
	for ( int i = 0; i < capacity; ++i )
	{
		const b2TreeNode* node = nodes + i;
		if ( node->flags & b2_allocatedNode )
		{
			B2_ASSERT( ( node->flags & b2_enlargedNode ) == 0 );
		}
	}
#else
	B2_UNUSED( tree );
#endif
}

int b2DynamicTree_GetByteCount( const b2DynamicTree* tree )
{
	size_t size = sizeof( b2DynamicTree ) + sizeof( b2TreeNode ) * tree->nodeCapacity +
				  tree->rebuildCapacity * ( sizeof( int ) + sizeof( b2AABB ) + sizeof( b2Vec2 ) + sizeof( int ) );

	return (int)size;
}

uint64_t b2DynamicTree_GetUserData( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	return tree->nodes[proxyId].userData;
}

b2AABB b2DynamicTree_GetAABB( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	return tree->nodes[proxyId].aabb;
}

b2TreeStats b2DynamicTree_Query( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits, b2TreeQueryCallbackFcn* callback,
								 void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];

		const b2TreeNode* node = tree->nodes + nodeId;
		result.nodeVisits += 1;

		if ( b2AABB_Overlaps( node->aabb, aabb ) && ( node->categoryBits & maskBits ) != 0 )
		{
			if ( b2IsLeaf( node ) )
			{
				// callback to user code with proxy id
				bool proceed = callback( nodeId, node->userData, context );
				result.leafVisits += 1;

				if ( proceed == false )
				{
					return result;
				}
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
				else
				{
					B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
				}
			}
		}
	}

	return result;
}

b2TreeStats b2DynamicTree_QueryAll( const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];

		const b2TreeNode* node = tree->nodes + nodeId;
		result.nodeVisits += 1;

		if ( b2AABB_Overlaps( node->aabb, aabb ) )
		{
			if ( b2IsLeaf( node ) )
			{
				// callback to user code with proxy id
				bool proceed = callback( nodeId, node->userData, context );
				result.leafVisits += 1;

				if ( proceed == false )
				{
					return result;
				}
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
				else
				{
					B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
				}
			}
		}
	}

	return result;
}

b2TreeStats b2DynamicTree_RayCast( const b2DynamicTree* tree, const b2RayCastInput* input, uint64_t maskBits,
								   b2TreeRayCastCallbackFcn* callback, void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	b2Vec2 p1 = input->origin;
	b2Vec2 d = input->translation;

	b2Vec2 r = b2Normalize( d );

	// v is perpendicular to the segment.
	b2Vec2 v = b2CrossSV( 1.0f, r );
	b2Vec2 abs_v = b2Abs( v );

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	float maxFraction = input->maxFraction;

	b2Vec2 p2 = b2MulAdd( p1, maxFraction, d );

	// Build a bounding box for the segment.
	b2AABB segmentAABB = { b2Min( p1, p2 ), b2Max( p1, p2 ) };

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	const b2TreeNode* nodes = tree->nodes;

	b2RayCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];
		if ( nodeId == B2_NULL_INDEX )
		{
			// todo is this possible?
			B2_ASSERT( false );
			continue;
		}

		const b2TreeNode* node = nodes + nodeId;
		result.nodeVisits += 1;

		b2AABB nodeAABB = node->aabb;

		if ( ( node->categoryBits & maskBits ) == 0 || b2AABB_Overlaps( nodeAABB, segmentAABB ) == false )
		{
			continue;
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		// radius extension is added to the node in this case
		b2Vec2 c = b2AABB_Center( nodeAABB );
		b2Vec2 h = b2AABB_Extents( nodeAABB );
		float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
		float term2 = b2Dot( abs_v, h );
		if ( term2 < term1 )
		{
			continue;
		}

		if ( b2IsLeaf( node ) )
		{
			subInput.maxFraction = maxFraction;

			float value = callback( &subInput, nodeId, node->userData, context );
			result.leafVisits += 1;

			// The user may return -1 to indicate this shape should be skipped

			if ( value == 0.0f )
			{
				// The client has terminated the ray cast.
				return result;
			}

			if ( 0.0f < value && value <= maxFraction )
			{
				// Update segment bounding box.
				maxFraction = value;
				p2 = b2MulAdd( p1, maxFraction, d );
				segmentAABB.lowerBound = b2Min( p1, p2 );
				segmentAABB.upperBound = b2Max( p1, p2 );
			}
		}
		else
		{
			if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				b2Vec2 c1 = b2AABB_Center( nodes[node->children.child1].aabb );
				b2Vec2 c2 = b2AABB_Center( nodes[node->children.child2].aabb );
				if ( b2DistanceSquared( c1, p1 ) < b2DistanceSquared( c2, p1 ) )
				{
					stack[stackCount++] = node->children.child2;
					stack[stackCount++] = node->children.child1;
				}
				else
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
			}
		}
	}

	return result;
}

b2TreeStats b2DynamicTree_ShapeCast( const b2DynamicTree* tree, const b2ShapeCastInput* input, uint64_t maskBits,
									 b2TreeShapeCastCallbackFcn* callback, void* context )
{
	b2TreeStats stats = { 0 };

	if ( tree->nodeCount == 0 || input->proxy.count == 0 )
	{
		return stats;
	}

	b2AABB originAABB = { input->proxy.points[0], input->proxy.points[0] };
	for ( int i = 1; i < input->proxy.count; ++i )
	{
		originAABB.lowerBound = b2Min( originAABB.lowerBound, input->proxy.points[i] );
		originAABB.upperBound = b2Max( originAABB.upperBound, input->proxy.points[i] );
	}

	b2Vec2 radius = { input->proxy.radius, input->proxy.radius };

	originAABB.lowerBound = b2Sub( originAABB.lowerBound, radius );
	originAABB.upperBound = b2Add( originAABB.upperBound, radius );

	b2Vec2 p1 = b2AABB_Center( originAABB );
	b2Vec2 extension = b2AABB_Extents( originAABB );

	// v is perpendicular to the segment.
	b2Vec2 r = input->translation;
	b2Vec2 v = b2CrossSV( 1.0f, r );
	b2Vec2 abs_v = b2Abs( v );

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	float maxFraction = input->maxFraction;

	// Build total box for the shape cast
	b2Vec2 t = b2MulSV( maxFraction, input->translation );
	b2AABB totalAABB = {
		b2Min( originAABB.lowerBound, b2Add( originAABB.lowerBound, t ) ),
		b2Max( originAABB.upperBound, b2Add( originAABB.upperBound, t ) ),
	};

	b2ShapeCastInput subInput = *input;
	const b2TreeNode* nodes = tree->nodes;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];
		if ( nodeId == B2_NULL_INDEX )
		{
			// todo is this possible?
			B2_ASSERT( false );
			continue;
		}

		const b2TreeNode* node = nodes + nodeId;
		stats.nodeVisits += 1;

		if ( ( node->categoryBits & maskBits ) == 0 || b2AABB_Overlaps( node->aabb, totalAABB ) == false )
		{
			continue;
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		// radius extension is added to the node in this case
		b2Vec2 c = b2AABB_Center( node->aabb );
		b2Vec2 h = b2Add( b2AABB_Extents( node->aabb ), extension );
		float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
		float term2 = b2Dot( abs_v, h );
		if ( term2 < term1 )
		{
			continue;
		}

		if ( b2IsLeaf( node ) )
		{
			subInput.maxFraction = maxFraction;

			float value = callback( &subInput, nodeId, node->userData, context );
			stats.leafVisits += 1;

			if ( value == 0.0f )
			{
				// The client has terminated the ray cast.
				return stats;
			}

			if ( 0.0f < value && value < maxFraction )
			{
				// Update segment bounding box.
				maxFraction = value;
				t = b2MulSV( maxFraction, input->translation );
				totalAABB.lowerBound = b2Min( originAABB.lowerBound, b2Add( originAABB.lowerBound, t ) );
				totalAABB.upperBound = b2Max( originAABB.upperBound, b2Add( originAABB.upperBound, t ) );
			}
		}
		else
		{
			if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				b2Vec2 c1 = b2AABB_Center( nodes[node->children.child1].aabb );
				b2Vec2 c2 = b2AABB_Center( nodes[node->children.child2].aabb );
				if ( b2DistanceSquared( c1, p1 ) < b2DistanceSquared( c2, p1 ) )
				{
					stack[stackCount++] = node->children.child2;
					stack[stackCount++] = node->children.child1;
				}
				else
				{
					stack[stackCount++] = node->children.child1;
					stack[stackCount++] = node->children.child2;
				}
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
			}
		}
	}

	return stats;
}

// Median split == 0, Surface area heuristic == 1
#define B2_TREE_HEURISTIC 0

#if B2_TREE_HEURISTIC == 0

// Median split heuristic
static int b2PartitionMid( int* indices, b2Vec2* centers, int count )
{
	// Handle trivial case
	if ( count <= 2 )
	{
		return count / 2;
	}

	b2Vec2 lowerBound = centers[0];
	b2Vec2 upperBound = centers[0];

	for ( int i = 1; i < count; ++i )
	{
		lowerBound = b2Min( lowerBound, centers[i] );
		upperBound = b2Max( upperBound, centers[i] );
	}

	b2Vec2 d = b2Sub( upperBound, lowerBound );
	b2Vec2 c = { 0.5f * ( lowerBound.x + upperBound.x ), 0.5f * ( lowerBound.y + upperBound.y ) };

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	if ( d.x > d.y )
	{
		float pivot = c.x;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].x < pivot )
			{
				i1 += 1;
			};

			while ( i1 < i2 && centers[i2 - 1].x >= pivot )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b2Vec2 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else
	{
		float pivot = c.y;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].y < pivot )
			{
				i1 += 1;
			};

			while ( i1 < i2 && centers[i2 - 1].y >= pivot )
			{
				i2 -= 1;
			};

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b2Vec2 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	B2_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}

	return count / 2;
}

#else

#define B2_BIN_COUNT 8

typedef struct b2TreeBin
{
	b2AABB aabb;
	int count;
} b2TreeBin;

typedef struct b2TreePlane
{
	b2AABB leftAABB;
	b2AABB rightAABB;
	int leftCount;
	int rightCount;
} b2TreePlane;

// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
// Returns the left child count
static int b2PartitionSAH( int* indices, int* binIndices, b2AABB* boxes, int count )
{
	B2_ASSERT( count > 0 );

	b2TreeBin bins[B2_BIN_COUNT];
	b2TreePlane planes[B2_BIN_COUNT - 1];

	b2Vec2 center = b2AABB_Center( boxes[0] );
	b2AABB centroidAABB;
	centroidAABB.lowerBound = center;
	centroidAABB.upperBound = center;

	for ( int i = 1; i < count; ++i )
	{
		center = b2AABB_Center( boxes[i] );
		centroidAABB.lowerBound = b2Min( centroidAABB.lowerBound, center );
		centroidAABB.upperBound = b2Max( centroidAABB.upperBound, center );
	}

	b2Vec2 d = b2Sub( centroidAABB.upperBound, centroidAABB.lowerBound );

	// Find longest axis
	int axisIndex;
	float invD;
	if ( d.x > d.y )
	{
		axisIndex = 0;
		invD = d.x;
	}
	else
	{
		axisIndex = 1;
		invD = d.y;
	}

	invD = invD > 0.0f ? 1.0f / invD : 0.0f;

	// Initialize bin bounds and count
	for ( int i = 0; i < B2_BIN_COUNT; ++i )
	{
		bins[i].aabb.lowerBound = (b2Vec2){ FLT_MAX, FLT_MAX };
		bins[i].aabb.upperBound = (b2Vec2){ -FLT_MAX, -FLT_MAX };
		bins[i].count = 0;
	}

	// Assign boxes to bins and compute bin boxes
	// TODO_ERIN optimize
	float binCount = B2_BIN_COUNT;
	float lowerBoundArray[2] = { centroidAABB.lowerBound.x, centroidAABB.lowerBound.y };
	float minC = lowerBoundArray[axisIndex];
	for ( int i = 0; i < count; ++i )
	{
		b2Vec2 c = b2AABB_Center( boxes[i] );
		float cArray[2] = { c.x, c.y };
		int binIndex = (int)( binCount * ( cArray[axisIndex] - minC ) * invD );
		binIndex = b2ClampInt( binIndex, 0, B2_BIN_COUNT - 1 );
		binIndices[i] = binIndex;
		bins[binIndex].count += 1;
		bins[binIndex].aabb = b2AABB_Union( bins[binIndex].aabb, boxes[i] );
	}

	int planeCount = B2_BIN_COUNT - 1;

	// Prepare all the left planes, candidates for left child
	planes[0].leftCount = bins[0].count;
	planes[0].leftAABB = bins[0].aabb;
	for ( int i = 1; i < planeCount; ++i )
	{
		planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
		planes[i].leftAABB = b2AABB_Union( planes[i - 1].leftAABB, bins[i].aabb );
	}

	// Prepare all the right planes, candidates for right child
	planes[planeCount - 1].rightCount = bins[planeCount].count;
	planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
	for ( int i = planeCount - 2; i >= 0; --i )
	{
		planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
		planes[i].rightAABB = b2AABB_Union( planes[i + 1].rightAABB, bins[i + 1].aabb );
	}

	// Find best split to minimize SAH
	float minCost = FLT_MAX;
	int bestPlane = 0;
	for ( int i = 0; i < planeCount; ++i )
	{
		float leftArea = b2Perimeter( planes[i].leftAABB );
		float rightArea = b2Perimeter( planes[i].rightAABB );
		int leftCount = planes[i].leftCount;
		int rightCount = planes[i].rightCount;

		float cost = leftCount * leftArea + rightCount * rightArea;
		if ( cost < minCost )
		{
			bestPlane = i;
			minCost = cost;
		}
	}

	// Partition node indices and boxes using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	while ( i1 < i2 )
	{
		while ( i1 < i2 && binIndices[i1] < bestPlane )
		{
			i1 += 1;
		};

		while ( i1 < i2 && binIndices[i2 - 1] >= bestPlane )
		{
			i2 -= 1;
		};

		if ( i1 < i2 )
		{
			// Swap indices
			{
				int temp = indices[i1];
				indices[i1] = indices[i2 - 1];
				indices[i2 - 1] = temp;
			}

			// Swap boxes
			{
				b2AABB temp = boxes[i1];
				boxes[i1] = boxes[i2 - 1];
				boxes[i2 - 1] = temp;
			}

			i1 += 1;
			i2 -= 1;
		}
	}
	B2_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}
	else
	{
		return count / 2;
	}
}

#endif

// Temporary data used to track the rebuild of a tree node
struct b2RebuildItem
{
	int nodeIndex;
	int childCount;

	// Leaf indices
	int startIndex;
	int splitIndex;
	int endIndex;
};

// Returns root node index
static int b2BuildTree( b2DynamicTree* tree, int leafCount )
{
	b2TreeNode* nodes = tree->nodes;
	int* leafIndices = tree->leafIndices;

	if ( leafCount == 1 )
	{
		nodes[leafIndices[0]].parent = B2_NULL_INDEX;
		return leafIndices[0];
	}

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
	int* binIndices = tree->binIndices;
#endif

	// todo large stack item
	struct b2RebuildItem stack[B2_TREE_STACK_SIZE];
	int top = 0;

	stack[0].nodeIndex = b2AllocateNode( tree );
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
#if B2_TREE_HEURISTIC == 0
	stack[0].splitIndex = b2PartitionMid( leafIndices, leafCenters, leafCount );
#else
	stack[0].splitIndex = b2PartitionSAH( leafIndices, binIndices, leafBoxes, leafCount );
#endif

	while ( true )
	{
		struct b2RebuildItem* item = stack + top;

		item->childCount += 1;

		if ( item->childCount == 2 )
		{
			// This internal node has both children established

			if ( top == 0 )
			{
				// all done
				break;
			}

			struct b2RebuildItem* parentItem = stack + ( top - 1 );
			b2TreeNode* parentNode = nodes + parentItem->nodeIndex;

			if ( parentItem->childCount == 0 )
			{
				B2_ASSERT( parentNode->children.child1 == B2_NULL_INDEX );
				parentNode->children.child1 = item->nodeIndex;
			}
			else
			{
				B2_ASSERT( parentItem->childCount == 1 );
				B2_ASSERT( parentNode->children.child2 == B2_NULL_INDEX );
				parentNode->children.child2 = item->nodeIndex;
			}

			b2TreeNode* node = nodes + item->nodeIndex;

			B2_ASSERT( node->parent == B2_NULL_INDEX );
			node->parent = parentItem->nodeIndex;

			B2_ASSERT( node->children.child1 != B2_NULL_INDEX );
			B2_ASSERT( node->children.child2 != B2_NULL_INDEX );
			b2TreeNode* child1 = nodes + node->children.child1;
			b2TreeNode* child2 = nodes + node->children.child2;

			node->aabb = b2AABB_Union( child1->aabb, child2->aabb );
			node->height = 1 + b2MaxUInt16( child1->height, child2->height );
			node->categoryBits = child1->categoryBits | child2->categoryBits;

			// Pop stack
			top -= 1;
		}
		else
		{
			int startIndex, endIndex;
			if ( item->childCount == 0 )
			{
				startIndex = item->startIndex;
				endIndex = item->splitIndex;
			}
			else
			{
				B2_ASSERT( item->childCount == 1 );
				startIndex = item->splitIndex;
				endIndex = item->endIndex;
			}

			int count = endIndex - startIndex;

			if ( count == 1 )
			{
				int childIndex = leafIndices[startIndex];
				b2TreeNode* node = nodes + item->nodeIndex;

				if ( item->childCount == 0 )
				{
					B2_ASSERT( node->children.child1 == B2_NULL_INDEX );
					node->children.child1 = childIndex;
				}
				else
				{
					B2_ASSERT( item->childCount == 1 );
					B2_ASSERT( node->children.child2 == B2_NULL_INDEX );
					node->children.child2 = childIndex;
				}

				b2TreeNode* childNode = nodes + childIndex;
				B2_ASSERT( childNode->parent == B2_NULL_INDEX );
				childNode->parent = item->nodeIndex;
			}
			else
			{
				B2_ASSERT( count > 0 );
				B2_ASSERT( top < B2_TREE_STACK_SIZE );

				top += 1;
				struct b2RebuildItem* newItem = stack + top;
				newItem->nodeIndex = b2AllocateNode( tree );
				newItem->childCount = -1;
				newItem->startIndex = startIndex;
				newItem->endIndex = endIndex;
#if B2_TREE_HEURISTIC == 0
				newItem->splitIndex = b2PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
#else
				newItem->splitIndex =
					b2PartitionSAH( leafIndices + startIndex, binIndices + startIndex, leafBoxes + startIndex, count );
#endif
				newItem->splitIndex += startIndex;
			}
		}
	}

	b2TreeNode* rootNode = nodes + stack[0].nodeIndex;
	B2_ASSERT( rootNode->parent == B2_NULL_INDEX );
	B2_ASSERT( rootNode->children.child1 != B2_NULL_INDEX );
	B2_ASSERT( rootNode->children.child2 != B2_NULL_INDEX );

	b2TreeNode* child1 = nodes + rootNode->children.child1;
	b2TreeNode* child2 = nodes + rootNode->children.child2;

	rootNode->aabb = b2AABB_Union( child1->aabb, child2->aabb );
	rootNode->height = 1 + b2MaxUInt16( child1->height, child2->height );
	rootNode->categoryBits = child1->categoryBits | child2->categoryBits;

	return stack[0].nodeIndex;
}

// Not safe to access tree during this operation because it may grow
int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild )
{
	int proxyCount = tree->proxyCount;
	if ( proxyCount == 0 )
	{
		return 0;
	}

	// Ensure capacity for rebuild space
	if ( proxyCount > tree->rebuildCapacity )
	{
		int newCapacity = proxyCount + proxyCount / 2;

		b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int ) );
		tree->leafIndices = b2Alloc( newCapacity * sizeof( int ) );

#if B2_TREE_HEURISTIC == 0
		b2Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b2Vec2 ) );
		tree->leafCenters = b2Alloc( newCapacity * sizeof( b2Vec2 ) );
#else
		b2Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b2AABB ) );
		tree->leafBoxes = b2Alloc( newCapacity * sizeof( b2AABB ) );
		b2Free( tree->binIndices, tree->rebuildCapacity * sizeof( int ) );
		tree->binIndices = b2Alloc( newCapacity * sizeof( int ) );
#endif
		tree->rebuildCapacity = newCapacity;
	}

	int leafCount = 0;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;

	int nodeIndex = tree->root;
	b2TreeNode* nodes = tree->nodes;
	b2TreeNode* node = nodes + nodeIndex;

	// These are the nodes that get sorted to rebuild the tree.
	// I'm using indices because the node pool may grow during the build.
	int* leafIndices = tree->leafIndices;

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
#endif

	// Gather all proxy nodes that have grown and all internal nodes that haven't grown. Both are
	// considered leaves in the tree rebuild.
	// Free all internal nodes that have grown.
	// todo use a node growth metric instead of simply enlarged to reduce rebuild size and frequency
	// this should be weighed against B2_AABB_MARGIN
	while ( true )
	{
		if ( node->height == 0 || ( ( node->flags & b2_enlargedNode ) == 0 && fullBuild == false ) )
		{
			leafIndices[leafCount] = nodeIndex;
#if B2_TREE_HEURISTIC == 0
			leafCenters[leafCount] = b2AABB_Center( node->aabb );
#else
			leafBoxes[leafCount] = node->aabb;
#endif
			leafCount += 1;

			// Detach
			node->parent = B2_NULL_INDEX;
		}
		else
		{
			int doomedNodeIndex = nodeIndex;

			// Handle children
			nodeIndex = node->children.child1;

			if ( stackCount < B2_TREE_STACK_SIZE )
			{
				stack[stackCount++] = node->children.child2;
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
			}

			node = nodes + nodeIndex;

			// Remove doomed node
			b2FreeNode( tree, doomedNodeIndex );

			continue;
		}

		if ( stackCount == 0 )
		{
			break;
		}

		nodeIndex = stack[--stackCount];
		node = nodes + nodeIndex;
	}

#if B2_ENABLE_VALIDATION == 1
	int capacity = tree->nodeCapacity;
	for ( int i = 0; i < capacity; ++i )
	{
		if ( nodes[i].flags & b2_allocatedNode )
		{
			B2_ASSERT( ( nodes[i].flags & b2_enlargedNode ) == 0 );
		}
	}
#endif

	B2_ASSERT( leafCount <= proxyCount );

	tree->root = b2BuildTree( tree, leafCount );

	b2DynamicTree_Validate( tree );

	return leafCount;
}

// ============================================================================
// Parallel build from a flat leaf array.
//
// Input is owned by the caller (a flat array of AABBs plus optional userData
// and categoryBits). Output is a fresh tree with the input element at slot i
// becoming leaf node i, and N-1 internal nodes at slots [N, 2N-1) in DFS
// preorder. The build uses the midpoint-of-centroid heuristic and a stable
// count+scatter partition so the result is bit-identical regardless of the
// number of workers used.
// ============================================================================

// Below this leaf count, the entire build runs serially even when a world is
// supplied. Avoids paying task-spawn overhead on small inputs.
#define B2_BUILD_MIN_PARALLEL_LEAVES 1024

// A subtree with this many leaves or fewer builds entirely on one worker (no
// task spawn, no parallel partition). Tuned to be roughly the smallest range
// where parallel_for's dispatch is cheaper than a serial pass.
#define B2_BUILD_SUBTREE_TASK_THRESHOLD 2048

typedef struct b2BuildContext
{
	b2DynamicTree* tree;
	b2World* world;

	// Source for this depth (read by partition); destination is the alt pair.
	// Roles flip each level via the ping-pong invariant.
	int* srcIndices;
	b2Vec2* srcCenters;
	int* dstIndices;
	b2Vec2* dstCenters;

	int rangeFirst;
	int rangeCount;

	// Pre-allocated internal-node slots for this subtree, DFS preorder.
	int internalFirst;
	int internalCount;

	// Output: read by the parent after the recursive call (or task) completes.
	int rootIndex;
	b2AABB rootAabb;
	uint16_t rootHeight;
	uint64_t rootCategoryBits;
} b2BuildContext;

static int b2BuildShouldParallelize( const b2BuildContext* ctx )
{
	return ctx->world != NULL && ctx->world->workerCount > 1 && ctx->rangeCount >= B2_BUILD_SUBTREE_TASK_THRESHOLD;
}

// ---------------------------------------------------------------------------
// Phase 2: leaf node initialization (reset all leaves + seed scratch buffers).
// ---------------------------------------------------------------------------

typedef struct b2BuildLeafInitShared
{
	b2TreeNode* nodes;
	const b2AABB* aabbs;
	const uint64_t* userData;
	const uint64_t* categoryBits;
	int* indices;
	b2Vec2* centers;
} b2BuildLeafInitShared;

static void b2BuildLeafInitRange( b2BuildLeafInitShared* shared, int startIndex, int endIndex )
{
	b2TreeNode* nodes = shared->nodes;
	const b2AABB* aabbs = shared->aabbs;
	const uint64_t* userData = shared->userData;
	const uint64_t* categoryBits = shared->categoryBits;
	int* indices = shared->indices;
	b2Vec2* centers = shared->centers;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2AABB aabb = aabbs[i];
		b2TreeNode* node = nodes + i;
		node->aabb = aabb;
		node->userData = userData != NULL ? userData[i] : 0;
		node->categoryBits = categoryBits != NULL ? categoryBits[i] : B2_DEFAULT_CATEGORY_BITS;
		node->parent = B2_NULL_INDEX;
		node->height = 0;
		node->flags = b2_allocatedNode | b2_leafNode;

		indices[i] = i;
		centers[i] = b2AABB_Center( aabb );
	}
}

static void b2BuildLeafInitTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	(void)workerIndex;
	b2BuildLeafInitRange( (b2BuildLeafInitShared*)context, startIndex, endIndex );
}

// ---------------------------------------------------------------------------
// Phase 3 helper: centroid-AABB reduction over a sub-range.
// ---------------------------------------------------------------------------

typedef struct b2BuildCentroidShared
{
	const b2Vec2* centers;
	int rangeFirst;
	b2Vec2* perWorkerLower;
	b2Vec2* perWorkerUpper;
} b2BuildCentroidShared;

static void b2BuildCentroidTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2BuildCentroidShared* shared = (b2BuildCentroidShared*)context;
	const b2Vec2* centers = shared->centers + shared->rangeFirst;

	b2Vec2 lower = shared->perWorkerLower[workerIndex];
	b2Vec2 upper = shared->perWorkerUpper[workerIndex];

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2Vec2 c = centers[i];
		lower = b2Min( lower, c );
		upper = b2Max( upper, c );
	}

	shared->perWorkerLower[workerIndex] = lower;
	shared->perWorkerUpper[workerIndex] = upper;
}

static void b2BuildCentroidBounds( const b2BuildContext* ctx, b2Vec2* outLower, b2Vec2* outUpper )
{
	const b2Vec2* centers = ctx->srcCenters + ctx->rangeFirst;
	int n = ctx->rangeCount;

	if ( ctx->world == NULL || ctx->world->workerCount == 1 || n < B2_BUILD_SUBTREE_TASK_THRESHOLD )
	{
		b2Vec2 lower = centers[0];
		b2Vec2 upper = centers[0];
		for ( int i = 1; i < n; ++i )
		{
			b2Vec2 c = centers[i];
			lower = b2Min( lower, c );
			upper = b2Max( upper, c );
		}
		*outLower = lower;
		*outUpper = upper;
		return;
	}

	int workerCount = ctx->world->workerCount;
	b2Vec2 perWorkerLower[B2_MAX_WORKERS];
	b2Vec2 perWorkerUpper[B2_MAX_WORKERS];
	b2Vec2 sentinelLow = { FLT_MAX, FLT_MAX };
	b2Vec2 sentinelHigh = { -FLT_MAX, -FLT_MAX };
	for ( int i = 0; i < workerCount; ++i )
	{
		perWorkerLower[i] = sentinelLow;
		perWorkerUpper[i] = sentinelHigh;
	}

	b2BuildCentroidShared shared = {
		.centers = ctx->srcCenters,
		.rangeFirst = ctx->rangeFirst,
		.perWorkerLower = perWorkerLower,
		.perWorkerUpper = perWorkerUpper,
	};

	b2ParallelFor( ctx->world, b2BuildCentroidTask, n, 1024, &shared );

	b2Vec2 lower = sentinelLow;
	b2Vec2 upper = sentinelHigh;
	for ( int i = 0; i < workerCount; ++i )
	{
		lower = b2Min( lower, perWorkerLower[i] );
		upper = b2Max( upper, perWorkerUpper[i] );
	}
	*outLower = lower;
	*outUpper = upper;
}

// ---------------------------------------------------------------------------
// Phase 3 helper: stable count+scatter partition over a sub-range.
//
// Predicate is independent per leaf (compares srcCenters[i] on the chosen
// axis to a fixed pivot), so the partition is deterministic. Output is
// stable: relative order of "left" items is preserved, same for "right".
// ---------------------------------------------------------------------------

static int b2BuildPartitionPredicate( b2Vec2 c, int axis, float pivot )
{
	return ( axis == 0 ? c.x : c.y ) < pivot;
}

static int b2BuildPartitionSerial( b2BuildContext* ctx, int axis, float pivot )
{
	const int* srcIndices = ctx->srcIndices + ctx->rangeFirst;
	const b2Vec2* srcCenters = ctx->srcCenters + ctx->rangeFirst;
	int* dstIndices = ctx->dstIndices + ctx->rangeFirst;
	b2Vec2* dstCenters = ctx->dstCenters + ctx->rangeFirst;
	int n = ctx->rangeCount;

	int leftCount = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( b2BuildPartitionPredicate( srcCenters[i], axis, pivot ) )
		{
			leftCount += 1;
		}
	}

	int li = 0;
	int ri = leftCount;
	for ( int i = 0; i < n; ++i )
	{
		if ( b2BuildPartitionPredicate( srcCenters[i], axis, pivot ) )
		{
			dstIndices[li] = srcIndices[i];
			dstCenters[li] = srcCenters[i];
			li += 1;
		}
		else
		{
			dstIndices[ri] = srcIndices[i];
			dstCenters[ri] = srcCenters[i];
			ri += 1;
		}
	}

	B2_ASSERT( li == leftCount );
	B2_ASSERT( ri == n );
	return leftCount;
}

// Replicates the block layout that b2ParallelFor would compute. Keeping it
// here lets each block know its own index from the (start,end) it receives.
static void b2BuildComputeBlocks( int itemCount, int workerCount, int minRange, int* outBlockSize, int* outBlockCount )
{
	int blocksPerWorker = 4;
	int maxBlockCount = blocksPerWorker * workerCount;
	int blockSize;
	int blockCount;
	if ( itemCount <= minRange * maxBlockCount )
	{
		blockSize = minRange;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	else
	{
		blockSize = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	*outBlockSize = blockSize;
	*outBlockCount = blockCount;
}

typedef struct b2BuildPartitionShared
{
	const int* srcIndices;
	const b2Vec2* srcCenters;
	int* dstIndices;
	b2Vec2* dstCenters;
	int rangeFirst;
	int axis;
	float pivot;
	int blockSize;
	int* perBlockLeftCount;
	int* perBlockLeftOffset;
	int* perBlockRightOffset;
} b2BuildPartitionShared;

static void b2BuildPartitionCountTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	(void)workerIndex;
	b2BuildPartitionShared* shared = (b2BuildPartitionShared*)context;
	const b2Vec2* centers = shared->srcCenters + shared->rangeFirst;
	int axis = shared->axis;
	float pivot = shared->pivot;

	int count = 0;
	for ( int i = startIndex; i < endIndex; ++i )
	{
		if ( b2BuildPartitionPredicate( centers[i], axis, pivot ) )
		{
			count += 1;
		}
	}

	int blockIndex = startIndex / shared->blockSize;
	shared->perBlockLeftCount[blockIndex] = count;
}

static void b2BuildPartitionScatterTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	(void)workerIndex;
	b2BuildPartitionShared* shared = (b2BuildPartitionShared*)context;
	const int* srcIndices = shared->srcIndices + shared->rangeFirst;
	const b2Vec2* srcCenters = shared->srcCenters + shared->rangeFirst;
	int* dstIndices = shared->dstIndices + shared->rangeFirst;
	b2Vec2* dstCenters = shared->dstCenters + shared->rangeFirst;
	int axis = shared->axis;
	float pivot = shared->pivot;

	int blockIndex = startIndex / shared->blockSize;
	int li = shared->perBlockLeftOffset[blockIndex];
	int ri = shared->perBlockRightOffset[blockIndex];

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2Vec2 c = srcCenters[i];
		int idx = srcIndices[i];
		if ( b2BuildPartitionPredicate( c, axis, pivot ) )
		{
			dstIndices[li] = idx;
			dstCenters[li] = c;
			li += 1;
		}
		else
		{
			dstIndices[ri] = idx;
			dstCenters[ri] = c;
			ri += 1;
		}
	}
}

static int b2BuildPartitionParallel( b2BuildContext* ctx, int axis, float pivot )
{
	int n = ctx->rangeCount;
	int workerCount = ctx->world->workerCount;
	int minRange = 1024;

	int blockSize;
	int blockCount;
	b2BuildComputeBlocks( n, workerCount, minRange, &blockSize, &blockCount );

	// Block counts share fixed-size on-stack arrays; max possible block count
	// is B2_MAX_WORKERS * blocksPerWorker = 32 * 4 = 128.
	int perBlockLeftCount[B2_MAX_WORKERS * 4];
	int perBlockLeftOffset[B2_MAX_WORKERS * 4];
	int perBlockRightOffset[B2_MAX_WORKERS * 4];
	B2_ASSERT( blockCount <= (int)( sizeof( perBlockLeftCount ) / sizeof( perBlockLeftCount[0] ) ) );

	b2BuildPartitionShared shared = {
		.srcIndices = ctx->srcIndices,
		.srcCenters = ctx->srcCenters,
		.dstIndices = ctx->dstIndices,
		.dstCenters = ctx->dstCenters,
		.rangeFirst = ctx->rangeFirst,
		.axis = axis,
		.pivot = pivot,
		.blockSize = blockSize,
		.perBlockLeftCount = perBlockLeftCount,
		.perBlockLeftOffset = perBlockLeftOffset,
		.perBlockRightOffset = perBlockRightOffset,
	};

	// Pass 1: per-block left count.
	b2ParallelFor( ctx->world, b2BuildPartitionCountTask, n, minRange, &shared );

	// Exclusive prefix sums to get destination offsets per block.
	int leftAccum = 0;
	int rightAccum = 0;
	for ( int b = 0; b < blockCount; ++b )
	{
		int blockStart = b * blockSize;
		int blockEnd = blockStart + blockSize;
		if ( blockEnd > n )
		{
			blockEnd = n;
		}
		int blockLen = blockEnd - blockStart;
		int blockLeft = perBlockLeftCount[b];
		int blockRight = blockLen - blockLeft;
		perBlockLeftOffset[b] = leftAccum;
		perBlockRightOffset[b] = rightAccum;
		leftAccum += blockLeft;
		rightAccum += blockRight;
	}
	int totalLeft = leftAccum;

	// Right items are placed after all left items in dst.
	for ( int b = 0; b < blockCount; ++b )
	{
		perBlockRightOffset[b] += totalLeft;
	}

	// Pass 2: scatter.
	b2ParallelFor( ctx->world, b2BuildPartitionScatterTask, n, minRange, &shared );

	return totalLeft;
}

static int b2BuildPartition( b2BuildContext* ctx, int axis, float pivot )
{
	if ( b2BuildShouldParallelize( ctx ) )
	{
		return b2BuildPartitionParallel( ctx, axis, pivot );
	}
	return b2BuildPartitionSerial( ctx, axis, pivot );
}

// ---------------------------------------------------------------------------
// Recursive build core. Operates on a single subtree described by ctx and
// writes the subtree-root summary back into ctx (rootIndex/rootAabb/...).
// ---------------------------------------------------------------------------

static void b2BuildRecursive( b2BuildContext* ctx );

static void b2BuildTrampoline( void* context )
{
	b2BuildRecursive( (b2BuildContext*)context );
}

// Iterative stack-based serial build used for subtrees small enough that
// task-spawn overhead would dominate. Mirrors b2BuildTree but consumes
// internal node indices from the pre-allocated ctx range and partitions with
// the stable count+scatter scheme over the ping-pong scratch buffers.
//
// The stack frame mirrors b2RebuildItem with the addition of buffer pointers
// and the internal-range bookkeeping.
typedef struct b2BuildSerialFrame
{
	int nodeIndex;
	int childCount;       // -1 (not started), 0 (left in flight), 1 (right in flight)
	int splitIndex;       // leftCount produced by partition
	int rangeFirst;
	int rangeCount;
	int internalFirst;
	int internalCount;
	int* srcIndices;
	b2Vec2* srcCenters;
	int* dstIndices;
	b2Vec2* dstCenters;
	int childRoot[2];
} b2BuildSerialFrame;

static void b2BuildSerialPartitionAndSplit( b2BuildSerialFrame* frame )
{
	const b2Vec2* centers = frame->srcCenters + frame->rangeFirst;
	int n = frame->rangeCount;

	b2Vec2 lower = centers[0];
	b2Vec2 upper = centers[0];
	for ( int i = 1; i < n; ++i )
	{
		b2Vec2 c = centers[i];
		lower = b2Min( lower, c );
		upper = b2Max( upper, c );
	}

	float dx = upper.x - lower.x;
	float dy = upper.y - lower.y;
	int axis = dx > dy ? 0 : 1;
	float pivot = 0.5f * ( axis == 0 ? lower.x + upper.x : lower.y + upper.y );

	const int* srcIndices = frame->srcIndices + frame->rangeFirst;
	int* dstIndices = frame->dstIndices + frame->rangeFirst;
	b2Vec2* dstCenters = frame->dstCenters + frame->rangeFirst;

	int leftCount = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( b2BuildPartitionPredicate( centers[i], axis, pivot ) )
		{
			leftCount += 1;
		}
	}

	// Degenerate: all centers on one side. Force a balanced split by index.
	if ( leftCount == 0 || leftCount == n )
	{
		// Copy through unchanged — order is already stable in src — and pick n/2.
		for ( int i = 0; i < n; ++i )
		{
			dstIndices[i] = srcIndices[i];
			dstCenters[i] = centers[i];
		}
		frame->splitIndex = n / 2;
		return;
	}

	int li = 0;
	int ri = leftCount;
	for ( int i = 0; i < n; ++i )
	{
		if ( b2BuildPartitionPredicate( centers[i], axis, pivot ) )
		{
			dstIndices[li] = srcIndices[i];
			dstCenters[li] = centers[i];
			li += 1;
		}
		else
		{
			dstIndices[ri] = srcIndices[i];
			dstCenters[ri] = centers[i];
			ri += 1;
		}
	}

	frame->splitIndex = leftCount;
}

static void b2BuildSubtreeSerial( b2BuildContext* ctx )
{
	b2DynamicTree* tree = ctx->tree;
	b2TreeNode* nodes = tree->nodes;

	if ( ctx->rangeCount == 1 )
	{
		int leafIndex = ctx->srcIndices[ctx->rangeFirst];
		ctx->rootIndex = leafIndex;
		ctx->rootAabb = nodes[leafIndex].aabb;
		ctx->rootHeight = 0;
		ctx->rootCategoryBits = nodes[leafIndex].categoryBits;
		return;
	}

	b2BuildSerialFrame stack[B2_TREE_STACK_SIZE];
	int top = 0;
	stack[0] = (b2BuildSerialFrame){
		.nodeIndex = ctx->internalFirst,
		.childCount = -1,
		.rangeFirst = ctx->rangeFirst,
		.rangeCount = ctx->rangeCount,
		.internalFirst = ctx->internalFirst,
		.internalCount = ctx->internalCount,
		.srcIndices = ctx->srcIndices,
		.srcCenters = ctx->srcCenters,
		.dstIndices = ctx->dstIndices,
		.dstCenters = ctx->dstCenters,
	};
	b2BuildSerialPartitionAndSplit( &stack[0] );

	for ( ;; )
	{
		b2BuildSerialFrame* frame = stack + top;
		frame->childCount += 1;

		if ( frame->childCount == 2 )
		{
			b2TreeNode* node = nodes + frame->nodeIndex;
			int left = frame->childRoot[0];
			int right = frame->childRoot[1];
			node->aabb = b2AABB_Union( nodes[left].aabb, nodes[right].aabb );
			node->height = 1 + b2MaxUInt16( nodes[left].height, nodes[right].height );
			node->categoryBits = nodes[left].categoryBits | nodes[right].categoryBits;
			node->parent = B2_NULL_INDEX;
			node->userData = 0;
			node->children.child1 = left;
			node->children.child2 = right;
			node->flags = b2_allocatedNode;
			nodes[left].parent = frame->nodeIndex;
			nodes[right].parent = frame->nodeIndex;

			if ( top == 0 )
			{
				ctx->rootIndex = frame->nodeIndex;
				ctx->rootAabb = node->aabb;
				ctx->rootHeight = node->height;
				ctx->rootCategoryBits = node->categoryBits;
				return;
			}
			int childRoot = frame->nodeIndex;
			top -= 1;
			stack[top].childRoot[stack[top].childCount] = childRoot;
			continue;
		}

		// Descend into next child (childCount == 0 -> left, childCount == 1 -> right).
		int childIndex = frame->childCount;
		int rangeFirst = frame->rangeFirst + ( childIndex == 0 ? 0 : frame->splitIndex );
		int rangeCount = childIndex == 0 ? frame->splitIndex : frame->rangeCount - frame->splitIndex;

		if ( rangeCount == 1 )
		{
			// Single leaf: read from the destination produced by frame's partition,
			// since that is where the partition wrote the items.
			int leafIndex = frame->dstIndices[rangeFirst];
			frame->childRoot[childIndex] = leafIndex;
			continue;
		}

		// Allocate child internals from the parent's range. Left first, then right.
		int leftLeaves = frame->splitIndex;
		int rightLeaves = frame->rangeCount - frame->splitIndex;
		int leftInternals = leftLeaves > 1 ? leftLeaves - 1 : 0;
		int rightInternals = rightLeaves > 1 ? rightLeaves - 1 : 0;
		(void)rightInternals;
		int childInternalFirst;
		int childInternalCount;
		if ( childIndex == 0 )
		{
			childInternalFirst = frame->internalFirst + 1;
			childInternalCount = leftInternals;
		}
		else
		{
			childInternalFirst = frame->internalFirst + 1 + leftInternals;
			childInternalCount = frame->internalCount - 1 - leftInternals;
		}

		B2_ASSERT( top + 1 < B2_TREE_STACK_SIZE );
		top += 1;
		// Children read from the parent's destination buffer (where the partition
		// wrote) and write to the parent's source buffer.
		stack[top] = (b2BuildSerialFrame){
			.nodeIndex = childInternalFirst,
			.childCount = -1,
			.rangeFirst = rangeFirst,
			.rangeCount = rangeCount,
			.internalFirst = childInternalFirst,
			.internalCount = childInternalCount,
			.srcIndices = frame->dstIndices,
			.srcCenters = frame->dstCenters,
			.dstIndices = frame->srcIndices,
			.dstCenters = frame->srcCenters,
		};
		b2BuildSerialPartitionAndSplit( &stack[top] );
	}
}

static void b2BuildRecursive( b2BuildContext* ctx )
{
	if ( ctx->rangeCount == 1 )
	{
		b2TreeNode* nodes = ctx->tree->nodes;
		int leafIndex = ctx->srcIndices[ctx->rangeFirst];
		ctx->rootIndex = leafIndex;
		ctx->rootAabb = nodes[leafIndex].aabb;
		ctx->rootHeight = 0;
		ctx->rootCategoryBits = nodes[leafIndex].categoryBits;
		return;
	}

	if ( ctx->rangeCount <= B2_BUILD_SUBTREE_TASK_THRESHOLD || ctx->world == NULL || ctx->world->workerCount == 1 )
	{
		b2BuildSubtreeSerial( ctx );
		return;
	}

	b2Vec2 lower, upper;
	b2BuildCentroidBounds( ctx, &lower, &upper );

	float dx = upper.x - lower.x;
	float dy = upper.y - lower.y;
	int axis = dx > dy ? 0 : 1;
	float pivot = 0.5f * ( axis == 0 ? lower.x + upper.x : lower.y + upper.y );

	int leftCount = b2BuildPartition( ctx, axis, pivot );
	int rightCount = ctx->rangeCount - leftCount;

	// Pathological centroid layout: force a balanced split by index. The dst
	// buffer already holds a stable-order copy of src, so we just choose the
	// midpoint; no re-scatter needed.
	if ( leftCount == 0 || rightCount == 0 )
	{
		// Re-emit dst as a copy of src in source order, then split at n/2.
		// b2BuildPartition already wrote dst[0..totalLeft) and dst[totalLeft..n);
		// for a degenerate predicate one side is empty and the other holds all
		// items in source order, so dst is correct as-is.
		leftCount = ctx->rangeCount / 2;
		rightCount = ctx->rangeCount - leftCount;
	}

	int thisInternal = ctx->internalFirst;
	int leftInternals = leftCount > 1 ? leftCount - 1 : 0;
	int rightInternals = rightCount > 1 ? rightCount - 1 : 0;
	(void)rightInternals;

	b2BuildContext leftCtx = {
		.tree = ctx->tree,
		.world = ctx->world,
		.srcIndices = ctx->dstIndices,
		.srcCenters = ctx->dstCenters,
		.dstIndices = ctx->srcIndices,
		.dstCenters = ctx->srcCenters,
		.rangeFirst = ctx->rangeFirst,
		.rangeCount = leftCount,
		.internalFirst = ctx->internalFirst + 1,
		.internalCount = leftInternals,
	};
	b2BuildContext rightCtx = {
		.tree = ctx->tree,
		.world = ctx->world,
		.srcIndices = ctx->dstIndices,
		.srcCenters = ctx->dstCenters,
		.dstIndices = ctx->srcIndices,
		.dstCenters = ctx->srcCenters,
		.rangeFirst = ctx->rangeFirst + leftCount,
		.rangeCount = rightCount,
		.internalFirst = ctx->internalFirst + 1 + leftInternals,
		.internalCount = ctx->internalCount - 1 - leftInternals,
	};

	// Continue-the-bigger-half locally, spawn the smaller half as a task.
	// Bounds live tasks at O(log N) and gives us idle workers for the bigger
	// half's parallel partitions.
	b2BuildContext* bigger = leftCount >= rightCount ? &leftCtx : &rightCtx;
	b2BuildContext* smaller = leftCount >= rightCount ? &rightCtx : &leftCtx;
	b2World* world = ctx->world;

	int spawn = smaller->rangeCount > B2_BUILD_SUBTREE_TASK_THRESHOLD && world->taskCount < B2_MAX_TASKS;
	if ( spawn )
	{
		// If the task system runs synchronously it returns NULL from enqueue
		// (the task is already complete by the time enqueue returns). Either
		// way, smaller's work is done before bigger starts only if NULL was
		// returned; the async case overlaps with the bigger build below.
		void* smallerTask = world->enqueueTaskFcn( &b2BuildTrampoline, smaller, world->userTaskContext );
		world->taskCount += 1;
		b2BuildRecursive( bigger );
		if ( smallerTask != NULL )
		{
			world->finishTaskFcn( smallerTask, world->userTaskContext );
		}
	}
	else
	{
		b2BuildRecursive( &leftCtx );
		b2BuildRecursive( &rightCtx );
	}

	b2TreeNode* nodes = ctx->tree->nodes;
	int leftRoot = leftCtx.rootIndex;
	int rightRoot = rightCtx.rootIndex;

	b2TreeNode* node = nodes + thisInternal;
	node->aabb = b2AABB_Union( leftCtx.rootAabb, rightCtx.rootAabb );
	node->height = 1 + b2MaxUInt16( leftCtx.rootHeight, rightCtx.rootHeight );
	node->categoryBits = leftCtx.rootCategoryBits | rightCtx.rootCategoryBits;
	node->parent = B2_NULL_INDEX;
	node->userData = 0;
	node->children.child1 = leftRoot;
	node->children.child2 = rightRoot;
	node->flags = b2_allocatedNode;
	nodes[leftRoot].parent = thisInternal;
	nodes[rightRoot].parent = thisInternal;

	ctx->rootIndex = thisInternal;
	ctx->rootAabb = node->aabb;
	ctx->rootHeight = node->height;
	ctx->rootCategoryBits = node->categoryBits;
}

// ---------------------------------------------------------------------------
// Public entry point.
// ---------------------------------------------------------------------------

void b2DynamicTree_BuildFromLeaves( b2DynamicTree* tree, const b2AABB* leafAabbs, const uint64_t* leafUserData,
									const uint64_t* leafCategoryBits, int leafCount, b2World* world )
{
	B2_ASSERT( tree != NULL );
	B2_ASSERT( leafCount >= 0 );
	B2_ASSERT( leafCount == 0 || leafAabbs != NULL );

	if ( leafCount == 0 )
	{
		// Reset to empty.
		int oldCapacity = tree->nodeCapacity;
		for ( int i = 0; i < oldCapacity - 1; ++i )
		{
			tree->nodes[i].next = i + 1;
			tree->nodes[i].flags = 0;
		}
		tree->nodes[oldCapacity - 1].next = B2_NULL_INDEX;
		tree->nodes[oldCapacity - 1].flags = 0;
		tree->freeList = 0;
		tree->nodeCount = 0;
		tree->proxyCount = 0;
		tree->root = B2_NULL_INDEX;
		return;
	}

	// Phase 1: ensure the node pool has room for leafCount + (leafCount - 1) nodes.
	int totalNodes = leafCount == 1 ? 1 : 2 * leafCount - 1;
	if ( tree->nodeCapacity < totalNodes )
	{
		b2Free( tree->nodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
		int newCapacity = totalNodes;
		tree->nodes = (b2TreeNode*)b2Alloc( newCapacity * sizeof( b2TreeNode ) );
		// Zero so that uninitialized internal-node slots have deterministic
		// padding bytes (we overwrite all relevant fields before reading).
		memset( tree->nodes, 0, newCapacity * sizeof( b2TreeNode ) );
		tree->nodeCapacity = newCapacity;
	}

	// Slots [totalNodes, nodeCapacity) are unused by this build. Chain them
	// onto the freelist so future incremental ops can use them and so the
	// nodeCount + freeCount == nodeCapacity invariant holds.
	tree->nodeCount = totalNodes;
	tree->proxyCount = leafCount;
	if ( tree->nodeCapacity > totalNodes )
	{
		for ( int i = totalNodes; i < tree->nodeCapacity - 1; ++i )
		{
			tree->nodes[i].next = i + 1;
			tree->nodes[i].flags = 0;
		}
		tree->nodes[tree->nodeCapacity - 1].next = B2_NULL_INDEX;
		tree->nodes[tree->nodeCapacity - 1].flags = 0;
		tree->freeList = totalNodes;
	}
	else
	{
		tree->freeList = B2_NULL_INDEX;
	}

	// Ensure rebuild scratch capacity for ping-pong buffers.
	if ( leafCount > tree->rebuildCapacity )
	{
		int newCapacity = leafCount + leafCount / 2;
		b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
		b2Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b2Vec2 ) );
		b2Free( tree->leafIndicesAlt, tree->rebuildCapacity * sizeof( int32_t ) );
		b2Free( tree->leafCentersAlt, tree->rebuildCapacity * sizeof( b2Vec2 ) );
		tree->leafIndices = (int32_t*)b2Alloc( newCapacity * sizeof( int32_t ) );
		tree->leafCenters = (b2Vec2*)b2Alloc( newCapacity * sizeof( b2Vec2 ) );
		tree->leafIndicesAlt = (int32_t*)b2Alloc( newCapacity * sizeof( int32_t ) );
		tree->leafCentersAlt = (b2Vec2*)b2Alloc( newCapacity * sizeof( b2Vec2 ) );
		tree->rebuildCapacity = newCapacity;
	}
	else
	{
		// Lazily allocate alt buffers if the existing rebuild path created the
		// primaries but never used the parallel build before.
		if ( tree->leafIndicesAlt == NULL )
		{
			tree->leafIndicesAlt = (int32_t*)b2Alloc( tree->rebuildCapacity * sizeof( int32_t ) );
		}
		if ( tree->leafCentersAlt == NULL )
		{
			tree->leafCentersAlt = (b2Vec2*)b2Alloc( tree->rebuildCapacity * sizeof( b2Vec2 ) );
		}
		if ( tree->leafIndices == NULL )
		{
			tree->leafIndices = (int32_t*)b2Alloc( tree->rebuildCapacity * sizeof( int32_t ) );
		}
		if ( tree->leafCenters == NULL )
		{
			tree->leafCenters = (b2Vec2*)b2Alloc( tree->rebuildCapacity * sizeof( b2Vec2 ) );
		}
	}

	// Phase 2: initialize leaves and seed scratch.
	b2BuildLeafInitShared init = {
		.nodes = tree->nodes,
		.aabbs = leafAabbs,
		.userData = leafUserData,
		.categoryBits = leafCategoryBits,
		.indices = tree->leafIndices,
		.centers = tree->leafCenters,
	};
	if ( world != NULL && world->workerCount > 1 && leafCount >= B2_BUILD_MIN_PARALLEL_LEAVES )
	{
		b2ParallelFor( world, b2BuildLeafInitTask, leafCount, 1024, &init );
	}
	else
	{
		b2BuildLeafInitRange( &init, 0, leafCount );
	}

	if ( leafCount == 1 )
	{
		tree->root = 0;
		b2DynamicTree_Validate( tree );
		return;
	}

	// Phase 3: recursive build.
	b2World* effectiveWorld = ( world != NULL && world->workerCount > 1 && leafCount >= B2_BUILD_MIN_PARALLEL_LEAVES ) ? world : NULL;

	b2BuildContext rootCtx = {
		.tree = tree,
		.world = effectiveWorld,
		.srcIndices = tree->leafIndices,
		.srcCenters = tree->leafCenters,
		.dstIndices = tree->leafIndicesAlt,
		.dstCenters = tree->leafCentersAlt,
		.rangeFirst = 0,
		.rangeCount = leafCount,
		.internalFirst = leafCount,
		.internalCount = leafCount - 1,
	};
	b2BuildRecursive( &rootCtx );

	// Phase 4: finalize.
	tree->root = rootCtx.rootIndex;
	tree->nodes[tree->root].parent = B2_NULL_INDEX;

	b2DynamicTree_Validate( tree );
}
