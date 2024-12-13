// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "constants.h"
#include "core.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <string.h>

#define B2_TREE_STACK_SIZE 1024

static b2TreeNode b2_defaultTreeNode = {
	.aabb = { { 0.0f, 0.0f }, { 0.0f, 0.0f } },
	.categoryBits = B2_DEFAULT_CATEGORY_BITS,
	.parent = B2_NULL_INDEX,
	.child1 = B2_NULL_INDEX,
	.child2 = B2_NULL_INDEX,
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

b2DynamicTree b2DynamicTree_Create( void )
{
	b2DynamicTree tree;
	tree.root = B2_NULL_INDEX;

	tree.nodeCapacity = 16;
	tree.nodeCount = 0;
	tree.nodes = (b2TreeNode*)b2Alloc( tree.nodeCapacity * sizeof( b2TreeNode ) );
	memset( tree.nodes, 0, tree.nodeCapacity * sizeof( b2TreeNode ) );

	// Build a linked list for the free list.
	for ( int32_t i = 0; i < tree.nodeCapacity - 1; ++i )
	{
		tree.nodes[i].next = i + 1;
	}

	tree.nodes[tree.nodeCapacity - 1].next = B2_NULL_INDEX;
	tree.freeList = 0;

	tree.proxyCount = 0;

	tree.leafIndices = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
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
	b2Free( tree->binIndices, tree->rebuildCapacity * sizeof( int32_t ) );

	memset( tree, 0, sizeof( b2DynamicTree ) );
}

// Allocate a node from the pool. Grow the pool if necessary.
static int32_t b2AllocateNode( b2DynamicTree* tree )
{
	// Expand the node pool as needed.
	if ( tree->freeList == B2_NULL_INDEX )
	{
		B2_ASSERT( tree->nodeCount == tree->nodeCapacity );

		// The free list is empty. Rebuild a bigger pool.
		b2TreeNode* oldNodes = tree->nodes;
		int32_t oldCapacity = tree->nodeCapacity;
		tree->nodeCapacity += oldCapacity >> 1;
		tree->nodes = (b2TreeNode*)b2Alloc( tree->nodeCapacity * sizeof( b2TreeNode ) );
		B2_ASSERT( oldNodes != NULL );
		memcpy( tree->nodes, oldNodes, tree->nodeCount * sizeof( b2TreeNode ) );
		memset( tree->nodes + tree->nodeCount, 0, ( tree->nodeCapacity - tree->nodeCount ) * sizeof( b2TreeNode ) );
		b2Free( oldNodes, oldCapacity * sizeof( b2TreeNode ) );

		// Build a linked list for the free list. The parent pointer becomes the "next" pointer.
		// todo avoid building freelist?
		for ( int32_t i = tree->nodeCount; i < tree->nodeCapacity - 1; ++i )
		{
			tree->nodes[i].next = i + 1;
		}

		tree->nodes[tree->nodeCapacity - 1].next = B2_NULL_INDEX;
		tree->freeList = tree->nodeCount;
	}

	// Peel a node off the free list.
	int32_t nodeIndex = tree->freeList;
	b2TreeNode* node = tree->nodes + nodeIndex;
	tree->freeList = node->next;
	*node = b2_defaultTreeNode;
	++tree->nodeCount;
	return nodeIndex;
}

// Return a node to the pool.
static void b2FreeNode( b2DynamicTree* tree, int32_t nodeId )
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
static int32_t b2FindBestSibling( const b2DynamicTree* tree, b2AABB boxD )
{
	b2Vec2 centerD = b2AABB_Center( boxD );
	float areaD = b2Perimeter( boxD );

	const b2TreeNode* nodes = tree->nodes;
	int32_t rootIndex = tree->root;

	b2AABB rootBox = nodes[rootIndex].aabb;

	// Area of current node
	float areaBase = b2Perimeter( rootBox );

	// Area of inflated node
	float directCost = b2Perimeter( b2AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;

	int32_t bestSibling = rootIndex;
	float bestCost = directCost;

	// Descend the tree from root, following a single greedy path.
	int32_t index = rootIndex;
	while ( nodes[index].height > 0 )
	{
		int32_t child1 = nodes[index].child1;
		int32_t child2 = nodes[index].child2;

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

			// Lower bound cost of inserting under child 1.
			lowerCost1 = inheritedCost + directCost1 + b2MinFloat( areaD - area1, 0.0f );
		}

		// Cost of descending into child 2
		float lowerCost2 = FLT_MAX;
		b2AABB box2 = nodes[child2].aabb;
		float directCost2 = b2Perimeter( b2AABB_Union( box2, boxD ) );
		float area2 = 0.0f;
		if ( leaf2 )
		{
			// Child 2 is a leaf
			// Cost of creating new node and increasing area of node P
			float cost2 = directCost2 + inheritedCost;

			// Need this here due to while condition above
			if ( cost2 < bestCost )
			{
				bestSibling = child2;
				bestCost = cost2;
			}
		}
		else
		{
			// Child 2 is an internal node
			area2 = b2Perimeter( box2 );

			// Lower bound cost of inserting under child 2. This is not the cost
			// of child 2, it is the best we can hope for under child 2.
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
static void b2RotateNodes( b2DynamicTree* tree, int32_t iA )
{
	B2_ASSERT( iA != B2_NULL_INDEX );

	b2TreeNode* nodes = tree->nodes;

	b2TreeNode* A = nodes + iA;
	if ( A->height < 2 )
	{
		return;
	}

	int32_t iB = A->child1;
	int32_t iC = A->child2;
	B2_ASSERT( 0 <= iB && iB < tree->nodeCapacity );
	B2_ASSERT( 0 <= iC && iC < tree->nodeCapacity );

	b2TreeNode* B = nodes + iB;
	b2TreeNode* C = nodes + iC;

	if ( B->height == 0 )
	{
		// B is a leaf and C is internal
		B2_ASSERT( C->height > 0 );

		int32_t iF = C->child1;
		int32_t iG = C->child2;
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
			A->child1 = iF;
			C->child1 = iB;

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
			A->child1 = iG;
			C->child2 = iB;

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

		int iD = B->child1;
		int iE = B->child2;
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
			A->child2 = iD;
			B->child1 = iC;

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
			A->child2 = iE;
			B->child2 = iC;

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
		int iD = B->child1;
		int iE = B->child2;
		int iF = C->child1;
		int iG = C->child2;

		b2TreeNode* D = nodes + iD;
		b2TreeNode* E = nodes + iE;
		b2TreeNode* F = nodes + iF;
		b2TreeNode* G = nodes + iG;

		B2_ASSERT( 0 <= iD && iD < tree->nodeCapacity );
		B2_ASSERT( 0 <= iE && iE < tree->nodeCapacity );
		B2_ASSERT( 0 <= iF && iF < tree->nodeCapacity );
		B2_ASSERT( 0 <= iG && iG < tree->nodeCapacity );

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
				A->child1 = iF;
				C->child1 = iB;

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
				A->child1 = iG;
				C->child2 = iB;

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
				A->child2 = iD;
				B->child1 = iC;

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
				A->child2 = iE;
				B->child2 = iC;

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

static void b2InsertLeaf( b2DynamicTree* tree, int32_t leaf, bool shouldRotate )
{
	if ( tree->root == B2_NULL_INDEX )
	{
		tree->root = leaf;
		tree->nodes[tree->root].parent = B2_NULL_INDEX;
		return;
	}

	// Stage 1: find the best sibling for this node
	b2AABB leafAABB = tree->nodes[leaf].aabb;
	int32_t sibling = b2FindBestSibling( tree, leafAABB );

	// Stage 2: create a new parent for the leaf and sibling
	int32_t oldParent = tree->nodes[sibling].parent;
	int32_t newParent = b2AllocateNode( tree );

	// warning: node pointer can change after allocation
	b2TreeNode* nodes = tree->nodes;
	nodes[newParent].parent = oldParent;
	nodes[newParent].userData = -1;
	nodes[newParent].aabb = b2AABB_Union( leafAABB, nodes[sibling].aabb );
	nodes[newParent].categoryBits = nodes[leaf].categoryBits | nodes[sibling].categoryBits;
	nodes[newParent].height = nodes[sibling].height + 1;

	if ( oldParent != B2_NULL_INDEX )
	{
		// The sibling was not the root.
		if ( nodes[oldParent].child1 == sibling )
		{
			nodes[oldParent].child1 = newParent;
		}
		else
		{
			nodes[oldParent].child2 = newParent;
		}

		nodes[newParent].child1 = sibling;
		nodes[newParent].child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		nodes[newParent].child1 = sibling;
		nodes[newParent].child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
		tree->root = newParent;
	}

	// Stage 3: walk back up the tree fixing heights and AABBs
	int32_t index = nodes[leaf].parent;
	while ( index != B2_NULL_INDEX )
	{
		int32_t child1 = nodes[index].child1;
		int32_t child2 = nodes[index].child2;

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

static void b2RemoveLeaf( b2DynamicTree* tree, int32_t leaf )
{
	if ( leaf == tree->root )
	{
		tree->root = B2_NULL_INDEX;
		return;
	}

	b2TreeNode* nodes = tree->nodes;

	int32_t parent = nodes[leaf].parent;
	int32_t grandParent = nodes[parent].parent;
	int32_t sibling;
	if ( nodes[parent].child1 == leaf )
	{
		sibling = nodes[parent].child2;
	}
	else
	{
		sibling = nodes[parent].child1;
	}

	if ( grandParent != B2_NULL_INDEX )
	{
		// Destroy parent and connect sibling to grandParent.
		if ( nodes[grandParent].child1 == parent )
		{
			nodes[grandParent].child1 = sibling;
		}
		else
		{
			nodes[grandParent].child2 = sibling;
		}
		nodes[sibling].parent = grandParent;
		b2FreeNode( tree, parent );

		// Adjust ancestor bounds.
		int32_t index = grandParent;
		while ( index != B2_NULL_INDEX )
		{
			b2TreeNode* node = nodes + index;
			b2TreeNode* child1 = nodes + node->child1;
			b2TreeNode* child2 = nodes + node->child2;

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
int32_t b2DynamicTree_CreateProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, int32_t userData )
{
	B2_ASSERT( -B2_HUGE < aabb.lowerBound.x && aabb.lowerBound.x < B2_HUGE );
	B2_ASSERT( -B2_HUGE < aabb.lowerBound.y && aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( -B2_HUGE < aabb.upperBound.x && aabb.upperBound.x < B2_HUGE );
	B2_ASSERT( -B2_HUGE < aabb.upperBound.y && aabb.upperBound.y < B2_HUGE );

	int32_t proxyId = b2AllocateNode( tree );
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

void b2DynamicTree_DestroyProxy( b2DynamicTree* tree, int32_t proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B2_ASSERT( b2IsLeaf( tree->nodes + proxyId ) );

	b2RemoveLeaf( tree, proxyId );
	b2FreeNode( tree, proxyId );

	B2_ASSERT( tree->proxyCount > 0 );
	tree->proxyCount -= 1;
}

int32_t b2DynamicTree_GetProxyCount( const b2DynamicTree* tree )
{
	return tree->proxyCount;
}

void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int32_t proxyId, b2AABB aabb )
{
	B2_ASSERT( b2IsValidAABB( aabb ) );
	B2_ASSERT( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_ASSERT( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B2_ASSERT( b2IsLeaf( tree->nodes + proxyId ) );

	b2RemoveLeaf( tree, proxyId );

	tree->nodes[proxyId].aabb = aabb;

	bool shouldRotate = false;
	b2InsertLeaf( tree, proxyId, shouldRotate );
}

void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int32_t proxyId, b2AABB aabb )
{
	b2TreeNode* nodes = tree->nodes;

	B2_ASSERT( b2IsValidAABB( aabb ) );
	B2_ASSERT( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_ASSERT( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->nodeCapacity );
	B2_ASSERT( b2IsLeaf( tree->nodes + proxyId ) );

	// Caller must ensure this
	B2_ASSERT( b2AABB_Contains( nodes[proxyId].aabb, aabb ) == false );

	nodes[proxyId].aabb = aabb;

	int32_t parentIndex = nodes[proxyId].parent;
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
	for ( int32_t i = 0; i < tree->nodeCapacity; ++i )
	{
		const b2TreeNode* node = tree->nodes + i;
		if ( b2IsAllocated(node) == false || b2IsLeaf( node ) || i == tree->root )
		{
			continue;
		}

		totalArea += b2Perimeter( node->aabb );
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
static int b2ComputeHeight( const b2DynamicTree* tree, int32_t nodeId )
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	b2TreeNode* node = tree->nodes + nodeId;

	if ( b2IsLeaf( node ) )
	{
		return 0;
	}

	int32_t height1 = b2ComputeHeight( tree, node->child1 );
	int32_t height2 = b2ComputeHeight( tree, node->child2 );
	return 1 + b2MaxInt( height1, height2 );
}

int b2DynamicTree_ComputeHeight( const b2DynamicTree* tree )
{
	int height = b2ComputeHeight( tree, tree->root );
	return height;
}

#if B2_VALIDATE
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

	int32_t child1 = node->child1;
	int32_t child2 = node->child2;

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

static void b2ValidateMetrics( const b2DynamicTree* tree, int32_t index )
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

	int child1 = node->child1;
	int child2 = node->child2;

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
#if B2_VALIDATE
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
	int computedHeight = b2DynamicTree_ComputeHeight( tree );
	B2_ASSERT( height == computedHeight );

	B2_ASSERT( tree->nodeCount + freeCount == tree->nodeCapacity );
#else
	B2_MAYBE_UNUSED( tree );
#endif
}

int b2DynamicTree_GetByteCount( const b2DynamicTree* tree )
{
	size_t size = sizeof( b2DynamicTree ) + sizeof( b2TreeNode ) * tree->nodeCapacity +
				  tree->rebuildCapacity * ( sizeof( int32_t ) + sizeof( b2AABB ) + sizeof( b2Vec2 ) + sizeof( int32_t ) );

	return (int)size;
}

b2TreeStats b2DynamicTree_Query( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits, b2TreeQueryCallbackFcn* callback,
								 void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	int32_t stack[B2_TREE_STACK_SIZE];
	int32_t stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int32_t nodeId = stack[--stackCount];
		if ( nodeId == B2_NULL_INDEX )
		{
			// todo huh?
			B2_ASSERT( false );
			continue;
		}

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
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = node->child1;
					stack[stackCount++] = node->child2;
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

	int32_t stack[B2_TREE_STACK_SIZE];
	int32_t stackCount = 0;
	stack[stackCount++] = tree->root;

	const b2TreeNode* nodes = tree->nodes;

	b2RayCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int32_t nodeId = stack[--stackCount];
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
			B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
			if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				b2Vec2 c1 = b2AABB_Center( nodes[node->child1].aabb );
				b2Vec2 c2 = b2AABB_Center( nodes[node->child2].aabb );
				if ( b2DistanceSquared( c1, p1 ) < b2DistanceSquared( c2, p1 ) )
				{
					stack[stackCount++] = node->child2;
					stack[stackCount++] = node->child1;
				}
				else
				{
					stack[stackCount++] = node->child1;
					stack[stackCount++] = node->child2;
				}
			}
		}
	}

	return result;
}

b2TreeStats b2DynamicTree_ShapeCast( const b2DynamicTree* tree, const b2ShapeCastInput* input, uint64_t maskBits,
									 b2TreeShapeCastCallbackFcn* callback, void* context )
{
	b2TreeStats stats = { 0 };

	if ( tree->nodeCount == 0 || input->count == 0 )
	{
		return stats;
	}

	b2AABB originAABB = { input->points[0], input->points[0] };
	for ( int i = 1; i < input->count; ++i )
	{
		originAABB.lowerBound = b2Min( originAABB.lowerBound, input->points[i] );
		originAABB.upperBound = b2Max( originAABB.upperBound, input->points[i] );
	}

	b2Vec2 radius = { input->radius, input->radius };

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

	int32_t stack[B2_TREE_STACK_SIZE];
	int32_t stackCount = 0;
	stack[stackCount++] = tree->root;

	while ( stackCount > 0 )
	{
		int32_t nodeId = stack[--stackCount];
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
			B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
			if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				b2Vec2 c1 = b2AABB_Center( nodes[node->child1].aabb );
				b2Vec2 c2 = b2AABB_Center( nodes[node->child2].aabb );
				if ( b2DistanceSquared( c1, p1 ) < b2DistanceSquared( c2, p1 ) )
				{
					stack[stackCount++] = node->child2;
					stack[stackCount++] = node->child1;
				}
				else
				{
					stack[stackCount++] = node->child1;
					stack[stackCount++] = node->child2;
				}
			}
		}
	}

	return stats;
}

// Median split == 0, Surface area heuristic == 1
#define B2_TREE_HEURISTIC 0

#if B2_TREE_HEURISTIC == 0

// Median split heuristic
static int32_t b2PartitionMid( int32_t* indices, b2Vec2* centers, int32_t count )
{
	// Handle trivial case
	if ( count <= 2 )
	{
		return count / 2;
	}

	b2Vec2 lowerBound = centers[0];
	b2Vec2 upperBound = centers[0];

	for ( int32_t i = 1; i < count; ++i )
	{
		lowerBound = b2Min( lowerBound, centers[i] );
		upperBound = b2Max( upperBound, centers[i] );
	}

	b2Vec2 d = b2Sub( upperBound, lowerBound );
	b2Vec2 c = { 0.5f * ( lowerBound.x + upperBound.x ), 0.5f * ( lowerBound.y + upperBound.y ) };

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int32_t i1 = 0, i2 = count;
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
					int32_t temp = indices[i1];
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
					int32_t temp = indices[i1];
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

#define B2_BIN_COUNT 64

typedef struct b2TreeBin
{
	b2AABB aabb;
	int32_t count;
} b2TreeBin;

typedef struct b2TreePlane
{
	b2AABB leftAABB;
	b2AABB rightAABB;
	int32_t leftCount;
	int32_t rightCount;
} b2TreePlane;

// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
// Returns the left child count
static int32_t b2PartitionSAH( int32_t* indices, int32_t* binIndices, b2AABB* boxes, int32_t count )
{
	B2_ASSERT( count > 0 );

	b2TreeBin bins[B2_BIN_COUNT];
	b2TreePlane planes[B2_BIN_COUNT - 1];

	b2Vec2 center = b2AABB_Center( boxes[0] );
	b2AABB centroidAABB;
	centroidAABB.lowerBound = center;
	centroidAABB.upperBound = center;

	for ( int32_t i = 1; i < count; ++i )
	{
		center = b2AABB_Center( boxes[i] );
		centroidAABB.lowerBound = b2Min( centroidAABB.lowerBound, center );
		centroidAABB.upperBound = b2Max( centroidAABB.upperBound, center );
	}

	b2Vec2 d = b2Sub( centroidAABB.upperBound, centroidAABB.lowerBound );

	// Find longest axis
	int32_t axisIndex;
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
	for ( int32_t i = 0; i < B2_BIN_COUNT; ++i )
	{
		bins[i].aabb.lowerBound = ( b2Vec2 ){ FLT_MAX, FLT_MAX };
		bins[i].aabb.upperBound = ( b2Vec2 ){ -FLT_MAX, -FLT_MAX };
		bins[i].count = 0;
	}

	// Assign boxes to bins and compute bin boxes
	// TODO_ERIN optimize
	float binCount = B2_BIN_COUNT;
	float lowerBoundArray[2] = { centroidAABB.lowerBound.x, centroidAABB.lowerBound.y };
	float minC = lowerBoundArray[axisIndex];
	for ( int32_t i = 0; i < count; ++i )
	{
		b2Vec2 c = b2AABB_Center( boxes[i] );
		float cArray[2] = { c.x, c.y };
		int32_t binIndex = (int32_t)( binCount * ( cArray[axisIndex] - minC ) * invD );
		binIndex = b2ClampInt( binIndex, 0, B2_BIN_COUNT - 1 );
		binIndices[i] = binIndex;
		bins[binIndex].count += 1;
		bins[binIndex].aabb = b2AABB_Union( bins[binIndex].aabb, boxes[i] );
	}

	int32_t planeCount = B2_BIN_COUNT - 1;

	// Prepare all the left planes, candidates for left child
	planes[0].leftCount = bins[0].count;
	planes[0].leftAABB = bins[0].aabb;
	for ( int32_t i = 1; i < planeCount; ++i )
	{
		planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
		planes[i].leftAABB = b2AABB_Union( planes[i - 1].leftAABB, bins[i].aabb );
	}

	// Prepare all the right planes, candidates for right child
	planes[planeCount - 1].rightCount = bins[planeCount].count;
	planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
	for ( int32_t i = planeCount - 2; i >= 0; --i )
	{
		planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
		planes[i].rightAABB = b2AABB_Union( planes[i + 1].rightAABB, bins[i + 1].aabb );
	}

	// Find best split to minimize SAH
	float minCost = FLT_MAX;
	int32_t bestPlane = 0;
	for ( int32_t i = 0; i < planeCount; ++i )
	{
		float leftArea = b2Perimeter( planes[i].leftAABB );
		float rightArea = b2Perimeter( planes[i].rightAABB );
		int32_t leftCount = planes[i].leftCount;
		int32_t rightCount = planes[i].rightCount;

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
	int32_t i1 = 0, i2 = count;
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
				int32_t temp = indices[i1];
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
	int32_t nodeIndex;
	int32_t childCount;

	// Leaf indices
	int32_t startIndex;
	int32_t splitIndex;
	int32_t endIndex;
};

// Returns root node index
static int32_t b2BuildTree( b2DynamicTree* tree, int32_t leafCount )
{
	b2TreeNode* nodes = tree->nodes;
	int32_t* leafIndices = tree->leafIndices;

	if ( leafCount == 1 )
	{
		nodes[leafIndices[0]].parent = B2_NULL_INDEX;
		return leafIndices[0];
	}

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
	int32_t* binIndices = tree->binIndices;
#endif

	// todo large stack item
	struct b2RebuildItem stack[B2_TREE_STACK_SIZE];
	int32_t top = 0;

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
				B2_ASSERT( parentNode->child1 == B2_NULL_INDEX );
				parentNode->child1 = item->nodeIndex;
			}
			else
			{
				B2_ASSERT( parentItem->childCount == 1 );
				B2_ASSERT( parentNode->child2 == B2_NULL_INDEX );
				parentNode->child2 = item->nodeIndex;
			}

			b2TreeNode* node = nodes + item->nodeIndex;

			B2_ASSERT( node->parent == B2_NULL_INDEX );
			node->parent = parentItem->nodeIndex;

			B2_ASSERT( node->child1 != B2_NULL_INDEX );
			B2_ASSERT( node->child2 != B2_NULL_INDEX );
			b2TreeNode* child1 = nodes + node->child1;
			b2TreeNode* child2 = nodes + node->child2;

			node->aabb = b2AABB_Union( child1->aabb, child2->aabb );
			node->height = 1 + b2MaxUInt16( child1->height, child2->height );
			node->categoryBits = child1->categoryBits | child2->categoryBits;

			// Pop stack
			top -= 1;
		}
		else
		{
			int32_t startIndex, endIndex;
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

			int32_t count = endIndex - startIndex;

			if ( count == 1 )
			{
				int32_t childIndex = leafIndices[startIndex];
				b2TreeNode* node = nodes + item->nodeIndex;

				if ( item->childCount == 0 )
				{
					B2_ASSERT( node->child1 == B2_NULL_INDEX );
					node->child1 = childIndex;
				}
				else
				{
					B2_ASSERT( item->childCount == 1 );
					B2_ASSERT( node->child2 == B2_NULL_INDEX );
					node->child2 = childIndex;
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
	B2_ASSERT( rootNode->child1 != B2_NULL_INDEX );
	B2_ASSERT( rootNode->child2 != B2_NULL_INDEX );

	b2TreeNode* child1 = nodes + rootNode->child1;
	b2TreeNode* child2 = nodes + rootNode->child2;

	rootNode->aabb = b2AABB_Union( child1->aabb, child2->aabb );
	rootNode->height = 1 + b2MaxUInt16( child1->height, child2->height );
	rootNode->categoryBits = child1->categoryBits | child2->categoryBits;

	return stack[0].nodeIndex;
}

// Not safe to access tree during this operation because it may grow
int32_t b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild )
{
	int32_t proxyCount = tree->proxyCount;
	if ( proxyCount == 0 )
	{
		return 0;
	}

	// Ensure capacity for rebuild space
	if ( proxyCount > tree->rebuildCapacity )
	{
		int32_t newCapacity = proxyCount + proxyCount / 2;

		b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
		tree->leafIndices = b2Alloc( newCapacity * sizeof( int32_t ) );

#if B2_TREE_HEURISTIC == 0
		b2Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b2Vec2 ) );
		tree->leafCenters = b2Alloc( newCapacity * sizeof( b2Vec2 ) );
#else
		b2Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b2AABB ) );
		tree->leafBoxes = b2Alloc( newCapacity * sizeof( b2AABB ) );
		b2Free( tree->binIndices, tree->rebuildCapacity * sizeof( int32_t ) );
		tree->binIndices = b2Alloc( newCapacity * sizeof( int32_t ) );
#endif
		tree->rebuildCapacity = newCapacity;
	}

	int32_t leafCount = 0;
	int32_t stack[B2_TREE_STACK_SIZE];
	int32_t stackCount = 0;

	int32_t nodeIndex = tree->root;
	b2TreeNode* nodes = tree->nodes;
	b2TreeNode* node = nodes + nodeIndex;

	// These are the nodes that get sorted to rebuild the tree.
	// I'm using indices because the node pool may grow during the build.
	int32_t* leafIndices = tree->leafIndices;

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
#endif

	// Gather all proxy nodes that have grown and all internal nodes that haven't grown. Both are
	// considered leaves in the tree rebuild.
	// Free all internal nodes that have grown.
	// todo use a node growth metric instead of simply enlarged to reduce rebuild size and frequency
	// this should be weighed against b2_aabbMargin
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
			int32_t doomedNodeIndex = nodeIndex;

			// Handle children
			nodeIndex = node->child1;

			B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
			if ( stackCount < B2_TREE_STACK_SIZE )
			{
				stack[stackCount++] = node->child2;
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

#if B2_VALIDATE == 1
	int32_t capacity = tree->nodeCapacity;
	for ( int32_t i = 0; i < capacity; ++i )
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
