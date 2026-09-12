// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_tree.h"

#include "aabb.h"
#include "atomic.h"
#include "core.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <string.h>

_Static_assert( B2_ROOT_NODE == 0, "bad root" );
_Static_assert( sizeof( b2TreeNode ) == 32, "expected size" );
_Static_assert( sizeof( b2TreeProxy ) == 24, "expected size" );

// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
// allocations can just bump.
static int b2AllocateSiblingPair( b2DynamicTree* tree )
{
	// Allocate from free list.
	if ( tree->pairFreeList != B2_NULL_INDEX )
	{
		int pair = tree->pairFreeList;
		tree->pairFreeList = tree->parents[pair];
		return pair;
	}

	// Extend the bump allocator if needed.
	if ( tree->nodeEnd + 2 > tree->nodeCapacity )
	{
		int oldCapacity = tree->nodeCapacity;
		int newCapacity = oldCapacity + ( oldCapacity >> 1 );
		newCapacity += newCapacity & 1;
		tree->nodes = B2_GROW_ZERO( tree->nodes, oldCapacity, newCapacity );
		tree->parents = B2_GROW_ZERO( tree->parents, oldCapacity, newCapacity );
		tree->nodeCapacity = newCapacity;

		// The spare has to match, the rebuild allocates it again.
		b2Free( tree->swapNodes, oldCapacity * sizeof( b2TreeNode ) );
		tree->swapNodes = NULL;
	}

	int pair = tree->nodeEnd;
	tree->nodeEnd += 2;
	return pair;
}

static void b2FreePair( b2DynamicTree* tree, int pair )
{
	B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
	tree->nodes[pair] = b2MakeEmptyNode();
	tree->nodes[pair + 1] = b2MakeEmptyNode();
	tree->parents[pair] = tree->pairFreeList;
	tree->parents[pair + 1] = B2_NULL_INDEX;
	tree->pairFreeList = pair;
}

b2DynamicTree b2DynamicTree_Create( int proxyCapacity )
{
	int capacity = b2MaxInt( proxyCapacity, 16 );

	// Intentionally _not_ initialized with brace initialization, which can leave
	// uninitialized gaps.
	b2DynamicTree tree;

	// memset needed for deterministic serialization.
	memset( &tree, 0, sizeof( b2DynamicTree ) );

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
	tree.nodeCapacity = 2 * capacity;
	tree.nodes = (b2TreeNode*)b2AllocZero( tree.nodeCapacity * sizeof( b2TreeNode ) );
	tree.parents = (int32_t*)b2AllocZero( tree.nodeCapacity * sizeof( int32_t ) );
	tree.pairFreeList = B2_NULL_INDEX;

	// The root and the empty node always exist
	tree.nodes[B2_ROOT_NODE] = b2MakeEmptyNode();
	tree.nodes[B2_ROOT_NODE + 1] = b2MakeEmptyNode();
	tree.parents[B2_ROOT_NODE] = B2_NULL_INDEX;
	tree.parents[B2_ROOT_NODE + 1] = B2_NULL_INDEX;

	// This is the bump index.
	tree.nodeEnd = 2;
	tree.dfsOrdered = true;

	tree.proxyCapacity = capacity;
	tree.proxyCount = 0;
	tree.proxies = (b2TreeProxy*)b2AllocZero( tree.proxyCapacity * sizeof( b2TreeProxy ) );

	// Build a linked list for the free list.
	for ( int i = 0; i < tree.proxyCapacity - 1; ++i )
	{
		tree.proxies[i].node = B2_NULL_INDEX;
		tree.proxies[i].next = i + 1;
	}

	tree.proxies[tree.proxyCapacity - 1].node = B2_NULL_INDEX;
	tree.proxies[tree.proxyCapacity - 1].next = B2_NULL_INDEX;
	tree.proxyFreeList = 0;

	tree.leafIndices = NULL;
	tree.leafNodes = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
	tree.binIndices = NULL;
	tree.rebuildCapacity = 0;

	return tree;
}

void b2DynamicTree_Destroy( b2DynamicTree* tree )
{
	b2Free( tree->nodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->parents, tree->nodeCapacity * sizeof( int32_t ) );
	b2Free( tree->proxies, tree->proxyCapacity * sizeof( b2TreeProxy ) );
	b2Free( tree->swapNodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
	b2Free( tree->leafNodes, tree->rebuildCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b2AABB ) );
	b2Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b2Vec2 ) );
	b2Free( tree->binIndices, tree->rebuildCapacity * sizeof( int32_t ) );

	memset( tree, 0, sizeof( b2DynamicTree ) );
}

// Allocate a proxy from the pool. Grow the pool if necessary.
static int b2AllocateProxy( b2DynamicTree* tree )
{
	// Expand the pool as needed.
	if ( tree->proxyFreeList == B2_NULL_INDEX )
	{
		B2_ASSERT( tree->proxyCount == tree->proxyCapacity );

		// The free list is empty. Rebuild a bigger pool.
		int oldCapacity = tree->proxyCapacity;
		tree->proxyCapacity += oldCapacity >> 1;
		tree->proxies = B2_GROW_ZERO( tree->proxies, oldCapacity, tree->proxyCapacity );

		// Build a linked list for the free list.
		for ( int i = oldCapacity; i < tree->proxyCapacity - 1; ++i )
		{
			tree->proxies[i].node = B2_NULL_INDEX;
			tree->proxies[i].next = i + 1;
		}

		tree->proxies[tree->proxyCapacity - 1].node = B2_NULL_INDEX;
		tree->proxies[tree->proxyCapacity - 1].next = B2_NULL_INDEX;
		tree->proxyFreeList = oldCapacity;
	}

	// Peel a proxy off the free list.
	int proxyIndex = tree->proxyFreeList;
	tree->proxyFreeList = tree->proxies[proxyIndex].next;
	memset( tree->proxies + proxyIndex, 0, sizeof( b2TreeProxy ) );
	tree->proxies[proxyIndex].node = B2_NULL_INDEX;
	tree->proxies[proxyIndex].next = B2_NULL_INDEX;
	++tree->proxyCount;
	return proxyIndex;
}

// Return a proxy to the pool.
static void b2FreeProxy( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	B2_ASSERT( 0 < tree->proxyCount );
	tree->proxies[proxyId].node = B2_NULL_INDEX;
	tree->proxies[proxyId].next = tree->proxyFreeList;
	tree->proxyFreeList = proxyId;
	--tree->proxyCount;
}

static inline int b2GetLeafCount( const b2TreeNode* node )
{
	return b2IsLeaf( node ) ? 1 : node->leafCount;
}

// The internal node above a children pair
static inline b2TreeNode b2MakeInternalNode( const b2TreeNode* nodes, int pair )
{
	const b2TreeNode* c1 = nodes + pair;
	const b2TreeNode* c2 = nodes + pair + 1;

	b2TreeNode node = { 0 };
	node.aabb = b2AABB_Union( c1->aabb, c2->aabb );
	node.flagIndex = (uint32_t)pair | ( ( c1->flagIndex | c2->flagIndex ) & B2_MOVED_NODE );
	node.leafCount = b2GetLeafCount( c1 ) + b2GetLeafCount( c2 );
	return node;
}

static inline b2TreeNode b2MakeLeafNode( b2AABB aabb, int proxyId, uint64_t userData, bool moved )
{
	b2TreeNode node = { 0 };
	node.aabb = aabb;
	node.flagIndex = (uint32_t)proxyId | B2_LEAF_NODE | ( moved ? B2_MOVED_NODE : 0 );
	node.shapeIndex = (uint32_t)userData;
	return node;
}

// A node landed at a new index, so tell what hangs below it
static inline void b2LinkChildren( b2DynamicTree* tree, int nodeIndex )
{
	const b2TreeNode* node = tree->nodes + nodeIndex;
	if ( b2IsLeaf( node ) )
	{
		tree->proxies[b2GetProxyId( node )].node = nodeIndex;
	}
	else
	{
		int pair = b2GetLeftChild( node );
		tree->parents[pair] = nodeIndex;
		tree->parents[pair + 1] = nodeIndex;
	}
}

// The sweep refit visits indices from high to low, so it needs every child above its parent
static inline bool b2IsNodeOrdered( const b2TreeNode* nodes, int nodeIndex )
{
	const b2TreeNode* node = nodes + nodeIndex;
	return b2IsLeaf( node ) || nodeIndex < b2GetLeftChild( node );
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
	int nodeIndex = B2_ROOT_NODE;

	if ( b2IsLeaf( nodes + nodeIndex ) )
	{
		return nodeIndex;
	}

	b2AABB rootBox = nodes[nodeIndex].aabb;

	// Area of current node
	float areaBase = b2Perimeter( rootBox );

	// Area of inflated node
	float directCost = b2Perimeter( b2AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;

	int bestSibling = nodeIndex;
	float bestCost = directCost;

	// Descend the tree, following a single greedy path.
	for ( ;; )
	{
		int child1 = b2GetLeftChild( nodes + nodeIndex );
		int child2 = child1 + 1;

		// Cost of creating a new parent for this node and the new leaf
		float cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( cost < bestCost )
		{
			bestSibling = nodeIndex;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		bool leaf1 = b2IsLeaf( nodes + child1 );
		bool leaf2 = b2IsLeaf( nodes + child2 );

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
			nodeIndex = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			nodeIndex = child2;
			areaBase = area2;
			directCost = directCost2;
		}
	}

	return bestSibling;
}

// Swap a child of A with a grandchild under the other child.
// Example swapping B with G:
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
static void b2SwapNodes( b2DynamicTree* tree, int iDown, int iUp )
{
	b2TreeNode* nodes = tree->nodes;
	B2_SWAP( nodes[iDown], nodes[iUp] );
	b2LinkChildren( tree, iDown );
	b2LinkChildren( tree, iUp );

	if ( b2IsNodeOrdered( nodes, iDown ) == false || b2IsNodeOrdered( nodes, iUp ) == false )
	{
		tree->dfsOrdered = false;
	}

	// The sibling of the node that went down holds a different subtree now
	int iC = iDown ^ 1;
	nodes[iC] = b2MakeInternalNode( nodes, b2GetLeftChild( nodes + iC ) );
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
static void b2RotateNodes( b2DynamicTree* tree, int iA )
{
	b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* A = nodes + iA;
	B2_ASSERT( b2IsLeaf( A ) == false );

	int iB = b2GetLeftChild( A );
	int iC = iB + 1;
	const b2TreeNode* B = nodes + iB;
	const b2TreeNode* C = nodes + iC;

	bool leafB = b2IsLeaf( B );
	bool leafC = b2IsLeaf( C );
	if ( leafB && leafC )
	{
		return;
	}

	int bestDown = B2_NULL_INDEX;
	int bestUp = B2_NULL_INDEX;
	float bestDelta = 0.0f;

	if ( leafC == false )
	{
		// Swap B with F or G
		int iF = b2GetLeftChild( C );
		int iG = iF + 1;
		float areaC = b2Perimeter( C->aabb );

		// B <-> F then C (B G)
		float deltaBF = b2Perimeter( b2UnionV( B->aabb, nodes[iG].aabb ) ) - areaC;
		if ( deltaBF < bestDelta )
		{
			bestDown = iB;
			bestUp = iF;
			bestDelta = deltaBF;
		}

		// B <-> G then C (F B)
		float deltaBG = b2Perimeter( b2UnionV( B->aabb, nodes[iF].aabb ) ) - areaC;
		if ( deltaBG < bestDelta )
		{
			bestDown = iB;
			bestUp = iG;
			bestDelta = deltaBG;
		}
	}

	if ( leafB == false )
	{
		// Swap C with D or E
		int iD = b2GetLeftChild( B );
		int iE = iD + 1;
		float areaB = b2Perimeter( B->aabb );

		// C <-> D then B (C E)
		float deltaCD = b2Perimeter( b2UnionV( C->aabb, nodes[iE].aabb ) ) - areaB;
		if ( deltaCD < bestDelta )
		{
			bestDown = iC;
			bestUp = iD;
			bestDelta = deltaCD;
		}

		// C <-> E then B (D C)
		float deltaCE = b2Perimeter( b2UnionV( C->aabb, nodes[iD].aabb ) ) - areaB;
		if ( deltaCE < bestDelta )
		{
			bestDown = iC;
			bestUp = iE;
			bestDelta = deltaCE;
		}
	}

	if ( bestDown != B2_NULL_INDEX )
	{
		b2SwapNodes( tree, bestDown, bestUp );
	}
}

static void b2InsertLeaf( b2DynamicTree* tree, b2AABB aabb, int proxyId, bool moved, bool shouldRotate )
{
	b2TreeProxy* proxy = tree->proxies + proxyId;
	b2TreeNode leaf = b2MakeLeafNode( aabb, proxyId, proxy->userData, moved );

	if ( b2IsEmptyNode( tree->nodes + B2_ROOT_NODE ) )
	{
		tree->nodes[B2_ROOT_NODE] = leaf;
		proxy->node = B2_ROOT_NODE;
		return;
	}

	int sibling = b2FindBestSibling( tree, aabb );

	// The sibling's position becomes the new parent. The sibling moves down into a new pair
	// beside the leaf, which puts it below its own children when it is internal.
	int pair = b2AllocateSiblingPair( tree );
	b2TreeNode* nodes = tree->nodes;
	int32_t* parents = tree->parents;

	nodes[pair] = nodes[sibling];
	nodes[pair + 1] = leaf;
	parents[pair] = sibling;
	parents[pair + 1] = sibling;
	b2LinkChildren( tree, pair );
	proxy->node = pair + 1;

	nodes[sibling] = b2MakeInternalNode( nodes, pair );

	if ( b2IsNodeOrdered( nodes, sibling ) == false || b2IsNodeOrdered( nodes, pair ) == false )
	{
		tree->dfsOrdered = false;
	}

	// Walk back up the tree refitting ancestors, the root included.
	int index = sibling;
	while ( index != B2_NULL_INDEX )
	{
		if ( shouldRotate )
		{
			b2RotateNodes( tree, index );
		}

		nodes[index] = b2MakeInternalNode( nodes, b2GetLeftChild( nodes + index ) );
		index = parents[index];
	}
}

static void b2RemoveLeaf( b2DynamicTree* tree, int proxyId )
{
	b2TreeNode* nodes = tree->nodes;
	int32_t* parents = tree->parents;

	int leaf = tree->proxies[proxyId].node;
	B2_ASSERT( 0 <= leaf && leaf < tree->nodeEnd );
	B2_ASSERT( b2IsLeaf( nodes + leaf ) && b2GetProxyId( nodes + leaf ) == proxyId );

	if ( leaf == B2_ROOT_NODE )
	{
		nodes[B2_ROOT_NODE] = b2MakeEmptyNode();
		return;
	}

	// The sibling takes the parent's position and the pair is freed. This keeps the order,
	// the sibling only moves down.
	int parent = parents[leaf];
	nodes[parent] = nodes[leaf ^ 1];
	b2LinkChildren( tree, parent );
	b2FreePair( tree, leaf & ~1 );

	// Update ancestors.
	int index = parents[parent];
	while ( index != B2_NULL_INDEX )
	{
		nodes[index] = b2MakeInternalNode( nodes, b2GetLeftChild( nodes + index ) );
		index = parents[index];
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
int b2DynamicTree_CreateProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, uint64_t userData, bool markMoved )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );

	int proxyId = b2AllocateProxy( tree );

	b2TreeProxy* proxy = tree->proxies + proxyId;
	proxy->categoryBits = categoryBits;
	proxy->userData = userData;

	bool shouldRotate = true;
	b2InsertLeaf( tree, aabb, proxyId, markMoved, shouldRotate );

	return proxyId;
}

void b2DynamicTree_DestroyProxy( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b2RemoveLeaf( tree, proxyId );
	b2FreeProxy( tree, proxyId );
}

int b2DynamicTree_GetProxyCount( const b2DynamicTree* tree )
{
	return tree->proxyCount;
}

void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb, bool markMoved )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );
	B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b2RemoveLeaf( tree, proxyId );

	bool shouldRotate = false;
	b2InsertLeaf( tree, aabb, proxyId, markMoved, shouldRotate );
}

void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );
	B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b2TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;

	int index = tree->proxies[proxyId].node;
	b2TreeNode* node = nodes + index;
	B2_ASSERT( b2IsLeaf( node ) );

	// Caller must ensure this
	B2_VALIDATE( b2AABB_Contains( node->aabb, aabb ) == false );

	node->aabb = aabb;
	node->flagIndex |= B2_MOVED_NODE;

	index = parents[index];
	while ( index != B2_NULL_INDEX )
	{
		node = nodes + index;
		bool changed = b2EnlargeAABB( &node->aabb, aabb );

		// This is marked to ensure the root is marked in this loop or the one below.
		node->flagIndex |= B2_MOVED_NODE;

		index = parents[index];

		if ( changed == false )
		{
			break;
		}
	}

	// Mark all the way up to the root.
	while ( index != B2_NULL_INDEX )
	{
		node = nodes + index;
		if ( node->flagIndex & B2_MOVED_NODE )
		{
			// Early out because this ancestor was previously ascended and marked as moved.
			break;
		}

		node->flagIndex |= B2_MOVED_NODE;
		index = parents[index];
	}
}

void b2DynamicTree_SetCategoryBits( b2DynamicTree* tree, int proxyId, uint64_t categoryBits )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	tree->proxies[proxyId].categoryBits = categoryBits;
}

uint64_t b2DynamicTree_GetCategoryBits( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	return tree->proxies[proxyId].categoryBits;
}

static int b2ComputeHeight( const b2DynamicTree* tree, int nodeIndex )
{
	B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree->nodeEnd );
	const b2TreeNode* node = tree->nodes + nodeIndex;
	if ( b2IsLeaf( node ) )
	{
		return 0;
	}

	int pair = b2GetLeftChild( node );
	int height1 = b2ComputeHeight( tree, pair );
	int height2 = b2ComputeHeight( tree, pair + 1 );
	return 1 + b2MaxInt( height1, height2 );
}

int b2DynamicTree_GetHeight( const b2DynamicTree* tree )
{
	if ( tree->proxyCount == 0 )
	{
		return 0;
	}

	return b2ComputeHeight( tree, B2_ROOT_NODE );
}

// The area ratio is the thing that SAH seeks to minimize. SAH
// cannot do anything about leaf boxes or the root box. It seeks
// to minimize the area of all non-root internal nodes. Divide this
// by the root area to make the metric non-dimensional.
// So this becomes a meaningful measure of tree quality.
float b2DynamicTree_GetAreaRatio( const b2DynamicTree* tree )
{
	if ( tree->proxyCount == 0 )
	{
		return 0.0f;
	}

	const b2TreeNode* nodes = tree->nodes;
	float rootArea = b2Perimeter( nodes[B2_ROOT_NODE].aabb );
	if ( rootArea <= 0.0f )
	{
		return 0.0f;
	}

	// Free nodes and the empty node are leaf tagged.
	float internalArea = 0.0f;
	int nodeEnd = tree->nodeEnd;
	for ( int i = 2; i < nodeEnd; ++i )
	{
		if ( b2IsLeaf( nodes + i ) == false )
		{
			internalArea += b2Perimeter( nodes[i].aabb );
		}
	}

	return internalArea / rootArea;
}

b2AABB b2DynamicTree_GetRootBounds( const b2DynamicTree* tree )
{
	if ( tree->proxyCount == 0 )
	{
		return (b2AABB){ b2Vec2_zero, b2Vec2_zero };
	}

	return tree->nodes[B2_ROOT_NODE].aabb;
}

#if B2_ENABLE_VALIDATION

// Back links both ways, boxes containing the children, leaf counts and marks exact. Returns the leaf count.
static int b2ValidateSubtree( const b2DynamicTree* tree, int nodeIndex )
{
	B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree->nodeEnd );
	const b2TreeNode* node = tree->nodes + nodeIndex;
	B2_ASSERT( b2IsEmptyNode( node ) == false );

	if ( b2IsLeaf( node ) )
	{
		int proxyId = b2GetProxyId( node );
		B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
		B2_ASSERT( tree->proxies[proxyId].node == nodeIndex );
		B2_ASSERT( (int32_t)tree->proxies[proxyId].userData == node->shapeIndex );
		return 1;
	}

	int pair = b2GetLeftChild( node );
	B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
	B2_ASSERT( tree->parents[pair] == nodeIndex );
	B2_ASSERT( tree->parents[pair + 1] == nodeIndex );
	B2_ASSERT( tree->dfsOrdered == false || nodeIndex < pair );

	const b2TreeNode* c1 = tree->nodes + pair;
	const b2TreeNode* c2 = tree->nodes + pair + 1;
	B2_ASSERT( b2AABB_Contains( node->aabb, c1->aabb ) );
	B2_ASSERT( b2AABB_Contains( node->aabb, c2->aabb ) );
	B2_ASSERT( b2IsNodeMoved( node ) == ( b2IsNodeMoved( c1 ) || b2IsNodeMoved( c2 ) ) );

	int leafCount = b2ValidateSubtree( tree, pair ) + b2ValidateSubtree( tree, pair + 1 );
	B2_ASSERT( node->leafCount == leafCount );
	return leafCount;
}

#endif

void b2DynamicTree_Validate( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION
	B2_ASSERT( 2 <= tree->nodeEnd && tree->nodeEnd <= tree->nodeCapacity );
	B2_ASSERT( ( tree->nodeEnd & 1 ) == 0 );
	B2_ASSERT( tree->parents[B2_ROOT_NODE] == B2_NULL_INDEX );
	B2_ASSERT( b2IsEmptyNode( tree->nodes + B2_ROOT_NODE + 1 ) );

	// Free pairs are self describing and chained through the parent slot
	int freePairCount = 0;
	int pair = tree->pairFreeList;
	while ( pair != B2_NULL_INDEX )
	{
		B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
		B2_ASSERT( b2IsEmptyNode( tree->nodes + pair ) );
		B2_ASSERT( b2IsEmptyNode( tree->nodes + pair + 1 ) );
		pair = tree->parents[pair];
		++freePairCount;
		B2_ASSERT( 2 * freePairCount < tree->nodeEnd );
	}

	// Validate proxy free list.
	int freeProxyCount = 0;
	int freeIndex = tree->proxyFreeList;
	while ( freeIndex != B2_NULL_INDEX )
	{
		B2_ASSERT( 0 <= freeIndex && freeIndex < tree->proxyCapacity );
		B2_ASSERT( tree->proxies[freeIndex].node == B2_NULL_INDEX );
		freeIndex = tree->proxies[freeIndex].next;
		++freeProxyCount;
	}
	B2_ASSERT( tree->proxyCount + freeProxyCount == tree->proxyCapacity );

	// The root pair, a pair per proxy past the first, and the holes
	B2_ASSERT( tree->nodeEnd == 2 * b2MaxInt( tree->proxyCount, 1 ) + 2 * freePairCount );

	if ( tree->proxyCount == 0 )
	{
		B2_ASSERT( b2IsEmptyNode( tree->nodes + B2_ROOT_NODE ) );
		return;
	}

	int leafCount = b2ValidateSubtree( tree, B2_ROOT_NODE );
	B2_ASSERT( leafCount == tree->proxyCount );

#else
	B2_UNUSED( tree );
#endif
}

void b2DynamicTree_ValidateNoEnlarged( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION == 1
	const b2TreeNode* nodes = tree->nodes;
	int nodeEnd = tree->nodeEnd;
	for ( int i = 0; i < nodeEnd; ++i )
	{
		B2_ASSERT( b2IsNodeMoved( nodes + i ) == false );
	}
#else
	B2_UNUSED( tree );
#endif
}

int b2DynamicTree_GetByteCount( const b2DynamicTree* tree )
{
	size_t size = sizeof( b2DynamicTree );
	size += tree->nodeCapacity * sizeof( b2TreeNode );
	size += tree->nodeCapacity * sizeof( int32_t );
	size += tree->proxyCapacity * sizeof( b2TreeProxy );
	size += tree->swapNodes == NULL ? 0 : tree->nodeCapacity * sizeof( b2TreeNode );

	// leafIndices
	size += tree->rebuildCapacity * sizeof( int );
	// leafNodes
	size += tree->rebuildCapacity * sizeof( b2TreeNode );
	// leafBoxes
	size += tree->rebuildCapacity * sizeof( b2AABB );
	// leafCenters
	size += tree->rebuildCapacity * sizeof( b2Vec2 );
	// binIndices
	size += tree->rebuildCapacity * sizeof( int );

	return (int)size;
}

uint64_t b2DynamicTree_GetUserData( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	return tree->proxies[proxyId].userData;
}

b2AABB b2DynamicTree_GetAABB( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	int nodeIndex = tree->proxies[proxyId].node;
	B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree->nodeEnd );
	return tree->nodes[nodeIndex].aabb;
}

b2TreeStats b2DynamicTree_Query( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits, b2TreeQueryCallbackFcn* callback,
								 void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	const b2TreeNode* nodes = tree->nodes;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );

	b2AABBV boxv = b2LoadAABBV( &aabb );

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( b2OverlapNode( boxv, node ) )
			{
				if ( b2IsLeaf( node ) )
				{
					// callback to user code with proxy id
					int proxyId = b2GetProxyId( node );
					const b2TreeProxy* proxy = tree->proxies + proxyId;
					if ( proxy->categoryBits & maskBits )
					{
						bool proceed = callback( proxyId, proxy->userData, context );
						result.leafVisits += 1;

						if ( proceed == false )
						{
							return result;
						}
					}
				}
				else
				{
					if ( stackCount < B2_TREE_STACK_SIZE - 1 )
					{
						stack[stackCount++] = b2GetLeftChild( node );
					}
					else
					{
						B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
					}
				}
			}
		}
	}

	return result;
}

b2TreeStats b2DynamicTree_QueryAll( const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	const b2TreeNode* nodes = tree->nodes;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );

	b2AABBV boxv = b2LoadAABBV( &aabb );

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( b2OverlapNode( boxv, node ) )
			{
				if ( b2IsLeaf( node ) )
				{
					// callback to user code with proxy id
					int proxyId = b2GetProxyId( node );
					const b2TreeProxy* proxy = tree->proxies + proxyId;
					bool proceed = callback( proxyId, proxy->userData, context );
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
						stack[stackCount++] = b2GetLeftChild( node );
					}
					else
					{
						B2_ASSERT( stackCount < B2_TREE_STACK_SIZE - 1 );
					}
				}
			}
		}
	}

	return result;
}

// A lot of optimization work went into this. It beats the slab test by a significant margin.
// It is faster than having category bits in the nodes because of the cache line friendly node
// size (64 bytes).
b2TreeStats b2DynamicTree_RayCast( const b2DynamicTree* tree, const b2RayCastInput* input, uint64_t maskBits,
								   b2TreeRayCastCallbackFcn* callback, void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
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

	b2AABBV boxv = b2LoadAABBV( &segmentAABB );

	const b2TreeNode* nodes = tree->nodes;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );

	b2RayCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		const b2TreeNode* hit[2];
		bool isLeaf[2];
		int hitCount = 0;
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;

			if ( b2OverlapNode( boxv, node ) == false )
			{
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			b2AABB nodeAABB = node->aabb;
			b2Vec2 c = b2AABB_Center( nodeAABB );
			b2Vec2 h = b2AABB_Extents( nodeAABB );
			float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
			float term2 = b2Dot( abs_v, h );
			if ( term2 < term1 )
			{
				continue;
			}

			isLeaf[hitCount] = b2IsLeaf( node );
			hit[hitCount] = node;
			hitCount += 1;
		}

		if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
		{
			b2Vec2 center1 = b2AABB_Center( hit[0]->aabb );
			b2Vec2 center2 = b2AABB_Center( hit[1]->aabb );
			float d1 = b2DistanceSquared( center1, p1 );
			float d2 = b2DistanceSquared( center2, p1 );

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if ( d1 < d2 )
			{
				B2_SWAP( hit[0], hit[1] );
			}
		}

		for ( int i = 0; i < hitCount; ++i )
		{
			if ( isLeaf[i] )
			{
				int proxyId = b2GetProxyId( hit[i] );
				const b2TreeProxy* proxy = tree->proxies + proxyId;

				if ( ( proxy->categoryBits & maskBits ) == 0 )
				{
					continue;
				}

				subInput.maxFraction = maxFraction;

				float value = callback( &subInput, proxyId, proxy->userData, context );
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

					boxv = b2LoadAABBV( &segmentAABB );
				}
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b2GetLeftChild( hit[i] );
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

// Follows structure of ray cast with small tweaks to handle a swept box.
b2TreeStats b2DynamicTree_BoxCast( const b2DynamicTree* tree, const b2BoxCastInput* input, uint64_t maskBits,
								   b2TreeBoxCastCallbackFcn* callback, void* context )
{
	b2TreeStats result = { 0 };

	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	b2AABB originAABB = input->box;

	b2Vec2 p1 = b2AABB_Center( originAABB );
	b2Vec2 extension = b2AABB_Extents( originAABB );

	// v is perpendicular to the segment.
	b2Vec2 r = input->translation;
	b2Vec2 v = b2CrossSV( 1.0f, r );
	b2Vec2 abs_v = b2Abs( v );

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	float maxFraction = input->maxFraction;

	// Build total box for the cast
	b2Vec2 t = b2MulSV( maxFraction, input->translation );
	b2AABB totalAABB = {
		b2Min( originAABB.lowerBound, b2Add( originAABB.lowerBound, t ) ),
		b2Max( originAABB.upperBound, b2Add( originAABB.upperBound, t ) ),
	};

	b2AABBV boxv = b2LoadAABBV( &totalAABB );

	const b2TreeNode* nodes = tree->nodes;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );

	b2BoxCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;

		const b2TreeNode* hit[2];
		bool isLeaf[2];
		int hitCount = 0;
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;

			if ( b2OverlapNode( boxv, node ) == false )
			{
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			// radius extension is added to the node in this case
			b2AABB nodeAABB = node->aabb;
			b2Vec2 c = b2AABB_Center( nodeAABB );
			b2Vec2 h = b2Add( b2AABB_Extents( nodeAABB ), extension );
			float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
			float term2 = b2Dot( abs_v, h );
			if ( term2 < term1 )
			{
				continue;
			}

			isLeaf[hitCount] = b2IsLeaf( node );
			hit[hitCount] = node;
			hitCount += 1;
		}

		if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
		{
			b2Vec2 center1 = b2AABB_Center( hit[0]->aabb );
			b2Vec2 center2 = b2AABB_Center( hit[1]->aabb );
			float d1 = b2DistanceSquared( center1, p1 );
			float d2 = b2DistanceSquared( center2, p1 );

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if ( d1 < d2 )
			{
				B2_SWAP( hit[0], hit[1] );
			}
		}

		for ( int i = 0; i < hitCount; ++i )
		{
			if ( isLeaf[i] )
			{
				int proxyId = b2GetProxyId( hit[i] );
				const b2TreeProxy* proxy = tree->proxies + proxyId;

				if ( ( proxy->categoryBits & maskBits ) == 0 )
				{
					continue;
				}

				subInput.maxFraction = maxFraction;

				float value = callback( &subInput, proxyId, proxy->userData, context );
				result.leafVisits += 1;

				// The user may return -1 to indicate this shape should be skipped

				if ( value == 0.0f )
				{
					// The client has terminated the ray cast.
					return result;
				}

				if ( 0.0f < value && value < maxFraction )
				{
					// Update cast bounding box.
					maxFraction = value;
					t = b2MulSV( maxFraction, input->translation );
					totalAABB.lowerBound = b2Min( originAABB.lowerBound, b2Add( originAABB.lowerBound, t ) );
					totalAABB.upperBound = b2Max( originAABB.upperBound, b2Add( originAABB.upperBound, t ) );
					boxv = b2LoadAABBV( &totalAABB );
				}
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b2GetLeftChild( hit[i] );
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

// Temporary data used to track the rebuild of a tree node.
typedef struct b2RebuildItem
{
	// Where this node is written and the pair its children go in.
	int nodeIndex;
	int pair;
	int childCount;

	// Leaf indices
	int startIndex;
	int splitIndex;
	int endIndex;
} b2RebuildItem;

typedef struct b2CopyItem
{
	int oldPair;
	int newIndex;
} b2CopyItem;

// Bump allocate a pair of sibling nodes. Returns index to the first one.
static inline int b2BumpPair( b2DynamicTree* tree, int parent )
{
	int pair = tree->nodeEnd;
	B2_ASSERT( pair + 2 <= tree->nodeCapacity );
	tree->nodeEnd += 2;
	tree->parents[pair] = parent;
	tree->parents[pair + 1] = parent;
	return pair;
}

static inline void b2SetLeftChild( b2TreeNode* node, int pair )
{
	node->flagIndex = ( node->flagIndex & ~B2_NODE_INDEX_MASK ) | (uint32_t)pair;
}

// Copy a retained subtree from the old array into the rebuilt DFS array, a pair per step. The node is
// the old content of the subtree root, its children still index the old array. The left child is
// followed right away and the right child waits on the stack, its pair allocated once the left
// subtree is written, which keeps the DFS order.
static void b2CopySubtree( b2DynamicTree* tree, b2TreeNode node, int newIndex )
{
	const b2TreeNode* oldNodes = tree->nodes;
	b2TreeNode* newNodes = tree->swapNodes;
	b2TreeProxy* proxies = tree->proxies;

	if ( b2IsLeaf( &node ) )
	{
		newNodes[newIndex] = node;
		proxies[b2GetProxyId( &node )].node = newIndex;
		return;
	}

	b2CopyItem stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;

	int oldPair = b2GetLeftChild( &node );
	int newPair = b2BumpPair( tree, newIndex );
	b2SetLeftChild( &node, newPair );

	// Copy the subtree root. This can be a left or right sibling (even or odd).
	newNodes[newIndex] = node;

	for ( ;; )
	{
		b2TreeNode* pair = newNodes + newPair;

		// Copy the siblings.
		pair[0] = oldNodes[oldPair];
		pair[1] = oldNodes[oldPair + 1];

		if ( b2IsLeaf( pair + 1 ) )
		{
			// Hoop up proxy on right sibling.
			int proxyId = b2GetProxyId( pair + 1 );
			proxies[proxyId].node = newPair + 1;
		}
		else
		{
			// Push the left child of the right sibling.
			B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
			int leftChild = b2GetLeftChild( pair + 1 );
			stack[stackCount++] = (b2CopyItem){ leftChild, newPair + 1 };
		}

		if ( b2IsLeaf( pair ) == false )
		{
			// Descend into left subtree.
			int leftIndex = newPair;
			oldPair = b2GetLeftChild( pair );
			newPair = b2BumpPair( tree, leftIndex );
			b2SetLeftChild( newNodes + leftIndex, newPair );
			continue;
		}

		// Hook up proxy on left sibling.
		proxies[b2GetProxyId( pair )].node = newPair;

		if ( stackCount == 0 )
		{
			break;
		}

		// Descend a right subtree that was previously pushed.
		b2CopyItem item = stack[--stackCount];
		oldPair = item.oldPair;
		newPair = b2BumpPair( tree, item.newIndex );
		b2SetLeftChild( newNodes + item.newIndex, newPair );
	}
}

static void b2PlaceLeaf( b2DynamicTree* tree, b2TreeNode leaf, int newIndex )
{
	if ( b2IsLeaf( &leaf ) )
	{
		tree->swapNodes[newIndex] = leaf;
		tree->proxies[b2GetProxyId( &leaf )].node = newIndex;
	}
	else
	{
		b2CopySubtree( tree, leaf, newIndex );
	}
}

static void b2BuildTree( b2DynamicTree* tree, int leafCount )
{
	b2TreeNode* nodes = tree->swapNodes;
	const b2TreeNode* leaves = tree->leafNodes;
	int* leafIndices = tree->leafIndices;

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
	int* binIndices = tree->binIndices;
#endif

	// Bump allocation into the spare. The root pair is fixed.
	tree->nodeEnd = 2;
	nodes[B2_ROOT_NODE + 1] = b2MakeEmptyNode();
	tree->parents[B2_ROOT_NODE] = B2_NULL_INDEX;
	tree->parents[B2_ROOT_NODE + 1] = B2_NULL_INDEX;

	if ( leafCount == 1 )
	{
		b2PlaceLeaf( tree, leaves[leafIndices[0]], B2_ROOT_NODE );
		return;
	}

	b2RebuildItem stack[B2_TREE_STACK_SIZE];
	int top = 0;

	stack[0].nodeIndex = B2_ROOT_NODE;
	stack[0].pair = b2BumpPair( tree, B2_ROOT_NODE );
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
		b2RebuildItem* item = stack + top;
		item->childCount += 1;

		if ( item->childCount == 2 )
		{
			// Both children written, so the node above them can be made
			nodes[item->nodeIndex] = b2MakeInternalNode( nodes, item->pair );
			if ( top == 0 )
			{
				break;
			}

			top -= 1;
			continue;
		}

		int slot = item->childCount;
		int startIndex = slot == 0 ? item->startIndex : item->splitIndex;
		int endIndex = slot == 0 ? item->splitIndex : item->endIndex;
		int count = endIndex - startIndex;
		B2_ASSERT( count > 0 );

		int nodeIndex = item->pair + slot;
		if ( count == 1 )
		{
			b2PlaceLeaf( tree, leaves[leafIndices[startIndex]], nodeIndex );
			continue;
		}

		B2_ASSERT( top < B2_TREE_STACK_SIZE - 1 );
		top += 1;
		b2RebuildItem* newItem = stack + top;
		newItem->nodeIndex = nodeIndex;
		newItem->pair = b2BumpPair( tree, nodeIndex );
		newItem->childCount = -1;
		newItem->startIndex = startIndex;
		newItem->endIndex = endIndex;
		newItem->splitIndex = startIndex;
#if B2_TREE_HEURISTIC == 0
		newItem->splitIndex += b2PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
#else
		newItem->splitIndex += b2PartitionSAH( leafIndices + startIndex, binIndices + startIndex, leafBoxes + startIndex, count );
#endif
	}
}

// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
// refitting much faster. This is done async with threading, so the cost is hidden.
// Not safe to access tree during this operation.
int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild )
{
	int proxyCount = tree->proxyCount;
	if ( proxyCount == 0 )
	{
		return 0;
	}

	// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
	const b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	if ( fullBuild == false && b2IsNodeMoved( root ) == false && tree->dfsOrdered )
	{
		return 0;
	}

	if ( tree->swapNodes == NULL )
	{
		tree->swapNodes = b2Alloc( tree->nodeCapacity * sizeof( b2TreeNode ) );
	}

	if ( proxyCount > tree->rebuildCapacity )
	{
		int oldCapacity = tree->rebuildCapacity;
		int newCapacity = proxyCount + proxyCount / 2;

		tree->leafIndices = B2_GROW( tree->leafIndices, oldCapacity, newCapacity );
		tree->leafNodes = B2_GROW( tree->leafNodes, oldCapacity, newCapacity );
#if B2_TREE_HEURISTIC == 0
		tree->leafCenters = B2_GROW( tree->leafCenters, oldCapacity, newCapacity );
#else
		tree->leafBoxes = B2_GROW( tree->leafBoxes, oldCapacity, newCapacity );
		tree->binIndices = B2_GROW( tree->binIndices, oldCapacity, newCapacity );
#endif
		tree->rebuildCapacity = newCapacity;
	}

	int* leafIndices = tree->leafIndices;
	b2TreeNode* leaves = tree->leafNodes;
#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
#endif

	// Gather build leaves. A marked internal node is descended and abandoned in the old array.
	// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
	// on an unordered array, is the one build leaf.
	int leafCount = 0;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	if ( b2IsLeaf( root ) == false && ( fullBuild || b2IsNodeMoved( root ) ) )
	{
		stack[stackCount++] = b2GetLeftChild( root );
	}
	else
	{
		b2TreeNode node = *root;
		node.flagIndex &= ~B2_MOVED_NODE;
		leafIndices[0] = 0;
		leaves[0] = node;
#if B2_TREE_HEURISTIC == 0
		leafCenters[0] = b2AABB_Center( node.aabb );
#else
		leafBoxes[0] = node.aabb;
#endif
		leafCount = 1;
	}

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			b2TreeNode node = nodes[pair + i];
			if ( b2IsLeaf( &node ) == false && ( fullBuild || b2IsNodeMoved( &node ) ) )
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				stack[stackCount++] = b2GetLeftChild( &node );
				continue;
			}

			node.flagIndex &= ~B2_MOVED_NODE;
			leafIndices[leafCount] = leafCount;
			leaves[leafCount] = node;
#if B2_TREE_HEURISTIC == 0
			leafCenters[leafCount] = b2AABB_Center( node.aabb );
#else
			leafBoxes[leafCount] = node.aabb;
#endif
			leafCount += 1;
		}
	}

	B2_ASSERT( 0 < leafCount && leafCount <= proxyCount );

	b2BuildTree( tree, leafCount );

	// The spare is now the tree and the old array is the spare. The build is dense, so the
	// stale tail of the old array sits above the end and is never read.
	B2_SWAP( tree->nodes, tree->swapNodes );
	tree->pairFreeList = B2_NULL_INDEX;
	tree->dfsOrdered = true;

	b2DynamicTree_Validate( tree );
	b2DynamicTree_ValidateNoEnlarged( tree );

	return leafCount;
}

void b2DynamicTree_MarkEnlargedFlag( b2DynamicTree* tree, int proxyId )
{
	B2_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b2TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;

	int index = tree->proxies[proxyId].node;
	B2_VALIDATE( 0 <= index && index < tree->nodeEnd );
	B2_VALIDATE( b2IsLeaf( nodes + index ) );

	while ( index != B2_NULL_INDEX )
	{
		nodes[index].flagIndex |= B2_MOVED_NODE;
		index = parents[index];
	}
}

void b2DynamicTree_MarkEnlarged( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );

	b2TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;

	int index = tree->proxies[proxyId].node;
	B2_VALIDATE( 0 <= index && index < tree->nodeEnd );

	b2TreeNode* node = nodes + index;
	B2_VALIDATE( b2IsLeaf( node ) );
	B2_VALIDATE( b2AABB_Contains( node->aabb, aabb ) == false );

	// This is not raced since it is the leaf.
	node->aabb = aabb;
	node->flagIndex |= B2_MOVED_NODE;

	index = parents[index];
	while ( index != B2_NULL_INDEX )
	{
		node = nodes + index;

		// Read first to avoid the FetchOr if possible.
		if ( b2AtomicLoadU32Raw( &node->flagIndex ) & B2_MOVED_NODE )
		{
			break;
		}

		uint32_t previousFlags = b2AtomicFetchOrU32( &node->flagIndex, B2_MOVED_NODE );
		if ( previousFlags & B2_MOVED_NODE )
		{
			// Ancestor already visited.
			break;
		}

		index = parents[index];
	}
}

// todo call this during the async rebuild
void b2DynamicTree_ClearEnlarged( b2DynamicTree* tree )
{
	b2TreeNode* nodes = tree->nodes;

	b2TreeNode* root = nodes + B2_ROOT_NODE;
	if ( b2IsNodeMoved( root ) == false )
	{
		return;
	}

	root->flagIndex &= ~B2_MOVED_NODE;
	if ( b2IsLeaf( root ) )
	{
		return;
	}

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetLeftChild( root );

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			b2TreeNode* node = nodes + pair + i;
			if ( node->flagIndex & B2_MOVED_NODE )
			{
				node->flagIndex &= ~B2_MOVED_NODE;
				if ( b2IsLeaf( node ) == false )
				{
					stack[stackCount++] = b2GetLeftChild( node );
				}
			}
		}
	}
}

// Slow refit for unit tests.
static b2AABB b2RefitSubtree( b2TreeNode* nodes, int nodeIndex )
{
	b2TreeNode* node = nodes + nodeIndex;
	if ( b2IsLeaf( node ) || b2IsNodeMoved( node ) == false )
	{
		return node->aabb;
	}

	int pair = b2GetLeftChild( node );
	b2AABB box1 = b2RefitSubtree( nodes, pair );
	b2AABB box2 = b2RefitSubtree( nodes, pair + 1 );
	node->aabb = b2UnionV( box1, box2 );
	return node->aabb;
}

void b2DynamicTree_Refit( b2DynamicTree* tree )
{
	if ( b2HasTreeMoved( tree ) == false )
	{
		return;
	}

	b2TreeNode* nodes = tree->nodes;

	if ( tree->dfsOrdered == false )
	{
		// This only happens in unit tests.
		b2RefitSubtree( nodes, B2_ROOT_NODE );
		return;
	}

	for ( int pair = tree->nodeEnd - 2; pair >= 0; pair -= 2 )
	{
		b2TreeNode* node = nodes + pair;
		uint32_t flags1 = node[0].flagIndex;
		uint32_t flags2 = node[1].flagIndex;

		// Did either move?
		if ( ( ( flags1 | flags2 ) & B2_MOVED_NODE ) == 0 )
		{
			continue;
		}

		// Is the node internal and moved?
		bool refit1 = ( flags1 & ( B2_LEAF_NODE | B2_MOVED_NODE ) ) == B2_MOVED_NODE;
		bool refit2 = ( flags2 & ( B2_LEAF_NODE | B2_MOVED_NODE ) ) == B2_MOVED_NODE;

		// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
		// The store function won't actually modify the node at the dummy index. This
		// avoids an unpredictable branch. SIMD FTW.
		int children1 = refit1 ? (int)( flags1 & B2_NODE_INDEX_MASK ) : pair;
		int children2 = refit2 ? (int)( flags2 & B2_NODE_INDEX_MASK ) : pair;

		// Conditionally store the union.
		b2StoreAABBV( &node[0].aabb, b2UnionPairV( nodes + children1 ), refit1 );
		b2StoreAABBV( &node[1].aabb, b2UnionPairV( nodes + children2 ), refit2 );
	}
}
