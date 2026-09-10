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
_Static_assert( sizeof( b2TreeChild ) == 32, "expected size" );
_Static_assert( sizeof( b2TreeNode ) == 64, "expected size" );
_Static_assert( sizeof( b2TreeLink ) == 8, "expected size" );
_Static_assert( sizeof( b2TreeProxy ) == 24, "expected size" );

// Allocate a node from the pool. Grow the pool if necessary.
static int b2AllocateNode( b2DynamicTree* tree )
{
	// Expand the pool as needed.
	if ( tree->nodeFreeList == B2_NULL_INDEX )
	{
		B2_ASSERT( tree->nodeCount == tree->nodeCapacity );

		// The free list is empty. Rebuild a bigger pool.
		int oldCapacity = tree->nodeCapacity;
		tree->nodeCapacity += oldCapacity >> 1;
		tree->nodes = B2_GROW_ZERO( tree->nodes, oldCapacity, tree->nodeCapacity );
		tree->links = B2_GROW_ZERO( tree->links, oldCapacity, tree->nodeCapacity );

		b2Free( tree->swapNodes, oldCapacity * sizeof( b2TreeNode ) );
		tree->swapNodes = NULL;

		// Build a linked list for the free list. The parent pointer becomes the "next" pointer.
		for ( int i = tree->nodeCount; i < tree->nodeCapacity - 1; ++i )
		{
			tree->links[i].next = i + 1;
		}

		tree->links[tree->nodeCapacity - 1].next = B2_NULL_INDEX;
		tree->nodeFreeList = tree->nodeCount;
	}

	// Peel a node off the free list.
	int nodeIndex = tree->nodeFreeList;
	tree->nodeFreeList = tree->links[nodeIndex].next;
	memset( tree->nodes + nodeIndex, 0, sizeof( b2TreeNode ) );
	memset( tree->links + nodeIndex, 0, sizeof( b2TreeLink ) );
	tree->links[nodeIndex].flags |= B2_ALLOCATED_BIT;
	++tree->nodeCount;
	return nodeIndex;
}

// Return a node to the pool.
static void b2FreeNode( b2DynamicTree* tree, int nodeId )
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	B2_ASSERT( 0 < tree->nodeCount );
	tree->nodes[nodeId].children[0].flagIndex = B2_NODE_SENTINEL;
	tree->nodes[nodeId].children[1].flagIndex = B2_NODE_SENTINEL;
	tree->links[nodeId].next = tree->nodeFreeList;
	tree->links[nodeId].flags = 0;
	tree->nodeFreeList = nodeId;
	--tree->nodeCount;
}

b2DynamicTree b2DynamicTree_Create( int proxyCapacity )
{
	int capacity = b2MaxInt( proxyCapacity, 16 );

	// Intentionally _not_ initialized with brace initialization, which can leave
	// uninitialized gaps.
	b2DynamicTree tree;

	// memset needed for deterministic serialization.
	memset( &tree, 0, sizeof( b2DynamicTree ) );

	// maximum node count for a full binary tree is 2 * leafCount - 1
	tree.nodeCapacity = capacity - 1;
	tree.nodeCount = 0;
	tree.nodes = (b2TreeNode*)b2AllocZero( tree.nodeCapacity * sizeof( b2TreeNode ) );
	tree.links = (b2TreeLink*)b2AllocZero( tree.nodeCapacity * sizeof( b2TreeLink ) );

	// Build a linked list for the free list.
	for ( int i = 0; i < tree.nodeCapacity - 1; ++i )
	{
		tree.links[i].next = i + 1;
	}

	tree.links[tree.nodeCapacity - 1].next = B2_NULL_INDEX;
	tree.nodeFreeList = 0;

	tree.proxyCapacity = capacity;
	tree.proxyCount = 0;
	tree.proxies = (b2TreeProxy*)b2AllocZero( tree.proxyCapacity * sizeof( b2TreeProxy ) );

	// Build a linked list for the free list.
	for ( int i = 0; i < tree.proxyCapacity - 1; ++i )
	{
		tree.proxies[i].link.next = i + 1;
	}

	tree.proxies[tree.proxyCapacity - 1].link.next = B2_NULL_INDEX;
	tree.proxyFreeList = 0;

	tree.leafIndices = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
	tree.binIndices = NULL;
	tree.rebuildCapacity = 0;
	tree.dfsNodeCount = false;

	// Root node always exists for simplicity. nodeCount == 1 and proxyCount == 0.
	int root = b2AllocateNode( &tree );
	B2_ASSERT( root == B2_ROOT_NODE );
	tree.nodes[root].children[0] = b2MakeEmptyChild();
	tree.nodes[root].children[1] = b2MakeEmptyChild();

	return tree;
}

void b2DynamicTree_Destroy( b2DynamicTree* tree )
{
	b2Free( tree->nodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->links, tree->nodeCapacity * sizeof( b2TreeLink ) );
	b2Free( tree->proxies, tree->proxyCapacity * sizeof( b2TreeProxy ) );
	b2Free( tree->swapNodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
	b2Free( tree->leafChildren, tree->rebuildCapacity * sizeof( b2TreeChild ) );
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

		// Build a linked list for the free list. The parent pointer becomes the "next" pointer.
		for ( int i = tree->proxyCount; i < tree->proxyCapacity - 1; ++i )
		{
			tree->proxies[i].link.next = i + 1;
		}

		tree->proxies[tree->proxyCapacity - 1].link.next = B2_NULL_INDEX;
		tree->proxyFreeList = tree->proxyCount;
	}

	// Peel a proxy off the free list.
	int proxyIndex = tree->proxyFreeList;
	tree->proxyFreeList = tree->proxies[proxyIndex].link.next;
	memset( tree->proxies + proxyIndex, 0, sizeof( b2TreeProxy ) );
	tree->proxies[proxyIndex].link.flags |= B2_ALLOCATED_BIT;
	++tree->proxyCount;
	return proxyIndex;
}

// Return a proxy to the pool.
static void b2FreeProxy( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	B2_ASSERT( 0 < tree->proxyCount );
	tree->proxies[proxyId].link.next = tree->proxyFreeList;
	tree->proxies[proxyId].link.flags = 0;
	tree->proxyFreeList = proxyId;
	--tree->proxyCount;
}

typedef struct b2ChildId
{
	int parent;
	int slot;
} b2ChildId;

static inline b2ChildId b2GetChildId( const b2DynamicTree* tree, int nodeIndex )
{
	B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree->nodeCapacity );
	const b2TreeLink* link = tree->links + nodeIndex;
	B2_ASSERT( b2IsAllocated( link ) );
	int slot = b2GetChildSlot( link );
	return (b2ChildId){
		.parent = link->parent,
		.slot = slot,
	};
}

static inline int b2GetLeafCount( const b2TreeChild* child )
{
	if ( child->flagIndex == B2_NODE_SENTINEL )
	{
		return 0;
	}

	return b2IsLeaf( child ) ? 1 : child->leafCount;
}

// Given an node, make the child that goes in the parent.
static inline b2TreeChild b2MakeInternalChild( const b2TreeNode* node, int nodeIndex )
{
	const b2TreeChild* c1 = node->children + 0;
	const b2TreeChild* c2 = node->children + 1;

	b2TreeChild child = { 0 };
	child.aabb = b2AABB_Union( c1->aabb, c2->aabb );
	child.flagIndex = (uint32_t)nodeIndex | ( ( c1->flagIndex | c2->flagIndex ) & B2_MOVED_NODE );
	child.leafCount = b2GetLeafCount( c1 ) + b2GetLeafCount( c2 );
	return child;
}

// Given a proxy, make a child that goes in the parent.
static inline b2TreeChild b2MakeLeafChild( b2AABB aabb, int proxyId, uint64_t userData, bool moved )
{
	b2TreeChild child = { 0 };
	child.aabb = aabb;
	child.flagIndex = (uint32_t)proxyId | B2_LEAF_NODE | ( moved ? B2_MOVED_NODE : 0 );
	child.truncatedUserData = (uint32_t)userData;
	return child;
}

static inline void b2ConnectChild( b2DynamicTree* tree, b2ChildId id )
{
	B2_ASSERT( 0 <= id.parent && id.parent < tree->nodeCapacity );
	B2_ASSERT( id.slot == 0 || id.slot == 1 );
	const b2TreeChild* child = tree->nodes[id.parent].children + id.slot;
	int index = b2GetChildIndex( child );
	b2TreeLink* link = b2IsLeaf( child ) ? &tree->proxies[index].link : tree->links + index;
	link->parent = id.parent;
	link->flags &= ~B2_SLOT_BIT;
	link->flags |= id.slot == 0 ? 0 : B2_SLOT_BIT;
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
static b2ChildId b2FindBestSibling( const b2DynamicTree* tree, b2AABB boxD )
{
	b2Vec2 centerD = b2AABB_Center( boxD );
	float areaD = b2Perimeter( boxD );

	const b2TreeNode* nodes = tree->nodes;
	int rootIndex = B2_ROOT_NODE;

	b2AABB rootBox = b2AABB_Union( nodes[rootIndex].children[0].aabb, nodes[rootIndex].children[1].aabb );

	// Area of current node
	float areaBase = b2Perimeter( rootBox );

	// Area of inflated node
	float directCost = b2Perimeter( b2AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;

	b2ChildId bestSibling = { B2_NULL_INDEX, 0 };
	float bestCost = directCost;

	// Descend the tree, following a single greedy path.
	b2ChildId currentId = bestSibling;
	int nodeIndex = rootIndex;
	for ( ;; )
	{
		int child1 = b2GetChildIndex( nodes[nodeIndex].children + 0 );
		int child2 = b2GetChildIndex( nodes[nodeIndex].children + 1 );

		// Cost of creating a new parent for this node and the new leaf
		float cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( cost < bestCost )
		{
			bestSibling = currentId;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		bool leaf1 = b2IsLeaf( nodes[nodeIndex].children + 0 );
		bool leaf2 = b2IsLeaf( nodes[nodeIndex].children + 1 );

		// Cost of descending into child 1
		float lowerCost1 = FLT_MAX;
		b2AABB box1 = nodes[nodeIndex].children[0].aabb;
		float directCost1 = b2Perimeter( b2AABB_Union( box1, boxD ) );
		float area1 = 0.0f;
		if ( leaf1 )
		{
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			float cost1 = directCost1 + inheritedCost;
			if ( cost1 < bestCost )
			{
				bestSibling.parent = nodeIndex;
				bestSibling.slot = 0;
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
		b2AABB box2 = nodes[nodeIndex].children[1].aabb;
		float directCost2 = b2Perimeter( b2AABB_Union( box2, boxD ) );
		float area2 = 0.0f;
		if ( leaf2 )
		{
			float cost2 = directCost2 + inheritedCost;
			if ( cost2 < bestCost )
			{
				bestSibling.parent = nodeIndex;
				bestSibling.slot = 1;
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
			currentId.parent = nodeIndex;
			currentId.slot = 0;
			nodeIndex = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			currentId.parent = nodeIndex;
			currentId.slot = 1;
			nodeIndex = child2;
			areaBase = area2;
			directCost = directCost2;
		}
	}

	return bestSibling;
}

// Swap a child in A with a grandchild of the sibling. For example
// swap B with G. This includes the whole subtrees below B and G.
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
// Below I pretend I'm swapping B and G, but this function is more general.
static void b2SwapChildren( b2DynamicTree* tree, int iA, int slotDown, int slotUp )
{
	b2TreeNode* nodes = tree->nodes;
	b2TreeNode* A = nodes + iA;

	// C is the sibling of B
	int iC = b2GetChildIndex( A->children + ( slotDown ^ 1 ) );
	b2TreeNode* C = nodes + iC;

	// Swap B and G
	B2_SWAP( A->children[slotDown], C->children[slotUp] );

	// Connect G to parent A
	b2ConnectChild( tree, (b2ChildId){ iA, slotDown } );

	// Connect B to parent C
	b2ConnectChild( tree, (b2ChildId){ iC, slotUp } );

	// Refresh C since it now contains B instead of G.
	A->children[slotDown ^ 1] = b2MakeInternalChild( C, iC );
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
static void b2RotateNodes( b2DynamicTree* tree, int iA )
{
	B2_ASSERT( b2IsAllocated( tree->links + iA ) );

	b2TreeNode* nodes = tree->nodes;
	b2TreeNode* A = nodes + iA;
	const b2TreeChild* childB = A->children + 0;
	const b2TreeChild* childC = A->children + 1;

	bool leafB = b2IsLeaf( childB );
	bool leafC = b2IsLeaf( childC );
	if ( leafB && leafC )
	{
		return;
	}

	int bestSlotDown = B2_NULL_INDEX;
	int bestSlotUp = 0;
	float bestDelta = 0.0f;

	if ( leafC == false )
	{
		// Swap B with F or G
		const b2TreeNode* C = nodes + b2GetChildIndex( childC );
		b2AABB boxF = C->children[0].aabb;
		b2AABB boxG = C->children[1].aabb;
		float areaC = b2Perimeter( childC->aabb );

		// B <-> F then C (B G)
		float deltaBF = b2Perimeter( b2UnionV( childB->aabb, boxG ) ) - areaC;
		if ( deltaBF < bestDelta )
		{
			bestSlotDown = 0;
			bestSlotUp = 0;
			bestDelta = deltaBF;
		}

		// B <-> G then C (F B)
		float deltaBG = b2Perimeter( b2UnionV( childB->aabb, boxF ) ) - areaC;
		if ( deltaBG < bestDelta )
		{
			bestSlotDown = 0;
			bestSlotUp = 1;
			bestDelta = deltaBG;
		}
	}

	if ( leafB == false )
	{
		// Swap C with D or E
		const b2TreeNode* B = nodes + b2GetChildIndex( childB );
		b2AABB boxD = B->children[0].aabb;
		b2AABB boxE = B->children[1].aabb;
		float areaB = b2Perimeter( childB->aabb );

		// C <-> D then B (C E)
		float deltaCD = b2Perimeter( b2UnionV( childC->aabb, boxE ) ) - areaB;
		if ( deltaCD < bestDelta )
		{
			bestSlotDown = 1;
			bestSlotUp = 0;
			bestDelta = deltaCD;
		}

		// C <-> E then B (D C)
		float deltaCE = b2Perimeter( b2UnionV( childC->aabb, boxD ) ) - areaB;
		if ( deltaCE < bestDelta )
		{
			bestSlotDown = 1;
			bestSlotUp = 1;
			bestDelta = deltaCE;
		}
	}

	if ( bestSlotDown != B2_NULL_INDEX )
	{
		b2SwapChildren( tree, iA, bestSlotDown, bestSlotUp );
	}
}

static void b2InsertLeaf( b2DynamicTree* tree, b2AABB aabb, int leaf, bool moved, bool shouldRotate )
{
	tree->dfsNodeCount = 0;

	b2TreeProxy* proxy = tree->proxies + leaf;
	b2TreeChild leafChild = b2MakeLeafChild( aabb, leaf, proxy->userData, moved );

	// Stage 1: check for an empty slot in the root
	{
		b2TreeNode* root = tree->nodes + B2_ROOT_NODE;
		for ( int i = 0; i < 2; ++i )
		{
			if ( root->children[i].flagIndex == B2_NODE_SENTINEL )
			{
				root->children[i] = leafChild;
				b2ConnectChild( tree, (b2ChildId){ B2_ROOT_NODE, i } );
				return;
			}
		}
	}

	// Stage 2: find the best sibling for this node
	b2ChildId siblingId = b2FindBestSibling( tree, aabb );

	// Stage 3: create a new parent for the leaf and sibling
	int newParent = b2AllocateNode( tree );
	b2TreeNode* nodes = tree->nodes;
	b2TreeLink* links = tree->links;

	if ( siblingId.parent == B2_NULL_INDEX )
	{
		// Sibling is the root.
		// The children move down because the root node is fixed at index 0.
		int rootIndex = B2_ROOT_NODE;
		nodes[newParent] = nodes[rootIndex];
		links[newParent].parent = rootIndex;
		links[newParent].flags = B2_ALLOCATED_BIT;
		nodes[rootIndex].children[0] = b2MakeInternalChild( nodes + newParent, newParent );
		nodes[rootIndex].children[1] = leafChild;
		b2ConnectChild( tree, (b2ChildId){ rootIndex, 0 } );
		b2ConnectChild( tree, (b2ChildId){ rootIndex, 1 } );
	}
	else
	{
		b2TreeChild* siblingChild = nodes[siblingId.parent].children + siblingId.slot;
		nodes[newParent].children[0] = *siblingChild;
		nodes[newParent].children[1] = leafChild;
		links[newParent].parent = siblingId.parent;
		links[newParent].flags = B2_ALLOCATED_BIT | ( siblingId.slot == 0 ? 0 : B2_SLOT_BIT );

		// Install new parent into the grandparent.
		*siblingChild = b2MakeInternalChild( nodes + newParent, newParent );
	}

	b2ConnectChild( tree, (b2ChildId){ newParent, 0 } );
	b2ConnectChild( tree, (b2ChildId){ newParent, 1 } );

	// Stage 4: walk back up the tree refitting ancestor fields
	b2ChildId id = siblingId;
	while ( id.parent != B2_NULL_INDEX )
	{
		b2TreeChild* child = nodes[id.parent].children + id.slot;
		int nodeIndex = b2GetChildIndex( child );

		if ( shouldRotate )
		{
			b2RotateNodes( tree, nodeIndex );
		}

		*child = b2MakeInternalChild( nodes + nodeIndex, nodeIndex );
		id = b2GetChildId( tree, id.parent );
	}

	if ( shouldRotate )
	{
		b2RotateNodes( tree, B2_ROOT_NODE );
	}
}

static void b2RemoveLeaf( b2DynamicTree* tree, int leaf )
{
	b2TreeNode* nodes = tree->nodes;
	b2TreeProxy* proxy = tree->proxies + leaf;
	B2_ASSERT( b2IsAllocated( &proxy->link ) );

	int parent = proxy->link.parent;
	int slot = b2GetChildSlot( &proxy->link );
	b2TreeChild* sibling = nodes[parent].children + ( slot ^ 1 );

	if ( parent == B2_ROOT_NODE )
	{
		if ( b2IsLeaf( sibling ) )
		{
			nodes[parent].children[slot] = b2MakeEmptyChild();
			return;
		}

		// Raise sibling children up into both slots.
		int siblingIndex = b2GetChildIndex( sibling );
		nodes[parent] = nodes[siblingIndex];
		b2ConnectChild( tree, (b2ChildId){ parent, 0 } );
		b2ConnectChild( tree, (b2ChildId){ parent, 1 } );
		b2FreeNode( tree, siblingIndex );
		return;
	}

	b2ChildId parentId = b2GetChildId( tree, parent );
	nodes[parentId.parent].children[parentId.slot] = *sibling;
	b2ConnectChild( tree, parentId );
	b2FreeNode( tree, parent );

	int nodeIndex = parentId.parent;
	for ( ;; )
	{
		b2ChildId id = b2GetChildId( tree, nodeIndex );
		if ( id.parent == B2_NULL_INDEX )
		{
			break;
		}

		nodes[id.parent].children[id.slot] = b2MakeInternalChild( nodes + nodeIndex, nodeIndex );
		nodeIndex = id.parent;
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

	b2TreeProxy* proxy = tree->proxies + proxyId;

	int nodeIndex = proxy->link.parent;
	int slotIndex = b2GetChildSlot( &proxy->link );

	b2TreeNode* nodes = tree->nodes;
	b2TreeChild* child = tree->nodes[nodeIndex].children + slotIndex;

	B2_ASSERT( b2IsLeaf( child ) );

	// Caller must ensure this
	B2_VALIDATE( b2AABB_Contains( child->aabb, aabb ) == false );

	child->aabb = aabb;
	child->flagIndex |= B2_MOVED_NODE;

	const b2TreeLink* links = tree->links;

	slotIndex = b2GetChildSlot( links + nodeIndex );
	nodeIndex = links[nodeIndex].parent;
	while ( nodeIndex != B2_NULL_INDEX )
	{
		child = nodes[nodeIndex].children + slotIndex;
		bool changed = b2EnlargeAABB( &child->aabb, aabb );

		// This is marked to ensure the root is marked in this loop or
		// the one below.
		child->flagIndex |= B2_MOVED_NODE;

		slotIndex = b2GetChildSlot( links + nodeIndex );
		nodeIndex = links[nodeIndex].parent;

		if ( changed == false )
		{
			break;
		}
	}

	// Mark all the way up to the root.
	while ( nodeIndex != B2_NULL_INDEX )
	{
		child = nodes[nodeIndex].children + slotIndex;
		if ( child->flagIndex & B2_MOVED_NODE )
		{
			// Early out because this ancestor was previously ascended
			// and marked as moved.
			break;
		}

		child->flagIndex |= B2_MOVED_NODE;

		slotIndex = b2GetChildSlot( links + nodeIndex );
		nodeIndex = links[nodeIndex].parent;
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

static int b2ComputeHeight( const b2DynamicTree* tree, int nodeId )
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	const b2TreeNode* node = tree->nodes + nodeId;
	const b2TreeChild* c1 = node->children + 0;
	const b2TreeChild* c2 = node->children + 1;

	int height1;
	if ( c1->flagIndex == B2_NODE_SENTINEL || b2IsLeaf( c1 ) )
	{
		height1 = 0;
	}
	else
	{
		height1 = b2ComputeHeight( tree, b2GetChildIndex( c1 ) );
	}

	int height2;
	if ( c2->flagIndex == B2_NODE_SENTINEL || b2IsLeaf( c2 ) )
	{
		height2 = 0;
	}
	else
	{
		height2 = b2ComputeHeight( tree, b2GetChildIndex( c2 ) );
	}

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

// Internal node area. Skips leaves.
static inline float b2GetInternalNodeArea( const b2TreeNode* node )
{
	bool leaf1 = b2IsLeaf( node->children + 0 );
	bool leaf2 = b2IsLeaf( node->children + 1 );
	if ( leaf1 && leaf2 )
	{
		return 0.0f;
	}

	b2AABB aabb;
	if ( leaf1 )
	{
		aabb = node->children[1].aabb;
	}
	else if ( leaf2 )
	{
		aabb = node->children[0].aabb;
	}
	else
	{
		aabb = b2UnionV( node->children[0].aabb, node->children[1].aabb );
	}

	return b2Perimeter( aabb );
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

	const b2TreeNode* root = tree->nodes + B2_ROOT_NODE;
	b2AABB rootBounds = b2UnionV( root->children[0].aabb, root->children[1].aabb );
	float rootArea = b2Perimeter( rootBounds );
	if ( rootArea <= 0.0f )
	{
		return 0.0f;
	}

	float internalArea = 0.0f;
	for ( int i = 1; i < tree->nodeCapacity; ++i )
	{
		const b2TreeNode* node = tree->nodes + i;
		if ( b2IsAllocated( tree->links + i ) == false )
		{
			continue;
		}

		internalArea += b2GetInternalNodeArea( node );
	}

	return internalArea / rootArea;
}

b2AABB b2DynamicTree_GetRootBounds( const b2DynamicTree* tree )
{
	if ( tree->proxyCount == 0 )
	{
		return (b2AABB){ b2Vec2_zero, b2Vec2_zero };
	}

	const b2TreeNode* node = tree->nodes + B2_ROOT_NODE;
	return b2UnionV( node->children[0].aabb, node->children[1].aabb );
}

#if B2_ENABLE_VALIDATION
static int b2ComputeLeafCount( const b2DynamicTree* tree, int nodeId )
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree->nodeCapacity );
	const b2TreeNode* node = tree->nodes + nodeId;
	const b2TreeChild* c1 = node->children + 0;
	const b2TreeChild* c2 = node->children + 1;

	int count1;
	if ( c1->flagIndex == B2_NODE_SENTINEL )
	{
		count1 = 0;
	}
	else if ( b2IsLeaf( c1 ) )
	{
		count1 = 1;
	}
	else
	{
		count1 = b2ComputeLeafCount( tree, b2GetChildIndex( c1 ) );
	}

	int count2;
	if ( c2->flagIndex == B2_NODE_SENTINEL )
	{
		count2 = 0;
	}
	else if ( b2IsLeaf( c2 ) )
	{
		count2 = 1;
	}
	else
	{
		count2 = b2ComputeLeafCount( tree, b2GetChildIndex( c2 ) );
	}

	return count1 + count2;
}

static inline void b2ValidateChild( const b2DynamicTree* tree, const b2TreeNode* node, int nodeIndex, int slot )
{
	int childIndex = b2GetChildIndex( node->children + slot );
	b2ChildId childId = b2GetChildId( tree, childIndex );
	B2_ASSERT( childId.parent == nodeIndex );
	B2_ASSERT( childId.slot == slot );
}

// Compute the height of a sub-tree.
static void b2ValidateSubtree( const b2DynamicTree* tree, int index )
{
	if ( index == B2_NULL_INDEX )
	{
		return;
	}

	const b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* node = nodes + index;
	const b2TreeLink* link = tree->links + index;
	B2_ASSERT( ( link->flags & B2_ALLOCATED_BIT ) == B2_ALLOCATED_BIT );

	if ( index == B2_ROOT_NODE )
	{
		B2_ASSERT( index == 0 );
		if ( node->children[0].flagIndex != B2_NODE_SENTINEL )
		{
			b2ValidateChild( tree, node, index, 0 );
			b2ValidateSubtree( tree, b2GetChildIndex( node->children + 0 ) );
		}

		if ( node->children[1].flagIndex != B2_NODE_SENTINEL )
		{
			b2ValidateChild( tree, node, index, 1 );
			b2ValidateSubtree( tree, b2GetChildIndex( node->children + 1 ) );
		}
	}
	else
	{
		const b2TreeChild* c1 = node->children + 0;
		const b2TreeChild* c2 = node->children + 1;
		B2_ASSERT( c1->flagIndex != B2_NODE_SENTINEL );
		B2_ASSERT( c2->flagIndex != B2_NODE_SENTINEL );
		b2ValidateChild( tree, node, index, 0 );
		b2ValidateChild( tree, node, index, 1 );
		int leafCount1 = b2GetLeafCount( c1 );
		int leafCount2 = b2GetLeafCount( c2 );
		b2ChildId childId = b2GetChildId( tree, index );
		const b2TreeChild* self = nodes[childId.parent].children + childId.slot;
		B2_ASSERT( self->flagIndex != B2_NODE_SENTINEL && b2IsLeaf( self ) == false );
		B2_ASSERT( self->leafCount == leafCount1 + leafCount2 );
		bool moved1 = c1->flagIndex & B2_MOVED_NODE;
		bool moved2 = c2->flagIndex & B2_MOVED_NODE;
		bool selfMoved = self->flagIndex & B2_MOVED_NODE;
		B2_ASSERT( selfMoved == ( moved1 || moved2 ) );
		B2_ASSERT( b2AABB_Contains( self->aabb, c1->aabb ) );
		B2_ASSERT( b2AABB_Contains( self->aabb, c2->aabb ) );

		b2ValidateSubtree( tree, b2GetChildIndex( c1 ) );
		b2ValidateSubtree( tree, b2GetChildIndex( c2 ) );
	}
}

#endif

void b2DynamicTree_Validate( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION

	// Validate node free list.
	int freeCount = 0;
	int freeIndex = tree->nodeFreeList;
	while ( freeIndex != B2_NULL_INDEX )
	{
		B2_ASSERT( 0 <= freeIndex && freeIndex < tree->nodeCapacity );
		freeIndex = tree->links[freeIndex].next;
		++freeCount;
	}
	B2_ASSERT( tree->nodeCount + freeCount == tree->nodeCapacity );

	// Validate proxy free list.
	freeCount = 0;
	freeIndex = tree->proxyFreeList;
	while ( freeIndex != B2_NULL_INDEX )
	{
		B2_ASSERT( 0 <= freeIndex && freeIndex < tree->proxyCapacity );
		freeIndex = tree->proxies[freeIndex].link.next;
		++freeCount;
	}
	B2_ASSERT( tree->proxyCount + freeCount == tree->proxyCapacity );

	if ( tree->proxyCount == 0 )
	{
		return;
	}

	b2ValidateSubtree( tree, B2_ROOT_NODE );

	const b2TreeNode* root = tree->nodes + B2_ROOT_NODE;
	int leafCount1 = b2GetLeafCount( root->children + 0 );
	int leafCount2 = b2GetLeafCount( root->children + 1 );
	int leafCount = b2ComputeLeafCount( tree, B2_ROOT_NODE );
	B2_ASSERT( leafCount == leafCount1 + leafCount2 );
	B2_ASSERT( leafCount == tree->proxyCount );

	for ( int i = 0; i < tree->dfsNodeCount; ++i )
	{
		if ( b2IsAllocated( tree->links + i ) )
		{
			B2_ASSERT( tree->links[i].parent < i );
		}
	}

#else
	B2_UNUSED( tree );
#endif
}

void b2DynamicTree_ValidateNoEnlarged( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION == 1
	const b2TreeNode* nodes = tree->nodes;
	const b2TreeLink* links = tree->links;
	for ( int i = 0; i < tree->nodeCapacity; ++i )
	{
		const b2TreeNode* node = nodes + i;
		const b2TreeLink* link = links + i;
		if ( link->flags & B2_ALLOCATED_BIT )
		{
			B2_ASSERT( ( node->children[0].flagIndex & B2_MOVED_NODE ) == 0 );
			B2_ASSERT( ( node->children[1].flagIndex & B2_MOVED_NODE ) == 0 );
		}
	}
#else
	B2_UNUSED( tree );
#endif
}

int b2DynamicTree_GetByteCount( const b2DynamicTree* tree )
{
	size_t size = sizeof( b2DynamicTree );
	size += tree->nodeCapacity * sizeof( b2TreeNode );
	size += tree->nodeCapacity * sizeof( b2TreeLink );
	size += tree->proxyCapacity * sizeof( b2TreeProxy );
	size += tree->swapNodes == NULL ? 0 : tree->nodeCapacity * sizeof( b2TreeNode );

	// leafIndices
	size += tree->rebuildCapacity * sizeof( int );
	// leafChildren
	size += tree->rebuildCapacity * sizeof( b2TreeChild );
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

// This got more expensive but rarely used.
b2AABB b2DynamicTree_GetAABB( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	const b2TreeProxy* proxy = tree->proxies + proxyId;
	int parent = proxy->link.parent;
	b2TreeNode* node = tree->nodes + parent;
	int slot = b2GetChildSlot( &proxy->link );
	return node->children[slot].aabb;
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
	stack[stackCount++] = B2_ROOT_NODE;
	const b2TreeNode* nodes = tree->nodes;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];

		const b2TreeNode* node = nodes + nodeId;
		result.nodeVisits += 1;

		for ( int i = 0; i < 2; ++i )
		{
			if ( b2OverlapsV( node->children[i].aabb, aabb ) )
			{
				if ( b2IsLeaf( node->children + i ) )
				{
					// callback to user code with proxy id
					int proxyId = b2GetChildIndex( node->children + i );
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
						stack[stackCount++] = b2GetChildIndex( node->children + i );
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

	if ( tree->nodeCount == 0 )
	{
		return result;
	}

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = B2_ROOT_NODE;
	const b2TreeNode* nodes = tree->nodes;

	while ( stackCount > 0 )
	{
		int nodeId = stack[--stackCount];

		const b2TreeNode* node = nodes + nodeId;
		result.nodeVisits += 1;

		for ( int i = 0; i < 2; ++i )
		{
			if ( b2OverlapsV( node->children[i].aabb, aabb ) )
			{
				if ( b2IsLeaf( node->children + i ) )
				{
					// callback to user code with proxy id
					int proxyId = b2GetChildIndex( node->children + i );
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
						stack[stackCount++] = b2GetChildIndex( node->children + i );
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

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = B2_ROOT_NODE;

	const b2TreeNode* nodes = tree->nodes;

	b2RayCastInput subInput = *input;

	while ( stackCount > 0 )
	{
		int nodeIndex = stack[--stackCount];

		const b2TreeNode* node = nodes + nodeIndex;
		result.nodeVisits += 1;

		const b2TreeChild* children[2];
		children[0] = node->children + 0;
		children[1] = node->children + 1;

		bool leaf1 = b2IsLeaf( children[0] );
		bool leaf2 = b2IsLeaf( children[1] );

		// Push the farthest child first so it gets processed second. This is
		// only relevant if both nodes are internal.
		if ( leaf1 == false && leaf2 == false )
		{
			b2Vec2 center1 = b2AABB_Center( children[0]->aabb );
			b2Vec2 center2 = b2AABB_Center( children[1]->aabb );

			if ( b2DistanceSquared( center1, p1 ) < b2DistanceSquared( center2, p1 ) )
			{
				B2_SWAP( children[0], children[1] );
			}
		}

		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeChild* child = children[i];

			b2AABB nodeAABB = child->aabb;
			if ( b2OverlapsV( nodeAABB, segmentAABB ) == false )
			{
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			b2Vec2 c = b2AABB_Center( nodeAABB );
			b2Vec2 h = b2AABB_Extents( nodeAABB );
			float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
			float term2 = b2Dot( abs_v, h );
			if ( term2 < term1 )
			{
				continue;
			}

			if ( b2IsLeaf( child ) )
			{
				int proxyId = b2GetChildIndex( child );
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
				}
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b2GetChildIndex( child );
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

	b2BoxCastInput subInput = *input;
	const b2TreeNode* nodes = tree->nodes;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = B2_ROOT_NODE;

	while ( stackCount > 0 )
	{
		int nodeIndex = stack[--stackCount];

		const b2TreeNode* node = nodes + nodeIndex;
		result.nodeVisits += 1;

		const b2TreeChild* children[2];
		children[0] = node->children + 0;
		children[1] = node->children + 1;

		bool leaf1 = b2IsLeaf( children[0] );
		bool leaf2 = b2IsLeaf( children[1] );

		// Push the farthest child first so it gets processed second. This is
		// only relevant if both nodes are internal.
		if ( leaf1 == false && leaf2 == false )
		{
			b2Vec2 center1 = b2AABB_Center( children[0]->aabb );
			b2Vec2 center2 = b2AABB_Center( children[1]->aabb );

			if ( b2DistanceSquared( center1, p1 ) < b2DistanceSquared( center2, p1 ) )
			{
				B2_SWAP( children[0], children[1] );
			}
		}

		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeChild* child = children[i];

			b2AABB nodeAABB = child->aabb;
			if ( b2OverlapsV( nodeAABB, totalAABB ) == false )
			{
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			// radius extension is added to the node in this case
			b2Vec2 c = b2AABB_Center( nodeAABB );
			b2Vec2 h = b2Add( b2AABB_Extents( nodeAABB ), extension );
			float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
			float term2 = b2Dot( abs_v, h );
			if ( term2 < term1 )
			{
				continue;
			}

			if ( b2IsLeaf( child ) )
			{
				int proxyId = b2GetChildIndex( child );
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
				}
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE - 1 )
				{
					stack[stackCount++] = b2GetChildIndex( child );
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

// Temporary data used to track the rebuild of a tree node
typedef struct b2RebuildItem
{
	int nodeIndex;
	int childCount;

	// Leaf indices
	int startIndex;
	int splitIndex;
	int endIndex;
} b2RebuildItem;

typedef struct b2CopyItem
{
	int oldIndex;
	int parent;
	int slot;
} b2CopyItem;

// Copy a subtree from the old tree into the rebuilt DFS tree. This puts
// the subtree is contiguous depth first order.
static void b2CopySubtree( b2DynamicTree* tree, int oldIndex, int parent, int slot )
{
	const b2TreeNode* oldNodes = tree->nodes;
	b2TreeNode* newNodes = tree->swapNodes;
	b2TreeLink* links = tree->links;
	b2TreeProxy* proxies = tree->proxies;

	b2CopyItem stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = (b2CopyItem){ oldIndex, parent, slot };

	while ( stackCount > 0 )
	{
		b2CopyItem item = stack[--stackCount];

		// Index assigned at the pop, so a the left subtree preceeds the right child.
		int newIndex = tree->nodeCount++;
		newNodes[newIndex] = oldNodes[item.oldIndex];
		links[newIndex].parent = item.parent;
		links[newIndex].flags = B2_ALLOCATED_BIT | ( item.slot == 0 ? 0 : B2_SLOT_BIT );
		newNodes[item.parent].children[item.slot].flagIndex = (uint32_t)newIndex;

		// Push the right child first so the left pops first.
		for ( int i = 1; i >= 0; --i )
		{
			const b2TreeChild* child = newNodes[newIndex].children + i;
			if ( b2IsLeaf( child ) )
			{
				int proxyId = b2GetChildIndex( child );
				b2TreeLink* link = &proxies[proxyId].link;
				link->parent = newIndex;
				link->flags = B2_ALLOCATED_BIT | ( i == 0 ? 0 : B2_SLOT_BIT );
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				int index = b2GetChildIndex( child );
				stack[stackCount++] = (b2CopyItem){ index, newIndex, i };
			}
		}
	}
}

static void b2PlaceLeaf( b2DynamicTree* tree, b2TreeChild leaf, int parent, int slot )
{
	tree->swapNodes[parent].children[slot] = leaf;

	if ( b2IsLeaf( &leaf ) )
	{
		b2TreeLink* link = &tree->proxies[b2GetChildIndex( &leaf )].link;
		link->parent = parent;
		link->flags = B2_ALLOCATED_BIT | ( slot == 0 ? 0 : B2_SLOT_BIT );
	}
	else
	{
		b2CopySubtree( tree, b2GetChildIndex( &leaf ), parent, slot );
	}
}

// Returns root node index
static int b2BuildTree( b2DynamicTree* tree, int leafCount )
{
	b2TreeNode* nodes = tree->swapNodes;
	b2TreeLink* links = tree->links;
	const b2TreeChild* leaves = tree->leafChildren;
	int* leafIndices = tree->leafIndices;

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
	int* binIndices = tree->binIndices;
#endif

	// Bump allocation into the spare. The root is index zero.
	tree->nodeCount = 0;
	int rootIndex = tree->nodeCount++;
	links[rootIndex].parent = B2_NULL_INDEX;
	links[rootIndex].flags = B2_ALLOCATED_BIT;

	if ( leafCount == 1 )
	{
		b2PlaceLeaf( tree, leaves[leafIndices[0]], rootIndex, 0 );
		nodes[rootIndex].children[1] = b2MakeEmptyChild();
		return rootIndex;
	}

	b2RebuildItem stack[B2_TREE_STACK_SIZE];
	int top = 0;

	stack[0].nodeIndex = rootIndex;
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
			// Both slots written, so the record above this node can be made
			if ( top == 0 )
			{
				break;
			}

			b2RebuildItem* parentItem = stack + ( top - 1 );
			nodes[parentItem->nodeIndex].children[parentItem->childCount] =
				b2MakeInternalChild( nodes + item->nodeIndex, item->nodeIndex );
			top -= 1;
			continue;
		}

		int slot = item->childCount;
		int startIndex = slot == 0 ? item->startIndex : item->splitIndex;
		int endIndex = slot == 0 ? item->splitIndex : item->endIndex;
		int count = endIndex - startIndex;
		B2_ASSERT( count > 0 );

		if ( count == 1 )
		{
			b2PlaceLeaf( tree, leaves[leafIndices[startIndex]], item->nodeIndex, slot );
			continue;
		}

		B2_ASSERT( top < B2_TREE_STACK_SIZE - 1 );
		top += 1;
		b2RebuildItem* newItem = stack + top;
		newItem->nodeIndex = tree->nodeCount++;
		newItem->childCount = -1;
		newItem->startIndex = startIndex;
		newItem->endIndex = endIndex;
		newItem->splitIndex = startIndex;
#if B2_TREE_HEURISTIC == 0
		newItem->splitIndex += b2PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
#else
		newItem->splitIndex += b2PartitionSAH( leafIndices + startIndex, binIndices + startIndex, leafBoxes + startIndex, count );
#endif

		links[newItem->nodeIndex].parent = item->nodeIndex;
		links[newItem->nodeIndex].flags = B2_ALLOCATED_BIT | ( slot == 0 ? 0 : B2_SLOT_BIT );
	}

	return rootIndex;
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

	const b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	if ( fullBuild == false && b2IsChildMoved( root->children + 0 ) == false && b2IsChildMoved( root->children + 1 ) == false )
	{
		// Nothing moved.
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
		tree->leafChildren = B2_GROW( tree->leafChildren, oldCapacity, newCapacity );
#if B2_TREE_HEURISTIC == 0
		tree->leafCenters = B2_GROW( tree->leafCenters, oldCapacity, newCapacity );
#else
		tree->leafBoxes = B2_GROW( tree->leafBoxes, oldCapacity, newCapacity );
		tree->binIndices = B2_GROW( tree->binIndices, oldCapacity, newCapacity );
#endif
		tree->rebuildCapacity = newCapacity;
	}

	int* leafIndices = tree->leafIndices;
	b2TreeChild* leaves = tree->leafChildren;
#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
#endif

	// Gather build leaves. A marked internal child is descended and its node is abandoned in the
	// old array. Everything else is a build leaf, kept subtrees included.
	int leafCount = 0;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = B2_ROOT_NODE;

	while ( stackCount > 0 )
	{
		const b2TreeNode* node = nodes + stack[--stackCount];
		for ( int slot = 0; slot < 2; ++slot )
		{
			b2TreeChild child = node->children[slot];
			if ( child.flagIndex == B2_NODE_SENTINEL )
			{
				continue;
			}

			if ( b2IsLeaf( &child ) == false && ( fullBuild || b2IsChildMoved( &child ) ) )
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				stack[stackCount++] = b2GetChildIndex( &child );
				continue;
			}

			child.flagIndex &= ~B2_MOVED_NODE;
			leafIndices[leafCount] = leafCount;
			leaves[leafCount] = child;
#if B2_TREE_HEURISTIC == 0
			leafCenters[leafCount] = b2AABB_Center( child.aabb );
#else
			leafBoxes[leafCount] = child.aabb;
#endif
			leafCount += 1;
		}
	}

	B2_ASSERT( 0 < leafCount && leafCount <= proxyCount );

	int rootIndex = b2BuildTree( tree, leafCount );
	B2_ASSERT( rootIndex == 0 );

	// The spare is now the tree and the old array is the spare
	B2_SWAP( tree->nodes, tree->swapNodes );

	// Everything above the DFS range is free. Chain it upward so the next allocations extend the
	// dense range instead of scattering.
	int nodeCount = tree->nodeCount;
	int nodeCapacity = tree->nodeCapacity;
	for ( int i = nodeCount; i < nodeCapacity; ++i )
	{
		tree->links[i].next = i + 1 < nodeCapacity ? i + 1 : B2_NULL_INDEX;
		tree->links[i].flags = 0;
	}
	tree->nodeFreeList = nodeCount < nodeCapacity ? nodeCount : B2_NULL_INDEX;
	tree->dfsNodeCount = nodeCount;

	b2DynamicTree_Validate( tree );
	b2DynamicTree_ValidateNoEnlarged( tree );

	return leafCount;
}

void b2DynamicTree_MarkEnlargedFlag( b2DynamicTree* tree, int proxyId )
{
	B2_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2TreeProxy* proxy = tree->proxies + proxyId;
	B2_VALIDATE( b2IsAllocated( &proxy->link ) );

	int nodeIndex = proxy->link.parent;
	int slotIndex = b2GetChildSlot( &proxy->link );

	b2TreeNode* nodes = tree->nodes;
	B2_VALIDATE( b2IsLeaf( nodes[nodeIndex].children + slotIndex ) );

	b2TreeLink* links = tree->links;
	while ( nodeIndex != B2_NULL_INDEX )
	{
		nodes[nodeIndex].children[slotIndex].flagIndex |= B2_MOVED_NODE;
		slotIndex = b2GetChildSlot( links + nodeIndex );
		nodeIndex = links[nodeIndex].parent;
	}
}

void b2DynamicTree_MarkEnlarged( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2TreeProxy* proxy = tree->proxies + proxyId;
	B2_VALIDATE( b2IsAllocated( &proxy->link ) );

	int slotIndex = b2GetChildSlot( &proxy->link );
	int nodeIndex = proxy->link.parent;

	b2TreeNode* nodes = tree->nodes;
	b2TreeChild* child = nodes[nodeIndex].children + slotIndex;
	B2_VALIDATE( b2IsLeaf( child ) );
	B2_VALIDATE( b2AABB_Contains( child->aabb, aabb ) == false );

	// This is not raced since it is the leaf.
	child->aabb = aabb;
	child->flagIndex |= B2_MOVED_NODE;

	b2TreeLink* links = tree->links;
	slotIndex = b2GetChildSlot( links + nodeIndex );
	nodeIndex = links[nodeIndex].parent;

	while ( nodeIndex != B2_NULL_INDEX )
	{
		child = nodes[nodeIndex].children + slotIndex;

		// Read first to avoid the FetchOr if possible.
		if ( b2AtomicLoadU32Raw( &child->flagIndex ) & B2_MOVED_NODE )
		{
			break;
		}

		uint32_t previousFlags = b2AtomicFetchOrU32( &child->flagIndex, B2_MOVED_NODE );
		if ( previousFlags & B2_MOVED_NODE )
		{
			// Ancestor already visited.
			break;
		}

		slotIndex = b2GetChildSlot( links + nodeIndex );
		nodeIndex = links[nodeIndex].parent;
	}
}

void b2DynamicTree_RefitEnlarged( b2DynamicTree* tree, int proxyId )
{
	B2_VALIDATE( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2TreeProxy* proxy = tree->proxies + proxyId;
	int slotIndex = b2GetChildSlot( &proxy->link );
	int nodeIndex = proxy->link.parent;

	b2TreeNode* nodes = tree->nodes;
	b2TreeLink* links = tree->links;
	// B2_VALIDATE( b2IsLeaf( nodes + proxyId ) );
	// B2_VALIDATE( b2AtomicLoadU16( &nodes[proxyId].flags ) & b2_enlargedNode );

	// int childIndex = proxyId;
	// int parentIndex = nodes[proxyId].parent;
	while ( nodeIndex != B2_ROOT_NODE )
	{
		b2TreeNode* node = nodes + nodeIndex;
		B2_VALIDATE( b2AtomicLoadU32Raw( &node->children[slotIndex].flagIndex ) & B2_MOVED_NODE );
		b2TreeLink* link = links + nodeIndex;

		// int child1 = parentNode->children.child1;
		// int child2 = parentNode->children.child2;
		int siblingIndex = 1 ^ slotIndex;

		// Is the sibling also enlarged?
		if ( b2AtomicLoadU32Raw( &node->children[siblingIndex].flagIndex ) & B2_MOVED_NODE )
		{
			// Leave a tag for the sibling or maybe the sibling already tagged (since they know
			// this node is enlarged).
			// Internal nodes will be freed in the rebuild so this flag never needs to be cleared.
			uint32_t previousFlags = b2AtomicFetchOrU32( &link->flags, B2_REFIT_BIT );

			// If the sibling didn't arrive here yet, then bail to avoid a race on the bounds.
			if ( ( previousFlags & B2_REFIT_BIT ) == 0 )
			{
				// Sibling will handle it once they arrive. You got me bro!
				return;
			}
		}

		int parentIndex = link->parent;
		slotIndex = b2AtomicLoadU32Raw( &link->flags ) & B2_SLOT_BIT;

		// Reaching this line means either:
		// 1. Only one child got enlarged
		// 2. The second child has arrived and both siblings have up to date bounds.
		nodes[parentIndex].children[slotIndex].aabb = b2AABB_Union( node->children[0].aabb, node->children[1].aabb );

		nodeIndex = parentIndex;
	}
}

// todo call this during the async rebuild
void b2DynamicTree_ClearEnlarged( b2DynamicTree* tree )
{
	b2TreeNode* nodes = tree->nodes;

	// if ( ( nodes[root].flags & b2_enlargedNode ) == 0 )
	//{
	//	return;
	// }

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = B2_ROOT_NODE;

	while ( stackCount > 0 )
	{
		b2TreeNode* node = nodes + stack[--stackCount];
		b2TreeChild* child1 = node->children + 0;
		if ( child1->flagIndex & B2_MOVED_NODE )
		{
			child1->flagIndex &= ~B2_MOVED_NODE;
			if ( b2IsLeaf( child1 ) == false )
			{
				stack[stackCount++] = b2GetChildIndex( child1 );
			}
		}

		b2TreeChild* child2 = node->children + 1;
		if ( child2->flagIndex & B2_MOVED_NODE )
		{
			child2->flagIndex &= ~B2_MOVED_NODE;
			if ( b2IsLeaf( child2 ) == false )
			{
				stack[stackCount++] = b2GetChildIndex( child2 );
			}
		}
	}
}

void b2DynamicTree_Refit( b2DynamicTree* tree )
{
	B2_ASSERT( tree->dfsNodeCount > 0 );
	b2TreeNode* nodes = tree->nodes;

	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	if ( b2IsChildMoved( root->children + 0 ) == false && b2IsChildMoved( root->children + 1 ) == false )
	{
		return;
	}

	const b2TreeLink* links = tree->links;

	for ( int i = tree->dfsNodeCount - 1; i > 0; --i )
	{
		const b2TreeLink* link = links + i;
		if ( b2IsAllocated( link ) == false )
		{
			continue;
		}

		b2TreeNode* node = nodes + i;
		if ( ( ( node->children[0].flagIndex | node->children[1].flagIndex ) & B2_MOVED_NODE ) == 0 )
		{
			continue;
		}

		node = nodes + link->parent;
		int slotIndex = b2GetChildSlot( link );
		node->children[slotIndex].aabb = b2UnionV( node->children[0].aabb, node->children[1].aabb );
	}
}
