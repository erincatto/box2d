/*
* Copyright (c) 2009 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <Box2D/Collision/b2DynamicTree.h>
#include <cstring>
#include <cfloat>
using namespace std;

b2DynamicTree::b2DynamicTree()
{
	m_root = b2_nullNode;

	m_nodeCapacity = 16;
	m_nodeCount = 0;
	m_nodes = (b2DynamicTreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2DynamicTreeNode));
	memset(m_nodes, 0, m_nodeCapacity * sizeof(b2DynamicTreeNode));

	// Build a linked list for the free list.
	for (int32 i = 0; i < m_nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
	}
	m_nodes[m_nodeCapacity-1].next = b2_nullNode;
	m_freeList = 0;

	m_path = 0;

	m_insertionCount = 0;
}

b2DynamicTree::~b2DynamicTree()
{
	// This frees the entire tree in one shot.
	b2Free(m_nodes);
}

// Allocate a node from the pool. Grow the pool if necessary.
int32 b2DynamicTree::AllocateNode()
{
	// Expand the node pool as needed.
	if (m_freeList == b2_nullNode)
	{
		b2Assert(m_nodeCount == m_nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		b2DynamicTreeNode* oldNodes = m_nodes;
		m_nodeCapacity *= 2;
		m_nodes = (b2DynamicTreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2DynamicTreeNode));
		memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(b2DynamicTreeNode));
		b2Free(oldNodes);

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for (int32 i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
		}
		m_nodes[m_nodeCapacity-1].next = b2_nullNode;
		m_freeList = m_nodeCount;
	}

	// Peel a node off the free list.
	int32 nodeId = m_freeList;
	m_freeList = m_nodes[nodeId].next;
	m_nodes[nodeId].parent = b2_nullNode;
	m_nodes[nodeId].child1 = b2_nullNode;
	m_nodes[nodeId].child2 = b2_nullNode;
	m_nodes[nodeId].leafCount = 0;
	++m_nodeCount;
	return nodeId;
}

// Return a node to the pool.
void b2DynamicTree::FreeNode(int32 nodeId)
{
	b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	b2Assert(0 < m_nodeCount);
	m_nodes[nodeId].next = m_freeList;
	m_freeList = nodeId;
	--m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
int32 b2DynamicTree::CreateProxy(const b2AABB& aabb, void* userData)
{
	int32 proxyId = AllocateNode();

	// Fatten the aabb.
	b2Vec2 r(b2_aabbExtension, b2_aabbExtension);
	m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
	m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].leafCount = 1;

	InsertLeaf(proxyId);

	return proxyId;
}

void b2DynamicTree::DestroyProxy(int32 proxyId)
{
	b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
	b2Assert(m_nodes[proxyId].IsLeaf());

	RemoveLeaf(proxyId);
	FreeNode(proxyId);
}

bool b2DynamicTree::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
	b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);

	b2Assert(m_nodes[proxyId].IsLeaf());

	if (m_nodes[proxyId].aabb.Contains(aabb))
	{
		return false;
	}

	RemoveLeaf(proxyId);

	// Extend AABB.
	b2AABB b = aabb;
	b2Vec2 r(b2_aabbExtension, b2_aabbExtension);
	b.lowerBound = b.lowerBound - r;
	b.upperBound = b.upperBound + r;

	// Predict AABB displacement.
	b2Vec2 d = b2_aabbMultiplier * displacement;

	if (d.x < 0.0f)
	{
		b.lowerBound.x += d.x;
	}
	else
	{
		b.upperBound.x += d.x;
	}

	if (d.y < 0.0f)
	{
		b.lowerBound.y += d.y;
	}
	else
	{
		b.upperBound.y += d.y;
	}

	m_nodes[proxyId].aabb = b;

	InsertLeaf(proxyId);
	return true;
}

void b2DynamicTree::InsertLeaf(int32 leaf)
{
	++m_insertionCount;

	if (m_root == b2_nullNode)
	{
		m_root = leaf;
		m_nodes[m_root].parent = b2_nullNode;
		return;
	}

	// Find the best sibling for this node
	b2AABB leafAABB = m_nodes[leaf].aabb;
	int32 sibling = m_root;
	while (m_nodes[sibling].IsLeaf() == false)
	{
		int32 child1 = m_nodes[sibling].child1;
		int32 child2 = m_nodes[sibling].child2;

		// Expand the node's AABB.
		m_nodes[sibling].aabb.Combine(leafAABB);
		m_nodes[sibling].leafCount += 1;

		float32 siblingArea = m_nodes[sibling].aabb.GetPerimeter();
		b2AABB parentAABB;
		parentAABB.Combine(m_nodes[sibling].aabb, leafAABB);
		float32 parentArea = parentAABB.GetPerimeter();
		float32 cost1 = 2.0f * parentArea;

		float32 inheritanceCost = 2.0f * (parentArea - siblingArea);

		float32 cost2;
		if (m_nodes[child1].IsLeaf())
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			cost2 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			float32 oldArea = m_nodes[child1].aabb.GetPerimeter();
			float32 newArea = aabb.GetPerimeter();
			cost2 = (newArea - oldArea) + inheritanceCost;
		}

		float32 cost3;
		if (m_nodes[child2].IsLeaf())
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			cost3 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			float32 oldArea = m_nodes[child2].aabb.GetPerimeter();
			float32 newArea = aabb.GetPerimeter();
			cost3 = newArea - oldArea + inheritanceCost;
		}

		// Descend according to the minimum cost.
		if (cost1 < cost2 && cost1 < cost3)
		{
			break;
		}

		// Expand the node's AABB to account for the new leaf.
		m_nodes[sibling].aabb.Combine(leafAABB);

		// Descend
		if (cost2 < cost3)
		{
			sibling = child1;
		}
		else
		{
			sibling = child2;
		}
	}

	// Create a new parent for the siblings.
	int32 oldParent = m_nodes[sibling].parent;
	int32 newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = NULL;
	m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
	m_nodes[newParent].leafCount = m_nodes[sibling].leafCount + 1;

	if (oldParent != b2_nullNode)
	{
		// The sibling was not the root.
		if (m_nodes[oldParent].child1 == sibling)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}
}

void b2DynamicTree::RemoveLeaf(int32 leaf)
{
	if (leaf == m_root)
	{
		m_root = b2_nullNode;
		return;
	}

	int32 parent = m_nodes[leaf].parent;
	int32 grandParent = m_nodes[parent].parent;
	int32 sibling;
	if (m_nodes[parent].child1 == leaf)
	{
		sibling = m_nodes[parent].child2;
	}
	else
	{
		sibling = m_nodes[parent].child1;
	}

	if (grandParent != b2_nullNode)
	{
		// Destroy parent and connect sibling to grandParent.
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = sibling;
		}
		else
		{
			m_nodes[grandParent].child2 = sibling;
		}
		m_nodes[sibling].parent = grandParent;
		FreeNode(parent);

		// Adjust ancestor bounds.
		parent = grandParent;
		while (parent != b2_nullNode)
		{
			b2AABB oldAABB = m_nodes[parent].aabb;
			m_nodes[parent].aabb.Combine(m_nodes[m_nodes[parent].child1].aabb, m_nodes[m_nodes[parent].child2].aabb);

			b2Assert(m_nodes[parent].leafCount > 0);
			m_nodes[parent].leafCount -= 1;

			parent = m_nodes[parent].parent;
		}
	}
	else
	{
		m_root = sibling;
		m_nodes[sibling].parent = b2_nullNode;
		FreeNode(parent);
	}
}

void b2DynamicTree::Rebalance(int32 iterations)
{
	if (m_root == b2_nullNode)
	{
		return;
	}

	// Rebalance the tree by removing and re-inserting leaves.
	for (int32 i = 0; i < iterations; ++i)
	{
		int32 node = m_root;

		uint32 bit = 0;
		while (m_nodes[node].IsLeaf() == false)
		{
			int32* children = &m_nodes[node].child1;
			
			// Child selector based on a bit in the path
			int32 selector = (m_path >> bit) & 1;

			// Select the child nod
			node = children[selector];

			// Keep bit between 0 and 31 because m_path has 32 bits
			// bit = (bit + 1) % 31
			bit = (bit + 1) & 0x1F;
		}
		++m_path;

		RemoveLeaf(node);
		InsertLeaf(node);
	}
}

// Compute the height of a sub-tree.
int32 b2DynamicTree::ComputeHeight(int32 nodeId) const
{
	if (nodeId == b2_nullNode)
	{
		return 0;
	}

	b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	b2DynamicTreeNode* node = m_nodes + nodeId;
	int32 height1 = ComputeHeight(node->child1);
	int32 height2 = ComputeHeight(node->child2);
	return 1 + b2Max(height1, height2);
}

int32 b2DynamicTree::ComputeHeight() const
{
	return ComputeHeight(m_root);
}

int32 b2DynamicTree::CountLeaves(int32 nodeId) const
{
	if (nodeId == b2_nullNode)
	{
		return 0;
	}

	b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	b2DynamicTreeNode* node = m_nodes + nodeId;

	if (node->IsLeaf())
	{
		b2Assert(node->leafCount == 1);
		return 1;
	}

	int32 count1 = CountLeaves(node->child1);
	int32 count2 = CountLeaves(node->child2);
	int32 count = count1 + count2;
	b2Assert(count == node->leafCount);
	return count;
}

void b2DynamicTree::Validate() const
{
	CountLeaves(m_root);	
}
