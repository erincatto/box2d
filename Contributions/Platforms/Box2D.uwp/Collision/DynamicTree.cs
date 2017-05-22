/* Box2D.uwp port of Box2D.Xna: Copyright (c) 2015 Nukepayload2
* Box2D.Xna port of Box2D:
* Copyright (c) 2009 Brandon Furtwangler, Nathan Furtwangler
*
* Original source Box2D:
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com 
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


using System;
using System.Diagnostics;
using System.Numerics;
using Box2D.uwp.UWPExtensions;

namespace Box2D.UWP
{
    /// A node in the dynamic tree. The client does not interact with this directly.
    internal struct DynamicTreeNode
    {
        internal bool IsLeaf() 
	    {
		    return child1 == DynamicTree.NullNode;
	    }

	    internal AABB aabb;
        internal int userData;

        internal int parentOrNext;

        internal int child1;
        internal int child2;
    };

    /// A dynamic tree arranges data in a binary tree to accelerate
    /// queries such as volume queries and ray casts. Leafs are proxies
    /// with an AABB. In the tree we expand the proxy AABB by Settings.b2_fatAABBFactor
    /// so that the proxy AABB is bigger than the client object. This allows the client
    /// object to move by small amounts without triggering a tree update.
    ///
    /// Nodes are pooled and relocatable, so we use node indices rather than pointers.
    public class DynamicTree
    {
        internal static int NullNode = -1;

	    /// ructing the tree initializes the node pool.
	    public DynamicTree()
        {
	        _root = NullNode;

	        _nodeCapacity = 16;
	        _nodeCount = 0;
	        _nodes = new DynamicTreeNode[_nodeCapacity];
	        
	        // Build a linked list for the free list.
	        for (int i = 0; i < _nodeCapacity - 1; ++i)
	        {
                _nodes[i].parentOrNext = i + 1;
	        }
            _nodes[_nodeCapacity - 1].parentOrNext = NullNode;
	        _freeList = 0;

	        _path = 0;
        }

	    /// Create a proxy. Provide a tight fitting AABB and a userData pointer.
	    public int CreateProxy(ref AABB aabb, int userData)
        {
	        int proxyId = AllocateNode();

	        // Fatten the aabb.
            Vector2 r = new Vector2(Settings.b2_aabbExtension, Settings.b2_aabbExtension);
	        _nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
	        _nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
	        _nodes[proxyId].userData = userData;

	        InsertLeaf(proxyId);

	        return proxyId;
        }

	    /// Destroy a proxy. This asserts if the id is invalid.
	    public void DestroyProxy(int proxyId)
        {
	        Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
	        Debug.Assert(_nodes[proxyId].IsLeaf());

	        RemoveLeaf(proxyId);
	        FreeNode(proxyId);
        }

	    /// Move a proxy. If the proxy has moved outside of its fattened AABB,
	    /// then the proxy is removed from the tree and re-inserted. Otherwise
	    /// the function returns immediately.
	    public void MoveProxy(int proxyId, ref AABB aabb)
        {
	        Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);

	        Debug.Assert(_nodes[proxyId].IsLeaf());

	        if (_nodes[proxyId].aabb.Contains(ref aabb))
	        {
		        return;
	        }

	        RemoveLeaf(proxyId);

	        Vector2 r = new Vector2(Settings.b2_aabbExtension, Settings.b2_aabbExtension);
	        _nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
	        _nodes[proxyId].aabb.upperBound = aabb.upperBound + r;

	        InsertLeaf(proxyId);
        }

	    /// Perform some iterations to re-balance the tree.
	    public void Rebalance(int iterations)
        {
	        if (_root == NullNode)
	        {
		        return;
	        }

	        for (int i = 0; i < iterations; ++i)
	        {
		        int node = _root;

		        int bit = 0;
		        while (_nodes[node].IsLeaf() == false)
		        {
                    node = ((_path >> bit) & 1) == 0 ? _nodes[node].child1 : _nodes[node].child1;
			        bit = (bit + 1) & (8 * sizeof(uint) - 1);
		        }
		        ++_path;

		        RemoveLeaf(node);
		        InsertLeaf(node);
	        }
        }

	    /// Get proxy user data.
	    /// @return the proxy user data or 0 if the id is invalid.
	    public int GetUserData(int proxyId)
        {
	        if (proxyId < _nodeCapacity)
	        {
		        return _nodes[proxyId].userData;
	        }
	        else
	        {
		        return 0;
	        }
        }

	    /// Compute the height of the tree.
	    public int ComputeHeight()
        {
	        return ComputeHeight(_root);
        }

        static int k_stackSize = 128;
        static int[] stack = new int[k_stackSize];

	    /// Query an AABB for overlapping proxies. The callback class
	    /// is called for each proxy that overlaps the supplied AABB.
	    public void Query(Action<int> callback, ref AABB aabb)
        {
	        int count = 0;
	        stack[count++] = _root;

	        while (count > 0)
	        {
		        int nodeId = stack[--count];
		        if (nodeId == NullNode)
		        {
			        continue;
		        }

		        DynamicTreeNode node = _nodes[nodeId];

                if (AABB.TestOverlap(ref node.aabb, ref aabb))
		        {
			        if (node.IsLeaf())
			        {
				        callback(node.userData);
			        }
			        else
			        {
				        Debug.Assert(count + 1 < k_stackSize);
				        stack[count++] = node.child1;
				        stack[count++] = node.child2;
			        }
		        }
	        }
        }

        public delegate void RayCastCallback(out RayCastOutput output, ref RayCastInput input, int userData);

	    /// Ray-cast against the proxies in the tree. This relies on the callback
	    /// to perform a exact ray-cast in the case were the proxy contains a shape.
	    /// The callback also performs the any collision filtering. This has performance
	    /// roughly equal to k * log(n), where k is the number of collisions and n is the
	    /// number of proxies in the tree.
	    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	    /// @param callback a callback class that is called for each proxy that is hit by the ray.
	    public void RayCast(RayCastCallback callback, ref RayCastInput input) 
        {
	        Vector2 p1 = input.p1;
	        Vector2 p2 = input.p2;
	        Vector2 r = p2 - p1;
	        Debug.Assert(r.LengthSquared() > 0.0f);
	        r.Normalize();

	        // v is perpendicular to the segment.
	        Vector2 v = MathUtils.Cross(1.0f, r);
	        Vector2 abs_v = MathUtils.Abs(v);

	        // Separating axis for segment (Gino, p80).
	        // |dot(v, p1 - c)| > dot(|v|, h)

	        float maxFraction = input.maxFraction;

	        // Build a bounding box for the segment.
	        AABB segmentAABB = new AABB();
	        {
		        Vector2 t = p1 + maxFraction * (p2 - p1);
		        segmentAABB.lowerBound = Vector2.Min(p1, t);
		        segmentAABB.upperBound = Vector2.Max(p1, t);
	        }

	        int count = 0;
	        stack[count++] = _root;

	        while (count > 0)
	        {
		        int nodeId = stack[--count];
		        if (nodeId == NullNode)
		        {
			        continue;
		        }

		        DynamicTreeNode node = _nodes[nodeId];

		        if (AABB.TestOverlap(ref node.aabb, ref segmentAABB) == false)
		        {
			        continue;
		        }

		        // Separating axis for segment (Gino, p80).
		        // |dot(v, p1 - c)| > dot(|v|, h)
		        Vector2 c = node.aabb.GetCenter();
		        Vector2 h = node.aabb.GetExtents();
		        float separation = Math.Abs(Vector2.Dot(v, p1 - c)) - Vector2.Dot(abs_v, h);
		        if (separation > 0.0f)
		        {
			        continue;
		        }

		        if (node.IsLeaf())
		        {
			        RayCastInput subInput;
			        subInput.p1 = input.p1;
			        subInput.p2 = input.p2;
			        subInput.maxFraction = maxFraction;

			        RayCastOutput output;

			        callback(out output, ref subInput, node.userData);

			        if (output.hit)
			        {
				        // Early exit.
				        if (output.fraction == 0.0f)
				        {
					        return;
				        }

				        maxFraction = output.fraction;

				        // Update segment bounding box.
				        {
					        Vector2 t = p1 + maxFraction * (p2 - p1);
					        segmentAABB.lowerBound = Vector2.Min(p1, t);
                            segmentAABB.upperBound = Vector2.Max(p1, t);
				        }
			        }
		        }
		        else
		        {
			        Debug.Assert(count + 1 < k_stackSize);
			        stack[count++] = node.child1;
			        stack[count++] = node.child2;
		        }
	        }
        }

        private int AllocateNode()
        {
	        // Expand the node pool as needed.
	        if (_freeList == NullNode)
	        {
		        Debug.Assert(_nodeCount == _nodeCapacity);

		        // The free list is empty. Rebuild a bigger pool.
		        DynamicTreeNode[] oldNodes = _nodes;
		        _nodeCapacity *= 2;
                _nodes = new DynamicTreeNode[_nodeCapacity];
                Array.Copy(oldNodes, _nodes, _nodeCount);

		        // Build a linked list for the free list. The parent
		        // pointer becomes the "next" pointer.
		        for (int i = _nodeCount; i < _nodeCapacity - 1; ++i)
		        {
                    _nodes[i].parentOrNext = i + 1;
		        }
		        _nodes[_nodeCapacity-1].parentOrNext = NullNode;
		        _freeList = _nodeCount;
	        }

	        // Peel a node off the free list.
	        int nodeId = _freeList;
            _freeList = _nodes[nodeId].parentOrNext;
            _nodes[nodeId].parentOrNext = NullNode;
	        _nodes[nodeId].child1 = NullNode;
	        _nodes[nodeId].child2 = NullNode;
	        ++_nodeCount;
	        return nodeId;
        }

        private void FreeNode(int nodeId)
        {
	        Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
	        Debug.Assert(0 < _nodeCount);
            _nodes[nodeId].parentOrNext = _freeList;
	        _freeList = nodeId;
	        --_nodeCount;
        }

        private void InsertLeaf(int leaf)
        {
	        if (_root == NullNode)
	        {
		        _root = leaf;
                _nodes[_root].parentOrNext = NullNode;
		        return;
	        }

	        // Find the best sibling for this node.
	        Vector2 center = _nodes[leaf].aabb.GetCenter();
	        int sibling = _root;
	        if (_nodes[sibling].IsLeaf() == false)
	        {
		        do 
		        {
			        int child1 = _nodes[sibling].child1;
			        int child2 = _nodes[sibling].child2;

			        Vector2 delta1 = MathUtils.Abs(_nodes[child1].aabb.GetCenter() - center);
			        Vector2 delta2 = MathUtils.Abs(_nodes[child2].aabb.GetCenter() - center);

			        float norm1 = delta1.X + delta1.Y;
			        float norm2 = delta2.X + delta2.Y;

			        if (norm1 < norm2)
			        {
				        sibling = child1;
			        }
			        else
			        {
				        sibling = child2;
			        }

		        }
		        while(_nodes[sibling].IsLeaf() == false);
	        }

	        // Create a parent for the siblings.
            int node1 = _nodes[sibling].parentOrNext;
	        int node2 = AllocateNode();
            _nodes[node2].parentOrNext = node1;
	        _nodes[node2].userData = 0;
	        _nodes[node2].aabb.Combine(ref _nodes[leaf].aabb, ref _nodes[sibling].aabb);

	        if (node1 != NullNode)
	        {
                if (_nodes[_nodes[sibling].parentOrNext].child1 == sibling)
		        {
			        _nodes[node1].child1 = node2;
		        }
		        else
		        {
			        _nodes[node1].child2 = node2;
		        }

		        _nodes[node2].child1 = sibling;
		        _nodes[node2].child2 = leaf;
                _nodes[sibling].parentOrNext = node2;
                _nodes[leaf].parentOrNext = node2;

		        do 
		        {
			        if (_nodes[node1].aabb.Contains(ref _nodes[node2].aabb))
			        {
				        break;
			        }

			        _nodes[node1].aabb.Combine(ref _nodes[_nodes[node1].child1].aabb, ref _nodes[_nodes[node1].child2].aabb);
			        node2 = node1;
                    node1 = _nodes[node1].parentOrNext;
		        }
		        while(node1 != NullNode);
	        }
	        else
	        {
		        _nodes[node2].child1 = sibling;
		        _nodes[node2].child2 = leaf;
                _nodes[sibling].parentOrNext = node2;
                _nodes[leaf].parentOrNext = node2;
		        _root = node2;
	        }
        }

        private void RemoveLeaf(int leaf)
        {
            if (leaf == _root)
	        {
		        _root = NullNode;
		        return;
	        }

            int node2 = _nodes[leaf].parentOrNext;
            int node1 = _nodes[node2].parentOrNext;
	        int sibling;
	        if (_nodes[node2].child1 == leaf)
	        {
		        sibling = _nodes[node2].child2;
	        }
	        else
	        {
		        sibling = _nodes[node2].child1;
	        }

	        if (node1 != NullNode)
	        {
		        // Destroy node2 and connect node1 to sibling.
		        if (_nodes[node1].child1 == node2)
		        {
			        _nodes[node1].child1 = sibling;
		        }
		        else
		        {
			        _nodes[node1].child2 = sibling;
		        }
                _nodes[sibling].parentOrNext = node1;
		        FreeNode(node2);

		        // Adjust ancestor bounds.
		        while (node1 != NullNode)
		        {
			        AABB oldAABB = _nodes[node1].aabb;
			        _nodes[node1].aabb.Combine(ref _nodes[_nodes[node1].child1].aabb, ref _nodes[_nodes[node1].child2].aabb);

			        if (oldAABB.Contains(ref _nodes[node1].aabb))
			        {
				        break;
			        }

                    node1 = _nodes[node1].parentOrNext;
		        }
	        }
	        else
	        {
		        _root = sibling;
                _nodes[sibling].parentOrNext = NullNode;
		        FreeNode(node2);
	        }
        }

        private int ComputeHeight(int nodeId)
        {
		    if (nodeId == NullNode)
	        {
		        return 0;
	        }

	        Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
	        DynamicTreeNode node = _nodes[nodeId];
	        int height1 = ComputeHeight(node.child1);
	        int height2 = ComputeHeight(node.child2);
	        return 1 + Math.Max(height1, height2);
        }

	    int _root;

	    DynamicTreeNode[] _nodes;
	    int _nodeCount;
	    int _nodeCapacity;

	    int _freeList;

	    /// This is used incrementally traverse the tree for re-balancing.
	    int _path;
    }
}
