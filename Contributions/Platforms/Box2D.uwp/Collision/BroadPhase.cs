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
namespace Box2D.UWP
{
    /// A proxy for the broadphase. This is used to represent geometry as an AABB.
    /// Note: We can't filter at this level because we still need to filter back
    /// in the client to confirm existing pairs.
    internal struct Proxy
    {
        public AABB aabb;
        public object userData;
        public int treeProxyId;
        public int next;
    };

    internal struct Pair : IComparable<Pair>
    {
        public int proxyIdA;
        public int proxyIdB;
        public int next;

        public int CompareTo(Pair other)
        {
            if (proxyIdA < other.proxyIdA)
            {
                return -1;
            }
            else if (proxyIdA == other.proxyIdA)
            {
                if (proxyIdB < other.proxyIdB)
                {
                    return -1;
                }
                else if (proxyIdB == other.proxyIdB)
                {
                    return 0;
                }
            }

            return 1;
        }
    };

    /// The broad-phase is used for computing pairs and performing volume queries and ray casts.
    /// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
    /// It is up to the client to consume the new pairs and to track subsequent overlap.
    public class BroadPhase
    {
        internal static int NullProxy = -1;

	    public BroadPhase()
        {
            _queryCallback = new Action<int>(QueryCallback);

	        // Build initial proxy pool and free list.
	        _proxyCapacity = 16;
	        _proxyCount = 0;
	        _proxyPool = new Proxy[_proxyCapacity];
        	
	        _freeProxy = 0;
	        for (int i = 0; i < _proxyCapacity - 1; ++i)
	        {
                _proxyPool[i].next = i + 1;
	        }
            _proxyPool[_proxyCapacity - 1].next = NullProxy;

	        _pairCapacity = 16;
	        _pairCount = 0;
            _pairBuffer = new Pair[_pairCapacity];

	        _moveCapacity = 16;
	        _moveCount = 0;
            _moveBuffer = new int[_moveCapacity];
        }

	    /// Create a proxy with an initial AABB. Pairs are not reported until
	    /// UpdatePairs is called.
	    public int CreateProxy(ref AABB aabb, object userData)
        {
        	int proxyId = AllocateProxy();
	        Proxy proxy = _proxyPool[proxyId];
	        proxy.aabb = aabb;
	        proxy.treeProxyId = _tree.CreateProxy(ref aabb, proxyId);
	        proxy.userData = userData;
            _proxyPool[proxyId] = proxy;

	        BufferMove(proxyId);
	        return proxyId;
        }

	    /// Destroy a proxy. It is up to the client to remove any pairs.
	    public void DestroyProxy(int proxyId)
        {
    	    Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);
	        UnBufferMove(proxyId);
            _tree.DestroyProxy(_proxyPool[proxyId].treeProxyId);
	        FreeProxy(proxyId);
        }

	    /// Call MoveProxy as many times as you like, then when you are done
	    /// call UpdatePairs to finalized the proxy pairs (for your time step).
	    public void MoveProxy(int proxyId, ref AABB aabb)
        {
    	    Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);
	        Proxy proxy = _proxyPool[proxyId];
	        proxy.aabb = aabb;
	        _tree.MoveProxy(proxy.treeProxyId, ref aabb);
	        BufferMove(proxyId);
            _proxyPool[proxyId] = proxy;
        }

	    /// Get the AABB for a proxy.
	    public void GetAABB(int proxyId, out AABB aabb)
        {
    	    Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);
	        aabb = _proxyPool[proxyId].aabb;
        }

	    /// Get user data from a proxy. Returns null if the id is invalid.
	    public object GetUserData(int proxyId)
        {
	        Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);
            return _proxyPool[proxyId].userData;
        }

	    /// Get the number of proxies.
	    public int ProxyCount
        {
            get
            {
                return _proxyCount;
            }
        }

	    /// Update the pairs. This results in pair callbacks. This can only add pairs.
	    public void UpdatePairs<T>(Action<T,T> callback)
        {
            // Reset pair buffer
	        _pairCount = 0;

	        // Perform tree queries for all moving proxies.
	        for (int j = 0; j < _moveCount; ++j)
	        {
		        _queryProxyId = _moveBuffer[j];
		        if (_queryProxyId == NullProxy)
		        {
			        continue;
		        }

		        // query tree, create pairs and add them pair buffer.
                _tree.Query(_queryCallback, ref _proxyPool[_queryProxyId].aabb);
	        }

	        // Reset move buffer
	        _moveCount = 0;

	        // Sort the pair buffer to expose duplicates.
            Array.Sort(_pairBuffer, 0, _pairCount);

	        // Send the pairs back to the client.
	        int i = 0;
	        while (i < _pairCount)
	        {
		        Pair primaryPair = _pairBuffer[i];
		        Proxy proxyA = _proxyPool[primaryPair.proxyIdA];
		        Proxy proxyB = _proxyPool[primaryPair.proxyIdB];

		        callback((T)(proxyA.userData), (T)(proxyB.userData));
		        ++i;

		        // Skip any duplicate pairs.
		        while (i < _pairCount)
		        {
			        Pair pair = _pairBuffer[i];
			        if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
			        {
				        break;
			        }
			        ++i;
		        }
	        }
        }

	    /// Query an AABB for overlapping proxies. The callback class
	    /// is called for each proxy that overlaps the supplied AABB.
	    public void Query<T>(Func<T, bool> callback, ref AABB aabb)
        {
	        _tree.Query((userData) =>
		                {
			                Debug.Assert(0 <= userData && userData < _proxyCapacity);
			                callback((T)(_proxyPool[userData].userData));
		                }, ref aabb);
        }

	    /// Compute the height of the embedded tree.
	    public int ComputeHeight()
        {
            return _tree.ComputeHeight();
        }

	    internal int AllocateProxy()
        {
	        if (_freeProxy == NullProxy)
	        {
		        Debug.Assert(_proxyCount == _proxyCapacity);
		        Proxy[] oldPool = _proxyPool;

		        _proxyCapacity *= 2;
		        _proxyPool = new Proxy[_proxyCapacity];

                Array.Copy(oldPool, _proxyPool, _proxyCount);

		        _freeProxy = _proxyCount;
		        for (int i = _proxyCount; i < _proxyCapacity - 1; ++i)
		        {
			        _proxyPool[i].next = i + 1;
		        }
                _proxyPool[_proxyCapacity - 1].next = NullProxy;
	        }

	        int proxyId = _freeProxy;
            _freeProxy = _proxyPool[proxyId].next;
            _proxyPool[proxyId].next = NullProxy;
	        ++_proxyCount;
	        return proxyId;
        }

	    internal void FreeProxy(int proxyId)
        {
	        Debug.Assert(0 < _proxyCount);
	        Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);
            _proxyPool[proxyId].next = _freeProxy;
	        _freeProxy = proxyId;
	        --_proxyCount;
        }

	    internal void BufferMove(int proxyId)
        {
	        Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);

	        if (_moveCount == _moveCapacity)
	        {
		        int[] oldBuffer = _moveBuffer;
		        _moveCapacity *= 2;
		        _moveBuffer = new int[_moveCapacity];
                Array.Copy(oldBuffer, _moveBuffer, _moveCount);
	        }

	        _moveBuffer[_moveCount] = proxyId;
	        ++_moveCount;
        }

	    internal void UnBufferMove(int proxyId)
        {
	        for (int i = 0; i < _moveCount; ++i)
	        {
		        if (_moveBuffer[i] == proxyId)
		        {
			        _moveBuffer[i] = NullProxy;
			        return;
		        }
	        }
        }

        internal void QueryCallback(int proxyId)
        {
	        Debug.Assert(0 <= proxyId && proxyId < _proxyCapacity);

	        // A proxy cannot form a pair with itself.
	        if (proxyId == _queryProxyId)
	        {
		        return;
	        }

	        // Check the tight fitting AABBs for overlap.
            if (AABB.TestOverlap(ref _proxyPool[proxyId].aabb, ref _proxyPool[_queryProxyId].aabb) == false)
	        {
		        return;
	        }

	        // Grow the pair buffer as needed.
	        if (_pairCount == _pairCapacity)
	        {
		        Pair[] oldBuffer = _pairBuffer;
		        _pairCapacity *= 2;
                _pairBuffer = new Pair[_pairCapacity];
                Array.Copy(oldBuffer, _pairBuffer, _pairCount);
	        }

	        _pairBuffer[_pairCount].proxyIdA = Math.Min(proxyId, _queryProxyId);
	        _pairBuffer[_pairCount].proxyIdB = Math.Max(proxyId, _queryProxyId);
	        ++_pairCount;
        }

	    internal DynamicTree _tree = new DynamicTree();

	    internal int[] _moveBuffer;
	    internal int _moveCapacity;
	    internal int _moveCount;

	    internal Proxy[] _proxyPool;
	    internal int _proxyCapacity;
	    internal int _proxyCount;
	    internal int _freeProxy;

	    internal Pair[] _pairBuffer;
	    internal int _pairCapacity;
	    internal int _pairCount;

	    internal int _queryProxyId;

        Action<int> _queryCallback;
    }
}
