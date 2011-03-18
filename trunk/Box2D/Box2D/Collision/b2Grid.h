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

#ifndef B2_GRID_H
#define B2_GRID_H

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Common/b2BlockAllocator.h>

#define b2_nullProxy (-1)

struct b2Proxy
{
	/// Enlarge AABB.
	b2AABB aabb;
	void* userData;
	int32 timeStamp;
	bool large;
};

struct b2ProxyNode
{
	/// Duplicate of b2Proxy::aabb to reduce cache misses.
	b2AABB aabb;
	int32 index;
};

struct b2Cell
{
	b2ProxyNode* nodes;
	int32 nodeCapacity;
	int32 nodeCount;
};

/// This implementation is not a tree at all. It is just a cache friendly array of AABBs.
class b2Grid
{
public:

	/// Constructing the grid. Provide the expected object size and count.
	b2Grid(float32 objectSize, int32 proxyCount);

	/// Destroy the grid. Frees all allocated memory.
	~b2Grid();

	/// Create a proxy. Provide a tight fitting AABB and a userData pointer.
	/// The user data pointer must be non-null.
	int32 CreateProxy(const b2AABB& aabb, void* userData);

	/// Destroy a proxy. This asserts if the id is invalid.
	void DestroyProxy(int32 proxyId);

	/// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
	/// then the proxy is removed from the tree and re-inserted. Otherwise
	/// the function returns immediately.
	/// @return true if the proxy was re-inserted.
	bool MoveProxy(int32 proxyId, const b2AABB& aabb1, const b2Vec2& displacement);

	/// Get proxy user data.
	/// @return the proxy user data or 0 if the id is invalid.
	void* GetUserData(int32 proxyId) const;

	/// Get the fat AABB for a proxy.
	const b2AABB& GetFatAABB(int32 proxyId) const;

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const b2AABB& aabb) const;

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input) const;

	void Validate() const;

private:

	b2Proxy* m_proxies;
	int32 m_proxyCapacity;
	int32 m_proxyCount;
	int32 m_freeProxy;

	b2Cell* m_table;
	int32 m_tableCount;

	// Array of proxies that don't fit within a single cell.
	b2Proxy* m_largeProxies;
	int32 m_largeCount;
	int32 m_largeCapacity;

	b2BlockAllocator m_allocator;
	int32 m_timeStamp;
};

inline void* b2DynamicTree::GetUserData(int32 proxyId) const
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);
	return m_proxies[proxyId].userData;
}

inline const b2AABB& b2DynamicTree::GetFatAABB(int32 proxyId) const
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);
	int32 index = m_proxyMap[proxyId];
	return m_proxies[index].aabb;
}

template <typename T>
inline void b2DynamicTree::Query(T* callback, const b2AABB& aabb) const
{
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		if (b2TestOverlap(m_proxies[i].aabb, aabb))
		{
			bool proceed = callback->QueryCallback(m_proxies[i].id);
			if (proceed == false)
			{
				return;
			}
		}
	}
}

template <typename T>
inline void b2DynamicTree::RayCast(T* callback, const b2RayCastInput& input) const
{
	b2Vec2 p1 = input.p1;
	b2Vec2 p2 = input.p2;
	b2Vec2 r = p2 - p1;
	b2Assert(r.LengthSquared() > 0.0f);
	r.Normalize();

	// v is perpendicular to the segment.
	b2Vec2 v = b2Cross(1.0f, r);
	b2Vec2 abs_v = b2Abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	float32 maxFraction = input.maxFraction;

	// Build a bounding box for the segment.
	b2AABB segmentAABB;
	{
		b2Vec2 t = p1 + maxFraction * (p2 - p1);
		segmentAABB.lowerBound = b2Min(p1, t);
		segmentAABB.upperBound = b2Max(p1, t);
	}

	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		const b2Proxy* proxy = m_proxies + i;
		b2AABB proxyAABB = proxy->aabb;

		if (b2TestOverlap(proxyAABB, segmentAABB) == false)
		{
			continue;
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		b2Vec2 c = proxyAABB.GetCenter();
		b2Vec2 h = proxyAABB.GetExtents();
		float32 separation = b2Abs(b2Dot(v, p1 - c)) - b2Dot(abs_v, h);
		if (separation > 0.0f)
		{
			continue;
		}

		b2RayCastInput subInput;
		subInput.p1 = input.p1;
		subInput.p2 = input.p2;
		subInput.maxFraction = maxFraction;

		float32 value = callback->RayCastCallback(subInput, proxy->id);

		if (value == 0.0f)
		{
			// The client has terminated the ray cast.
			return;
		}

		if (value > 0.0f)
		{
			// Update segment bounding box.
			maxFraction = value;
			b2Vec2 t = p1 + maxFraction * (p2 - p1);
			segmentAABB.lowerBound = b2Min(p1, t);
			segmentAABB.upperBound = b2Max(p1, t);
		}
	}
}

#endif
