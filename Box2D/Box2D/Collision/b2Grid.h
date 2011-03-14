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
#include <Box2D/Common/b2GrowableStack.h>

#define b2_nullNode (-1)

// This must be a power of 2
#define b2_blockCount	(1 << 4)

struct b2Proxy
{
	/// This is the fattened AABB.
	b2AABB aabb;
	void* userData;
	int32 id;
};

struct b2Block
{
	int16 x, y, z;
	b2Proxy proxies[b2_blockCount];
	b2Block* next;
};

/// This implementation is not a tree at all. It is just a cache friendly array of AABBs.
class b2Grid
{
public:

	/// Constructing the grid. Provide the expected world AABB, object size, and proxy count.
	b2Grid(const b2Vec2& worldAABB, float32 objectSize, int32 proxyCount);

	/// Destroy the tree, freeing the node pool.
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

	enum
	{
		// Number of proxies per block
		e_blockSize = 16
	};

	// Map of ids to proxies indices. This may have holes (which contain a free list).
	int32* m_proxyMap;
	int32 m_mapCapacity;
	int32 m_freeId;

	b2Block* m_pool;
	int32 m_poolCapacity;
	b2Bloc* m_freeBlock;

	// Array of proxies that don't fit within a single cell.
	b2Block* m_looseProxies;
	int32 m_looseCount;
	int32 m_looseCapacity;

	b2Block** m_table;
	int32 m_tableCount;

	int32 m_proxyCount;
};

inline void* b2DynamicTree::GetUserData(int32 proxyId) const
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);
	int32 index = m_proxyMap[proxyId];
	return m_proxies[index].userData;
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
