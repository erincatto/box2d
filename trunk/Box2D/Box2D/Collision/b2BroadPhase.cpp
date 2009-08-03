/*
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

#include <Box2D/Collision/b2BroadPhase.h>
#include <cstring>

b2BroadPhase::b2BroadPhase()
{
	// Build initial proxy pool and free list.
	m_proxyCapacity = 16;
	m_proxyCount = 0;
	m_proxyPool = (b2Proxy*)b2Alloc(m_proxyCapacity * sizeof(b2Proxy));
	
	m_freeProxy = 0;
	for (int32 i = 0; i < m_proxyCapacity - 1; ++i)
	{
		m_proxyPool[i].next = i + 1;
	}
	m_proxyPool[m_proxyCapacity-1].next = e_nullProxy;

	m_pairCapacity = 16;
	m_pairCount = 0;
	m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));

	m_moveCapacity = 16;
	m_moveCount = 0;
	m_moveBuffer = (int32*)b2Alloc(m_moveCapacity * sizeof(int32));
}

b2BroadPhase::~b2BroadPhase()
{
	b2Free(m_moveBuffer);
	b2Free(m_proxyPool);
	b2Free(m_pairBuffer);
}

int32 b2BroadPhase::AllocateProxy()
{
	if (m_freeProxy == e_nullProxy)
	{
		b2Assert(m_proxyCount == m_proxyCapacity);
		b2Proxy* oldPool = m_proxyPool;

		m_proxyCapacity *= 2;
		m_proxyPool = (b2Proxy*)b2Alloc(m_proxyCapacity * sizeof(b2Proxy));

		memcpy(m_proxyPool, oldPool, m_proxyCount * sizeof(b2Proxy));
		b2Free(oldPool);

		m_freeProxy = m_proxyCount;
		for (int32 i = m_proxyCount; i < m_proxyCapacity - 1; ++i)
		{
			m_proxyPool[i].next = i + 1;
		}
		m_proxyPool[m_proxyCapacity-1].next = e_nullProxy;
	}

	int32 proxyId = m_freeProxy;
	m_freeProxy = m_proxyPool[proxyId].next;
	m_proxyPool[proxyId].next = e_nullProxy;
	++m_proxyCount;
	return proxyId;
}

void b2BroadPhase::FreeProxy(int32 proxyId)
{
	b2Assert(0 < m_proxyCount);
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);
	m_proxyPool[proxyId].next = m_freeProxy;
	m_freeProxy = proxyId;
	--m_proxyCount;
}

int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
{
	int32 proxyId = AllocateProxy();
	b2Proxy* proxy = m_proxyPool + proxyId;
	proxy->aabb = aabb;
	proxy->treeProxyId = m_tree.CreateProxy(aabb, proxyId);
	proxy->userData = userData;
	BufferMove(proxyId);
	return proxyId;
}

void b2BroadPhase::DestroyProxy(int32 proxyId)
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);
	b2Proxy* proxy = m_proxyPool + proxyId;
	UnBufferMove(proxyId);
	m_tree.DestroyProxy(proxy->treeProxyId);
	FreeProxy(proxyId);
}

void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb)
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);
	b2Proxy* proxy = m_proxyPool + proxyId;
	proxy->aabb = aabb;
	m_tree.MoveProxy(proxy->treeProxyId, aabb);
	BufferMove(proxyId);
}

void b2BroadPhase::BufferMove(int32 proxyId)
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);

	if (m_moveCount == m_moveCapacity)
	{
		int32* oldBuffer = m_moveBuffer;
		m_moveCapacity *= 2;
		m_moveBuffer = (int32*)b2Alloc(m_moveCapacity * sizeof(int32));
		memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(int32));
		b2Free(oldBuffer);
	}

	m_moveBuffer[m_moveCount] = proxyId;
	++m_moveCount;
}

void b2BroadPhase::UnBufferMove(int32 proxyId)
{
	for (int32 i = 0; i < m_moveCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = e_nullProxy;
			return;
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
void b2BroadPhase::QueryCallback(int32 proxyId)
{
	b2Assert(0 <= proxyId && proxyId < m_proxyCapacity);

	// A proxy cannot form a pair with itself.
	if (proxyId == m_queryProxyId)
	{
		return;
	}

	// Check the tight fitting AABBs for overlap.
	if (b2TestOverlap(m_proxyPool[proxyId].aabb, m_proxyPool[m_queryProxyId].aabb) == false)
	{
		return;
	}

	// Grow the pair buffer as needed.
	if (m_pairCount == m_pairCapacity)
	{
		b2Pair* oldBuffer = m_pairBuffer;
		m_pairCapacity *= 2;
		m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));
		memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(b2Pair));
		b2Free(oldBuffer);
	}

	m_pairBuffer[m_pairCount].proxyIdA = b2Min(proxyId, m_queryProxyId);
	m_pairBuffer[m_pairCount].proxyIdB = b2Max(proxyId, m_queryProxyId);
	++m_pairCount;
}
