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

#include <Box2D/Box2D.h>
#include "../Framework/Test.h"
#include "BroadPhaseTest.h"

#include <cstdio>
#include <cstring>

const float32 k_width = 1.0f;

void* Callback::PairAdded(void* proxyUserData1, void* proxyUserData2)
{
	BroadPhaseTest::Actor* actor1 = (BroadPhaseTest::Actor*)proxyUserData1;
	BroadPhaseTest::Actor* actor2 = (BroadPhaseTest::Actor*)proxyUserData2;

	int32 id1 = (int32)(actor1 - m_test->m_actors);
	int32 id2 = (int32)(actor2 - m_test->m_actors);
	b2Assert(id1 < BroadPhaseTest::e_actorCount);
	b2Assert(id2 < BroadPhaseTest::e_actorCount);

	b2Assert(m_test->m_overlaps[id1][id2] == false);
	m_test->m_overlaps[id1][id2] = true;
	m_test->m_overlaps[id2][id1] = true;
	++m_test->m_overlapCount;

	++actor1->overlapCount;
	++actor2->overlapCount;

	return NULL;
}

void Callback::PairRemoved(void* proxyUserData1, void* proxyUserData2, void* pairUserData)
{
	B2_NOT_USED(pairUserData);

	BroadPhaseTest::Actor* actor1 = (BroadPhaseTest::Actor*)proxyUserData1;
	BroadPhaseTest::Actor* actor2 = (BroadPhaseTest::Actor*)proxyUserData2;

	// The pair may have been removed by destroying a proxy.
	int32 id1 = (int32)(actor1 - m_test->m_actors);
	int32 id2 = (int32)(actor2 - m_test->m_actors);
	b2Assert(id1 < BroadPhaseTest::e_actorCount);
	b2Assert(id2 < BroadPhaseTest::e_actorCount);

	m_test->m_overlaps[id1][id2] = false;
	m_test->m_overlaps[id2][id1] = false;
	--m_test->m_overlapCount;

	--actor1->overlapCount;
	--actor2->overlapCount;
}

Test* BroadPhaseTest::Create()
{
	return new BroadPhaseTest;
}

BroadPhaseTest::BroadPhaseTest()
{
	m_extent = 15.0f;

	b2BroadPhase::s_validate = true;

	srand(888);

	b2AABB worldAABB;
	worldAABB.lowerBound.Set(-5.0f * m_extent, -5.0f * m_extent);
	worldAABB.upperBound.Set(5.0f * m_extent, 5.0f * m_extent);

	m_overlapCount = 0;
	m_overlapCountExact = 0;
	m_callback.m_test = this;

	m_broadPhase = new b2BroadPhase(worldAABB, &m_callback);

	memset(m_overlaps, 0, sizeof(m_overlaps));

	for (int32 i = 0; i < e_actorCount; ++i)
	{
		Actor* actor = m_actors + i;
		GetRandomAABB(&actor->aabb);
		//actor->aabb.minVertex.Set(0.0f, 0.0f);
		//actor->aabb.maxVertex.Set(k_width, k_width);
		actor->proxyId = m_broadPhase->CreateProxy(actor->aabb, actor);
		actor->overlapCount = 0;
		m_broadPhase->Validate();
	}

	m_automated = true;
	m_stepCount = 0;
}

BroadPhaseTest::~BroadPhaseTest()
{
	b2BroadPhase::s_validate = false;

	delete m_broadPhase;
}

void BroadPhaseTest::GetRandomAABB(b2AABB* aabb)
{
	b2Vec2 w; w.Set(k_width, k_width);
	aabb->lowerBound.x = RandomFloat(-m_extent, m_extent);
	aabb->lowerBound.y = RandomFloat(0.0f, 2.0f * m_extent);
	aabb->upperBound = aabb->lowerBound + w;
}

void BroadPhaseTest::MoveAABB(b2AABB* aabb)
{
	b2Vec2 d;
	d.x = RandomFloat(-0.5f, 0.5f);
	d.y = RandomFloat(-0.5f, 0.5f);
	//d.x = 2.0f;
	//d.y = 0.0f;
	aabb->lowerBound += d;
	aabb->upperBound += d;

	b2Vec2 c0 = 0.5f * (aabb->lowerBound + aabb->upperBound);
	b2Vec2 min; min.Set(-m_extent, 0.0f);
	b2Vec2 max; max.Set(m_extent, 2.0f * m_extent);
	b2Vec2 c = b2Clamp(c0, min, max);

	aabb->lowerBound += c - c0;
	aabb->upperBound += c - c0;
}

void BroadPhaseTest::CreateProxy()
{
	for (int32 i = 0; i < e_actorCount; ++i)
	{
		int32 j = rand() % e_actorCount;
		Actor* actor = m_actors + j;
		if (actor->proxyId == b2_nullProxy)
		{
			actor->overlapCount = 0;
			GetRandomAABB(&actor->aabb);
			actor->proxyId = m_broadPhase->CreateProxy(actor->aabb, actor);
			return;
		}
	}
}

void BroadPhaseTest::DestroyProxy()
{
	for (int32 i = 0; i < e_actorCount; ++i)
	{
		int32 j = rand() % e_actorCount;
		Actor* actor = m_actors + j;
		if (actor->proxyId != b2_nullProxy)
		{
			m_broadPhase->DestroyProxy(actor->proxyId);
			actor->proxyId = b2_nullProxy;
			actor->overlapCount = 0;
			return;
		}
	}
}

void BroadPhaseTest::MoveProxy()
{
	for (int32 i = 0; i < e_actorCount; ++i)
	{
		int32 j = rand() % e_actorCount;
		//int32 j = 1;
		Actor* actor = m_actors + j;
		if (actor->proxyId == b2_nullProxy)
		{
			continue;
		}

		MoveAABB(&actor->aabb);
		m_broadPhase->MoveProxy(actor->proxyId, actor->aabb);
		return;
	}
}

void BroadPhaseTest::Action()
{
	int32 choice = rand() % 20;

	switch (choice)
	{
	case 0:
		CreateProxy();
		break;

	case 1:
		DestroyProxy();
		break;

	default:
		MoveProxy();
	}
}

void BroadPhaseTest::Step(Settings* settings)
{
	B2_NOT_USED(settings);

	if (m_automated == true)
	{
		int32 actionCount = b2Max(1, e_actorCount >> 2);

		for (int32 i = 0; i < actionCount; ++i)
		{
			Action();
		}
	}

	m_broadPhase->Commit();

	for (int32 i = 0; i < e_actorCount; ++i)
	{
		Actor* actor = m_actors + i;
		if (actor->proxyId == b2_nullProxy)
			continue;

		b2Color c;
		switch (actor->overlapCount)
		{
		case 0:
			c.r = 0.9f; c.g = 0.9f; c.b = 0.9f;
			break;

		case 1:
			c.r = 0.6f; c.g = 0.9f; c.b = 0.6f;
			break;

		default:
			c.r = 0.9f; c.g = 0.6f; c.b = 0.6f;
			break;
		}

		m_debugDraw.DrawAABB(&actor->aabb, c);
	}

	char buffer[64];
	sprintf(buffer, "overlaps = %d, exact = %d, diff = %d", m_overlapCount, m_overlapCountExact, m_overlapCount - m_overlapCountExact);
	m_debugDraw.DrawString(5, 30, buffer);
	Validate();

	++m_stepCount;
}

void BroadPhaseTest::Keyboard(unsigned char key)
{
	switch (key)
	{
	case 'a':
		m_automated = !m_automated;
		break;

	case 'c':
		CreateProxy();
		break;

	case 'd':
		DestroyProxy();
		break;

	case 'm':
		MoveProxy();
		break;
	}
}

void BroadPhaseTest::Validate()
{
	m_broadPhase->Validate();

	m_overlapCountExact = 0;

	for (int32 i = 0; i < e_actorCount; ++i)
	{
		Actor* actor1 = m_actors + i;
		if (actor1->proxyId == b2_nullProxy)
			continue;

		for (int32 j = i + 1; j < e_actorCount; ++j)
		{
			Actor* actor2 = m_actors + j;
			if (actor2->proxyId == b2_nullProxy)
				continue;

			bool overlap = b2TestOverlap(actor1->aabb, actor2->aabb);
			if (overlap) ++m_overlapCountExact;

			if (overlap)
			{
				b2Assert(m_overlaps[actor1-m_actors][actor2-m_actors] == true);
			}
		}
	}

	b2Assert(m_overlapCount >= m_overlapCountExact);
}
