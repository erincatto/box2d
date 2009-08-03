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

#ifndef BROAD_PHASE_TEST_H
#define BROAD_PHASE_TEST_H

class BroadPhaseTest;

class Callback : public b2PairCallback
{
public:
	void* PairAdded(void* proxyUserData1, void* proxyUserData2);

	void PairRemoved(void* proxyUserData1, void* proxyUserData2, void* pairUserData);

	BroadPhaseTest* m_test;
};

class BroadPhaseTest : public Test
{
public:
	enum
	{
		e_actorCount = 256,
	};

	struct Actor
	{
		b2AABB aabb;
		int32 overlapCount;
		uint16 proxyId;
	};

	static Test* Create();

	BroadPhaseTest();
	~BroadPhaseTest();

	float GetExtent() { return 1.5f * m_extent; }
	void Step(Settings* settings);

	void Keyboard(unsigned char key);

private:

	friend class Callback;

	void GetRandomAABB(b2AABB* aabb);
	void MoveAABB(b2AABB* aabb);

	void CreateProxy();
	void DestroyProxy();
	void MoveProxy();
	void Action();
	void Validate();

	float32 m_extent;

	int32 m_overlapCount;
	int32 m_overlapCountExact;

	Callback m_callback;
	b2BroadPhase* m_broadPhase;
	Actor m_actors[e_actorCount];
	bool m_overlaps[e_actorCount][e_actorCount];
	bool m_automated;
	int32 m_stepCount;
};

#endif
