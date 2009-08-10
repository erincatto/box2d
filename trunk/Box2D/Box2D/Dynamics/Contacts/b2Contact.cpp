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

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Contacts/b2CircleContact.h>
#include <Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h>
#include <Box2D/Dynamics/Contacts/b2PolygonContact.h>
#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>

b2ContactRegister b2Contact::s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
bool b2Contact::s_initialized = false;

void b2Contact::InitializeRegisters()
{
	AddType(b2CircleContact::Create, b2CircleContact::Destroy, b2Shape::e_circle, b2Shape::e_circle);
	AddType(b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, b2Shape::e_polygon, b2Shape::e_circle);
	AddType(b2PolygonContact::Create, b2PolygonContact::Destroy, b2Shape::e_polygon, b2Shape::e_polygon);
}

void b2Contact::AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destoryFcn,
						b2Shape::Type type1, b2Shape::Type type2)
{
	b2Assert(b2Shape::e_unknown < type1 && type1 < b2Shape::e_typeCount);
	b2Assert(b2Shape::e_unknown < type2 && type2 < b2Shape::e_typeCount);
	
	s_registers[type1][type2].createFcn = createFcn;
	s_registers[type1][type2].destroyFcn = destoryFcn;
	s_registers[type1][type2].primary = true;

	if (type1 != type2)
	{
		s_registers[type2][type1].createFcn = createFcn;
		s_registers[type2][type1].destroyFcn = destoryFcn;
		s_registers[type2][type1].primary = false;
	}
}

b2Contact* b2Contact::Create(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator)
{
	if (s_initialized == false)
	{
		InitializeRegisters();
		s_initialized = true;
	}

	b2Shape::Type type1 = fixtureA->GetType();
	b2Shape::Type type2 = fixtureB->GetType();

	b2Assert(b2Shape::e_unknown < type1 && type1 < b2Shape::e_typeCount);
	b2Assert(b2Shape::e_unknown < type2 && type2 < b2Shape::e_typeCount);
	
	b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
	if (createFcn)
	{
		if (s_registers[type1][type2].primary)
		{
			return createFcn(fixtureA, fixtureB, allocator);
		}
		else
		{
			return createFcn(fixtureB, fixtureA, allocator);
		}
	}
	else
	{
		return NULL;
	}
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	b2Assert(s_initialized == true);

	if (contact->m_manifold.m_pointCount > 0)
	{
		contact->GetFixtureA()->GetBody()->WakeUp();
		contact->GetFixtureB()->GetBody()->WakeUp();
	}

	b2Shape::Type typeA = contact->GetFixtureA()->GetType();
	b2Shape::Type typeB = contact->GetFixtureB()->GetType();

	b2Assert(b2Shape::e_unknown < typeA && typeB < b2Shape::e_typeCount);
	b2Assert(b2Shape::e_unknown < typeA && typeB < b2Shape::e_typeCount);

	b2ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
	destroyFcn(contact, allocator);
}

b2Contact::b2Contact(b2Fixture* fA, b2Fixture* fB)
{
	m_flags = 0;

	if (fA->IsSensor() || fB->IsSensor())
	{
		m_flags |= e_sensorFlag;
	}

	b2Body* bodyA = fA->GetBody();
	b2Body* bodyB = fB->GetBody();

	if (bodyA->IsStatic() || bodyA->IsBullet() || bodyB->IsStatic() || bodyB->IsBullet())
	{
		m_flags |= e_continuousFlag;
	}
	else
	{
		m_flags &= ~e_continuousFlag;
	}

	m_fixtureA = fA;
	m_fixtureB = fB;

	m_manifold.m_pointCount = 0;

	m_prev = NULL;
	m_next = NULL;

	m_nodeA.contact = NULL;
	m_nodeA.prev = NULL;
	m_nodeA.next = NULL;
	m_nodeA.other = NULL;

	m_nodeB.contact = NULL;
	m_nodeB.prev = NULL;
	m_nodeB.next = NULL;
	m_nodeB.other = NULL;
}

void b2Contact::Update(b2ContactListener* listener)
{
	b2Manifold oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags &= ~e_disabledFlag;

	if (b2TestOverlap(m_fixtureA->m_aabb, m_fixtureB->m_aabb))
	{
		Evaluate();
	}
	else
	{
		m_manifold.m_pointCount = 0;
	}

	b2Body* bodyA = m_fixtureA->GetBody();
	b2Body* bodyB = m_fixtureB->GetBody();

	int32 oldCount = oldManifold.m_pointCount;
	int32 newCount = m_manifold.m_pointCount;

	if (newCount == 0 && oldCount > 0)
	{
		bodyA->WakeUp();
		bodyB->WakeUp();
	}

	// Slow contacts don't generate TOI events.
	if (bodyA->IsStatic() || bodyA->IsBullet() || bodyB->IsStatic() || bodyB->IsBullet())
	{
		m_flags |= e_continuousFlag;
	}
	else
	{
		m_flags &= ~e_continuousFlag;
	}

	// Match old contact ids to new contact ids and copy the
	// stored impulses to warm start the solver.
	for (int32 i = 0; i < m_manifold.m_pointCount; ++i)
	{
		b2ManifoldPoint* mp2 = m_manifold.m_points + i;
		mp2->m_normalImpulse = 0.0f;
		mp2->m_tangentImpulse = 0.0f;
		b2ContactID id2 = mp2->m_id;

		for (int32 j = 0; j < oldManifold.m_pointCount; ++j)
		{
			b2ManifoldPoint* mp1 = oldManifold.m_points + j;

			if (mp1->m_id.key == id2.key)
			{
				mp2->m_normalImpulse = mp1->m_normalImpulse;
				mp2->m_tangentImpulse = mp1->m_tangentImpulse;
				break;
			}
		}
	}

	if (newCount > 0)
	{
		m_flags |= e_touchingFlag;
	}
	else
	{
		m_flags &= ~e_touchingFlag;
	}

	if (oldCount == 0 && newCount > 0)
	{
		listener->BeginContact(this);
	}

	if (oldCount > 0 && newCount == 0)
	{
		listener->EndContact(this);
	}

	if ((m_flags & e_sensorFlag) == 0)
	{
		listener->PreSolve(this, &oldManifold);
	}
}

float32 b2Contact::ComputeTOI(const b2Sweep& sweepA, const b2Sweep& sweepB) const
{
	b2TOIInput input;
	input.proxyA.Set(m_fixtureA->GetShape());
	input.proxyB.Set(m_fixtureB->GetShape());
	input.sweepA = sweepA;
	input.sweepB = sweepB;
	input.tolerance = b2_linearSlop;

	return b2TimeOfImpact(&input);
}
