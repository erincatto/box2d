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

#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Common/b2BlockAllocator.h>


b2Fixture::b2Fixture()
{
	m_userData = NULL;
	m_body = NULL;
	m_next = NULL;
	m_proxyId = b2BroadPhase::e_nullProxy;
	m_shape = NULL;
}

b2Fixture::~b2Fixture()
{
	b2Assert(m_shape == NULL);
	b2Assert(m_proxyId == b2BroadPhase::e_nullProxy);
}

void b2Fixture::Create(b2BlockAllocator* allocator, b2BroadPhase* broadPhase, b2Body* body, const b2Transform& xf, const b2FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;
	m_density = def->density;

	m_body = body;
	m_next = NULL;

	m_filter = def->filter;

	m_isSensor = def->isSensor;

	m_shape = def->shape->Clone(allocator);

	// Create proxy in the broad-phase.
	m_shape->ComputeAABB(&m_aabb, xf);

	m_proxyId = broadPhase->CreateProxy(m_aabb, this);
}

void b2Fixture::Destroy(b2BlockAllocator* allocator, b2BroadPhase* broadPhase)
{
	// Remove proxy from the broad-phase.
	if (m_proxyId != b2BroadPhase::e_nullProxy)
	{
		broadPhase->DestroyProxy(m_proxyId);
		m_proxyId = b2BroadPhase::e_nullProxy;
	}

	// Free the child shape.
	switch (m_shape->m_type)
	{
	case b2Shape::e_circle:
		{
			b2CircleShape* s = (b2CircleShape*)m_shape;
			s->~b2CircleShape();
			allocator->Free(s, sizeof(b2CircleShape));
		}
		break;

	case b2Shape::e_polygon:
		{
			b2PolygonShape* s = (b2PolygonShape*)m_shape;
			s->~b2PolygonShape();
			allocator->Free(s, sizeof(b2PolygonShape));
		}
		break;

	default:
		b2Assert(false);
		break;
	}

	m_shape = NULL;
}

void b2Fixture::Synchronize(b2BroadPhase* broadPhase, const b2Transform& transform1, const b2Transform& transform2)
{
	if (m_proxyId == b2BroadPhase::e_nullProxy)
	{	
		return;
	}

	// Compute an AABB that covers the swept shape (may miss some rotation effect).
	b2AABB aabb1, aabb2;
	m_shape->ComputeAABB(&aabb1, transform1);
	m_shape->ComputeAABB(&aabb2, transform2);
	
	m_aabb.Combine(aabb1, aabb2);
	broadPhase->MoveProxy(m_proxyId, m_aabb);
}

void b2Fixture::SetFilterData(const b2Filter& filter)
{
	m_filter = filter;

	if (m_body == NULL)
	{
		return;
	}

	// Flag associated contacts for filtering.
	b2ContactEdge* edge = m_body->GetConactList();
	while (edge)
	{
		b2Contact* contact = edge->contact;
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA == this || fixtureB == this)
		{
			contact->FlagForFiltering();
		}
	}
}

void b2Fixture::SetSensor(bool sensor)
{
	if (m_isSensor == sensor)
	{
		return;
	}

	m_isSensor = sensor;

	if (m_body == NULL)
	{
		return;
	}

	// Flag associated contacts for filtering.
	b2ContactEdge* edge = m_body->GetConactList();
	while (edge)
	{
		b2Contact* contact = edge->contact;
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA == this || fixtureB == this)
		{
			contact->SetAsSensor(m_isSensor);
		}
	}
}

