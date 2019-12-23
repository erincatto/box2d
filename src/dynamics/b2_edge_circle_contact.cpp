/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include "b2_edge_circle_contact.h"

#include "box2d/b2_block_allocator.h"
#include "box2d/b2_fixture.h"

#include <new>

b2Contact* b2EdgeAndCircleContact::Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b2EdgeAndCircleContact));
	return new (mem) b2EdgeAndCircleContact(fixtureA, fixtureB);
}

void b2EdgeAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	((b2EdgeAndCircleContact*)contact)->~b2EdgeAndCircleContact();
	allocator->Free(contact, sizeof(b2EdgeAndCircleContact));
}

b2EdgeAndCircleContact::b2EdgeAndCircleContact(b2Fixture* fixtureA, b2Fixture* fixtureB)
: b2Contact(fixtureA, 0, fixtureB, 0)
{
	b2Assert(m_fixtureA->GetType() == b2Shape::e_edge);
	b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
}

void b2EdgeAndCircleContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
{
	b2CollideEdgeAndCircle(	manifold,
								(b2EdgeShape*)m_fixtureA->GetShape(), xfA,
								(b2CircleShape*)m_fixtureB->GetShape(), xfB);
}
