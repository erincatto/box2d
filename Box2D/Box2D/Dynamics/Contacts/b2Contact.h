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

#ifndef B2_CONTACT_H
#define B2_CONTACT_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Fixture.h>

class b2Body;
class b2Contact;
class b2Fixture;
class b2World;
class b2BlockAllocator;
class b2StackAllocator;
class b2ContactListener;

typedef b2Contact* b2ContactCreateFcn(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator);
typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);

struct b2ContactRegister
{
	b2ContactCreateFcn* createFcn;
	b2ContactDestroyFcn* destroyFcn;
	bool primary;
};

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct b2ContactEdge
{
	b2Body* other;			///< provides quick access to the other body attached.
	b2Contact* contact;		///< the contact
	b2ContactEdge* prev;	///< the previous contact edge in the body's contact list
	b2ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class b2Contact
{
public:

	/// Get the contact manifold.
	b2Manifold* GetManifold();

	/// Get the world manifold.
	void GetWorldManifold(b2WorldManifold* worldManifold) const;

	/// Is this contact solid?
	/// @return true if this contact should generate a response.
	bool IsSolid() const;

    /// Change the solidity of this contact. Used for sensors.
	void SetSolid(bool solid);

	/// Is this contact invalid?
	/// Contacts created or modified during a step are invalid,
	/// and won't participate until the next step.
	bool IsInvalid() const;

	/// Has this contact been removed from the world
	/// And is about to be destroyed.
	bool IsDestroyed() const;

	/// Are fixtures touching?
	bool AreTouching() const;

	/// Get the next contact in the world's contact list.
	b2Contact* GetNext();

	/// Get the first fixture in this contact.
	b2Fixture* GetFixtureA();

	/// Get the second fixture in this contact.
	b2Fixture* GetFixtureB();

	/// Flag this contact for filtering. Filtering will occur the next time step.
	void FlagForFiltering();

	//--------------- Internals Below -------------------
protected:
	friend class b2ContactManager;
	friend class b2World;
	friend class b2ContactSolver;

	// m_flags
	enum
	{
		// This contact should not participate in Solve
		// The contact equivalent of sensors
		e_nonSolidFlag	= 0x0001,
		// Do not use TOI solve.
		e_slowFlag		= 0x0002,
		// Used when crawling contact graph when forming islands.
		e_islandFlag	= 0x0004,
		// Used in SolveTOI to indicate the cached toi value is still valid.
		e_toiFlag		= 0x0008,
        // TODO: Doc
		e_touchFlag		= 0x0010,
		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag	= 0x0020,
	};

	static void AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destroyFcn,
						b2Shape::Type typeA, b2Shape::Type typeB);
	static void InitializeRegisters();
	static b2Contact* Create(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2Shape::Type typeA, b2Shape::Type typeB, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

	b2Contact() : m_fixtureA(NULL), m_fixtureB(NULL) {}
	b2Contact(b2Fixture* fixtureA, b2Fixture* fixtureB);
	virtual ~b2Contact() {}

	void Update(b2ContactListener* listener);
	virtual void Evaluate() = 0;

	virtual float32 ComputeTOI(const b2Sweep& sweepA, const b2Sweep& sweepB) const = 0;

	static b2ContactRegister s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
	static bool s_initialized;

	uint32 m_flags;

	// World pool and list pointers.
	b2Contact* m_prev;
	b2Contact* m_next;

	// Nodes for connecting bodies.
	b2ContactEdge m_nodeA;
	b2ContactEdge m_nodeB;

	b2Fixture* m_fixtureA;
	b2Fixture* m_fixtureB;

	b2Manifold m_manifold;

	float32 m_toi;
};

inline b2Manifold* b2Contact::GetManifold()
{
	return &m_manifold;
}

inline void b2Contact::GetWorldManifold(b2WorldManifold* worldManifold) const
{
	const b2Body* bodyA = m_fixtureA->GetBody();
	const b2Body* bodyB = m_fixtureB->GetBody();
	const b2Shape* shapeA = m_fixtureA->GetShape();
	const b2Shape* shapeB = m_fixtureB->GetShape();

	worldManifold->Initialize(&m_manifold, bodyA->GetXForm(), shapeA->m_radius, bodyB->GetXForm(), shapeB->m_radius);
}

inline bool b2Contact::IsSolid() const
{
	return (m_flags & e_nonSolidFlag) == 0;
}

inline void b2Contact::SetSolid(bool solid)
{
	if (solid)
	{
		m_flags &= ~e_nonSolidFlag;
	}
	else
	{
		m_flags |= e_nonSolidFlag;
	}
}

inline bool b2Contact::AreTouching() const
{
	return (m_flags & e_touchFlag) == e_touchFlag;
}

inline b2Contact* b2Contact::GetNext()
{
	return m_next;
}

inline b2Fixture* b2Contact::GetFixtureA()
{
	return m_fixtureA;
}

inline b2Fixture* b2Contact::GetFixtureB()
{
	return m_fixtureB;
}

inline void b2Contact::FlagForFiltering()
{
	m_flags |= e_filterFlag;
}

#endif
