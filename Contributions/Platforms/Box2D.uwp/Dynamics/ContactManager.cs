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
namespace Box2D.UWP
{
    internal class ContactManager
    {
        internal ContactManager()
        {
            _addPair = AddPair;

            _contactList = null;
            _contactCount = 0;
            ContactFilter = new DefaultContactFilter();
            ContactListener = new DefaultContactListener();
        }

	    // Broad-phase callback.
        internal void AddPair(Fixture proxyUserDataA, Fixture proxyUserDataB)
        {
            Fixture fixtureA = proxyUserDataA;
	        Fixture fixtureB = proxyUserDataB;

	        Body bodyA = fixtureA.GetBody();
	        Body bodyB = fixtureB.GetBody();

	        // Are the fixtures on the same body?
	        if (bodyA == bodyB)
	        {
		        return;
	        }

	        // Are both bodies static?
	        if (bodyA.IsStatic && bodyB.IsStatic)
	        {
		        return;
	        }

	        // Does a contact already exist?
	        ContactEdge edge = bodyB.GetConactList();
	        while (edge != null)
	        {
		        if (edge.Other == bodyA)
		        {
			        Fixture fA = edge.Contact.GetFixtureA();
			        Fixture fB = edge.Contact.GetFixtureB();
			        if (fA == fixtureA && fB == fixtureB)
			        {
				        // A contact already exists.
				        return;
			        }

			        if (fA == fixtureB && fB == fixtureA)
			        {
				        // A contact already exists.
				        return;
			        }
		        }

		        edge = edge.Next;
	        }

	        // Does a joint override collision?
	        if (bodyB.IsConnected(bodyA))
	        {
		        return;
	        }

	        // Check user filtering.
	        if (ContactFilter.ShouldCollide(fixtureA, fixtureB) == false)
	        {
		        return;
	        }

	        // Call the factory.
	        Contact c = Contact.Create(fixtureA, fixtureB);

	        // Contact creation may swap fixtures.
	        fixtureA = c.GetFixtureA();
	        fixtureB = c.GetFixtureB();
	        bodyA = fixtureA.GetBody();
	        bodyB = fixtureB.GetBody();

	        // Insert into the world.
	        c._prev = null;
	        c._next = _contactList;
	        if (_contactList != null)
	        {
		        _contactList._prev = c;
	        }
	        _contactList = c;

	        // Connect to island graph.

	        // Connect to body A
	        c._nodeA.Contact = c;
	        c._nodeA.Other = bodyB;

	        c._nodeA.Prev = null;
	        c._nodeA.Next = bodyA._contactList;
	        if (bodyA._contactList != null)
	        {
		        bodyA._contactList.Prev = c._nodeA;
	        }
	        bodyA._contactList = c._nodeA;

	        // Connect to body B
	        c._nodeB.Contact = c;
	        c._nodeB.Other = bodyA;

	        c._nodeB.Prev = null;
	        c._nodeB.Next = bodyB._contactList;
	        if (bodyB._contactList != null)
	        {
		        bodyB._contactList.Prev = c._nodeB;
	        }
	        bodyB._contactList = c._nodeB;

	        ++_contactCount;
        }

	    internal void FindNewContacts()
        {
            _broadPhase.UpdatePairs<Fixture>(_addPair);
        }

	    internal void Destroy(Contact c)
        {
            Fixture fixtureA = c.GetFixtureA();
	        Fixture fixtureB = c.GetFixtureB();
	        Body bodyA = fixtureA.GetBody();
	        Body bodyB = fixtureB.GetBody();

	        if (c._manifold._pointCount > 0)
	        {
		        ContactListener.EndContact(c);
	        }

	        // Remove from the world.
	        if (c._prev != null)
	        {
		        c._prev._next = c._next;
	        }

            if (c._next != null)
	        {
		        c._next._prev = c._prev;
	        }

	        if (c == _contactList)
	        {
		        _contactList = c._next;
	        }

	        // Remove from body 1
            if (c._nodeA.Prev != null)
	        {
		        c._nodeA.Prev.Next = c._nodeA.Next;
	        }

            if (c._nodeA.Next != null)
	        {
		        c._nodeA.Next.Prev = c._nodeA.Prev;
	        }

	        if (c._nodeA == bodyA._contactList)
	        {
		        bodyA._contactList = c._nodeA.Next;
	        }

	        // Remove from body 2
            if (c._nodeB.Prev != null)
	        {
		        c._nodeB.Prev.Next = c._nodeB.Next;
	        }

            if (c._nodeB.Next != null)
	        {
		        c._nodeB.Next.Prev = c._nodeB.Prev;
	        }

	        if (c._nodeB == bodyB._contactList)
	        {
		        bodyB._contactList = c._nodeB.Next;
	        }

	        --_contactCount;
        }

	    internal void Collide()
        {
            // Update awake contacts.
	        Contact c = _contactList;
	        while (c != null)
	        {
		        Fixture fixtureA = c.GetFixtureA();
		        Fixture fixtureB = c.GetFixtureB();
		        Body bodyA = fixtureA.GetBody();
		        Body bodyB = fixtureB.GetBody();

		        if (bodyA.IsSleeping && bodyB.IsSleeping)
		        {
			        c = c.GetNext();
			        continue;
		        }

		        // Is this contact flagged for filtering?
                if ((c._flags & ContactFlags.Filter) == ContactFlags.Filter)
		        {
			        // Are both bodies static?
			        if (bodyA.IsStatic && bodyB.IsStatic)
			        {
				        Contact cNuke = c;
				        c = cNuke.GetNext();
				        Destroy(cNuke);
				        continue;
			        }

			        // Does a joint override collision?
			        if (bodyB.IsConnected(bodyA))
			        {
				        Contact cNuke = c;
				        c = cNuke.GetNext();
				        Destroy(cNuke);
				        continue;
			        }

			        // Check user filtering.
			        if (ContactFilter.ShouldCollide(fixtureA, fixtureB) == false)
			        {
				        Contact cNuke = c;
				        c = cNuke.GetNext();
				        Destroy(cNuke);
				        continue;
			        }

			        // Clear the filtering flag.
			        c._flags &= ~ContactFlags.Filter;
		        }

		        int proxyIdA = fixtureA._proxyId;
		        int proxyIdB = fixtureB._proxyId;
                AABB aabbA;
                _broadPhase.GetAABB(proxyIdA, out aabbA);
                AABB aabbB;
                _broadPhase.GetAABB(proxyIdB, out aabbB);

		        // Here we cull out contacts that cease to overlap.
                if (AABB.TestOverlap(ref aabbA, ref aabbB) == false)
		        {
			        Contact cNuke = c;
			        c = cNuke.GetNext();
			        Destroy(cNuke);
			        continue;
		        }

		        // The contact persists.
		        c.Update(ContactListener);
		        c = c.GetNext();
	        }
        }
                
	    internal BroadPhase _broadPhase = new BroadPhase();
	    internal Contact _contactList;
	    internal int _contactCount;

        internal IContactFilter ContactFilter { get; set; }
        internal IContactListener ContactListener { get; set; }

        Action<Fixture, Fixture> _addPair;
    }
}
