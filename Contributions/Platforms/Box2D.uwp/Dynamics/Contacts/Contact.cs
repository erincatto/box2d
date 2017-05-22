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
using System.Diagnostics;
namespace Box2D.UWP
{
    /// A contact edge is used to connect bodies and contacts together
    /// in a contact graph where each body is a node and each contact
    /// is an edge. A contact edge belongs to a doubly linked list
    /// maintained in each attached body. Each contact has two contact
    /// nodes, one for each attached body.
    public class ContactEdge
    {
        public Body Other;			///< provides quick access to the other body attached.
        public Contact Contact;		///< the contact
        public ContactEdge Prev;	///< the previous contact edge in the body's contact list
        public ContactEdge Next;	///< the next contact edge in the body's contact list
    };

    [Flags]
    public enum ContactFlags
    {
        None = 0,

	    // This contact should not participate in Solve
	    // The contact equivalent of sensors
	    NonSolid	= 0x0001,
	    // Do not use TOI solve.
	    Slow		= 0x0002,
	    // Used when crawling contact graph when forming islands.
	    Island	= 0x0004,
	    // Used in SolveTOI to indicate the cached toi value is still valid.
	    Toi		= 0x0008,
        // TODO: Doc
	    Touch		= 0x0010,
	    // This contact needs filtering because a fixture filter was changed.
	    Filter	= 0x0020,
    };

    /// The class manages contact between two shapes. A contact exists for each overlapping
    /// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
    /// that has no contact points.
    public abstract class Contact
    {
	    /// Get the contact manifold.
	    public void GetManifold(out Manifold manifold)
        {
            manifold = _manifold;
        }

	    /// Get the world manifold.
	    public void GetWorldManifold(out WorldManifold worldManifold)
        {
            Body bodyA = _fixtureA.GetBody();
	        Body bodyB = _fixtureB.GetBody();
	        Shape shapeA = _fixtureA.GetShape();
	        Shape shapeB = _fixtureB.GetShape();

            XForm xfA, xfB;
            bodyA.GetXForm(out xfA);
            bodyA.GetXForm(out xfB);

            worldManifold = new WorldManifold(ref _manifold, ref xfA, shapeA._radius, ref xfB, shapeB._radius);
        }

	    /// Is this contact solid?
	    /// @return true if this contact should generate a response.
	    public bool IsSolid()
        {
            return (_flags & ContactFlags.NonSolid) == ContactFlags.None;
        }

        /// Change the solidity of this contact. Used for sensors.
	    public void SetSolid(bool solid)
        {
            if (solid)
	        {
		        _flags &= ~ContactFlags.NonSolid;
	        }
	        else
	        {
		        _flags |= ContactFlags.NonSolid;
	        }
        }

	    /// Are fixtures touching?
	    public bool AreTouching()
        {
            return (_flags & ContactFlags.Touch) == ContactFlags.Touch;
        }

	    /// Get the next contact in the world's contact list.
	    public Contact GetNext()
        {
            return _next;
        }

	    /// Get the first fixture in this contact.
	    public Fixture GetFixtureA()
        {
            return _fixtureA;
        }

	    /// Get the second fixture in this contact.
	    public Fixture GetFixtureB()
        {
            return _fixtureB;
        }

	    /// Flag this contact for filtering. Filtering will occur the next time step.
	    public void FlagForFiltering()
        {
            _flags |= ContactFlags.Filter;
        }

	    internal Contact()
        {
            _fixtureA = null;
            _fixtureB = null;
        }

        internal Contact(Fixture fA, Fixture fB)
        {
            _flags = 0;

	        if (fA.IsSensor() || fB.IsSensor())
	        {
		        _flags |= ContactFlags.NonSolid;
	        }

	        _fixtureA = fA;
	        _fixtureB = fB;

	        _manifold._pointCount = 0;

	        _prev = null;
	        _next = null;

	        _nodeA.Contact = null;
	        _nodeA.Prev = null;
	        _nodeA.Next = null;
	        _nodeA.Other = null;

	        _nodeB.Contact = null;
	        _nodeB.Prev = null;
	        _nodeB.Next = null;
	        _nodeB.Other = null;
        }

	    internal void Update(IContactListener listener)
        {
            Manifold oldManifold = _manifold;

	        Evaluate();

	        Body bodyA = _fixtureA.GetBody();
	        Body bodyB = _fixtureB.GetBody();

	        int oldCount = oldManifold._pointCount;
	        int newCount = _manifold._pointCount;

	        if (newCount == 0 && oldCount > 0)
	        {
		        bodyA.WakeUp();
		        bodyB.WakeUp();
	        }

	        // Slow contacts don't generate TOI events.
	        if (bodyA.IsStatic || bodyA.IsBullet || bodyB.IsStatic || bodyB.IsBullet)
	        {
		        _flags &= ~ContactFlags.Slow;
	        }
	        else
	        {
		        _flags |= ContactFlags.Slow;
	        }

	        // Match old contact ids to new contact ids and copy the
	        // stored impulses to warm start the solver.
	        for (int i = 0; i < _manifold._pointCount; ++i)
	        {
		        ManifoldPoint mp2 = _manifold._points[i];
		        mp2.NormalImpulse = 0.0f;
		        mp2.TangentImpulse = 0.0f;
		        ContactID id2 = mp2.Id;

		        for (int j = 0; j < oldManifold._pointCount; ++j)
		        {
			        ManifoldPoint mp1 = oldManifold._points[j];

			        if (mp1.Id.Key == id2.Key)
			        {
				        mp2.NormalImpulse = mp1.NormalImpulse;
				        mp2.TangentImpulse = mp1.TangentImpulse;
				        break;
			        }
		        }

                _manifold._points[i] = mp2;
	        }

	        if (oldCount == 0 && newCount > 0)
	        {
		        _flags |= ContactFlags.Touch;
		        listener.BeginContact(this);
	        }

	        if (oldCount > 0 && newCount == 0)
	        {
		        _flags &= ~ContactFlags.Touch;
		        listener.EndContact(this);
	        }

	        if ((_flags & ContactFlags.NonSolid) == 0)
	        {
		        listener.PreSolve(this, ref oldManifold);

		        // The user may have disabled contact.
		        if (_manifold._pointCount == 0)
		        {
			        _flags &= ~ContactFlags.Touch;
		        }
	        }
        }

	    internal abstract void Evaluate();

	    internal abstract float ComputeTOI(ref Sweep sweepA, ref Sweep sweepB);

	    internal static Func<Fixture, Fixture, Contact>[,]  s_registers = new Func<Fixture, Fixture, Contact>[,] 
        {
            { 
              (f1, f2) => { return new CircleContact(f1, f2); }, 
              (f1, f2) => { return new PolygonAndCircleContact(f1, f2); }
            },
            { 
              (f1, f2) => { return new PolygonAndCircleContact(f1, f2); }, 
              (f1, f2) => { return new PolygonContact(f1, f2); }
            },
        };

        internal static Contact Create(Fixture fixtureA, Fixture fixtureB)
        {
            ShapeType type1 = fixtureA.ShapeType;
	        ShapeType type2 = fixtureB.ShapeType;

	        Debug.Assert(ShapeType.Unknown < type1 && type1 < ShapeType.TypeCount);
            Debug.Assert(ShapeType.Unknown < type2 && type2 < ShapeType.TypeCount);
        	
	        if (type1 > type2)
	        {
                // primary
                return s_registers[(int)type1, (int)type2](fixtureA, fixtureB);
	        }
	        else
	        {
                return s_registers[(int)type1, (int)type2](fixtureB, fixtureA);
	        }
        }

	    internal ContactFlags _flags;

	    // World pool and list pointers.
	    internal Contact _prev;
	    internal Contact _next;

	    // Nodes for connecting bodies.
	    internal ContactEdge _nodeA = new ContactEdge();
        internal ContactEdge _nodeB = new ContactEdge();

	    internal Fixture _fixtureA;
	    internal Fixture _fixtureB;

	    internal Manifold _manifold;

	    internal float _toi;
    };
}
