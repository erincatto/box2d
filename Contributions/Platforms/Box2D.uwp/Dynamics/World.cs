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

using Box2D.uwp.UWPExtensions;
using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Numerics;
using Windows.UI;

namespace Box2D.UWP
{
    [Flags]
    public enum WorldFlags
    {
        NewFixture = (1 << 0),
        Locked = (1 << 1),
    };

    /// The world class manages all physics entities, dynamic simulation,
    /// and asynchronous queries. The world also contains efficient memory
    /// management facilities.
    public class World
    {
        /// ruct a world object.
        /// @param gravity the world gravity vector.
        /// @param doSleep improve performance by not simulating inactive bodies.
        public World(Vector2 gravity, bool doSleep)
        {
            WarmStarting = true;
            ContinuousPhysics = true;

            _allowSleep = doSleep;
            Gravity = gravity;
        }

	    /// Register a destruction listener.
	    public IDestructionListener DestructionListener { get; set; }

	    /// Register a contact filter to provide specific control over collision.
	    /// Otherwise the default filter is used (Settings.b2_defaultFilter).
	    public IContactFilter ContactFilter
        {
            get
            {
                return _contactManager.ContactFilter;
            }
            set
            {
                _contactManager.ContactFilter = value;
            }
        }

	    /// Register a contact event listener
	    public IContactListener ContactListener
        {
            get
            {
                return _contactManager.ContactListener;
            }
            set
            {
                _contactManager.ContactListener = value;
            }
        }

	    /// Register a routine for debug drawing. The debug draw functions are called
	    /// inside the World.Step method, so make sure your renderer is ready to
	    /// consume draw commands when you call Step().
	    public DebugDraw DebugDraw { get; set; }

	    /// Create a rigid body given a definition. No reference to the definition
	    /// is retained.
	    /// @warning This function is locked during callbacks.
	    public Body CreateBody(BodyDef def)
        {
            Debug.Assert(!IsLocked);
	        if (IsLocked)
	        {
		        return null;
	        }

	        var b = new Body(def, this);

	        // Add to world doubly linked list.
	        b._prev = null;
	        b._next = _bodyList;
	        if (_bodyList != null)
	        {
		        _bodyList._prev = b;
	        }
	        _bodyList = b;
	        ++_bodyCount;

	        return b;
        }

	    /// Destroy a rigid body given a definition. No reference to the definition
	    /// is retained. This function is locked during callbacks.
	    /// @warning This automatically deletes all associated shapes and joints.
	    /// @warning This function is locked during callbacks.
	    public void DestroyBody(Body b)
        {
            Debug.Assert(_bodyCount > 0);
	        Debug.Assert(!IsLocked);
	        if (IsLocked)
	        {
		        return;
	        }

	        // Delete the attached joints.
	        JointEdge je = b._jointList;
	        while (je != null)
	        {
		        JointEdge je0 = je;
		        je = je.Next;

		        if (DestructionListener != null)
		        {
			        DestructionListener.SayGoodbye(je0.Joint);
		        }

		        DestroyJoint(je0.Joint);
	        }
	        b._jointList = null;

	        // Delete the attached contacts.
	        ContactEdge ce = b._contactList;
	        while (ce != null)
	        {
		        ContactEdge ce0 = ce;
		        ce = ce.Next;
		        _contactManager.Destroy(ce0.Contact);
	        }
	        b._contactList = null;

	        // Delete the attached fixtures. This destroys broad-phase proxies.
	        Fixture f = b._fixtureList;
	        while (f != null)
	        {
		        Fixture f0 = f;
		        f = f._next;

		        if (DestructionListener != null)
		        {
			        DestructionListener.SayGoodbye(f0);
		        }

		        f0.Destroy(_contactManager._broadPhase);
	        }
	        b._fixtureList = null;
	        b._fixtureCount = 0;

	        // Remove world body list.
	        if (b._prev != null)
	        {
		        b._prev._next = b._next;
	        }

            if (b._next != null)
	        {
		        b._next._prev = b._prev;
	        }

	        if (b == _bodyList)
	        {
		        _bodyList = b._next;
	        }

	        --_bodyCount;
        }

	    /// Create a joint to rain bodies together. No reference to the definition
	    /// is retained. This may cause the connected bodies to cease colliding.
	    /// @warning This function is locked during callbacks.
	    public Joint CreateJoint(JointDef def)
        {
	        Debug.Assert(!IsLocked);
	        if (IsLocked)
	        {
		        return null;
	        }

	        Joint j = Joint.Create(def);

	        // Connect to the world list.
	        j._prev = null;
	        j._next = _jointList;
	        if (_jointList != null)
	        {
		        _jointList._prev = j;
	        }
	        _jointList = j;
	        ++_jointCount;

	        // Connect to the bodies' doubly linked lists.
	        j._edgeA.Joint = j;
	        j._edgeA.Other = j._bodyB;
	        j._edgeA.Prev = null;
	        j._edgeA.Next = j._bodyA._jointList;

	        if (j._bodyA._jointList != null) 
                j._bodyA._jointList.Prev = j._edgeA;

	        j._bodyA._jointList = j._edgeA;

	        j._edgeB.Joint = j;
	        j._edgeB.Other = j._bodyA;
	        j._edgeB.Prev = null;
	        j._edgeB.Next = j._bodyB._jointList;

	        if (j._bodyB._jointList != null) 
                j._bodyB._jointList.Prev = j._edgeB;

	        j._bodyB._jointList = j._edgeB;

	        Body bodyA = def.body1;
	        Body bodyB = def.body2;

	        bool staticA = bodyA.IsStatic;
	        bool staticB = bodyB.IsStatic;

	        // If the joint prevents collisions, then flag any contacts for filtering.
	        if (def.collideConnected == false && (staticA == false || staticB == false))
	        {
		        // Ensure we iterate over contacts on a dynamic body (usually have less contacts
		        // than a static body). Ideally we will have a contact count on both bodies.
		        if (staticB)
		        {
			        MathUtils.Swap(ref bodyA, ref bodyB);
		        }

		        ContactEdge edge = bodyB.GetConactList();
		        while (edge != null)
		        {
			        if (edge.Other == bodyA)
			        {
				        // Flag the contact for filtering at the next time step (where either
				        // body is awake).
				        edge.Contact.FlagForFiltering();
			        }

			        edge = edge.Next;
		        }
	        }

	        // Note: creating a joint doesn't wake the bodies.

	        return j;
        }

	    /// Destroy a joint. This may cause the connected bodies to begin colliding.
	    /// @warning This function is locked during callbacks.
	    public void DestroyJoint(Joint j)
        {
	        Debug.Assert(!IsLocked);
	        if (IsLocked)
	        {
		        return;
	        }

	        bool collideConnected = j._collideConnected;

	        // Remove from the doubly linked list.
	        if (j._prev != null)
	        {
		        j._prev._next = j._next;
	        }

	        if (j._next != null)
	        {
		        j._next._prev = j._prev;
	        }

	        if (j == _jointList)
	        {
		        _jointList = j._next;
	        }

	        // Disconnect from island graph.
	        Body bodyA = j._bodyA;
	        Body bodyB = j._bodyB;

	        // Wake up connected bodies.
	        bodyA.WakeUp();
	        bodyB.WakeUp();

	        // Remove from body 1.
	        if (j._edgeA.Prev != null)
	        {
		        j._edgeA.Prev.Next = j._edgeA.Next;
	        }

	        if (j._edgeA.Next != null)
	        {
		        j._edgeA.Next.Prev = j._edgeA.Prev;
	        }

	        if (j._edgeA == bodyA._jointList)
	        {
		        bodyA._jointList = j._edgeA.Next;
	        }

	        j._edgeA.Prev = null;
	        j._edgeA.Next = null;

	        // Remove from body 2
	        if (j._edgeB.Prev != null)
	        {
		        j._edgeB.Prev.Next = j._edgeB.Next;
	        }

	        if (j._edgeB.Next != null)
	        {
		        j._edgeB.Next.Prev = j._edgeB.Prev;
	        }

	        if (j._edgeB == bodyB._jointList)
	        {
		        bodyB._jointList = j._edgeB.Next;
	        }

	        j._edgeB.Prev = null;
	        j._edgeB.Next = null;

	        Debug.Assert(_jointCount > 0);
	        --_jointCount;

	        // If the joint prevents collisions, then flag any contacts for filtering.
	        if (collideConnected == false)
	        {
		        ContactEdge edge = bodyB.GetConactList();
		        while (edge != null)
		        {
			        if (edge.Other == bodyA)
			        {
				        // Flag the contact for filtering at the next time step (where either
				        // body is awake).
				        edge.Contact.FlagForFiltering();
			        }

			        edge = edge.Next;
		        }
	        }
        }

	    /// Take a time step. This performs collision detection, integration,
	    /// and raint solution.
	    /// @param timeStep the amount of time to simulate, this should not vary.
	    /// @param velocityIterations for the velocity raint solver.
	    /// @param positionIterations for the position raint solver.
	    public void Step(float dt, int velocityIterations, int positionIterations)
        {
	        int height;
	        height = _contactManager._broadPhase.ComputeHeight();

	        // If new fixtures were added, we need to find the new contacts.
	        if ((_flags & WorldFlags.NewFixture) == WorldFlags.NewFixture)
	        {
		        _contactManager.FindNewContacts();
		        _flags &= ~WorldFlags.NewFixture;
	        }

	        _flags |= WorldFlags.Locked;

	        TimeStep step;
	        step.dt = dt;
	        step.velocityIterations	= velocityIterations;
	        step.positionIterations = positionIterations;
	        if (dt > 0.0f)
	        {
		        step.inv_dt = 1.0f / dt;
	        }
	        else
	        {
		        step.inv_dt = 0.0f;
	        }

	        step.dtRatio = _inv_dt0 * dt;

	        step.warmStarting = WarmStarting;

	        // Update contacts. This is where some contacts are destroyed.
	        _contactManager.Collide();

	        // Integrate velocities, solve velocity raints, and integrate positions.
	        if (step.dt > 0.0f)
	        {
		        Solve(ref step);
	        }

	        // Handle TOI events.
	        if (ContinuousPhysics && step.dt > 0.0f)
	        {
		        SolveTOI(ref step);
	        }

	        if (step.dt > 0.0f)
	        {
		        _inv_dt0 = step.inv_dt;
	        }

	        _flags &= ~WorldFlags.Locked;
        }

	    /// Call this to draw shapes and other debug draw data.
	    public void DrawDebugData()
        {
	        if (DebugDraw == null)
	        {
		        return;
	        }

	        DebugDrawFlags flags = DebugDraw.Flags;

	        if ((flags & DebugDrawFlags.Shape) == DebugDrawFlags.Shape)
	        {
		        for (Body b = _bodyList; b != null; b = b.GetNext())
		        {
                    XForm xf;
                    b.GetXForm(out xf);
			        for (Fixture f = b.GetFixtureList(); f != null; f = f.GetNext())
			        {
				        if (b.IsStatic)
				        {
					        DrawShape(f, xf, ColorEx.FromScRgb(0.5f, 0.9f, 0.5f));
				        }
				        else if (b.IsSleeping)
				        {
					        DrawShape(f, xf, ColorEx.FromScRgb(0.5f, 0.5f, 0.9f));
				        }
				        else
				        {
					        DrawShape(f, xf, ColorEx.FromScRgb(0.9f, 0.9f, 0.9f));
				        }
			        }
		        }
	        }

	        if ((flags & DebugDrawFlags.Joint) == DebugDrawFlags.Joint)
	        {
		        for (Joint j = _jointList; j != null; j = j.GetNext())
		        {
			        if (j.JointType != JointType.Mouse)
			        {
				        DrawJoint(j);
			        }
		        }
	        }

	        if ((flags & DebugDrawFlags.Pair) == DebugDrawFlags.Pair)
	        {
		        // TODO
	        }

	        if ((flags & DebugDrawFlags.AABB) == DebugDrawFlags.AABB)
	        {
		        Color color = ColorEx.FromScRgb(0.9f, 0.3f, 0.9f);
		        BroadPhase bp = _contactManager._broadPhase;

		        for (Body b = _bodyList; b != null; b = b.GetNext())
		        {
			        for (Fixture f = b.GetFixtureList(); f != null; f = f.GetNext())
			        {
                        AABB aabb;
                        bp.GetAABB(f._proxyId, out aabb);
				        FixedArray8<Vector2> vs = new FixedArray8<Vector2>();
				        vs[0] = new Vector2(aabb.lowerBound.X, aabb.lowerBound.Y);
				        vs[1] = new Vector2(aabb.upperBound.X, aabb.lowerBound.Y);
				        vs[2] = new Vector2(aabb.upperBound.X, aabb.upperBound.Y);
				        vs[3] = new Vector2(aabb.lowerBound.X, aabb.upperBound.Y);

				        DebugDraw.DrawPolygon(ref vs, 4, color);
			        }
		        }
	        }

	        if ((flags & DebugDrawFlags.CenterOfMass) == DebugDrawFlags.CenterOfMass)
	        {
		        for (Body b = _bodyList; b != null; b = b.GetNext())
		        {
                    XForm xf;
                    b.GetXForm(out xf);
			        xf.Position = b.GetWorldCenter();
			        DebugDraw.DrawXForm(ref xf);
		        }
	        }
        }

	    /// Query the world for all fixtures that potentially overlap the
	    /// provided AABB.
	    /// @param callback a user implemented callback class.
	    /// @param aabb the query box.
	    public void Query(Func<Fixture, bool> callback, ref AABB aabb)
        {
            _contactManager._broadPhase.Query(callback, ref aabb);
        }

	    /// Get the world body list. With the returned body, use Body.GetNext to get
	    /// the next body in the world list. A null body indicates the end of the list.
	    /// @return the head of the world body list.
	    public Body GetBodyList()
        {
            return _bodyList;
        }

	    /// Get the world joint list. With the returned joint, use Joint.GetNext to get
	    /// the next joint in the world list. A null joint indicates the end of the list.
	    /// @return the head of the world joint list.
	    public Joint GetJointList()
        {
            return _jointList;
        }

	    /// Get the world contact list. With the returned contact, use Contact.GetNext to get
	    /// the next contact in the world list. A null contact indicates the end of the list.
	    /// @return the head of the world contact list.
	    /// @warning contacts are 
	    public Contact GetContactList()
        {
            return _contactManager._contactList;
        }

	    /// Enable/disable warm starting. For testing.
	    public bool WarmStarting { get; set; }

	    /// Enable/disable continuous physics. For testing.
	    public bool ContinuousPhysics { get; set; }

	    /// Get the number of broad-phase proxies.
	    public int ProxyCount 
        {  
            get
            {
                return _contactManager._broadPhase.ProxyCount;
            }
        }

	    /// Get the number of bodies.
	    public int BodyCount
        {
            get
            {
                return _bodyCount;
            }
        }

	    /// Get the number of joints.
	    public int JointCount
        {
            get
            {
                return _jointCount;
            }
        }
	    /// Get the number of contacts (each may have 0 or more contact points).
	    public int ContactCount
        {
            get
            {
                return _contactManager._contactCount;
            }
        }

	    /// Change the global gravity vector.
	    public Vector2 Gravity { get; set; }

	    /// Is the world locked (in the middle of a time step).
	    public bool IsLocked 
        { 
            get 
            { 
                return (_flags & WorldFlags.Locked) == WorldFlags.Locked; 
            }
            set 
            { 
                if (value)
                {
                    _flags |= WorldFlags.Locked;
                }
                else
                {
                    _flags &= ~WorldFlags.Locked;
                }
            }
        }

	    void Solve(ref TimeStep step)
        {
	        // Size the island for the worst case.
            _island.Reset(_bodyCount,
	                      _contactManager._contactCount,
	                      _jointCount,
	                      _contactManager.ContactListener);

	        // Clear all the island flags.
	        for (Body b = _bodyList; b != null; b = b._next)
	        {
		        b._flags &= ~BodyFlags.Island;
	        }
	        for (Contact c = _contactManager._contactList; c != null; c = c._next)
	        {
		        c._flags &= ~ContactFlags.Island;
	        }
	        for (Joint j = _jointList; j != null; j = j._next)
	        {
		        j._islandFlag = false;
	        }

	        // Build and simulate all awake islands.
//#warning Remove extra allocs

            int stackSize = _bodyCount;
	        Body[] stack = new Body[_bodyCount];
	        for (Body seed = _bodyList; seed != null; seed = seed._next)
	        {
		        if ((seed._flags & (BodyFlags.Island | BodyFlags.Sleep | BodyFlags.Frozen)) != BodyFlags.None)
		        {
			        continue;
		        }

		        if (seed.IsStatic)
		        {
			        continue;
		        }

		        // Reset island and stack.
		        _island.Clear();
		        int stackCount = 0;
		        stack[stackCount++] = seed;
		        seed._flags |= BodyFlags.Island;

		        // Perform a depth first search (DFS) on the raint graph.
		        while (stackCount > 0)
		        {
			        // Grab the next body off the stack and add it to the island.
			        Body b = stack[--stackCount];
			        _island.Add(b);

			        // Make sure the body is awake.
			        b._flags &= ~BodyFlags.Sleep;

			        // To keep islands as small as possible, we don't
			        // propagate islands across static bodies.
			        if (b.IsStatic)
			        {
				        continue;
			        }

			        // Search all contacts connected to this body.
			        for (ContactEdge ce = b._contactList; ce != null; ce = ce.Next)
			        {
				        // Has this contact already been added to an island?
				        // Is this contact non-solid (involves a sensor).
				        if ((ce.Contact._flags & (ContactFlags.Island | ContactFlags.NonSolid)) != ContactFlags.None)
				        {
					        continue;
				        }

				        // Is this contact touching?
				        if ((ce.Contact._flags & ContactFlags.Touch) == ContactFlags.None)
				        {
					        continue;
				        }

                        _island.Add(ce.Contact);
				        ce.Contact._flags |= ContactFlags.Island;

				        Body other = ce.Other;

				        // Was the other body already added to this island?
				        if ((other._flags & BodyFlags.Island) != BodyFlags.None)
				        {
					        continue;
				        }

				        Debug.Assert(stackCount < stackSize);
				        stack[stackCount++] = other;
				        other._flags |= BodyFlags.Island;
			        }

			        // Search all joints connect to this body.
			        for (JointEdge je = b._jointList; je != null; je = je.Next)
			        {
				        if (je.Joint._islandFlag == true)
				        {
					        continue;
				        }

                        _island.Add(je.Joint);
				        je.Joint._islandFlag = true;

				        Body other = je.Other;
				        if ((other._flags & BodyFlags.Island) != BodyFlags.None)
				        {
					        continue;
				        }

				        Debug.Assert(stackCount < stackSize);
				        stack[stackCount++] = other;
				        other._flags |= BodyFlags.Island;
			        }
		        }

		        _island.Solve(ref step, Gravity, _allowSleep);

		        // Post solve cleanup.
		        for (int i = 0; i < _island._bodyCount; ++i)
		        {
			        // Allow static bodies to participate in other islands.
			        Body b = _island._bodies[i];
			        if (b.IsStatic)
			        {
				        b._flags &= ~BodyFlags.Island;
			        }
		        }
	        }

	        // Synchronize fixtures, check for out of range bodies.
	        for (Body b = _bodyList; b != null; b = b.GetNext())
	        {
		        if ((b._flags & (BodyFlags.Sleep | BodyFlags.Frozen)) != BodyFlags.None)
		        {
			        continue;
		        }

		        if (b.IsStatic)
		        {
			        continue;
		        }

		        // Update fixtures (for broad-phase).
		        b.SynchronizeFixtures();
	        }

	        // Look for new contacts.
	        _contactManager.FindNewContacts();
        }

	    void SolveTOI(ref TimeStep step)
        {
	        // Reserve an island and a queue for TOI island solution.
	        _island.Reset(  _bodyCount,
	                        Settings.b2_maxTOIContactsPerIsland,
                            Settings.b2_maxTOIJointsPerIsland,
	                        _contactManager.ContactListener);

	        //Simple one pass queue
	        //Relies on the fact that we're only making one pass
	        //through and each body can only be pushed/popped once.
	        //To push: 
	        //  queue[queueStart+queueSize++] = newElement;
	        //To pop: 
	        //	poppedElement = queue[queueStart++];
	        //  --queueSize;
//#warning More Body array Allocs
            int queueCapacity = _bodyCount;
            Body[] queue = new Body[_bodyCount];

	        for (Body b = _bodyList; b != null; b = b._next)
	        {
		        b._flags &= ~BodyFlags.Island;
		        b._sweep.t0 = 0.0f;
	        }

	        for (Contact c = _contactManager._contactList; c != null; c = c._next)
	        {
		        // Invalidate TOI
		        c._flags &= ~(ContactFlags.Toi | ContactFlags.Island);
	        }

	        for (Joint j = _jointList; j != null; j = j._next)
	        {
		        j._islandFlag = false;
	        }

	        // Find TOI events and solve them.
	        for (;;)
	        {
		        // Find the first TOI.
		        Contact minContact = null;
		        float minTOI = 1.0f;

		        for (Contact c = _contactManager._contactList; c != null; c = c._next)
		        {
			        if ((c._flags & (ContactFlags.Slow | ContactFlags.NonSolid)) != ContactFlags.None)
			        {
				        continue;
			        }

			        // TODO_ERIN keep a counter on the contact, only respond to M TOIs per contact.

			        float toi = 1.0f;
			        if ((c._flags & ContactFlags.Toi) != ContactFlags.None)
			        {
				        // This contact has a valid cached TOI.
				        toi = c._toi;
			        }
			        else
			        {
				        // Compute the TOI for this contact.
				        Fixture s1 = c.GetFixtureA();
				        Fixture s2 = c.GetFixtureB();
				        Body b1 = s1.GetBody();
				        Body b2 = s2.GetBody();

				        if ((b1.IsStatic || b1.IsSleeping) && (b2.IsStatic || b2.IsSleeping))
				        {
					        continue;
				        }

				        // Put the sweeps onto the same time interval.
				        float t0 = b1._sweep.t0;

				        if (b1._sweep.t0 < b2._sweep.t0)
				        {
					        t0 = b2._sweep.t0;
					        b1._sweep.Advance(t0);
				        }
				        else if (b2._sweep.t0 < b1._sweep.t0)
				        {
					        t0 = b1._sweep.t0;
					        b2._sweep.Advance(t0);
				        }

				        Debug.Assert(t0 < 1.0f);

				        // Compute the time of impact.
				        toi = c.ComputeTOI(ref b1._sweep, ref b2._sweep);
				        //CalculateTimeOfImpact(c._fixtureA.GetShape(), b1._sweep, c._fixtureB.GetShape(), b2._sweep);

				        Debug.Assert(0.0f <= toi && toi <= 1.0f);

				        // If the TOI is in range ...
				        if (0.0f < toi && toi < 1.0f)
				        {
					        // Interpolate on the actual range.
					        toi = Math.Min((1.0f - toi) * t0 + toi, 1.0f);
				        }


				        c._toi = toi;
				        c._flags |= ContactFlags.Toi;
			        }

			        if (Settings.b2_FLT_EPSILON < toi && toi < minTOI)
			        {
				        // This is the minimum TOI found so far.
				        minContact = c;
				        minTOI = toi;
			        }
		        }

		        if (minContact == null || 1.0f - 100.0f * Settings.b2_FLT_EPSILON < minTOI)
		        {
			        // No more TOI events. Done!
			        break;
		        }

		        // Advance the bodies to the TOI.
		        Fixture s1_2 = minContact.GetFixtureA();
                Fixture s2_2 = minContact.GetFixtureB();
                Body b1_2 = s1_2.GetBody();
                Body b2_2 = s2_2.GetBody();
                b1_2.Advance(minTOI);
                b2_2.Advance(minTOI);

		        // The TOI contact likely has some new contact points.
		        minContact.Update(_contactManager.ContactListener);
		        minContact._flags &= ~ContactFlags.Toi;

		        if ((minContact._flags & ContactFlags.Touch) == 0)
		        {
			        // This shouldn't happen. Numerical error?
			        //Debug.Assert(false);
			        continue;
		        }

		        // Build the TOI island. We need a dynamic seed.
                Body seed = b1_2;
		        if (seed.IsStatic)
		        {
                    seed = b2_2;
		        }

		        // Reset island and queue.
		        _island.Clear();

		        int queueStart = 0; // starting index for queue
		        int queueSize = 0;  // elements in queue
		        queue[queueStart + queueSize++] = seed;
		        seed._flags |= BodyFlags.Island;

		        // Perform a breadth first search (BFS) on the contact/joint graph.
		        while (queueSize > 0)
		        {
			        // Grab the next body off the stack and add it to the island.
			        Body b = queue[queueStart++];
			        --queueSize;

			        _island.Add(b);

			        // Make sure the body is awake.
			        b._flags &= ~BodyFlags.Sleep;

			        // To keep islands as small as possible, we don't
			        // propagate islands across static bodies.
			        if (b.IsStatic)
			        {
				        continue;
			        }

			        // Search all contacts connected to this body.
			        for (ContactEdge cEdge = b._contactList; cEdge != null; cEdge = cEdge.Next)
			        {
				        // Does the TOI island still have space for contacts?
				        if (_island._contacts.Count == _island._contactCapacity)
				        {
					        continue;
				        }

				        // Has this contact already been added to an island? Skip slow or non-solid contacts.
				        if ((cEdge.Contact._flags & (ContactFlags.Island | ContactFlags.Slow | ContactFlags.NonSolid)) != ContactFlags.None)
				        {
					        continue;
				        }

				        // Is this contact touching? For performance we are not updating this contact.
				        if ((cEdge.Contact._flags & ContactFlags.Touch) == 0)
				        {
					        continue;
				        }

				        _island.Add(cEdge.Contact);
				        cEdge.Contact._flags |= ContactFlags.Island;

				        // Update other body.
				        Body other = cEdge.Other;

				        // Was the other body already added to this island?
				        if ((other._flags & BodyFlags.Island) != BodyFlags.None)
				        {
					        continue;
				        }

				        // March forward, this can do no harm since this is the min TOI.
				        if (other.IsStatic == false)
				        {
					        other.Advance(minTOI);
					        other.WakeUp();
				        }

				        Debug.Assert(queueStart + queueSize < queueCapacity);
				        queue[queueStart + queueSize] = other;
				        ++queueSize;
				        other._flags |= BodyFlags.Island;
			        }

			        for (JointEdge jEdge = b._jointList; jEdge != null; jEdge = jEdge.Next)
			        {
				        if (_island._jointCount == _island._jointCapacity)
				        {
					        continue;
				        }

				        if (jEdge.Joint._islandFlag == true)
				        {
					        continue;
				        }

				        _island.Add(jEdge.Joint);

				        jEdge.Joint._islandFlag = true;

				        Body other = jEdge.Other;

				        if ((other._flags & BodyFlags.Island) != BodyFlags.None)
				        {
					        continue;
				        }

				        if (!other.IsStatic)
				        {
					        other.Advance(minTOI);
					        other.WakeUp();
				        }

				        Debug.Assert(queueStart + queueSize < queueCapacity);
				        queue[queueStart + queueSize] = other;
				        ++queueSize;
				        other._flags |= BodyFlags.Island;
			        }
		        }

		        TimeStep subStep;
		        subStep.warmStarting = false;
		        subStep.dt = (1.0f - minTOI) * step.dt;
		        subStep.inv_dt = 1.0f / subStep.dt;
		        subStep.dtRatio = 0.0f;
		        subStep.velocityIterations = step.velocityIterations;
		        subStep.positionIterations = step.positionIterations;

		        _island.SolveTOI(ref subStep);

		        // Post solve cleanup.
		        for (int i = 0; i < _island._bodyCount; ++i)
		        {
			        // Allow bodies to participate in future TOI islands.
			        Body b = _island._bodies[i];
			        b._flags &= ~BodyFlags.Island;

			        if ((b._flags & (BodyFlags.Sleep | BodyFlags.Frozen)) != BodyFlags.None)
			        {
				        continue;
			        }

			        if (b.IsStatic)
			        {
				        continue;
			        }

			        // Update fixtures (for broad-phase).
			        b.SynchronizeFixtures();

			        // Invalidate all contact TOIs associated with this body. Some of these
			        // may not be in the island because they were not touching.
			        for (ContactEdge ce = b._contactList; ce != null; ce = ce.Next)
			        {
				        ce.Contact._flags &= ~ContactFlags.Toi;
			        }
		        }

                int contactCount = _island._contacts.Count;
                for (int i = 0; i < contactCount; ++i)
		        {
			        // Allow contacts to participate in future TOI islands.
			        Contact c = _island._contacts[i];
			        c._flags &= ~(ContactFlags.Toi | ContactFlags.Island);
		        }

		        for (int i = 0; i < _island._jointCount; ++i)
		        {
			        // Allow joints to participate in future TOI islands.
			        Joint j = _island._joints[i];
			        j._islandFlag = false;
		        }

		        // Commit fixture proxy movements to the broad-phase so that new contacts are created.
		        // Also, some contacts can be destroyed.
		        _contactManager.FindNewContacts();
	        }
        }

	    void DrawJoint(Joint joint)
        {
	        Body b1 = joint.GetBody1();
	        Body b2 = joint.GetBody2();
            XForm xf1, xf2;
            b1.GetXForm(out xf1);
	        b2.GetXForm(out xf2);
	        Vector2 x1 = xf1.Position;
	        Vector2 x2 = xf2.Position;
	        Vector2 p1 = joint.GetAnchor1();
	        Vector2 p2 = joint.GetAnchor2();

	        Color color = ColorEx.FromScRgb(0.5f, 0.8f, 0.8f);

	        switch (joint.JointType)
	        {
	        case JointType.Distance:
		        DebugDraw.DrawSegment(p1, p2, color);
		        break;

	        case JointType.Pulley:
		        {
			        PulleyJoint pulley = (PulleyJoint)joint;
			        Vector2 s1 = pulley.GetGroundAnchor1();
			        Vector2 s2 = pulley.GetGroundAnchor2();
			        DebugDraw.DrawSegment(s1, p1, color);
			        DebugDraw.DrawSegment(s2, p2, color);
			        DebugDraw.DrawSegment(s1, s2, color);
		        }
		        break;

	        case JointType.Mouse:
		        // don't draw this
		        break;

	        default:
		        DebugDraw.DrawSegment(x1, p1, color);
		        DebugDraw.DrawSegment(p1, p2, color);
		        DebugDraw.DrawSegment(x2, p2, color);
                break;
	        }
        }

	    void DrawShape(Fixture fixture, XForm xf, Color color)
        {
	        Color coreColor = ColorEx.FromScRgb(0.9f, 0.6f, 0.6f);

	        switch (fixture.ShapeType)
	        {
	        case ShapeType.Circle:
		        {
			        CircleShape circle = (CircleShape)fixture.GetShape();

			        Vector2 center = MathUtils.Multiply(ref xf, circle._p);
			        float radius = circle._radius;
			        Vector2 axis = xf.R.col1;

			        DebugDraw.DrawSolidCircle(center, radius, axis, color);
		        }
		        break;

	        case ShapeType.Polygon:
		        {
			        PolygonShape poly = (PolygonShape)fixture.GetShape();
			        int vertexCount = poly._vertexCount;
			        Debug.Assert(vertexCount <= Settings.b2_maxPolygonVertices);
			        FixedArray8<Vector2> vertices = new FixedArray8<Vector2>();

			        for (int i = 0; i < vertexCount; ++i)
			        {
				        vertices[i] = MathUtils.Multiply(ref xf, poly._vertices[i]);
			        }

			        DebugDraw.DrawSolidPolygon(ref vertices, vertexCount, color);
		        }
		        break;
	        }
        }

        internal Island _island = new Island();
        internal WorldFlags _flags;

	    internal ContactManager _contactManager = new ContactManager();

        internal Body _bodyList;
        internal Joint _jointList;

        internal int _bodyCount;
        internal int _jointCount;

        internal bool _allowSleep;

        internal Body _groundBody;

	    // This is used to compute the time step ratio to
	    // support a variable time step.
        internal float _inv_dt0;
    }
}
