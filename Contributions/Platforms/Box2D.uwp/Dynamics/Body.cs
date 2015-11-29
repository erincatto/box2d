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
using System.Numerics;

namespace Box2D.UWP
{
    /// A body definition holds all the data needed to ruct a rigid body.
    /// You can safely re-use body definitions.
    /// Shapes are added to a body after ruction.
    public class BodyDef
    {
        /// This ructor sets the body definition default values.
        public BodyDef()
        {
            massData.center = Vector2.Zero;
            massData.mass = 0.0f;
            massData.i = 0.0f;
            userData = null;
            position = new Vector2(0.0f, 0.0f);
            angle = 0.0f;
            linearVelocity = new Vector2(0.0f, 0.0f);
            angularVelocity = 0.0f;
            linearDamping = 0.0f;
            angularDamping = 0.0f;
            allowSleep = true;
            isSleeping = false;
            fixedRotation = false;
            isBullet = false;
        }

        /// You can use this to initialized the mass properties of the body.
        /// If you prefer, you can set the mass properties after the shapes
        /// have been added using Body.SetMassFromShapes.
        /// By default the mass data is set to zero, meaning the body is seen
        /// as static. If you intend the body to be dynamic, a small performance
        /// gain can be had by setting the mass to some positive value.
        public MassData massData;

        /// Use this to store application specific body data.
        public object userData;

        /// The world position of the body. Avoid creating bodies at the origin
        /// since this can lead to many overlapping shapes.
        public Vector2 position;

        /// The world angle of the body in radians.
        public float angle;

        /// The linear velocity of the body in world co-ordinates.
        public Vector2 linearVelocity;

        /// The angular velocity of the body.
        public float angularVelocity;

        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        public float linearDamping;

        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        public float angularDamping;

        /// Set this flag to false if this body should never fall asleep. Note that
        /// this increases CPU usage.
        public bool allowSleep;

        /// Is this body initially sleeping?
        public bool isSleeping;

        /// Should this body be prevented from rotating? Useful for characters.
        public bool fixedRotation;

        /// Is this a fast moving body that should be prevented from tunneling through
        /// other moving bodies? Note that all bodies are prevented from tunneling through
        /// static bodies.
        /// @warning You should use this flag sparingly since it increases processing time.
        public bool isBullet;
    };


    [Flags]
    public enum BodyFlags
    {
        None          = 0,
        Frozen        = (1 << 2),
        Island        = (1 << 3),
        Sleep         = (1 << 4),
        AllowSleep    = (1 << 5),
        Bullet        = (1 << 6),
        FixedRotation = (1 << 7),
    }

    public enum BodyType
    {
        Static,
        Dynamic,
    }

    public class Body
    {
	    /// Creates a fixture and attach it to this body. Use this function if you need
	    /// to set some fixture parameters, like friction. Otherwise you can create the
	    /// fixture directly from a shape.
	    /// @param def the fixture definition.
	    /// @warning This function is locked during callbacks.
	    public Fixture CreateFixture(FixtureDef def)
        {
            Debug.Assert(_world.IsLocked == false);
	        if (_world.IsLocked == true)
	        {
		        return null;
	        }

	        BroadPhase broadPhase = _world._contactManager._broadPhase;

	        Fixture fixture = new Fixture();
	        fixture.Create(broadPhase, this, ref _xf, def);

	        fixture._next = _fixtureList;
	        _fixtureList = fixture;
	        ++_fixtureCount;

	        fixture._body = this;

	        // Let the world know we have a new fixture.
	        _world._flags |= WorldFlags.NewFixture;

	        return fixture;
        }

	    /// Creates a fixture from a shape and attach it to this body.
	    /// This is a convenience function. Use FixtureDef if you need to set parameters
	    /// like friction, restitution, user data, or filtering.
	    /// @param shape the shape to be cloned.
	    /// @param density the shape density (set to zero for static bodies).
	    /// @warning This function is locked during callbacks.
	    public Fixture CreateFixture(Shape shape, float density)
        {
            Debug.Assert(_world.IsLocked == false);
	        if (_world.IsLocked == true)
	        {
		        return null;
	        }

	        BroadPhase broadPhase = _world._contactManager._broadPhase;

	        FixtureDef def = new FixtureDef();
	        def.shape = shape;
	        def.density = density;

	        Fixture fixture = new Fixture();
	        fixture.Create(broadPhase, this, ref _xf, def);

	        fixture._next = _fixtureList;
	        _fixtureList = fixture;
	        ++_fixtureCount;

	        fixture._body = this;

	        // Let the world know we have a new fixture.
	        _world._flags |= WorldFlags.NewFixture;

	        return fixture;
        }

	    /// Destroy a fixture. This removes the fixture from the broad-phase and
	    /// therefore destroys any contacts associated with this fixture. All fixtures
	    /// attached to a body are implicitly destroyed when the body is destroyed.
	    /// @param fixture the fixture to be removed.
	    /// @warning This function is locked during callbacks.
	    public void DestroyFixture(Fixture fixture)
        {
            Debug.Assert(_world.IsLocked == false);
	        if (_world.IsLocked == true)
	        {
		        return;
	        }

	        Debug.Assert(fixture._body == this);

	        // Remove the fixture from this body's singly linked list.
	        Debug.Assert(_fixtureCount > 0);
	        Fixture node = _fixtureList;
	        bool found = false;
	        while (node != null)
	        {
		        if (node == fixture)
		        {
                    _fixtureList = fixture._next;
			        found = true;
			        break;
		        }

		        node = node._next;
	        }

	        // You tried to remove a shape that is not attached to this body.
	        Debug.Assert(found);

	        // Destroy any contacts associated with the fixture.
	        ContactEdge edge = _contactList;
	        while (edge != null)
	        {
		        Contact c = edge.Contact;
		        edge = edge.Next;

		        Fixture fixtureA = c.GetFixtureA();
		        Fixture fixtureB = c.GetFixtureB();

		        if (fixture == fixtureA || fixture == fixtureB)
		        {
			        // This destroys the contact and removes it from
			        // this body's contact list.
			        _world._contactManager.Destroy(c);
		        }
	        }

	        BroadPhase broadPhase = _world._contactManager._broadPhase;

	        fixture.Destroy(broadPhase);
	        fixture._body = null;
	        fixture._next = null;
	        --_fixtureCount;
        }

	    /// Set the mass properties. Note that this changes the center of mass position.
	    /// If you are not sure how to compute mass properties, use SetMassFromShapes.
	    /// The inertia tensor is assumed to be relative to the center of mass.
	    /// You can make the body static by using a zero mass.
	    /// @param massData the mass properties.
	    public void SetMassData(MassData massData)
        {
            Debug.Assert(_world.IsLocked == false);
	        if (_world.IsLocked == true)
	        {
		        return;
	        }

	        _invMass = 0.0f;
	        _I = 0.0f;
	        _invI = 0.0f;

	        _mass = massData.mass;

	        if (_mass > 0.0f)
	        {
		        _invMass = 1.0f / _mass;
	        }

	        _I = massData.i;

	        if (_I > 0.0f && (_flags & BodyFlags.FixedRotation) == BodyFlags.None)
	        {
		        _invI = 1.0f / _I;
	        }

	        // Move center of mass.
	        _sweep.localCenter = massData.center;
	        _sweep.c0 = _sweep.c = MathUtils.Multiply(ref _xf, _sweep.localCenter);

	        BodyType oldType = _type;
	        if (_invMass == 0.0f && _invI == 0.0f)
	        {
		        _type = BodyType.Static;
	        }
	        else
	        {
		        _type = BodyType.Dynamic;
	        }

	        // If the body type changed, we need to flag contacts for filtering.
	        if (oldType != _type)
	        {
		        for (ContactEdge ce = _contactList; ce != null; ce = ce.Next)
		        {
			        ce.Contact.FlagForFiltering();
		        }
	        }
        }

	    /// Compute the mass properties from the attached fixture. You typically call this
	    /// after adding all the fixtures. If you add or remove fixtures later, you may want
	    /// to call this again. Note that this changes the center of mass position.
	    public void SetMassFromShapes()
        {
            Debug.Assert(_world.IsLocked == false);
	        if (_world.IsLocked == true)
	        {
		        return;
	        }

	        // Compute mass data from shapes. Each shape has its own density.
	        _mass = 0.0f;
	        _invMass = 0.0f;
	        _I = 0.0f;
	        _invI = 0.0f;

	        Vector2 center = Vector2.Zero;
	        for (Fixture f = _fixtureList; f != null; f = f._next)
	        {
		        MassData massData;
		        f.ComputeMass(out massData);
		        _mass += massData.mass;
		        center += massData.mass * massData.center;
		        _I += massData.i;
	        }

	        // Compute center of mass, and shift the origin to the COM.
	        if (_mass > 0.0f)
	        {
		        _invMass = 1.0f / _mass;
		        center *= _invMass;
	        }

	        if (_I > 0.0f && (_flags & BodyFlags.FixedRotation) == 0)
	        {
		        // Center the inertia about the center of mass.
		        _I -= _mass * Vector2.Dot(center, center);
		        Debug.Assert(_I > 0.0f);
		        _invI = 1.0f / _I;
	        }
	        else
	        {
		        _I = 0.0f;
		        _invI = 0.0f;
	        }

	        // Move center of mass.
	        _sweep.localCenter = center;
	        _sweep.c0 = _sweep.c = MathUtils.Multiply(ref _xf, _sweep.localCenter);

	        BodyType oldType = _type;
	        if (_invMass == 0.0f && _invI == 0.0f)
	        {
		        _type = BodyType.Static;
	        }
	        else
	        {
		        _type = BodyType.Dynamic;
	        }

	        // If the body type changed, we need to flag contacts for filtering.
	        if (oldType != _type)
	        {
		        for (ContactEdge ce = _contactList; ce != null; ce = ce.Next)
		        {
			        ce.Contact.FlagForFiltering();
		        }
	        }
        }

	    /// Set the position of the body's origin and rotation.
	    /// This breaks any contacts and wakes the other bodies.
	    /// @param position the world position of the body's local origin.
	    /// @param angle the world rotation in radians.
	    public void SetXForm(Vector2 position, float angle)
        {
            Debug.Assert(_world.IsLocked == false);
	        if (_world.IsLocked == true)
	        {
		        return;
	        }

	        _xf.R.Set(angle);
	        _xf.Position = position;

	        _sweep.c0 = _sweep.c = MathUtils.Multiply(ref _xf, _sweep.localCenter);
	        _sweep.a0 = _sweep.a = angle;

	        BroadPhase broadPhase = _world._contactManager._broadPhase;
	        for (Fixture f = _fixtureList; f != null; f = f._next)
	        {
		        f.Synchronize(broadPhase, ref _xf, ref _xf);
	        }

	        _world._contactManager.FindNewContacts();
        }

	    /// Get the body transform for the body's origin.
	    /// @return the world transform of the body's origin.
	    public void GetXForm(out XForm xf)
        {
            xf = _xf;
        }

	    /// Get the world body origin position.
	    /// @return the world position of the body's origin.
	    public Vector2 GetPosition()
        {
            return _xf.Position;
        }

	    /// Get the angle in radians.
	    /// @return the current world rotation angle in radians.
	    public float GetAngle() 
        {
            return _sweep.a;
        }

	    /// Get the world position of the center of mass.
	    public Vector2 GetWorldCenter() 
        {
            return _sweep.c;
        }

	    /// Get the local position of the center of mass.
	    public Vector2 GetLocalCenter() 
        {
            return _sweep.localCenter;
        }

	    /// Set the linear velocity of the center of mass.
	    /// @param v the new linear velocity of the center of mass.
	    public void SetLinearVelocity(Vector2 v)
        {
            _linearVelocity = v;
        }

	    /// Get the linear velocity of the center of mass.
	    /// @return the linear velocity of the center of mass.
	    public Vector2 GetLinearVelocity()
        {
            return _linearVelocity;
        }

	    /// Set the angular velocity.
	    /// @param omega the new angular velocity in radians/second.
	    public void SetAngularVelocity(float omega)
        {
            _angularVelocity = omega;
        }

	    /// Get the angular velocity.
	    /// @return the angular velocity in radians/second.
	    public float GetAngularVelocity()
        {
            return _angularVelocity;
        }

	    /// Apply a force at a world point. If the force is not
	    /// applied at the center of mass, it will generate a torque and
	    /// affect the angular velocity. This wakes up the body.
	    /// @param force the world force vector, usually in Newtons (N).
	    /// @param point the world position of the point of application.
	    public void ApplyForce(Vector2 force, Vector2 point)
        {
            if (IsSleeping)
	        {
		        WakeUp();
	        }
	        _force += force;
	        _torque += MathUtils.Cross(point - _sweep.c, force);
        }

	    /// Apply a torque. This affects the angular velocity
	    /// without affecting the linear velocity of the center of mass.
	    /// This wakes up the body.
	    /// @param torque about the z-axis (out of the screen), usually in N-m.
	    public void ApplyTorque(float torque)
        {
            if (IsSleeping)
	        {
		        WakeUp();
	        }
	        _torque += torque;
        }

	    /// Apply an impulse at a point. This immediately modifies the velocity.
	    /// It also modifies the angular velocity if the point of application
	    /// is not at the center of mass. This wakes up the body.
	    /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	    /// @param point the world position of the point of application.
	    public void ApplyImpulse(Vector2 impulse, Vector2 point)
        {
            if (IsSleeping)
	        {
		        WakeUp();
	        }
	        _linearVelocity += _invMass * impulse;
            _angularVelocity += _invI * MathUtils.Cross(point - _sweep.c, impulse);
        }

	    /// Get the total mass of the body.
	    /// @return the mass, usually in kilograms (kg).
	    public float GetMass()
        {
            return _mass;
        }

	    /// Get the central rotational inertia of the body.
	    /// @return the rotational inertia, usually in kg-m^2.
	    public float GetInertia()
        {
            return _I;
        }

	    /// Get the mass data of the body.
	    /// @return a struct containing the mass, inertia and center of the body.
	    public MassData GetMassData()
        {
            MassData massData;
            massData.mass = _mass;
            massData.i = _I;
            massData.center = GetWorldCenter();
            return massData;
        }

	    /// Get the world coordinates of a point given the local coordinates.
	    /// @param localPoint a point on the body measured relative the the body's origin.
	    /// @return the same point expressed in world coordinates.
	    public Vector2 GetWorldPoint(Vector2 localPoint)
        {
            return MathUtils.Multiply(ref _xf, localPoint);
        }

	    /// Get the world coordinates of a vector given the local coordinates.
	    /// @param localVector a vector fixed in the body.
	    /// @return the same vector expressed in world coordinates.
	    public Vector2 GetWorldVector(Vector2 localVector)
        {
            return MathUtils.Multiply(ref _xf.R, localVector);
        }

	    /// Gets a local point relative to the body's origin given a world point.
	    /// @param a point in world coordinates.
	    /// @return the corresponding local point relative to the body's origin.
	    public Vector2 GetLocalPoint(Vector2 worldPoint)
        {
            return MathUtils.MultiplyT(ref _xf, worldPoint);
        }

	    /// Gets a local vector given a world vector.
	    /// @param a vector in world coordinates.
	    /// @return the corresponding local vector.
	    public Vector2 GetLocalVector(Vector2 worldVector)
        {
            return MathUtils.MultiplyT(ref _xf.R, worldVector);
        }

	    /// Get the world linear velocity of a world point attached to this body.
	    /// @param a point in world coordinates.
	    /// @return the world velocity of a point.
	    public Vector2 GetLinearVelocityFromWorldPoint(Vector2 worldPoint)
        {
            return _linearVelocity + MathUtils.Cross(_angularVelocity, worldPoint - _sweep.c);
        }

	    /// Get the world velocity of a local point.
	    /// @param a point in local coordinates.
	    /// @return the world velocity of a point.
	    public Vector2 GetLinearVelocityFromLocalPoint(Vector2 localPoint)
        {
            return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
        }

	    /// Get the linear damping of the body.
	    public float GetLinearDamping()
        {
            return _linearDamping;
        }

	    /// Set the linear damping of the body.
	    public void SetLinearDamping(float linearDamping)
        {
            _linearDamping = linearDamping;
        }

	    /// Get the angular damping of the body.
	    public float GetAngularDamping()
        {
            return _angularDamping;
        }

	    /// Set the angular damping of the body.
	    public void SetAngularDamping(float angularDamping)
        {
            _angularDamping = angularDamping;
        }

	    /// Is this body treated like a bullet for continuous collision detection?
	    public bool IsBullet
        {
            get
            {
                return (_flags & BodyFlags.Bullet) == BodyFlags.Bullet;
            }
        }

	    /// Should this body be treated like a bullet for continuous collision detection?
	    public void SetBullet(bool flag)
        {
            if (flag)
	        {
		        _flags |= BodyFlags.Bullet;
	        }
	        else
	        {
		        _flags &= ~BodyFlags.Bullet;
	        }
        }

	    /// Is this body static (immovable)?
	    public bool IsStatic
        {
            get
            {
                return _type == BodyType.Static;
            }
        }

	    /// Is this body dynamic (movable)?
	    public bool IsDynamic
        {
            get
            {
                return _type == BodyType.Dynamic;
            }
        }

	    /// Is this body frozen?
	    public bool IsFrozen
        {
            get
            {
                return (_flags & BodyFlags.Frozen) == BodyFlags.Frozen;
            }
        }

	    /// Is this body sleeping (not simulating).
	    public bool IsSleeping
        {
            get
            {
                return (_flags & BodyFlags.Sleep) == BodyFlags.Sleep;
            }
        }

	    /// Is this body allowed to sleep
	    public bool IsAllowSleeping
        {
            get
            {
                return (_flags & BodyFlags.AllowSleep) == BodyFlags.AllowSleep;
            }
        }

	    /// You can disable sleeping on this body.
	    public void AllowSleeping(bool flag)
        {
            if (flag)
	        {
		        _flags |= BodyFlags.AllowSleep;
	        }
	        else
	        {
		        _flags &= ~BodyFlags.AllowSleep;
		        WakeUp();
	        }
        }

	    /// Wake up this body so it will begin simulating.
	    public void WakeUp()
        {
            _flags &= ~BodyFlags.Sleep;
	        _sleepTime = 0.0f;
        }

	    /// Put this body to sleep so it will stop simulating.
	    /// This also sets the velocity to zero.
	    public void PutToSleep()
        {
            _flags |= BodyFlags.Sleep;
	        _sleepTime = 0.0f;
	        _linearVelocity = Vector2.Zero;
	        _angularVelocity = 0.0f;
	        _force = Vector2.Zero;
	        _torque = 0.0f;
        }

	    /// Get the list of all fixtures attached to this body.
	    public Fixture GetFixtureList()
        {
            return _fixtureList;
        }

	    /// Get the list of all joints attached to this body.
	    public JointEdge GetJointList()
        {
            return _jointList;
        }

	    /// Get the list of all contacts attached to this body.
	    /// @warning this list changes during the time step and you may
	    /// miss some collisions if you don't use ContactListener.
	    public ContactEdge GetConactList()
        {
            return _contactList;
        }

	    /// Get the next body in the world's body list.
	    public Body GetNext()
        {
            return _next;
        }

	    /// Get the user data pointer that was provided in the body definition.
	    public object GetUserData()
        {
            return _userData;
        }

	    /// Set the user data. Use this to store your application specific data.
	    public void SetUserData(object data)
        {
            _userData = data;
        }

	    /// Get the parent world of this body.
	    public World GetWorld()
        {
            return _world;
        }

        internal Body(BodyDef bd, World world)
        {
            _flags = 0;

	        if (bd.isBullet)
	        {
		        _flags |= BodyFlags.Bullet;
	        }
	        if (bd.fixedRotation)
	        {
		        _flags |= BodyFlags.FixedRotation;
	        }
	        if (bd.allowSleep)
	        {
		        _flags |= BodyFlags.AllowSleep;
	        }
	        if (bd.isSleeping)
	        {
		        _flags |= BodyFlags.Sleep;
	        }

	        _world = world;

	        _xf.Position = bd.position;
	        _xf.R.Set(bd.angle);

	        _sweep.localCenter = bd.massData.center;
	        _sweep.t0 = 1.0f;
	        _sweep.a0 = _sweep.a = bd.angle;
	        _sweep.c0 = _sweep.c = MathUtils.Multiply(ref _xf, _sweep.localCenter);

	        _jointList = null;
	        _contactList = null;
	        _prev = null;
	        _next = null;

	        _linearVelocity = bd.linearVelocity;
	        _angularVelocity = bd.angularVelocity;

	        _linearDamping = bd.linearDamping;
	        _angularDamping = bd.angularDamping;

	        _force = new Vector2(0.0f, 0.0f);
	        _torque = 0.0f;

	        _linearVelocity = Vector2.Zero;
	        _angularVelocity = 0.0f;

	        _sleepTime = 0.0f;

	        _invMass = 0.0f;
	        _I = 0.0f;
	        _invI = 0.0f;

	        _mass = bd.massData.mass;

	        if (_mass > 0.0f)
	        {
		        _invMass = 1.0f / _mass;
	        }

	        _I = bd.massData.i;
        	
	        if (_I > 0.0f && (_flags & BodyFlags.FixedRotation) == 0)
	        {
		        _invI = 1.0f / _I;
	        }

	        if (_invMass == 0.0f && _invI == 0.0f)
	        {
		        _type = BodyType.Static;
	        }
	        else
	        {
		        _type = BodyType.Dynamic;
	        }

	        _userData = bd.userData;

	        _fixtureList = null;
	        _fixtureCount = 0;
        }

        internal void SynchronizeFixtures()
        {
            XForm xf1 = new XForm();
	        xf1.R.Set(_sweep.a0);
	        xf1.Position = _sweep.c0 - MathUtils.Multiply(ref xf1.R, _sweep.localCenter);

	        BroadPhase broadPhase = _world._contactManager._broadPhase;
	        for (Fixture f = _fixtureList; f != null; f = f._next)
	        {
		        f.Synchronize(broadPhase, ref xf1, ref _xf);
	        }
        }

	    internal void SynchronizeTransform()
        {
            _xf.R.Set(_sweep.a);
	        _xf.Position = _sweep.c - MathUtils.Multiply(ref _xf.R, _sweep.localCenter);
        }

	    // This is used to prevent connected bodies from colliding.
	    // It may lie, depending on the collideConnected flag.
	    internal bool IsConnected(Body other)
        {
            for (JointEdge jn = _jointList; jn != null; jn = jn.Next)
	        {
		        if (jn.Other == other)
		        {
			        return jn.Joint._collideConnected == false;
		        }
	        }

	        return false;
        }

	    internal void Advance(float t)
        {
            // Advance to the new safe time.
	        _sweep.Advance(t);
	        _sweep.c = _sweep.c0;
	        _sweep.a = _sweep.a0;
	        SynchronizeTransform();
        }

        internal BodyFlags _flags;
        internal BodyType _type;

        internal int _islandIndex;

        internal XForm _xf;		// the body origin transform
        internal Sweep _sweep;	// the swept motion for CCD

        internal Vector2 _linearVelocity;
        internal float _angularVelocity;

        internal Vector2 _force;
        internal float _torque;

        internal World _world;
        internal Body _prev;
        internal Body _next;

        internal Fixture _fixtureList;
        internal int _fixtureCount;

        internal JointEdge _jointList;
        internal ContactEdge _contactList;

        internal float _mass, _invMass;
        internal float _I, _invI;

        internal float _linearDamping;
        internal float _angularDamping;

        internal float _sleepTime;

        internal object _userData;
    }
}
