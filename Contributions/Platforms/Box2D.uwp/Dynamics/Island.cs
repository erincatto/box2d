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
using System.Collections.Generic;
using System.Numerics;
using Box2D.uwp.UWPExtensions;

namespace Box2D.UWP
{
    /// This is an internal class.
    class Island
    {
        public Island() { }

        public void Reset(int bodyCapacity, int contactCapacity, int jointCapacity, IContactListener listener)
        {
            _bodyCapacity = bodyCapacity;
	        _contactCapacity = contactCapacity;
	        _jointCapacity	 = jointCapacity;
	        _bodyCount = 0;
	        _jointCount = 0;

	        _listener = listener;

            if (_bodies == null || _bodies.Length < bodyCapacity)
            {
                _bodies = new Body[bodyCapacity];
            }

            _contacts.Clear();

            if (_joints == null || _joints.Length < jointCapacity)
            {
                _joints = new Joint[jointCapacity];
            }
        }

	    public void Clear()
	    {
		    _bodyCount = 0;
            _contacts.Clear();
		    _jointCount = 0;
	    }

	    public void Solve(ref TimeStep step, Vector2 gravity, bool allowSleep)
        {
            // Integrate velocities and apply damping.
	        for (int i = 0; i < _bodyCount; ++i)
	        {
		        Body b = _bodies[i];

		        if (b.IsStatic)
			        continue;

		        // Integrate velocities.
		        b._linearVelocity += step.dt * (gravity + b._invMass * b._force);
		        b._angularVelocity += step.dt * b._invI * b._torque;

		        // Reset forces.
		        b._force = new Vector2(0.0f, 0.0f);
		        b._torque = 0.0f;

		        // Apply damping.
		        // ODE: dv/dt + c * v = 0
		        // Solution: v(t) = v0 * exp(-c * t)
		        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		        // v2 = exp(-c * dt) * v1
		        // Taylor expansion:
		        // v2 = (1.0f - c * dt) * v1
		        b._linearVelocity *= MathUtils.Clamp(1.0f - step.dt * b._linearDamping, 0.0f, 1.0f);
		        b._angularVelocity *= MathUtils.Clamp(1.0f - step.dt * b._angularDamping, 0.0f, 1.0f);
	        }

            _contactSolver.Reset(ref step, _contacts);

	        // Initialize velocity raints.
            _contactSolver.InitVelocityConstraints(ref step);

	        for (int i = 0; i < _jointCount; ++i)
	        {
		        _joints[i].InitVelocityConstraints(ref step);
	        }

	        // Solve velocity raints.
	        for (int i = 0; i < step.velocityIterations; ++i)
	        {
		        for (int j = 0; j < _jointCount; ++j)
		        {
			        _joints[j].SolveVelocityConstraints(ref step);
		        }

                _contactSolver.SolveVelocityConstraints();
	        }

	        // Post-solve (store impulses for warm starting).
            _contactSolver.FinalizeVelocityConstraints();

	        // Integrate positions.
	        for (int i = 0; i < _bodyCount; ++i)
	        {
		        Body b = _bodies[i];

		        if (b.IsStatic)
			        continue;

		        // Check for large velocities.
		        Vector2 translation = step.dt * b._linearVelocity;
		        if (Vector2.Dot(translation, translation) > Settings.b2_maxTranslationSquared)
		        {
			        translation.Normalize();
			        b._linearVelocity = (Settings.b2_maxTranslation * step.inv_dt) * translation;
		        }

		        float rotation = step.dt * b._angularVelocity;
		        if (rotation * rotation > Settings.b2_maxRotationSquared)
		        {
			        if (rotation < 0.0)
			        {
				        b._angularVelocity = -step.inv_dt * Settings.b2_maxRotation;
			        }
			        else
			        {
				        b._angularVelocity = step.inv_dt * Settings.b2_maxRotation;
			        }
		        }

		        // Store positions for continuous collision.
		        b._sweep.c0 = b._sweep.c;
		        b._sweep.a0 = b._sweep.a;

		        // Integrate
		        b._sweep.c += step.dt * b._linearVelocity;
		        b._sweep.a += step.dt * b._angularVelocity;

		        // Compute new transform
		        b.SynchronizeTransform();

		        // Note: shapes are synchronized later.
	        }

	        // Iterate over raints.
            for (int i = 0; i < step.positionIterations; ++i)
	        {
                bool contactsOkay = _contactSolver.SolvePositionConstraints(Settings.b2_contactBaumgarte);

		        bool jointsOkay = true;
		        for (int j = 0; j < _jointCount; ++j)
		        {
			        bool jointOkay = _joints[j].SolvePositionConstraints(Settings.b2_contactBaumgarte);
			        jointsOkay = jointsOkay && jointOkay;
		        }

		        if (contactsOkay && jointsOkay)
		        {
			        // Exit early if the position errors are small.
			        break;
		        }
	        }

            Report(_contactSolver._constraints);

	        if (allowSleep)
	        {
		        float minSleepTime = Settings.b2_FLT_MAX;

        #if !TARGET_float_IS_FIXED
		        float linTolSqr = Settings.b2_linearSleepTolerance * Settings.b2_linearSleepTolerance;
		        float angTolSqr = Settings.b2_angularSleepTolerance * Settings.b2_angularSleepTolerance;
        #endif

		        for (int i = 0; i < _bodyCount; ++i)
		        {
			        Body b = _bodies[i];
			        if (b._invMass == 0.0f)
			        {
				        continue;
			        }

			        if ((b._flags & BodyFlags.AllowSleep) == 0)
			        {
				        b._sleepTime = 0.0f;
				        minSleepTime = 0.0f;
			        }

			        if ((b._flags & BodyFlags.AllowSleep) == 0 ||
        #if TARGET_float_IS_FIXED
				        MathUtils.Abs(b._angularVelocity) > Settings.b2_angularSleepTolerance ||
				        MathUtils.Abs(b._linearVelocity.X) > Settings.b2_linearSleepTolerance ||
				        MathUtils.Abs(b._linearVelocity.Y) > Settings.b2_linearSleepTolerance)
        #else
				        b._angularVelocity * b._angularVelocity > angTolSqr ||
				        Vector2.Dot(b._linearVelocity, b._linearVelocity) > linTolSqr)
        #endif
			        {
				        b._sleepTime = 0.0f;
				        minSleepTime = 0.0f;
			        }
			        else
			        {
				        b._sleepTime += step.dt;
				        minSleepTime = Math.Min(minSleepTime, b._sleepTime);
			        }
		        }

		        if (minSleepTime >= Settings.b2_timeToSleep)
		        {
			        for (int i = 0; i < _bodyCount; ++i)
			        {
				        Body b = _bodies[i];
				        b._flags |= BodyFlags.Sleep;
				        b._linearVelocity = Vector2.Zero;
				        b._angularVelocity = 0.0f;
			        }
		        }
	        }
        }

	    public void SolveTOI(ref TimeStep subStep)
        {
            _contactSolver.Reset(ref subStep, _contacts);

            // No warm starting is needed for TOI events because warm
            // starting impulses were applied in the discrete solver.

            // Warm starting for joints is off for now, but we need to
            // call this function to compute Jacobians.
            for (int i = 0; i < _jointCount; ++i)
            {
	            _joints[i].InitVelocityConstraints(ref subStep);
            }

            // Solve velocity raints.
            for (int i = 0; i < subStep.velocityIterations; ++i)
            {
                _contactSolver.SolveVelocityConstraints();
	            for (int j = 0; j < _jointCount; ++j)
	            {
		            _joints[j].SolveVelocityConstraints(ref subStep);
	            }
            }

            // Don't store the TOI contact forces for warm starting
            // because they can be quite large.

            // Integrate positions.
            for (int i = 0; i < _bodyCount; ++i)
            {
	            Body b = _bodies[i];

	            if (b.IsStatic)
		            continue;

	            // Check for large velocities.
	            Vector2 translation = subStep.dt * b._linearVelocity;
	            if (Vector2.Dot(translation, translation) > Settings.b2_maxTranslationSquared)
	            {
		            translation.Normalize();
		            b._linearVelocity = (Settings.b2_maxTranslation * subStep.inv_dt) * translation;
	            }

	            float rotation = subStep.dt * b._angularVelocity;
	            if (rotation * rotation > Settings.b2_maxRotationSquared)
	            {
		            if (rotation < 0.0)
		            {
			            b._angularVelocity = -subStep.inv_dt * Settings.b2_maxRotation;
		            }
		            else
		            {
			            b._angularVelocity = subStep.inv_dt * Settings.b2_maxRotation;
		            }
	            }

	            // Store positions for continuous collision.
	            b._sweep.c0 = b._sweep.c;
	            b._sweep.a0 = b._sweep.a;

	            // Integrate
	            b._sweep.c += subStep.dt * b._linearVelocity;
	            b._sweep.a += subStep.dt * b._angularVelocity;

	            // Compute new transform
	            b.SynchronizeTransform();

	            // Note: shapes are synchronized later.
            }


            // Solve position raints.
            float k_toiBaumgarte = 0.75f;
            for (int i = 0; i < subStep.positionIterations; ++i)
            {
                bool contactsOkay = _contactSolver.SolvePositionConstraints(k_toiBaumgarte);
	            bool jointsOkay = true;
	            for (int j = 0; j < _jointCount; ++j)
	            {
		            bool jointOkay = _joints[j].SolvePositionConstraints(k_toiBaumgarte);
		            jointsOkay = jointsOkay && jointOkay;
	            }
        		
	            if (contactsOkay && jointsOkay)
	            {
		            break;
	            }
            }

            Report(_contactSolver._constraints);
        }

	    public void Add(Body body)
	    {
		    Debug.Assert(_bodyCount < _bodyCapacity);
		    body._islandIndex = _bodyCount;
		    _bodies[_bodyCount++] = body;
	    }

	    public void Add(Contact contact)
	    {
            _contacts.Add(contact);
	    }

	    public void Add(Joint joint)
	    {
		    Debug.Assert(_jointCount < _jointCapacity);
		    _joints[_jointCount++] = joint;
	    }

	    public void Report(List<ContactConstraint> constraints)
        {
            if (_listener == null)
	        {
		        return;
	        }

            int contactCount = _contacts.Count;
            for (int i = 0; i < contactCount; ++i)
	        {
		        Contact c = _contacts[i];

                ContactConstraint cc = constraints[i];
        		
		        ContactImpulse impulse = new ContactImpulse();
		        for (int j = 0; j < cc.pointCount; ++j)
		        {
			        impulse.normalImpulses[j] = cc.points[j].normalImpulse;
			        impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
		        }

		        _listener.PostSolve(c, ref impulse);
	        }
        }

        private ContactSolver _contactSolver = new ContactSolver();
	    public IContactListener _listener;

	    public Body[] _bodies;
        public List<Contact> _contacts = new List<Contact>(50);
	    public Joint[] _joints;

	    public int _bodyCount;
	    public int _jointCount;

	    public int _bodyCapacity;
	    public int _contactCapacity;
	    public int _jointCapacity;

	    public int _positionIterationCount;
    };
}
