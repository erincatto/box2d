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


using System.Diagnostics;
using System;
using System.Numerics;

namespace Box2D.UWP
{
    /// Distance joint definition. This requires defining an
    /// anchor point on both bodies and the non-zero length of the
    /// distance joint. The definition uses local anchor points
    /// so that the initial configuration can violate the raint
    /// slightly. This helps when saving and loading a game.
    /// @warning Do not use a zero or short length.
    public class DistanceJointDef : JointDef
    {
	    public DistanceJointDef()
	    {
		    type = JointType.Distance;
		    localAnchor1 = new Vector2(0.0f, 0.0f);
		    localAnchor2 = new Vector2(0.0f, 0.0f);
		    length = 1.0f;
		    frequencyHz = 0.0f;
		    dampingRatio = 0.0f;
	    }

	    /// Initialize the bodies, anchors, and length using the world
	    /// anchors.
        // 1-D rained system
        // m (v2 - v1) = lambda
        // v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
        // x2 = x1 + h * v2

        // 1-D mass-damper-spring system
        // m (v2 - v1) + h * d * v2 + h * k * 

        // C = norm(p2 - p1) - L
        // u = (p2 - p1) / norm(p2 - p1)
        // Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
        // J = [-u -cross(r1, u) u cross(r2, u)]
        // K = J * invM * JT
        //   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2
	    public void Initialize(Body b1, Body b2,
					    Vector2 anchor1, Vector2 anchor2)
        {
	        body1 = b1;
	        body2 = b2;
	        localAnchor1 = body1.GetLocalPoint(anchor1);
	        localAnchor2 = body2.GetLocalPoint(anchor2);
	        Vector2 d = anchor2 - anchor1;
	        length = d.Length();
        }

	    /// The local anchor point relative to body1's origin.
	    public Vector2 localAnchor1;

	    /// The local anchor point relative to body2's origin.
	    public Vector2 localAnchor2;

	    /// The equilibrium length between the anchor points.
	    public float length;

	    /// The response speed.
	    public float frequencyHz;

	    /// The damping ratio. 0 = no damping, 1 = critical damping.
	    public float dampingRatio;
    };

    /// A distance joint rains two points on two bodies
    /// to remain at a fixed distance from each other. You can view
    /// this as a massless, rigid rod.
    public class DistanceJoint : Joint
    {
	    public override Vector2 GetAnchor1()
        {
            return _bodyA.GetWorldPoint(_localAnchor1);
        }

        public override Vector2 GetAnchor2()
        {
	        return _bodyB.GetWorldPoint(_localAnchor2);
        }

        public override Vector2 GetReactionForce(float inv_dt)
        {
	        Vector2 F = (inv_dt * _impulse) * _u;
	        return F;
        }

        public override float GetReactionTorque(float inv_dt)
        {
	        return 0.0f;
        }

	    internal DistanceJoint(DistanceJointDef def)
            : base(def)
        {
	        _localAnchor1 = def.localAnchor1;
	        _localAnchor2 = def.localAnchor2;
	        _length = def.length;
	        _frequencyHz = def.frequencyHz;
	        _dampingRatio = def.dampingRatio;
	        _impulse = 0.0f;
	        _gamma = 0.0f;
	        _bias = 0.0f;
        }

        internal override void InitVelocityConstraints(ref TimeStep step)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

	        // Compute the effective mass matrix.
            Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
            Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());
	        _u = b2._sweep.c + r2 - b1._sweep.c - r1;

	        // Handle singularity.
	        float length = _u.Length();
	        if (length > Settings.b2_linearSlop)
	        {
		        _u *= 1.0f / length;
	        }
	        else
	        {
		        _u = new Vector2(0.0f, 0.0f);
	        }

	        float cr1u = MathUtils.Cross(r1, _u);
	        float cr2u = MathUtils.Cross(r2, _u);
	        float invMass = b1._invMass + b1._invI * cr1u * cr1u + b2._invMass + b2._invI * cr2u * cr2u;
	        Debug.Assert(invMass > Settings.b2_FLT_EPSILON);
	        _mass = 1.0f / invMass;

	        if (_frequencyHz > 0.0f)
	        {
		        float C = length - _length;

		        // Frequency
		        float omega = 2.0f * Settings.b2_pi * _frequencyHz;

		        // Damping coefficient
		        float d = 2.0f * _mass * _dampingRatio * omega;

		        // Spring stiffness
		        float k = _mass * omega * omega;

		        // magic formulas
		        _gamma = 1.0f / (step.dt * (d + step.dt * k));
		        _bias = C * step.dt * k * _gamma;

		        _mass = 1.0f / (invMass + _gamma);
	        }

	        if (step.warmStarting)
	        {
		        // Scale the impulse to support a variable time step.
		        _impulse *= step.dtRatio;

		        Vector2 P = _impulse * _u;
		        b1._linearVelocity -= b1._invMass * P;
		        b1._angularVelocity -= b1._invI * MathUtils.Cross(r1, P);
		        b2._linearVelocity += b2._invMass * P;
		        b2._angularVelocity += b2._invI * MathUtils.Cross(r2, P);
	        }
	        else
	        {
		        _impulse = 0.0f;
	        }
        }

        internal override void SolveVelocityConstraints(ref TimeStep step)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

            Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
            Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

	        // Cdot = dot(u, v + cross(w, r))
	        Vector2 v1 = b1._linearVelocity + MathUtils.Cross(b1._angularVelocity, r1);
	        Vector2 v2 = b2._linearVelocity + MathUtils.Cross(b2._angularVelocity, r2);
	        float Cdot = Vector2.Dot(_u, v2 - v1);

	        float impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
	        _impulse += impulse;

	        Vector2 P = impulse * _u;
	        b1._linearVelocity -= b1._invMass * P;
	        b1._angularVelocity -= b1._invI * MathUtils.Cross(r1, P);
	        b2._linearVelocity += b2._invMass * P;
	        b2._angularVelocity += b2._invI * MathUtils.Cross(r2, P);
        }

        internal override bool SolvePositionConstraints(float baumgarte)
        {
	        if (_frequencyHz > 0.0f)
	        {
		        // There is no position correction for soft distance raints.
		        return true;
	        }

	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

	        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
	        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

	        Vector2 d = b2._sweep.c + r2 - b1._sweep.c - r1;

	        float length = d.Length();
            d /= length;
	        float C = length - _length;
	        C = MathUtils.Clamp(C, -Settings.b2_maxLinearCorrection, Settings.b2_maxLinearCorrection);

	        float impulse = -_mass * C;
	        _u = d;
	        Vector2 P = impulse * _u;

	        b1._sweep.c -= b1._invMass * P;
	        b1._sweep.a -= b1._invI * MathUtils.Cross(r1, P);
	        b2._sweep.c += b2._invMass * P;
	        b2._sweep.a += b2._invI * MathUtils.Cross(r2, P);

	        b1.SynchronizeTransform();
	        b2.SynchronizeTransform();

	        return Math.Abs(C) < Settings.b2_linearSlop;
        }

	    internal Vector2 _localAnchor1;
	    internal Vector2 _localAnchor2;
	    internal Vector2 _u;
	    internal float _frequencyHz;
	    internal float _dampingRatio;
	    internal float _gamma;
	    internal float _bias;
	    internal float _impulse;
	    internal float _mass;		// effective mass for the raint.
	    internal float _length;
    };
}
