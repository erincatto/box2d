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
    /// Pulley joint definition. This requires two ground anchors,
    /// two dynamic body anchor points, max lengths for each side,
    /// and a pulley ratio.
    public class PulleyJointDef : JointDef
    {
        internal static float b2_minPulleyLength = 2.0f;

	    public PulleyJointDef()
	    {
		    type = JointType.Pulley;
		    groundAnchor1 = new Vector2(-1.0f, 1.0f);
		    groundAnchor2 = new Vector2(1.0f, 1.0f);
		    localAnchor1 = new Vector2(-1.0f, 0.0f);
		    localAnchor2 = new Vector2(1.0f, 0.0f);
		    length1 = 0.0f;
		    maxLength1 = 0.0f;
		    length2 = 0.0f;
		    maxLength2 = 0.0f;
		    ratio = 1.0f;
		    collideConnected = true;
	    }

	    /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
        public void Initialize(Body b1, Body b2,
                        Vector2 ga1, Vector2 ga2,
					    Vector2 anchor1, Vector2 anchor2,
					    float r)
        {
	        body1 = b1;
	        body2 = b2;
	        groundAnchor1 = ga1;
	        groundAnchor2 = ga2;
	        localAnchor1 = body1.GetLocalPoint(anchor1);
	        localAnchor2 = body2.GetLocalPoint(anchor2);
	        Vector2 d1 = anchor1 - ga1;
	        length1 = d1.Length();
	        Vector2 d2 = anchor2 - ga2;
	        length2 = d2.Length();
	        ratio = r;
	        Debug.Assert(ratio > Settings.b2_FLT_EPSILON);
	        float C = length1 + ratio * length2;
	        maxLength1 = C - ratio * b2_minPulleyLength;
	        maxLength2 = (C - b2_minPulleyLength) / ratio;
        }

	    /// The first ground anchor in world coordinates. This point never moves.
	    public Vector2 groundAnchor1;

	    /// The second ground anchor in world coordinates. This point never moves.
	    public Vector2 groundAnchor2;

	    /// The local anchor point relative to body1's origin.
	    public Vector2 localAnchor1;

	    /// The local anchor point relative to body2's origin.
	    public Vector2 localAnchor2;

	    /// The a reference length for the segment attached to body1.
	    public float length1;

	    /// The maximum length of the segment attached to body1.
	    public float maxLength1;

	    /// The a reference length for the segment attached to body2.
	    public float length2;

	    /// The maximum length of the segment attached to body2.
	    public float maxLength2;

	    /// The pulley ratio, used to simulate a block-and-tackle.
	    public float ratio;
    };

    /// The pulley joint is connected to two bodies and two fixed ground points.
    /// The pulley supports a ratio such that:
    /// length1 + ratio * length2 <= ant
    /// Yes, the force transmitted is scaled by the ratio.
    /// The pulley also enforces a maximum length limit on both sides. This is
    /// useful to prevent one side of the pulley hitting the top.
    public class PulleyJoint : Joint
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
            Vector2 P = _impulse * _u2;
	        return inv_dt * P;
        }

	    public override float GetReactionTorque(float inv_dt)
        {
            return 0.0f;
        }

	    /// Get the first ground anchor.
	    public Vector2 GetGroundAnchor1()
        {
            return _groundAnchor1;
        }

	    /// Get the second ground anchor.
	    public Vector2 GetGroundAnchor2()
        {
            return _groundAnchor2;
        }

	    /// Get the current length of the segment attached to body1.
	    public float GetLength1()
        {
            Vector2 p = _bodyA.GetWorldPoint(_localAnchor1);
	        Vector2 s = _groundAnchor1;
	        Vector2 d = p - s;
	        return d.Length();
        }

	    /// Get the current length of the segment attached to body2.
	    public float GetLength2()
        {
            Vector2 p = _bodyB.GetWorldPoint(_localAnchor2);
	        Vector2 s = _groundAnchor2;
	        Vector2 d = p - s;
	        return d.Length();
        }

	    /// Get the pulley ratio.
	    public float GetRatio()
        {
            return _ratio;
        }

	    internal PulleyJoint(PulleyJointDef def)
            : base (def)
        {
	        _groundAnchor1 = def.groundAnchor1;
	        _groundAnchor2 = def.groundAnchor2;
	        _localAnchor1 = def.localAnchor1;
	        _localAnchor2 = def.localAnchor2;

	        Debug.Assert(def.ratio != 0.0f);
	        _ratio = def.ratio;

	        _ant = def.length1 + _ratio * def.length2;

            _maxLength1 = Math.Min(def.maxLength1, _ant - _ratio * PulleyJointDef.b2_minPulleyLength);
            _maxLength2 = Math.Min(def.maxLength2, (_ant - PulleyJointDef.b2_minPulleyLength) / _ratio);

	        _impulse = 0.0f;
	        _limitImpulse1 = 0.0f;
	        _limitImpulse2 = 0.0f;
        }

	    internal override void InitVelocityConstraints(ref TimeStep step)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

	        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
	        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

	        Vector2 p1 = b1._sweep.c + r1;
	        Vector2 p2 = b2._sweep.c + r2;

	        Vector2 s1 = _groundAnchor1;
	        Vector2 s2 = _groundAnchor2;

	        // Get the pulley axes.
	        _u1 = p1 - s1;
	        _u2 = p2 - s2;

	        float length1 = _u1.Length();
	        float length2 = _u2.Length();

	        if (length1 > Settings.b2_linearSlop)
	        {
		        _u1 *= 1.0f / length1;
	        }
	        else
	        {
		        _u1 = Vector2.Zero;
	        }

	        if (length2 > Settings.b2_linearSlop)
	        {
		        _u2 *= 1.0f / length2;
	        }
	        else
	        {
		        _u2 = Vector2.Zero;
	        }

	        float C = _ant - length1 - _ratio * length2;
	        if (C > 0.0f)
	        {
		        _state = LimitState.Inactive;
		        _impulse = 0.0f;
	        }
	        else
	        {
		        _state = LimitState.AtUpper;
	        }

	        if (length1 < _maxLength1)
	        {
		        _limitState1 = LimitState.Inactive;
		        _limitImpulse1 = 0.0f;
	        }
	        else
	        {
		        _limitState1 = LimitState.AtUpper;
	        }

	        if (length2 < _maxLength2)
	        {
		        _limitState2 = LimitState.Inactive;
		        _limitImpulse2 = 0.0f;
	        }
	        else
	        {
		        _limitState2 = LimitState.AtUpper;
	        }

	        // Compute effective mass.
	        float cr1u1 = MathUtils.Cross(r1, _u1);
	        float cr2u2 = MathUtils.Cross(r2, _u2);

	        _limitMass1 = b1._invMass + b1._invI * cr1u1 * cr1u1;
	        _limitMass2 = b2._invMass + b2._invI * cr2u2 * cr2u2;
	        _pulleyMass = _limitMass1 + _ratio * _ratio * _limitMass2;
	        Debug.Assert(_limitMass1 > Settings.b2_FLT_EPSILON);
	        Debug.Assert(_limitMass2 > Settings.b2_FLT_EPSILON);
	        Debug.Assert(_pulleyMass > Settings.b2_FLT_EPSILON);
	        _limitMass1 = 1.0f / _limitMass1;
	        _limitMass2 = 1.0f / _limitMass2;
	        _pulleyMass = 1.0f / _pulleyMass;

	        if (step.warmStarting)
	        {
		        // Scale impulses to support variable time steps.
		        _impulse *= step.dtRatio;
		        _limitImpulse1 *= step.dtRatio;
		        _limitImpulse2 *= step.dtRatio;

		        // Warm starting.
		        Vector2 P1 = -(_impulse + _limitImpulse1) * _u1;
		        Vector2 P2 = (-_ratio * _impulse - _limitImpulse2) * _u2;
		        b1._linearVelocity += b1._invMass * P1;
		        b1._angularVelocity += b1._invI * MathUtils.Cross(r1, P1);
		        b2._linearVelocity += b2._invMass * P2;
		        b2._angularVelocity += b2._invI * MathUtils.Cross(r2, P2);
	        }
	        else
	        {
		        _impulse = 0.0f;
		        _limitImpulse1 = 0.0f;
		        _limitImpulse2 = 0.0f;
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

	        if (_state == LimitState.AtUpper)
	        {
		        Vector2 v1 = b1._linearVelocity + MathUtils.Cross(b1._angularVelocity, r1);
		        Vector2 v2 = b2._linearVelocity + MathUtils.Cross(b2._angularVelocity, r2);

		        float Cdot = -Vector2.Dot(_u1, v1) - _ratio * Vector2.Dot(_u2, v2);
		        float impulse = _pulleyMass * (-Cdot);
		        float oldImpulse = _impulse;
		        _impulse = Math.Max(0.0f, _impulse + impulse);
		        impulse = _impulse - oldImpulse;

		        Vector2 P1 = -impulse * _u1;
		        Vector2 P2 = -_ratio * impulse * _u2;
		        b1._linearVelocity += b1._invMass * P1;
		        b1._angularVelocity += b1._invI * MathUtils.Cross(r1, P1);
		        b2._linearVelocity += b2._invMass * P2;
		        b2._angularVelocity += b2._invI * MathUtils.Cross(r2, P2);
	        }

	        if (_limitState1 == LimitState.AtUpper)
	        {
		        Vector2 v1 = b1._linearVelocity + MathUtils.Cross(b1._angularVelocity, r1);

		        float Cdot = -Vector2.Dot(_u1, v1);
		        float impulse = -_limitMass1 * Cdot;
		        float oldImpulse = _limitImpulse1;
		        _limitImpulse1 = Math.Max(0.0f, _limitImpulse1 + impulse);
		        impulse = _limitImpulse1 - oldImpulse;

		        Vector2 P1 = -impulse * _u1;
		        b1._linearVelocity += b1._invMass * P1;
		        b1._angularVelocity += b1._invI * MathUtils.Cross(r1, P1);
	        }

	        if (_limitState2 == LimitState.AtUpper)
	        {
		        Vector2 v2 = b2._linearVelocity + MathUtils.Cross(b2._angularVelocity, r2);

		        float Cdot = -Vector2.Dot(_u2, v2);
		        float impulse = -_limitMass2 * Cdot;
		        float oldImpulse = _limitImpulse2;
		        _limitImpulse2 = Math.Max(0.0f, _limitImpulse2 + impulse);
		        impulse = _limitImpulse2 - oldImpulse;

		        Vector2 P2 = -impulse * _u2;
		        b2._linearVelocity += b2._invMass * P2;
		        b2._angularVelocity += b2._invI * MathUtils.Cross(r2, P2);
	        }
        }

	    internal override bool SolvePositionConstraints(float baumgarte)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        Vector2 s1 = _groundAnchor1;
	        Vector2 s2 = _groundAnchor2;

	        float linearError = 0.0f;

	        if (_state == LimitState.AtUpper)
	        {
                XForm xf1, xf2;
                b1.GetXForm(out xf1);
                b2.GetXForm(out xf2);

		        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
		        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

		        Vector2 p1 = b1._sweep.c + r1;
		        Vector2 p2 = b2._sweep.c + r2;

		        // Get the pulley axes.
		        _u1 = p1 - s1;
		        _u2 = p2 - s2;

		        float length1 = _u1.Length();
		        float length2 = _u2.Length();

		        if (length1 > Settings.b2_linearSlop)
		        {
			        _u1 *= 1.0f / length1;
		        }
		        else
		        {
			        _u1 = Vector2.Zero;
		        }

		        if (length2 > Settings.b2_linearSlop)
		        {
			        _u2 *= 1.0f / length2;
		        }
		        else
		        {
			        _u2 = Vector2.Zero;
		        }

		        float C = _ant - length1 - _ratio * length2;
		        linearError = Math.Max(linearError, -C);

		        C = MathUtils.Clamp(C + Settings.b2_linearSlop, -Settings.b2_maxLinearCorrection, 0.0f);
		        float impulse = -_pulleyMass * C;

		        Vector2 P1 = -impulse * _u1;
		        Vector2 P2 = -_ratio * impulse * _u2;

		        b1._sweep.c += b1._invMass * P1;
		        b1._sweep.a += b1._invI * MathUtils.Cross(r1, P1);
		        b2._sweep.c += b2._invMass * P2;
		        b2._sweep.a += b2._invI * MathUtils.Cross(r2, P2);

		        b1.SynchronizeTransform();
		        b2.SynchronizeTransform();
	        }

	        if (_limitState1 == LimitState.AtUpper)
	        {
                XForm xf1;
                b1.GetXForm(out xf1);

		        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
		        Vector2 p1 = b1._sweep.c + r1;

		        _u1 = p1 - s1;
		        float length1 = _u1.Length();

		        if (length1 > Settings.b2_linearSlop)
		        {
			        _u1 *= 1.0f / length1;
		        }
		        else
		        {
			        _u1 = Vector2.Zero;
		        }

		        float C = _maxLength1 - length1;
		        linearError = Math.Max(linearError, -C);
		        C = MathUtils.Clamp(C + Settings.b2_linearSlop, -Settings.b2_maxLinearCorrection, 0.0f);
		        float impulse = -_limitMass1 * C;

		        Vector2 P1 = -impulse * _u1;
		        b1._sweep.c += b1._invMass * P1;
		        b1._sweep.a += b1._invI * MathUtils.Cross(r1, P1);

		        b1.SynchronizeTransform();
	        }

	        if (_limitState2 == LimitState.AtUpper)
	        {
                XForm xf2;
                b2.GetXForm(out xf2);

		        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());
		        Vector2 p2 = b2._sweep.c + r2;

		        _u2 = p2 - s2;
		        float length2 = _u2.Length();

		        if (length2 > Settings.b2_linearSlop)
		        {
			        _u2 *= 1.0f / length2;
		        }
		        else
		        {
			        _u2 = Vector2.Zero;
		        }

		        float C = _maxLength2 - length2;
		        linearError = Math.Max(linearError, -C);
		        C = MathUtils.Clamp(C + Settings.b2_linearSlop, -Settings.b2_maxLinearCorrection, 0.0f);
		        float impulse = -_limitMass2 * C;

		        Vector2 P2 = -impulse * _u2;
		        b2._sweep.c += b2._invMass * P2;
		        b2._sweep.a += b2._invI * MathUtils.Cross(r2, P2);

		        b2.SynchronizeTransform();
	        }

	        return linearError < Settings.b2_linearSlop;
        }

	    internal Vector2 _groundAnchor1;
	    internal Vector2 _groundAnchor2;
	    internal Vector2 _localAnchor1;
	    internal Vector2 _localAnchor2;

	    internal Vector2 _u1;
	    internal Vector2 _u2;
    	
	    internal float _ant;
	    internal float _ratio;
    	
	    internal float _maxLength1;
	    internal float _maxLength2;

	    // Effective masses
	    internal float _pulleyMass;
	    internal float _limitMass1;
	    internal float _limitMass2;

	    // Impulses for accumulation/warm starting.
	    internal float _impulse;
	    internal float _limitImpulse1;
	    internal float _limitImpulse2;

	    internal LimitState _state;
	    internal LimitState _limitState1;
	    internal LimitState _limitState2;
    };
}
