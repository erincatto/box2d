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

using System.Diagnostics;
using System;
using System.Numerics;

namespace Box2D.UWP
{
    /// Revolute joint definition. This requires defining an
    /// anchor point where the bodies are joined. The definition
    /// uses local anchor points so that the initial configuration
    /// can violate the raint slightly. You also need to
    /// specify the initial relative angle for joint limits. This
    /// helps when saving and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be.
    /// 2. if you add/remove shapes from a body and recompute the mass,
    ///    the joints will be broken.
    public class RevoluteJointDef : JointDef
    {
	    public RevoluteJointDef()
	    {
		    type = JointType.Revolute;
		    localAnchor1 = new Vector2(0.0f, 0.0f);
		    localAnchor2 = new Vector2(0.0f, 0.0f);
		    referenceAngle = 0.0f;
		    lowerAngle = 0.0f;
		    upperAngle = 0.0f;
		    maxMotorTorque = 0.0f;
		    motorSpeed = 0.0f;
		    enableLimit = false;
		    enableMotor = false;
	    }

	    /// Initialize the bodies, anchors, and reference angle using the world
	    /// anchor.
        public void Initialize(Body b1, Body b2, Vector2 anchor)
        {
	        body1 = b1;
	        body2 = b2;
	        localAnchor1 = body1.GetLocalPoint(anchor);
	        localAnchor2 = body2.GetLocalPoint(anchor);
	        referenceAngle = body2.GetAngle() - body1.GetAngle();
        }

	    /// The local anchor point relative to body1's origin.
	    public Vector2 localAnchor1;

	    /// The local anchor point relative to body2's origin.
	    public Vector2 localAnchor2;

	    /// The body2 angle minus body1 angle in the reference state (radians).
	    public float referenceAngle;

	    /// A flag to enable joint limits.
	    public bool enableLimit;

	    /// The lower angle for the joint limit (radians).
	    public float lowerAngle;

	    /// The upper angle for the joint limit (radians).
	    public float upperAngle;

	    /// A flag to enable the joint motor.
	    public bool enableMotor;

	    /// The desired motor speed. Usually in radians per second.
	    public float motorSpeed;

	    /// The maximum motor torque used to achieve the desired motor speed.
	    /// Usually in N-m.
	    public float maxMotorTorque;
    };

    /// A revolute joint rains to bodies to share a common point while they
    /// are free to rotate about the point. The relative rotation about the shared
    /// point is the joint angle. You can limit the relative rotation with
    /// a joint limit that specifies a lower and upper angle. You can use a motor
    /// to drive the relative rotation about the shared point. A maximum motor torque
    /// is provided so that infinite forces are not generated.
    public class RevoluteJoint : Joint
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
            Vector2 P = new Vector2(_impulse.X, _impulse.Y);
	        return inv_dt * P;
        }

	    public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * _impulse.Z;
        }

	    /// Get the current joint angle in radians.
	    public float GetJointAngle()
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;
	        return b2._sweep.a - b1._sweep.a - _referenceAngle;
        }

	    /// Get the current joint angle speed in radians per second.
	    public float GetJointSpeed()
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;
	        return b2._angularVelocity - b1._angularVelocity;
        }

	    /// Is the joint limit enabled?
	    public bool IsLimitEnabled()
        {
	        return _enableLimit;
        }

	    /// Enable/disable the joint limit.
	    public void EnableLimit(bool flag)
        {
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _enableLimit = flag;
        }

	    /// Get the lower joint limit in radians.
	    public float GetLowerLimit()
        {
            return _lowerAngle;
        }

	    /// Get the upper joint limit in radians.
	    public float GetUpperLimit()
        {
            return _upperAngle;
        }

	    /// Set the joint limits in radians.
	    public void SetLimits(float lower, float upper)
        {
	        Debug.Assert(lower <= upper);
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _lowerAngle = lower;
	        _upperAngle = upper;
        }

	    /// Is the joint motor enabled?
	    public bool IsMotorEnabled()
        {
            return _enableMotor;
        }

	    /// Enable/disable the joint motor.
	    public void EnableMotor(bool flag)
        {
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _enableMotor = flag;
        }

	    /// Set the motor speed in radians per second.
	    public void SetMotorSpeed(float speed)
        {
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _motorSpeed = speed;
        }

	    /// Get the motor speed in radians per second.
	    public float GetMotorSpeed()
        {
            return _motorSpeed;
        }

	    /// Set the maximum motor torque, usually in N-m.
	    public void SetMaxMotorTorque(float torque)
        {
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _maxMotorTorque = torque;
        }

	    /// Get the current motor torque, usually in N-m.
	    public float GetMotorTorque()
        {
	        return _motorImpulse;
        }

        internal RevoluteJoint(RevoluteJointDef def)
            : base (def)
        {
	        _localAnchor1 = def.localAnchor1;
	        _localAnchor2 = def.localAnchor2;
	        _referenceAngle = def.referenceAngle;

	        _impulse = Vector3.Zero;
	        _motorImpulse = 0.0f;

	        _lowerAngle = def.lowerAngle;
	        _upperAngle = def.upperAngle;
	        _maxMotorTorque = def.maxMotorTorque;
	        _motorSpeed = def.motorSpeed;
	        _enableLimit = def.enableLimit;
	        _enableMotor = def.enableMotor;
	        _limitState = LimitState.Inactive;
        }

	    internal override void InitVelocityConstraints(ref TimeStep step)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        if (_enableMotor || _enableLimit)
	        {
		        // You cannot create a rotation limit between bodies that
		        // both have fixed rotation.
		        Debug.Assert(b1._invI > 0.0f || b2._invI > 0.0f);
	        }

	        // Compute the effective mass matrix.
            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

	        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
	        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

	        // J = [-I -r1_skew I r2_skew]
	        //     [ 0       -1 0       1]
	        // r_skew = [-ry; rx]

	        // Matlab
	        // K = [ m1+r1y^2*i1+m2+r2y^2*i2,  -r1y*i1*r1x-r2y*i2*r2x,          -r1y*i1-r2y*i2]
	        //     [  -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2,           r1x*i1+r2x*i2]
	        //     [          -r1y*i1-r2y*i2,           r1x*i1+r2x*i2,                   i1+i2]

	        float m1 = b1._invMass, m2 = b2._invMass;
	        float i1 = b1._invI, i2 = b2._invI;

	        _mass.col1.X = m1 + m2 + r1.Y * r1.Y * i1 + r2.Y * r2.Y * i2;
	        _mass.col2.X = -r1.Y * r1.X * i1 - r2.Y * r2.X * i2;
	        _mass.col3.X = -r1.Y * i1 - r2.Y * i2;
	        _mass.col1.Y = _mass.col2.X;
	        _mass.col2.Y = m1 + m2 + r1.X * r1.X * i1 + r2.X * r2.X * i2;
	        _mass.col3.Y = r1.X * i1 + r2.X * i2;
	        _mass.col1.Z = _mass.col3.X;
	        _mass.col2.Z = _mass.col3.Y;
	        _mass.col3.Z = i1 + i2;

	        _motorMass = 1.0f / (i1 + i2);

	        if (_enableMotor == false)
	        {
		        _motorImpulse = 0.0f;
	        }

	        if (_enableLimit)
	        {
		        float jointAngle = b2._sweep.a - b1._sweep.a - _referenceAngle;
		        if (Math.Abs(_upperAngle - _lowerAngle) < 2.0f * Settings.b2_angularSlop)
		        {
			        _limitState = LimitState.Equal;
		        }
		        else if (jointAngle <= _lowerAngle)
		        {
			        if (_limitState != LimitState.AtLower)
			        {
				        _impulse.Z = 0.0f;
			        }
			        _limitState = LimitState.AtLower;
		        }
		        else if (jointAngle >= _upperAngle)
		        {
			        if (_limitState != LimitState.AtUpper)
			        {
				        _impulse.Z = 0.0f;
			        }
			        _limitState = LimitState.AtUpper;
		        }
		        else
		        {
			        _limitState = LimitState.Inactive;
			        _impulse.Z = 0.0f;
		        }
	        }
	        else
	        {
		        _limitState = LimitState.Inactive;
	        }

	        if (step.warmStarting)
	        {
		        // Scale impulses to support a variable time step.
		        _impulse *= step.dtRatio;
		        _motorImpulse *= step.dtRatio;

		        Vector2 P = new Vector2(_impulse.X, _impulse.Y);

		        b1._linearVelocity -= m1 * P;
		        b1._angularVelocity -= i1 * (MathUtils.Cross(r1, P) + _motorImpulse + _impulse.Z);

		        b2._linearVelocity += m2 * P;
		        b2._angularVelocity += i2 * (MathUtils.Cross(r2, P) + _motorImpulse + _impulse.Z);
	        }
	        else
	        {
		        _impulse = Vector3.Zero;
		        _motorImpulse = 0.0f;
	        }
        }

	    internal override void SolveVelocityConstraints(ref TimeStep step)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        Vector2 v1 = b1._linearVelocity;
	        float w1 = b1._angularVelocity;
	        Vector2 v2 = b2._linearVelocity;
	        float w2 = b2._angularVelocity;

	        float m1 = b1._invMass, m2 = b2._invMass;
	        float i1 = b1._invI, i2 = b2._invI;

	        // Solve motor raint.
	        if (_enableMotor && _limitState != LimitState.Equal)
	        {
		        float Cdot = w2 - w1 - _motorSpeed;
		        float impulse = _motorMass * (-Cdot);
		        float oldImpulse = _motorImpulse;
		        float maxImpulse = step.dt * _maxMotorTorque;
		        _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
		        impulse = _motorImpulse - oldImpulse;

		        w1 -= i1 * impulse;
		        w2 += i2 * impulse;
	        }

	        // Solve limit raint.
	        if (_enableLimit && _limitState != LimitState.Inactive)
	        {
                XForm xf1, xf2;
                b1.GetXForm(out xf1);
                b2.GetXForm(out xf2);

		        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
		        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

		        // Solve point-to-point raint
		        Vector2 Cdot1 = v2 + MathUtils.Cross(w2, r2) - v1 - MathUtils.Cross(w1, r1);
		        float Cdot2 = w2 - w1;
		        Vector3 Cdot = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

		        Vector3 impulse = _mass.Solve33(-Cdot);

		        if (_limitState == LimitState.Equal)
		        {
			        _impulse += impulse;
		        }
		        else if (_limitState == LimitState.AtLower)
		        {
			        float newImpulse = _impulse.Z + impulse.Z;
			        if (newImpulse < 0.0f)
			        {
				        Vector2 reduced = _mass.Solve22(-Cdot1);
				        impulse.X = reduced.X;
				        impulse.Y = reduced.Y;
				        impulse.Z = -_impulse.Z;
				        _impulse.X += reduced.X;
				        _impulse.Y += reduced.Y;
				        _impulse.Z = 0.0f;
			        }
		        }
		        else if (_limitState == LimitState.AtUpper)
		        {
			        float newImpulse = _impulse.Z + impulse.Z;
			        if (newImpulse > 0.0f)
			        {
				        Vector2 reduced = _mass.Solve22(-Cdot1);
				        impulse.X = reduced.X;
				        impulse.Y = reduced.Y;
				        impulse.Z = -_impulse.Z;
				        _impulse.X += reduced.X;
				        _impulse.Y += reduced.Y;
				        _impulse.Z = 0.0f;
			        }
		        }

		        Vector2 P = new Vector2(impulse.X, impulse.Y);

		        v1 -= m1 * P;
		        w1 -= i1 * (MathUtils.Cross(r1, P) + impulse.Z);

		        v2 += m2 * P;
		        w2 += i2 * (MathUtils.Cross(r2, P) + impulse.Z);
	        }
	        else
	        {
                XForm xf1, xf2;
                b1.GetXForm(out xf1);
                b2.GetXForm(out xf2);

		        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
		        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

		        // Solve point-to-point raint
		        Vector2 Cdot = v2 + MathUtils.Cross(w2, r2) - v1 - MathUtils.Cross(w1, r1);
		        Vector2 impulse = _mass.Solve22(-Cdot);

		        _impulse.X += impulse.X;
		        _impulse.Y += impulse.Y;

		        v1 -= m1 * impulse;
		        w1 -= i1 * MathUtils.Cross(r1, impulse);

		        v2 += m2 * impulse;
		        w2 += i2 * MathUtils.Cross(r2, impulse);
	        }

	        b1._linearVelocity = v1;
	        b1._angularVelocity = w1;
	        b2._linearVelocity = v2;
	        b2._angularVelocity = w2;
        }

	    internal override bool SolvePositionConstraints(float baumgarte)
        {
	        // TODO_ERIN block solve with limit. COME ON ERIN

	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        float angularError = 0.0f;
	        float positionError = 0.0f;

	        // Solve angular limit raint.
	        if (_enableLimit && _limitState != LimitState.Inactive)
	        {
		        float angle = b2._sweep.a - b1._sweep.a - _referenceAngle;
		        float limitImpulse = 0.0f;

		        if (_limitState == LimitState.Equal)
		        {
			        // Prevent large angular corrections
			        float C = MathUtils.Clamp(angle - _lowerAngle, -Settings.b2_maxAngularCorrection, Settings.b2_maxAngularCorrection);
			        limitImpulse = -_motorMass * C;
			        angularError = Math.Abs(C);
		        }
		        else if (_limitState == LimitState.AtLower)
		        {
			        float C = angle - _lowerAngle;
			        angularError = -C;

			        // Prevent large angular corrections and allow some slop.
			        C = MathUtils.Clamp(C + Settings.b2_angularSlop, -Settings.b2_maxAngularCorrection, 0.0f);
			        limitImpulse = -_motorMass * C;
		        }
		        else if (_limitState == LimitState.AtUpper)
		        {
			        float C = angle - _upperAngle;
			        angularError = C;

			        // Prevent large angular corrections and allow some slop.
			        C = MathUtils.Clamp(C - Settings.b2_angularSlop, 0.0f, Settings.b2_maxAngularCorrection);
			        limitImpulse = -_motorMass * C;
		        }

		        b1._sweep.a -= b1._invI * limitImpulse;
		        b2._sweep.a += b2._invI * limitImpulse;

		        b1.SynchronizeTransform();
		        b2.SynchronizeTransform();
	        }

	        // Solve point-to-point raint.
	        {
                XForm xf1, xf2;
                b1.GetXForm(out xf1);
                b2.GetXForm(out xf2);

		        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
		        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());

		        Vector2 C = b2._sweep.c + r2 - b1._sweep.c - r1;
		        positionError = C.Length();

		        float invMass1 = b1._invMass, invMass2 = b2._invMass;
		        float invI1 = b1._invI, invI2 = b2._invI;

		        // Handle large detachment.
		        float k_allowedStretch = 10.0f * Settings.b2_linearSlop;
		        if (C.LengthSquared() > k_allowedStretch * k_allowedStretch)
		        {
			        // Use a particle solution (no rotation).
			        Vector2 u = C; u.Normalize();
			        float k = invMass1 + invMass2;
			        Debug.Assert(k > Settings.b2_FLT_EPSILON);
			        float m = 1.0f / k;
			        Vector2 impulse2 = m * (-C);
			        float k_beta = 0.5f;
			        b1._sweep.c -= k_beta * invMass1 * impulse2;
			        b2._sweep.c += k_beta * invMass2 * impulse2;

			        C = b2._sweep.c + r2 - b1._sweep.c - r1;
		        }

                Mat22 K1 = new Mat22(new Vector2(invMass1 + invMass2, 0.0f), new Vector2(0.0f, invMass1 + invMass2));
		        Mat22 K2 = new Mat22(new Vector2(invI1 * r1.Y * r1.Y, -invI1 * r1.X * r1.Y), new Vector2(-invI1 * r1.X * r1.Y, invI1 * r1.X * r1.X));
                Mat22 K3 = new Mat22(new Vector2(invI2 * r2.Y * r2.Y, -invI2 * r2.X * r2.Y), new Vector2(-invI2 * r2.X * r2.Y, invI2 * r2.X * r2.X));

		        Mat22 Ka;
                Mat22 K;
                Mat22.Add(ref K1, ref K2, out Ka);
                Mat22.Add(ref Ka, ref K3, out K);

		        Vector2 impulse = K.Solve(-C);

		        b1._sweep.c -= b1._invMass * impulse;
		        b1._sweep.a -= b1._invI * MathUtils.Cross(r1, impulse);

		        b2._sweep.c += b2._invMass * impulse;
		        b2._sweep.a += b2._invI * MathUtils.Cross(r2, impulse);

		        b1.SynchronizeTransform();
		        b2.SynchronizeTransform();
	        }
        	
	        return positionError <= Settings.b2_linearSlop && angularError <= Settings.b2_angularSlop;
        }

        public Vector2 _localAnchor1;	// relative
        public Vector2 _localAnchor2;
        public Vector3 _impulse;
        public float _motorImpulse;

        public Mat33 _mass;			// effective mass for point-to-point raint.
        public float _motorMass;	// effective mass for motor/limit angular raint.

        public bool _enableMotor;
        public float _maxMotorTorque;
        public float _motorSpeed;

        public bool _enableLimit;
        public float _referenceAngle;
        public float _lowerAngle;
        public float _upperAngle;
        public LimitState _limitState;
    };
}
