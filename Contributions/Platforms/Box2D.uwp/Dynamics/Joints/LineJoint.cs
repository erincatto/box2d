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
    /// Line joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the raint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
    public class LineJointDef : JointDef
    {
	    public LineJointDef()
	    {
		    type = JointType.Line;
		    localAnchor1 = Vector2.Zero;
		    localAnchor2 = Vector2.Zero;
		    localAxis1 = new Vector2(1.0f, 0.0f);
		    enableLimit = false;
		    lowerTranslation = 0.0f;
		    upperTranslation = 0.0f;
		    enableMotor = false;
		    maxMotorForce = 0.0f;
		    motorSpeed = 0.0f;
	    }

	    /// Initialize the bodies, anchors, axis, and reference angle using the world
	    /// anchor and world axis.
        public void Initialize(Body b1, Body b2, Vector2 anchor, Vector2 axis)
        {
	        body1 = b1;
	        body2 = b2;
	        localAnchor1 = body1.GetLocalPoint(anchor);
	        localAnchor2 = body2.GetLocalPoint(anchor);
	        localAxis1 = body1.GetLocalVector(axis);
        }

	    /// The local anchor point relative to body1's origin.
	    public Vector2 localAnchor1;

	    /// The local anchor point relative to body2's origin.
	    public Vector2 localAnchor2;

	    /// The local translation axis in body1.
	    public Vector2 localAxis1;

	    /// Enable/disable the joint limit.
	    public bool enableLimit;

	    /// The lower translation limit, usually in meters.
	    public float lowerTranslation;

	    /// The upper translation limit, usually in meters.
	    public float upperTranslation;

	    /// Enable/disable the joint motor.
	    public bool enableMotor;

	    /// The maximum motor torque, usually in N-m.
	    public float maxMotorForce;

	    /// The desired motor speed in radians per second.
	    public float motorSpeed;
    };

    /// A line joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in body1. You can use a joint limit to restrict
    /// the range of motion and a joint motor to drive the motion or to
    /// model joint friction.
    public class LineJoint : Joint
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
            return inv_dt * (_impulse.X * _perp + (_motorImpulse + _impulse.Y) * _axis);
        }

        public override float GetReactionTorque(float inv_dt)
        {
	        return 0.0f;
        }

	    /// Get the current joint translation, usually in meters.
	    public float GetJointTranslation()
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        Vector2 p1 = b1.GetWorldPoint(_localAnchor1);
	        Vector2 p2 = b2.GetWorldPoint(_localAnchor2);
	        Vector2 d = p2 - p1;
	        Vector2 axis = b1.GetWorldVector(_localXAxis1);

	        float translation = Vector2.Dot(d, axis);
	        return translation;
        }

	    /// Get the current joint translation speed, usually in meters per second.
	    public float GetJointSpeed()
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
	        Vector2 d = p2 - p1;
	        Vector2 axis = b1.GetWorldVector(_localXAxis1);

	        Vector2 v1 = b1._linearVelocity;
	        Vector2 v2 = b2._linearVelocity;
	        float w1 = b1._angularVelocity;
	        float w2 = b2._angularVelocity;

	        float speed = Vector2.Dot(d, MathUtils.Cross(w1, axis)) + Vector2.Dot(axis, v2 + MathUtils.Cross(w2, r2) - v1 - MathUtils.Cross(w1, r1));
	        return speed;
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

	    /// Get the lower joint limit, usually in meters.
	    public float GetLowerLimit()
        {
            return _lowerTranslation;
        }

	    /// Get the upper joint limit, usually in meters.
	    public float GetUpperLimit()
        {
            return _upperTranslation;
        }

	    /// Set the joint limits, usually in meters.
	    public void SetLimits(float lower, float upper)
        {
	        Debug.Assert(lower <= upper);
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _lowerTranslation = lower;
	        _upperTranslation = upper;
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

	    /// Set the motor speed, usually in meters per second.
	    public void SetMotorSpeed(float speed)
        {
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _motorSpeed = speed;
        }

	    /// Get the motor speed, usually in meters per second.
	    public float GetMotorSpeed()
        {
            return _motorSpeed;
        }

	    /// Set the maximum motor force, usually in N.
	    public void SetMaxMotorForce(float force)
        {
	        _bodyA.WakeUp();
	        _bodyB.WakeUp();
	        _maxMotorForce = force;
        }

	    /// Get the current motor force, usually in N.
	    public float GetMotorForce()
        {
	        return _motorImpulse;
        }

	    internal LineJoint(LineJointDef def)
            : base (def)
        {
	        _localAnchor1 = def.localAnchor1;
	        _localAnchor2 = def.localAnchor2;
	        _localXAxis1 = def.localAxis1;
	        _localYAxis1 = MathUtils.Cross(1.0f, _localXAxis1);

	        _impulse = Vector2.Zero;
	        _motorMass = 0.0f;
	        _motorImpulse = 0.0f;

	        _lowerTranslation = def.lowerTranslation;
	        _upperTranslation = def.upperTranslation;
	        _maxMotorForce = def.maxMotorForce;
	        _motorSpeed = def.motorSpeed;
	        _enableLimit = def.enableLimit;
	        _enableMotor = def.enableMotor;
	        _limitState = LimitState.Inactive;

	        _axis = Vector2.Zero;
	        _perp = Vector2.Zero;
        }

	    internal override void InitVelocityConstraints(ref TimeStep step)
        {
            Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        _localCenter1 = b1.GetLocalCenter();
	        _localCenter2 = b2.GetLocalCenter();

            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

	        // Compute the effective masses.
	        Vector2 r1 = MathUtils.Multiply(ref xf1.R, _localAnchor1 - _localCenter1);
	        Vector2 r2 = MathUtils.Multiply(ref xf2.R, _localAnchor2 - _localCenter2);
	        Vector2 d = b2._sweep.c + r2 - b1._sweep.c - r1;

	        _invMass1 = b1._invMass;
	        _invI1 = b1._invI;
	        _invMass2 = b2._invMass;
	        _invI2 = b2._invI;

	        // Compute motor Jacobian and effective mass.
	        {
		        _axis = MathUtils.Multiply(ref xf1.R, _localXAxis1);
		        _a1 = MathUtils.Cross(d + r1, _axis);
		        _a2 = MathUtils.Cross(r2, _axis);

		        _motorMass = _invMass1 + _invMass2 + _invI1 * _a1 * _a1 + _invI2 * _a2 * _a2;
		        Debug.Assert(_motorMass > Settings.b2_FLT_EPSILON);
		        _motorMass = 1.0f / _motorMass;
	        }

	        // Prismatic raint.
	        {
		        _perp = MathUtils.Multiply(ref xf1.R, _localYAxis1);

		        _s1 = MathUtils.Cross(d + r1, _perp);
		        _s2 = MathUtils.Cross(r2, _perp);

		        float m1 = _invMass1, m2 = _invMass2;
		        float i1 = _invI1, i2 = _invI2;

		        float k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;
		        float k12 = i1 * _s1 * _a1 + i2 * _s2 * _a2;
		        float k22 = m1 + m2 + i1 * _a1 * _a1 + i2 * _a2 * _a2;

		        _K.col1 = new Vector2(k11, k12);
                _K.col2 = new Vector2(k12, k22);
	        }

	        // Compute motor and limit terms.
	        if (_enableLimit)
	        {
		        float jointTranslation = Vector2.Dot(_axis, d);
		        if (Math.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.b2_linearSlop)
		        {
			        _limitState = LimitState.Equal;
		        }
		        else if (jointTranslation <= _lowerTranslation)
		        {
			        if (_limitState != LimitState.AtLower)
			        {
				        _limitState = LimitState.AtLower;
				        _impulse.Y = 0.0f;
			        }
		        }
		        else if (jointTranslation >= _upperTranslation)
		        {
			        if (_limitState != LimitState.AtUpper)
			        {
				        _limitState = LimitState.AtUpper;
				        _impulse.Y = 0.0f;
			        }
		        }
		        else
		        {
			        _limitState = LimitState.Inactive;
			        _impulse.Y = 0.0f;
		        }
	        }
	        else
	        {
		        _limitState = LimitState.Inactive;
	        }

	        if (_enableMotor == false)
	        {
		        _motorImpulse = 0.0f;
	        }

	        if (step.warmStarting)
	        {
		        // Account for variable time step.
		        _impulse *= step.dtRatio;
		        _motorImpulse *= step.dtRatio;

		        Vector2 P = _impulse.X * _perp + (_motorImpulse + _impulse.Y) * _axis;
		        float L1 = _impulse.X * _s1 + (_motorImpulse + _impulse.Y) * _a1;
		        float L2 = _impulse.X * _s2 + (_motorImpulse + _impulse.Y) * _a2;

		        b1._linearVelocity -= _invMass1 * P;
		        b1._angularVelocity -= _invI1 * L1;

		        b2._linearVelocity += _invMass2 * P;
		        b2._angularVelocity += _invI2 * L2;
	        }
	        else
	        {
		        _impulse = Vector2.Zero;
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

	        // Solve linear motor raint.
	        if (_enableMotor && _limitState != LimitState.Equal)
	        {
		        float Cdot = Vector2.Dot(_axis, v2 - v1) + _a2 * w2 - _a1 * w1;
		        float impulse = _motorMass * (_motorSpeed - Cdot);
		        float oldImpulse = _motorImpulse;
		        float maxImpulse = step.dt * _maxMotorForce;
		        _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
		        impulse = _motorImpulse - oldImpulse;

		        Vector2 P = impulse * _axis;
		        float L1 = impulse * _a1;
		        float L2 = impulse * _a2;

		        v1 -= _invMass1 * P;
		        w1 -= _invI1 * L1;

		        v2 += _invMass2 * P;
		        w2 += _invI2 * L2;
	        }

	        float Cdot1 = Vector2.Dot(_perp, v2 - v1) + _s2 * w2 - _s1 * w1;

	        if (_enableLimit && _limitState != LimitState.Inactive)
	        {
		        // Solve prismatic and limit raint in block form.
		        float Cdot2 = Vector2.Dot(_axis, v2 - v1) + _a2 * w2 - _a1 * w1;
		        Vector2 Cdot = new Vector2(Cdot1, Cdot2);

		        Vector2 f1 = _impulse;
		        Vector2 df =  _K.Solve(-Cdot);
		        _impulse += df;

		        if (_limitState == LimitState.AtLower)
		        {
			        _impulse.Y = Math.Max(_impulse.Y, 0.0f);
		        }
		        else if (_limitState == LimitState.AtUpper)
		        {
                    _impulse.Y = Math.Min(_impulse.Y, 0.0f);
		        }

		        // f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
		        float b = -Cdot1 - (_impulse.Y - f1.Y) * _K.col2.X;
		        float f2r = b / _K.col1.X + f1.X;
		        _impulse.X = f2r;

		        df = _impulse - f1;

		        Vector2 P = df.X * _perp + df.Y * _axis;
		        float L1 = df.X * _s1 + df.Y * _a1;
		        float L2 = df.X * _s2 + df.Y * _a2;

		        v1 -= _invMass1 * P;
		        w1 -= _invI1 * L1;

		        v2 += _invMass2 * P;
		        w2 += _invI2 * L2;
	        }
	        else
	        {
		        // Limit is inactive, just solve the prismatic raint in block form.
		        float df = (-Cdot1) / _K.col1.X;
		        _impulse.X += df;

		        Vector2 P = df * _perp;
		        float L1 = df * _s1;
		        float L2 = df * _s2;

		        v1 -= _invMass1 * P;
		        w1 -= _invI1 * L1;

		        v2 += _invMass2 * P;
		        w2 += _invI2 * L2;
	        }

	        b1._linearVelocity = v1;
	        b1._angularVelocity = w1;
	        b2._linearVelocity = v2;
	        b2._angularVelocity = w2;
        }

	    internal override bool SolvePositionConstraints(float baumgarte)
        {
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        Vector2 c1 = b1._sweep.c;
	        float a1 = b1._sweep.a;

	        Vector2 c2 = b2._sweep.c;
	        float a2 = b2._sweep.a;

	        // Solve linear limit raint.
	        float linearError = 0.0f, angularError = 0.0f;
	        bool active = false;
	        float C2 = 0.0f;

            Mat22 R1 = new Mat22(a1);
            Mat22 R2 = new Mat22(a2);

	        Vector2 r1 = MathUtils.Multiply(ref R1, _localAnchor1 - _localCenter1);
	        Vector2 r2 = MathUtils.Multiply(ref R2, _localAnchor2 - _localCenter2);
	        Vector2 d = c2 + r2 - c1 - r1;

	        if (_enableLimit)
	        {
		        _axis = MathUtils.Multiply(ref R1, _localXAxis1);

		        _a1 = MathUtils.Cross(d + r1, _axis);
		        _a2 = MathUtils.Cross(r2, _axis);

		        float translation = Vector2.Dot(_axis, d);
		        if (Math.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.b2_linearSlop)
		        {
			        // Prevent large angular corrections
			        C2 = MathUtils.Clamp(translation, -Settings.b2_maxLinearCorrection, Settings.b2_maxLinearCorrection);
			        linearError = Math.Abs(translation);
			        active = true;
		        }
		        else if (translation <= _lowerTranslation)
		        {
			        // Prevent large linear corrections and allow some slop.
			        C2 = MathUtils.Clamp(translation - _lowerTranslation + Settings.b2_linearSlop, -Settings.b2_maxLinearCorrection, 0.0f);
			        linearError = _lowerTranslation - translation;
			        active = true;
		        }
		        else if (translation >= _upperTranslation)
		        {
			        // Prevent large linear corrections and allow some slop.
			        C2 = MathUtils.Clamp(translation - _upperTranslation - Settings.b2_linearSlop, 0.0f, Settings.b2_maxLinearCorrection);
			        linearError = translation - _upperTranslation;
			        active = true;
		        }
	        }

	        _perp = MathUtils.Multiply(ref R1, _localYAxis1);

	        _s1 = MathUtils.Cross(d + r1, _perp);
	        _s2 = MathUtils.Cross(r2, _perp);

	        Vector2 impulse;
	        float C1;
	        C1 = Vector2.Dot(_perp, d);

	        linearError = Math.Max(linearError, Math.Abs(C1));
	        angularError = 0.0f;

	        if (active)
	        {
		        float m1 = _invMass1, m2 = _invMass2;
		        float i1 = _invI1, i2 = _invI2;

		        float k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;
		        float k12 = i1 * _s1 * _a1 + i2 * _s2 * _a2;
		        float k22 = m1 + m2 + i1 * _a1 * _a1 + i2 * _a2 * _a2;

                _K.col1 = new Vector2(k11, k12);
		        _K.col2 = new Vector2(k12, k22);

		        Vector2 C = new Vector2(-C1, -C2);

		        impulse = _K.Solve(C); //note i inverted above
	        }
	        else
	        {
		        float m1 = _invMass1, m2 = _invMass2;
		        float i1 = _invI1, i2 = _invI2;

		        float k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;

		        float impulse1 = (-C1) / k11;
		        impulse.X = impulse1;
		        impulse.Y = 0.0f;
	        }

	        Vector2 P = impulse.X * _perp + impulse.Y * _axis;
	        float L1 = impulse.X * _s1 + impulse.Y * _a1;
	        float L2 = impulse.X * _s2 + impulse.Y * _a2;

	        c1 -= _invMass1 * P;
	        a1 -= _invI1 * L1;
	        c2 += _invMass2 * P;
	        a2 += _invI2 * L2;

	        // TODO_ERIN remove need for this.
	        b1._sweep.c = c1;
	        b1._sweep.a = a1;
	        b2._sweep.c = c2;
	        b2._sweep.a = a2;
	        b1.SynchronizeTransform();
	        b2.SynchronizeTransform();

	        return linearError <= Settings.b2_linearSlop && angularError <= Settings.b2_angularSlop;
        }

	    internal Vector2 _localAnchor1;
	    internal Vector2 _localAnchor2;
	    internal Vector2 _localXAxis1;
	    internal Vector2 _localYAxis1;

	    internal Vector2 _axis, _perp;
	    internal float _s1, _s2;
	    internal float _a1, _a2;

	    internal Mat22 _K;
	    internal Vector2 _impulse;

	    internal float _motorMass;			// effective mass for motor/limit translational raint.
	    internal float _motorImpulse;

	    internal float _lowerTranslation;
	    internal float _upperTranslation;
	    internal float _maxMotorForce;
	    internal float _motorSpeed;

	    internal bool _enableLimit;
	    internal bool _enableMotor;
	    internal LimitState _limitState;
    };
}
