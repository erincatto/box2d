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
    /// Prismatic joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the raint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
    public class PrismaticJointDef : JointDef
    {
	    public PrismaticJointDef()
	    {
		    type = JointType.Prismatic;
		    localAnchor1 = Vector2.Zero;
		    localAnchor2 = Vector2.Zero;
		    localAxis1 = new Vector2(1.0f, 0.0f);
		    referenceAngle = 0.0f;
		    enableLimit = false;
		    lowerTranslation = 0.0f;
		    upperTranslation = 0.0f;
		    enableMotor = false;
		    maxMotorForce = 0.0f;
		    motorSpeed = 0.0f;
	    }

	    /// Initialize the bodies, anchors, axis, and reference angle using the world
	    /// anchor and world axis.
        // Linear raint (point-to-line)
        // d = p2 - p1 = x2 + r2 - x1 - r1
        // C = dot(perp, d)
        // Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
        //      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
        // J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
        //
        // Angular raint
        // C = a2 - a1 + a_initial
        // Cdot = w2 - w1
        // J = [0 0 -1 0 0 1]
        //
        // K = J * invM * JT
        //
        // J = [-a -s1 a s2]
        //     [0  -1  0  1]
        // a = perp
        // s1 = cross(d + r1, a) = cross(p2 - x1, a)
        // s2 = cross(r2, a) = cross(p2 - x2, a)


        // Motor/Limit linear raint
        // C = dot(ax1, d)
        // Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
        // J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

        // Block Solver
        // We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
        // when the mass has poor distribution (leading to large torques about the joint anchor points).
        //
        // The Jacobian has 3 rows:
        // J = [-uT -s1 uT s2] // linear
        //     [0   -1   0  1] // angular
        //     [-vT -a1 vT a2] // limit
        //
        // u = perp
        // v = axis
        // s1 = cross(d + r1, u), s2 = cross(r2, u)
        // a1 = cross(d + r1, v), a2 = cross(r2, v)

        // M * (v2 - v1) = JT * df
        // J * v2 = bias
        //
        // v2 = v1 + invM * JT * df
        // J * (v1 + invM * JT * df) = bias
        // K * df = bias - J * v1 = -Cdot
        // K = J * invM * JT
        // Cdot = J * v1 - bias
        //
        // Now solve for f2.
        // df = f2 - f1
        // K * (f2 - f1) = -Cdot
        // f2 = invK * (-Cdot) + f1
        //
        // Clamp accumulated limit impulse.
        // lower: f2(3) = max(f2(3), 0)
        // upper: f2(3) = min(f2(3), 0)
        //
        // Solve for correct f2(1:2)
        // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
        //                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
        // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
        // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
        //
        // Now compute impulse to be applied:
        // df = f2 - f1
        public void Initialize(Body b1, Body b2, Vector2 anchor, Vector2 axis)
        {
	        body1 = b1;
	        body2 = b2;
	        localAnchor1 = body1.GetLocalPoint(anchor);
	        localAnchor2 = body2.GetLocalPoint(anchor);
	        localAxis1 = body1.GetLocalVector(axis);
	        referenceAngle = body2.GetAngle() - body1.GetAngle();
        }

	    /// The local anchor point relative to body1's origin.
	    public Vector2 localAnchor1;

	    /// The local anchor point relative to body2's origin.
	    public Vector2 localAnchor2;

	    /// The local translation axis in body1.
	    public Vector2 localAxis1;

	    /// The rained angle between the bodies: body2_angle - body1_angle.
	    public float referenceAngle;

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

    /// A prismatic joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in body1. Relative rotation is prevented. You can
    /// use a joint limit to restrict the range of motion and a joint motor to
    /// drive the motion or to model joint friction.
    public class PrismaticJoint : Joint
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
            return inv_dt * (_impulse.X * _perp + (_motorImpulse + _impulse.Z) * _axis);
        }

	    public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * _impulse.Y;
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

	    internal PrismaticJoint(PrismaticJointDef def)
            : base(def)
        {
	        _localAnchor1 = def.localAnchor1;
	        _localAnchor2 = def.localAnchor2;
	        _localXAxis1 = def.localAxis1;
	        _localYAxis1 = MathUtils.Cross(1.0f, _localXAxis1);
	        _refAngle = def.referenceAngle;

	        _impulse = Vector3.Zero;
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

	        // You cannot create a prismatic joint between bodies that
	        // both have fixed rotation.
	        Debug.Assert(b1._invI > 0.0f || b2._invI > 0.0f);

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
		        float k12 = i1 * _s1 + i2 * _s2;
		        float k13 = i1 * _s1 * _a1 + i2 * _s2 * _a2;
		        float k22 = i1 + i2;
		        float k23 = i1 * _a1 + i2 * _a2;
		        float k33 = m1 + m2 + i1 * _a1 * _a1 + i2 * _a2 * _a2;

		        _K.col1 = new Vector3(k11, k12, k13);
		        _K.col2 = new Vector3(k12, k22, k23);
		        _K.col3 = new Vector3(k13, k23, k33);
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
				        _impulse.Z = 0.0f;
			        }
		        }
		        else if (jointTranslation >= _upperTranslation)
		        {
			        if (_limitState != LimitState.AtUpper)
			        {
				        _limitState = LimitState.AtUpper;
				        _impulse.Z = 0.0f;
			        }
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

	        if (_enableMotor == false)
	        {
		        _motorImpulse = 0.0f;
	        }

	        if (step.warmStarting)
	        {
		        // Account for variable time step.
		        _impulse *= step.dtRatio;
		        _motorImpulse *= step.dtRatio;

		        Vector2 P = _impulse.X * _perp + (_motorImpulse + _impulse.Z) * _axis;
		        float L1 = _impulse.X * _s1 + _impulse.Y + (_motorImpulse + _impulse.Z) * _a1;
		        float L2 = _impulse.X * _s2 + _impulse.Y + (_motorImpulse + _impulse.Z) * _a2;

		        b1._linearVelocity -= _invMass1 * P;
		        b1._angularVelocity -= _invI1 * L1;

		        b2._linearVelocity += _invMass2 * P;
		        b2._angularVelocity += _invI2 * L2;
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

            Vector2 Cdot1 = new Vector2(Vector2.Dot(_perp, v2 - v1) + _s2 * w2 - _s1 * w1, w2 - w1);

	        if (_enableLimit && _limitState != LimitState.Inactive)
	        {
		        // Solve prismatic and limit raint in block form.
		        float Cdot2 = Vector2.Dot(_axis, v2 - v1) + _a2 * w2 - _a1 * w1;
		        Vector3 Cdot = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

		        Vector3 f1 = _impulse;
		        Vector3 df =  _K.Solve33(-Cdot);
		        _impulse += df;

		        if (_limitState == LimitState.AtLower)
		        {
			        _impulse.Z = Math.Max(_impulse.Z, 0.0f);
		        }
		        else if (_limitState == LimitState.AtUpper)
		        {
			        _impulse.Z = Math.Min(_impulse.Z, 0.0f);
		        }

		        // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		        Vector2 b = -Cdot1 - (_impulse.Z - f1.Z) * new Vector2(_K.col3.X, _K.col3.Y);
		        Vector2 f2r = _K.Solve22(b) + new Vector2(f1.X, f1.Y);
		        _impulse.X = f2r.X;
		        _impulse.Y = f2r.Y;

		        df = _impulse - f1;

		        Vector2 P = df.X * _perp + df.Z * _axis;
		        float L1 = df.X * _s1 + df.Y + df.Z * _a1;
		        float L2 = df.X * _s2 + df.Y + df.Z * _a2;

		        v1 -= _invMass1 * P;
		        w1 -= _invI1 * L1;

		        v2 += _invMass2 * P;
		        w2 += _invI2 * L2;
	        }
	        else
	        {
		        // Limit is inactive, just solve the prismatic raint in block form.
		        Vector2 df = _K.Solve22(-Cdot1);
		        _impulse.X += df.X;
		        _impulse.Y += df.Y;

		        Vector2 P = df.X * _perp;
		        float L1 = df.X * _s1 + df.Y;
		        float L2 = df.X * _s2 + df.Y;

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

	        Vector3 impulse;
            Vector2 C1 = new Vector2(Vector2.Dot(_perp, d), a2 - a1 - _refAngle);
	        
	        linearError = Math.Max(linearError, Math.Abs(C1.X));
	        angularError = Math.Abs(C1.Y);

	        if (active)
	        {
		        float m1 = _invMass1, m2 = _invMass2;
		        float i1 = _invI1, i2 = _invI2;

		        float k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;
		        float k12 = i1 * _s1 + i2 * _s2;
		        float k13 = i1 * _s1 * _a1 + i2 * _s2 * _a2;
		        float k22 = i1 + i2;
		        float k23 = i1 * _a1 + i2 * _a2;
		        float k33 = m1 + m2 + i1 * _a1 * _a1 + i2 * _a2 * _a2;

		        _K.col1 = new Vector3(k11, k12, k13);
		        _K.col2 = new Vector3(k12, k22, k23);
		        _K.col3 = new Vector3(k13, k23, k33);

                Vector3 C = new Vector3(-C1.X, -C1.Y, -C2);
		        impulse = _K.Solve33(C); // negated above
	        }
	        else
	        {
		        float m1 = _invMass1, m2 = _invMass2;
		        float i1 = _invI1, i2 = _invI2;

		        float k11 = m1 + m2 + i1 * _s1 * _s1 + i2 * _s2 * _s2;
		        float k12 = i1 * _s1 + i2 * _s2;
		        float k22 = i1 + i2;

		        _K.col1 = new Vector3(k11, k12, 0.0f);
		        _K.col2 = new Vector3(k12, k22, 0.0f);

		        Vector2 impulse1 = _K.Solve22(-C1);
		        impulse.X = impulse1.X;
		        impulse.Y = impulse1.Y;
		        impulse.Z = 0.0f;
	        }

	        Vector2 P = impulse.X * _perp + impulse.Z * _axis;
	        float L1 = impulse.X * _s1 + impulse.Y + impulse.Z * _a1;
	        float L2 = impulse.X * _s2 + impulse.Y + impulse.Z * _a2;

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

	    public Vector2 _localAnchor1;
        public Vector2 _localAnchor2;
        public Vector2 _localXAxis1;
        public Vector2 _localYAxis1;
        public float _refAngle;

        public Vector2 _axis, _perp;
        public float _s1, _s2;
        public float _a1, _a2;

        public Mat33 _K;
        public Vector3 _impulse;

        public float _motorMass;			// effective mass for motor/limit translational raint.
        public float _motorImpulse;

        public float _lowerTranslation;
        public float _upperTranslation;
        public float _maxMotorForce;
        public float _motorSpeed;

        public bool _enableLimit;
	    public bool _enableMotor;
        public LimitState _limitState;
    };
}
