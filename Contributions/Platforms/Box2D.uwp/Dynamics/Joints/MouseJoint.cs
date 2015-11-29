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
using System.Numerics;

namespace Box2D.UWP
{
/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
public class MouseJointDef : JointDef
{
	public MouseJointDef()
	{
		type = JointType.Mouse;
		target = new Vector2(0.0f, 0.0f);
		maxForce = 0.0f;
		frequencyHz = 5.0f;
		dampingRatio = 0.7f;
	}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	public Vector2 target;

	/// The maximum raint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	public float maxForce;

	/// The response speed.
	public float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	public float dampingRatio;
};

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft raint with a maximum
/// force. This allows the raint to stretch and without
/// applying huge forces.
public class MouseJoint : Joint
{
	/// Implements Joint.
	public override Vector2 GetAnchor1()
    {
        return _target;
    }

	/// Implements Joint.
	public override Vector2 GetAnchor2()
    {
        return _bodyB.GetWorldPoint(_localAnchor);
    }

	/// Implements Joint.
	public override Vector2 GetReactionForce(float inv_dt)
    {
        return inv_dt * _impulse;
    }

	/// Implements Joint.
	public override float GetReactionTorque(float inv_dt)
    {
        return inv_dt * 0.0f;
    }

	/// Use this to update the target point.
	public void SetTarget(Vector2 target)
    {
	    if (_bodyB.IsSleeping)
	    {
		    _bodyB.WakeUp();
	    }
	    _target = target;
    }

	internal MouseJoint(MouseJointDef def)
        : base(def)
    {
        XForm xf1;
        _bodyB.GetXForm(out xf1);

	    _target = def.target;
	    _localAnchor = MathUtils.MultiplyT(ref xf1, _target);

	    _maxForce = def.maxForce;
	    _impulse = Vector2.Zero;

	    _frequencyHz = def.frequencyHz;
	    _dampingRatio = def.dampingRatio;

	    _beta = 0.0f;
	    _gamma = 0.0f;
    }

	internal override void InitVelocityConstraints(ref TimeStep step)
    {
	    Body b = _bodyB;

	    float mass = b.GetMass();

	    // Frequency
	    float omega = 2.0f * Settings.b2_pi * _frequencyHz;

	    // Damping coefficient
	    float d = 2.0f * mass * _dampingRatio * omega;

	    // Spring stiffness
	    float k = mass * (omega * omega);

	    // magic formulas
	    // gamma has units of inverse mass.
	    // beta has units of inverse time.
	    Debug.Assert(d + step.dt * k > Settings.b2_FLT_EPSILON);
	    _gamma = 1.0f / (step.dt * (d + step.dt * k));
	    _beta = step.dt * k * _gamma;

	    // Compute the effective mass matrix.
        XForm xf1;
        b.GetXForm(out xf1);
	    Vector2 r = MathUtils.Multiply(ref xf1.R, _localAnchor - b.GetLocalCenter());

	    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	    //      = [1/m1+1/m2     0    ] + invI1 * [r1.Y*r1.Y -r1.X*r1.Y] + invI2 * [r1.Y*r1.Y -r1.X*r1.Y]
	    //        [    0     1/m1+1/m2]           [-r1.X*r1.Y r1.X*r1.X]           [-r1.X*r1.Y r1.X*r1.X]
	    float invMass = b._invMass;
	    float invI = b._invI;

        Mat22 K1 = new Mat22(new Vector2(invMass, 0.0f), new Vector2(0.0f, invMass));
        Mat22 K2 = new Mat22(new Vector2(invI * r.Y * r.Y, -invI * r.X * r.Y), new Vector2(-invI * r.X * r.Y, invI * r.X * r.X));

        Mat22 K;
        Mat22.Add(ref K1, ref K2, out K);

	    K.col1.X += _gamma;
	    K.col2.Y += _gamma;

	    _mass = K.GetInverse();

	    _C = b._sweep.c + r - _target;

	    // Cheat with some damping
	    b._angularVelocity *= 0.98f;

	    // Warm starting.
	    _impulse *= step.dtRatio;
	    b._linearVelocity += invMass * _impulse;
	    b._angularVelocity += invI * MathUtils.Cross(r, _impulse);
    }

	internal override void SolveVelocityConstraints(ref TimeStep step)
    {
        Body b = _bodyB;

        XForm xf1;
        b.GetXForm(out xf1);

	    Vector2 r = MathUtils.Multiply(ref xf1.R, _localAnchor - b.GetLocalCenter());

	    // Cdot = v + cross(w, r)
	    Vector2 Cdot = b._linearVelocity + MathUtils.Cross(b._angularVelocity, r);
	    Vector2 impulse = MathUtils.Multiply(ref _mass, -(Cdot + _beta * _C + _gamma * _impulse));

	    Vector2 oldImpulse = _impulse;
	    _impulse += impulse;
	    float maxImpulse = step.dt * _maxForce;
	    if (_impulse.LengthSquared() > maxImpulse * maxImpulse)
	    {
		    _impulse *= maxImpulse / _impulse.Length();
	    }
	    impulse = _impulse - oldImpulse;

	    b._linearVelocity += b._invMass * impulse;
	    b._angularVelocity += b._invI * MathUtils.Cross(r, impulse);
    }

	internal override bool SolvePositionConstraints(float baumgarte) 
    { 
        return true; 
    }

	public Vector2 _localAnchor;
    public Vector2 _target;
    public Vector2 _impulse;

    public Mat22 _mass;		// effective mass for point-to-point raint.
    public Vector2 _C;				// position error
    public float _maxForce;
    public float _frequencyHz;
    public float _dampingRatio;
    public float _beta;
    public float _gamma;
};
}
