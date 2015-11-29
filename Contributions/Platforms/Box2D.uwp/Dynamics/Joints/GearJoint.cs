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
    /// Gear joint definition. This definition requires two existing
    /// revolute or prismatic joints (any combination will work).
    /// The provided joints must attach a dynamic body to a static body.
    public class GearJointDef : JointDef
    {
	    public GearJointDef()
	    {
		    type = JointType.Gear;
		    joint1 = null;
		    joint2 = null;
		    ratio = 1.0f;
	    }

	    /// The first revolute/prismatic joint attached to the gear joint.
	    public Joint joint1;

	    /// The second revolute/prismatic joint attached to the gear joint.
	    public Joint joint2;

	    /// The gear ratio.
	    /// @see GearJoint for explanation.
	    public float ratio;
    };

    /// A gear joint is used to connect two joints together. Either joint
    /// can be a revolute or prismatic joint. You specify a gear ratio
    /// to bind the motions together:
    /// coordinate1 + ratio * coordinate2 = ant
    /// The ratio can be negative or positive. If one joint is a revolute joint
    /// and the other joint is a prismatic joint, then the ratio will have units
    /// of length or units of 1/length.
    /// @warning The revolute and prismatic joints must be attached to
    /// fixed bodies (which must be body1 on those joints).
    public class GearJoint : Joint
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
            Vector2 P = _impulse * _J.linear2;
	        return inv_dt * P;
        }

	    public override float GetReactionTorque(float inv_dt)
        {
            XForm xf1;
            _bodyB.GetXForm(out xf1);

            Vector2 r = MathUtils.Multiply(ref xf1.R, _localAnchor2 - _bodyB.GetLocalCenter());
	        Vector2 P = _impulse * _J.linear2;
	        float L = _impulse * _J.angular2 - MathUtils.Cross(r, P);
	        return inv_dt * L;
        }

	    /// Get the gear ratio.
	    public float GetRatio()
        {
            return _ratio;
        }

	    internal GearJoint(GearJointDef def)
            : base(def)
        {
	        JointType type1 = def.joint1.JointType;
            JointType type2 = def.joint2.JointType;

	        Debug.Assert(type1 == JointType.Revolute || type1 == JointType.Prismatic);
	        Debug.Assert(type2 == JointType.Revolute || type2 == JointType.Prismatic);
	        Debug.Assert(def.joint1.GetBody1().IsStatic);
	        Debug.Assert(def.joint2.GetBody1().IsStatic);

	        _revolute1 = null;
	        _prismatic1 = null;
	        _revolute2 = null;
	        _prismatic2 = null;

	        float coordinate1, coordinate2;

	        _ground1 = def.joint1.GetBody1();
	        _bodyA = def.joint1.GetBody2();
	        if (type1 == JointType.Revolute)
	        {
		        _revolute1 = (RevoluteJoint)def.joint1;
		        _groundAnchor1 = _revolute1._localAnchor1;
		        _localAnchor1 = _revolute1._localAnchor2;
		        coordinate1 = _revolute1.GetJointAngle();
	        }
	        else
	        {
		        _prismatic1 = (PrismaticJoint)def.joint1;
		        _groundAnchor1 = _prismatic1._localAnchor1;
		        _localAnchor1 = _prismatic1._localAnchor2;
		        coordinate1 = _prismatic1.GetJointTranslation();
	        }

	        _ground2 = def.joint2.GetBody1();
	        _bodyB = def.joint2.GetBody2();
	        if (type2 == JointType.Revolute)
	        {
		        _revolute2 = (RevoluteJoint)def.joint2;
		        _groundAnchor2 = _revolute2._localAnchor1;
		        _localAnchor2 = _revolute2._localAnchor2;
		        coordinate2 = _revolute2.GetJointAngle();
	        }
	        else
	        {
		        _prismatic2 = (PrismaticJoint)def.joint2;
		        _groundAnchor2 = _prismatic2._localAnchor1;
		        _localAnchor2 = _prismatic2._localAnchor2;
		        coordinate2 = _prismatic2.GetJointTranslation();
	        }

	        _ratio = def.ratio;

	        _ant = coordinate1 + _ratio * coordinate2;

	        _impulse = 0.0f;
        }

	    internal override void InitVelocityConstraints(ref TimeStep step)
        {
	        Body g1 = _ground1;
	        Body g2 = _ground2;
	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        float K = 0.0f;
            _J.SetZero();

	        if (_revolute1 != null)
	        {
		        _J.angular1 = -1.0f;
		        K += b1._invI;
	        }
	        else
	        {
                XForm xf1, xfg1;
                b1.GetXForm(out xf1);
                g1.GetXForm(out xfg1);

		        Vector2 ug = MathUtils.Multiply(ref xfg1.R, _prismatic1._localXAxis1);
		        Vector2 r = MathUtils.Multiply(ref xf1.R, _localAnchor1 - b1.GetLocalCenter());
		        float crug = MathUtils.Cross(r, ug);
		        _J.linear1 = -ug;
		        _J.angular1 = -crug;
		        K += b1._invMass + b1._invI * crug * crug;
	        }

	        if (_revolute2 != null)
	        {
		        _J.angular2 = -_ratio;
		        K += _ratio * _ratio * b2._invI;
	        }
	        else
	        {
                XForm xfg1, xf2;
                g1.GetXForm(out xfg1);
                b2.GetXForm(out xf2);

		        Vector2 ug = MathUtils.Multiply(ref xfg1.R, _prismatic2._localXAxis1);
		        Vector2 r = MathUtils.Multiply(ref xf2.R, _localAnchor2 - b2.GetLocalCenter());
		        float crug = MathUtils.Cross(r, ug);
		        _J.linear2 = -_ratio * ug;
		        _J.angular2 = -_ratio * crug;
		        K += _ratio * _ratio * (b2._invMass + b2._invI * crug * crug);
	        }

	        // Compute effective mass.
	        Debug.Assert(K > 0.0f);
	        _mass = 1.0f / K;

	        if (step.warmStarting)
	        {
		        // Warm starting.
		        b1._linearVelocity += b1._invMass * _impulse * _J.linear1;
		        b1._angularVelocity += b1._invI * _impulse * _J.angular1;
		        b2._linearVelocity += b2._invMass * _impulse * _J.linear2;
		        b2._angularVelocity += b2._invI * _impulse * _J.angular2;
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

	        float Cdot = _J.Compute(	b1._linearVelocity, b1._angularVelocity,
								        b2._linearVelocity, b2._angularVelocity);

	        float impulse = _mass * (-Cdot);
	        _impulse += impulse;

	        b1._linearVelocity += b1._invMass * impulse * _J.linear1;
	        b1._angularVelocity += b1._invI * impulse * _J.angular1;
	        b2._linearVelocity += b2._invMass * impulse * _J.linear2;
	        b2._angularVelocity += b2._invI * impulse * _J.angular2;
        }

	    internal override bool SolvePositionConstraints(float baumgarte)
        {
            float linearError = 0.0f;

	        Body b1 = _bodyA;
	        Body b2 = _bodyB;

	        float coordinate1, coordinate2;
	        if (_revolute1 != null)
	        {
		        coordinate1 = _revolute1.GetJointAngle();
	        }
	        else
	        {
		        coordinate1 = _prismatic1.GetJointTranslation();
	        }

            if (_revolute2 != null)
	        {
		        coordinate2 = _revolute2.GetJointAngle();
	        }
	        else
	        {
		        coordinate2 = _prismatic2.GetJointTranslation();
	        }

	        float C = _ant - (coordinate1 + _ratio * coordinate2);

	        float impulse = _mass * (-C);

	        b1._sweep.c += b1._invMass * impulse * _J.linear1;
	        b1._sweep.a += b1._invI * impulse * _J.angular1;
	        b2._sweep.c += b2._invMass * impulse * _J.linear2;
	        b2._sweep.a += b2._invI * impulse * _J.angular2;

	        b1.SynchronizeTransform();
	        b2.SynchronizeTransform();

	        // TODO_ERIN not implemented
	        return linearError < Settings.b2_linearSlop;
        }

	    internal Body _ground1;
	    internal Body _ground2;

	    // One of these is null.
	    internal RevoluteJoint _revolute1;
	    internal PrismaticJoint _prismatic1;

	    // One of these is null.
	    internal RevoluteJoint _revolute2;
	    internal PrismaticJoint _prismatic2;

	    internal Vector2 _groundAnchor1;
	    internal Vector2 _groundAnchor2;

	    internal Vector2 _localAnchor1;
	    internal Vector2 _localAnchor2;

	    internal Jacobian _J;

	    internal float _ant;
	    internal float _ratio;

	    // Effective mass
	    internal float _mass;

	    // Impulse for accumulation/warm starting.
	    internal float _impulse;
    };
}
