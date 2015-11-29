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
    public enum JointType
    {
        Unknown,
        Revolute,
        Prismatic,
        Distance,
        Pulley,
        Mouse,
        Gear,
        Line,
        Fixed,
    };

    public enum LimitState
    {
	    Inactive,
	    AtLower,
	    AtUpper,
	    Equal,
    };

    internal struct Jacobian
    {
	    public Vector2 linear1;
	    public float angular1;
	    public Vector2 linear2;
	    public float angular2;

	    public void SetZero()
        {
	        linear1 = Vector2.Zero; angular1 = 0.0f;
	        linear2 = Vector2.Zero; angular2 = 0.0f;
        }

	    public void Set(Vector2 x1, float a1, Vector2 x2, float a2)
        {
	        linear1 = x1; angular1 = a1;
	        linear2 = x2; angular2 = a2;
        }

	    public float Compute(Vector2 x1, float a1, Vector2 x2, float a2)
        {
            return Vector2.Dot(linear1, x1) + angular1 * a1 + Vector2.Dot(linear2, x2) + angular2 * a2;
        }
    };

     /// A joint edge is used to connect bodies and joints together
    /// in a joint graph where each body is a node and each joint
    /// is an edge. A joint edge belongs to a doubly linked list
    /// maintained in each attached body. Each joint has two joint
    /// nodes, one for each attached body.
    public class JointEdge
    {
        /// <summary>
        /// Provides quick access to the other body attached.
        /// </summary>
        public Body Other;

        /// <summary>
        /// The joint.
        /// </summary>
        public Joint Joint;

        /// <summary>
        /// The previous joint edge in the body's joint list.
        /// </summary>
        public JointEdge Prev;

        /// <summary>
        /// The next joint edge in the body's joint list.
        /// </summary>
        public JointEdge Next;
    }

    public class JointDef
    {
	    /// The joint type is set automatically for concrete joint types.
        internal JointType type;

	    /// Use this to attach application specific data to your joints.
        public object userData;

	    /// The first attached body.
        public Body body1;

	    /// The second attached body.
        public Body body2;

	    /// Set this flag to true if the attached bodies should collide.
	    public bool collideConnected;
    }

    public abstract class Joint
    {
	    /// Get the type of the concrete joint.
	    public JointType JointType
        {
            get
            {
                return _type;
            }
        }

	    /// Get the first body attached to this joint.
	    public Body GetBody1()
        {
            return _bodyA;
        }

	    /// Get the second body attached to this joint.
	    public Body GetBody2()
        {
            return _bodyB;
        }

	    /// Get the anchor point on body1 in world coordinates.
	    public abstract Vector2 GetAnchor1();

	    /// Get the anchor point on body2 in world coordinates.
	    public abstract Vector2 GetAnchor2();

	    /// Get the reaction force on body2 at the joint anchor.
	    public abstract Vector2 GetReactionForce(float inv_dt);

	    /// Get the reaction torque on body2.
	    public abstract float GetReactionTorque(float inv_dt);

	    /// Get the next joint the world joint list.
	    public Joint GetNext()
        {
	        return _next;
        }

	    /// Get the user data pointer.
	    public object GetUserData()
        {
            return _userData;
        }

	    /// Set the user data pointer.
	    public void SetUserData(object data)
        {
            _userData = data;
        }

	    internal static Joint Create(JointDef def)
        {
	        Joint joint = null;

	        switch (def.type)
	        {
	        case JointType.Distance:
		        {
			        joint = new DistanceJoint((DistanceJointDef)def);
		        }
		        break;

	        case JointType.Mouse:
		        {
			        joint = new MouseJoint((MouseJointDef)def);
		        }
		        break;

	        case JointType.Prismatic:
		        {
			        joint = new PrismaticJoint((PrismaticJointDef)def);
		        }
		        break;

	        case JointType.Revolute:
		        {
			        joint = new RevoluteJoint((RevoluteJointDef)def);
		        }
		        break;

	        case JointType.Pulley:
		        {
			        joint = new PulleyJoint((PulleyJointDef)def);
		        }
		        break;

	        case JointType.Gear:
		        {
			        joint = new GearJoint((GearJointDef)def);
		        }
		        break;

	        case JointType.Line:
		        {
			        joint = new LineJoint((LineJointDef)def);
		        }
		        break;
                
	        default:
		        Debug.Assert(false);
		        break;
	        }

	        return joint;
        }

	    protected Joint(JointDef def)
        {
	        _type = def.type;
	        _bodyA = def.body1;
	        _bodyB = def.body2;
	        _collideConnected = def.collideConnected;
	        _userData = def.userData;

            _edgeA = new JointEdge();
            _edgeB = new JointEdge();
        }

	    internal abstract void InitVelocityConstraints(ref TimeStep step);
	    internal abstract void SolveVelocityConstraints(ref TimeStep step);

	    // This returns true if the position errors are within tolerance.
	    internal abstract bool SolvePositionConstraints(float baumgarte);

	    internal void ComputeXForm(out XForm xf, Vector2 center, Vector2 localCenter, float angle)
        {
            xf = new XForm();
        	xf.R.Set(angle);
	        xf.Position = center - MathUtils.Multiply(ref xf.R, localCenter);
        }

	    internal JointType _type;
	    internal Joint _prev;
	    internal Joint _next;
	    internal JointEdge _edgeA;
	    internal JointEdge _edgeB;
	    internal Body _bodyA;
	    internal Body _bodyB;

	    internal bool _islandFlag;
	    internal bool _collideConnected;

	    internal object _userData;

	    // Cache here per time step to reduce cache misses.
	    internal Vector2 _localCenter1, _localCenter2;
	    internal float _invMass1, _invI1;
	    internal float _invMass2, _invI2;
    }
}
