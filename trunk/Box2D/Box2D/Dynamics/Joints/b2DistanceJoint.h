/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
struct b2DistanceJointDef : public b2JointDef
{
	b2DistanceJointDef()
	{
		type = e_distanceJoint;
		localAnchor1.Set(0.0f, 0.0f);
		localAnchor2.Set(0.0f, 0.0f);
		length = 1.0f;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
	}

	/// Initialize the bodies, anchors, and length using the world
	/// anchors.
	void Initialize(b2Body* body1, b2Body* body2,
					const b2Vec2& anchor1, const b2Vec2& anchor2);

	/// The local anchor point relative to body1's origin.
	b2Vec2 localAnchor1;

	/// The local anchor point relative to body2's origin.
	b2Vec2 localAnchor2;

	/// The equilibrium length between the anchor points.
	float32 length;

	/// The response speed.
	float32 frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float32 dampingRatio;
};

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
class b2DistanceJoint : public b2Joint
{
public:

	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;

	//--------------- Internals Below -------------------

	b2DistanceJoint(const b2DistanceJointDef* data);

	void InitVelocityConstraints(const b2TimeStep& step);
	void SolveVelocityConstraints(const b2TimeStep& step);
	bool SolvePositionConstraints(float32 baumgarte);

	b2Vec2 m_localAnchor1;
	b2Vec2 m_localAnchor2;
	b2Vec2 m_u;
	float32 m_frequencyHz;
	float32 m_dampingRatio;
	float32 m_gamma;
	float32 m_bias;
	float32 m_impulse;
	float32 m_mass;		// effective mass for the constraint.
	float32 m_length;
};

#endif
