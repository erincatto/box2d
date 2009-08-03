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

#ifndef B2_PULLEY_JOINT_H
#define B2_PULLEY_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

const float32 b2_minPulleyLength = 2.0f;

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, max lengths for each side,
/// and a pulley ratio.
struct b2PulleyJointDef : public b2JointDef
{
	b2PulleyJointDef()
	{
		type = e_pulleyJoint;
		groundAnchor1.Set(-1.0f, 1.0f);
		groundAnchor2.Set(1.0f, 1.0f);
		localAnchor1.Set(-1.0f, 0.0f);
		localAnchor2.Set(1.0f, 0.0f);
		length1 = 0.0f;
		maxLength1 = 0.0f;
		length2 = 0.0f;
		maxLength2 = 0.0f;
		ratio = 1.0f;
		collideConnected = true;
	}

	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	void Initialize(b2Body* body1, b2Body* body2,
					const b2Vec2& groundAnchor1, const b2Vec2& groundAnchor2,
					const b2Vec2& anchor1, const b2Vec2& anchor2,
					float32 ratio);

	/// The first ground anchor in world coordinates. This point never moves.
	b2Vec2 groundAnchor1;

	/// The second ground anchor in world coordinates. This point never moves.
	b2Vec2 groundAnchor2;

	/// The local anchor point relative to body1's origin.
	b2Vec2 localAnchor1;

	/// The local anchor point relative to body2's origin.
	b2Vec2 localAnchor2;

	/// The a reference length for the segment attached to body1.
	float32 length1;

	/// The maximum length of the segment attached to body1.
	float32 maxLength1;

	/// The a reference length for the segment attached to body2.
	float32 length2;

	/// The maximum length of the segment attached to body2.
	float32 maxLength2;

	/// The pulley ratio, used to simulate a block-and-tackle.
	float32 ratio;
};

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// The pulley also enforces a maximum length limit on both sides. This is
/// useful to prevent one side of the pulley hitting the top.
class b2PulleyJoint : public b2Joint
{
public:
	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;

	/// Get the first ground anchor.
	b2Vec2 GetGroundAnchor1() const;

	/// Get the second ground anchor.
	b2Vec2 GetGroundAnchor2() const;

	/// Get the current length of the segment attached to body1.
	float32 GetLength1() const;

	/// Get the current length of the segment attached to body2.
	float32 GetLength2() const;

	/// Get the pulley ratio.
	float32 GetRatio() const;

	//--------------- Internals Below -------------------

	b2PulleyJoint(const b2PulleyJointDef* data);

	void InitVelocityConstraints(const b2TimeStep& step);
	void SolveVelocityConstraints(const b2TimeStep& step);
	bool SolvePositionConstraints(float32 baumgarte);

	b2Vec2 m_groundAnchor1;
	b2Vec2 m_groundAnchor2;
	b2Vec2 m_localAnchor1;
	b2Vec2 m_localAnchor2;

	b2Vec2 m_u1;
	b2Vec2 m_u2;
	
	float32 m_constant;
	float32 m_ratio;
	
	float32 m_maxLength1;
	float32 m_maxLength2;

	// Effective masses
	float32 m_pulleyMass;
	float32 m_limitMass1;
	float32 m_limitMass2;

	// Impulses for accumulation/warm starting.
	float32 m_impulse;
	float32 m_limitImpulse1;
	float32 m_limitImpulse2;

	b2LimitState m_state;
	b2LimitState m_limitState1;
	b2LimitState m_limitState2;
};

#endif
