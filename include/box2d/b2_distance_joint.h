// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include "b2_joint.h"

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
		localAnchorA.Set(0.0f, 0.0f);
		localAnchorB.Set(0.0f, 0.0f);
		length = 1.0f;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
	}

	/// Initialize the bodies, anchors, and length using the world
	/// anchors.
	void Initialize(b2Body* bodyA, b2Body* bodyB,
					const b2Vec2& anchorA, const b2Vec2& anchorB);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The natural length between the anchor points.
	float length;

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float dampingRatio;
};

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
class b2DistanceJoint : public b2Joint
{
public:

	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	b2Vec2 GetReactionForce(float inv_dt) const override;

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
	float GetReactionTorque(float inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	void SetLength(float length);
	float GetLength() const;

	/// Set/get frequency in Hz.
	void SetFrequency(float hz);
	float GetFrequency() const;

	/// Set/get damping ratio.
	void SetDampingRatio(float ratio);
	float GetDampingRatio() const;

	/// Dump joint to dmLog
	void Dump() override;

protected:

	friend class b2Joint;
	b2DistanceJoint(const b2DistanceJointDef* data);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	float m_frequencyHz;
	float m_dampingRatio;
	float m_bias;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	float m_gamma;
	float m_impulse;
	float m_length;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_u;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	float m_mass;
};

inline void b2DistanceJoint::SetLength(float length)
{
	m_length = length;
}

inline float b2DistanceJoint::GetLength() const
{
	return m_length;
}

inline void b2DistanceJoint::SetFrequency(float hz)
{
	m_frequencyHz = hz;
}

inline float b2DistanceJoint::GetFrequency() const
{
	return m_frequencyHz;
}

inline void b2DistanceJoint::SetDampingRatio(float ratio)
{
	m_dampingRatio = ratio;
}

inline float b2DistanceJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

#endif
