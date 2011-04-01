/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2WheelJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2WheelJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axis);
}

b2WheelJoint::b2WheelJoint(const b2WheelJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localYAxisA = b2Cross(1.0f, m_localXAxisA);

	m_mass = 0.0f;
	m_impulse = 0.0f;
	m_motorMass = 0.0;
	m_motorImpulse = 0.0f;
	m_springMass = 0.0f;
	m_springImpulse = 0.0f;

	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableMotor = def->enableMotor;

	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_bias = 0.0f;
	m_gamma = 0.0f;

	m_ax.SetZero();
	m_ay.SetZero();
}

void b2WheelJoint::InitVelocityConstraints(const b2TimeStep& step)
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	m_localCenterA = bA->GetLocalCenter();
	m_localCenterB = bB->GetLocalCenter();

	b2Transform xfA = bA->GetTransform();
	b2Transform xfB = bB->GetTransform();

	// Compute the effective masses.
	b2Vec2 rA = b2Mul(xfA.R, m_localAnchorA - m_localCenterA);
	b2Vec2 rB = b2Mul(xfB.R, m_localAnchorB - m_localCenterB);
	b2Vec2 d = bB->m_sweep.c + rB - bA->m_sweep.c - rA;

	m_invMassA = bA->m_invMass;
	m_invIA = bA->m_invI;
	m_invMassB = bB->m_invMass;
	m_invIB = bB->m_invI;

	// Point to line constraint
	{
		m_ay = b2Mul(xfA.R, m_localYAxisA);
		m_sAy = b2Cross(d + rA, m_ay);
		m_sBy = b2Cross(rB, m_ay);

		m_mass = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

		if (m_mass > 0.0f)
		{
			m_mass = 1.0f / m_mass;
		}
	}

	// Spring constraint
	m_springMass = 0.0f;
	if (m_frequencyHz > 0.0f)
	{
		m_ax = b2Mul(xfA.R, m_localXAxisA);
		m_sAx = b2Cross(d + rA, m_ax);
		m_sBx = b2Cross(rB, m_ax);

		float32 invMass = m_invMassA + m_invMassB + m_invIA * m_sAx * m_sAx + m_invIB * m_sBx * m_sBx;

		if (invMass > 0.0f)
		{
			m_springMass = 1.0f / invMass;

			float32 C = b2Dot(d, m_ax);

			// Frequency
			float32 omega = 2.0f * b2_pi * m_frequencyHz;

			// Damping coefficient
			float32 d = 2.0f * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			float32 k = m_springMass * omega * omega;

			// magic formulas
			m_gamma = step.dt * (d + step.dt * k);
			if (m_gamma > 0.0f)
			{
				m_gamma = 1.0f / m_gamma;
			}

			m_bias = C * step.dt * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > 0.0f)
			{
				m_springMass = 1.0f / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = 0.0f;
		m_springMass = 0.0f;
	}

	// Rotational motor
	if (m_enableMotor)
	{
		m_motorMass = m_invIA + m_invIB;
		if (m_motorMass > 0.0f)
		{
			m_motorMass = 1.0f / m_motorMass;
		}
	}
	else
	{
		m_motorMass = 0.0f;
		m_motorImpulse = 0.0f;
	}

	if (step.warmStarting)
	{
		// Account for variable time step.
		m_impulse *= step.dtRatio;
		m_springImpulse *= step.dtRatio;
		m_motorImpulse *= step.dtRatio;

		b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
		float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
		float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

		bA->m_linearVelocity -= m_invMassA * P;
		bA->m_angularVelocity -= m_invIA * LA;

		bB->m_linearVelocity += m_invMassB * P;
		bB->m_angularVelocity += m_invIB * LB;
	}
	else
	{
		m_impulse = 0.0f;
		m_springImpulse = 0.0f;
		m_motorImpulse = 0.0f;
	}
}

void b2WheelJoint::SolveVelocityConstraints(const b2TimeStep& step)
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 vA = bA->m_linearVelocity;
	float32 wA = bA->m_angularVelocity;
	b2Vec2 vB = bB->m_linearVelocity;
	float32 wB = bB->m_angularVelocity;

	// Solve spring constraint
	{
		float32 Cdot = b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
		float32 impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		b2Vec2 P = impulse * m_ax;
		float32 LA = impulse * m_sAx;
		float32 LB = impulse * m_sBx;

		vA -= m_invMassA * P;
		wA -= m_invIA * LA;

		vB += m_invMassB * P;
		wB += m_invIB * LB;
	}

	// Solve rotational motor constraint
	{
		float32 Cdot = wB - wA - m_motorSpeed;
		float32 impulse = -m_motorMass * Cdot;

		float32 oldImpulse = m_motorImpulse;
		float32 maxImpulse = step.dt * m_maxMotorTorque;
		m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= m_invIA * impulse;
		wB += m_invIB * impulse;
	}

	// Solve point to line constraint
	{
		float32 Cdot = b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
		float32 impulse = m_mass * (-Cdot);
		m_impulse += impulse;

		b2Vec2 P = impulse * m_ay;
		float32 LA = impulse * m_sAy;
		float32 LB = impulse * m_sBy;

		vA -= m_invMassA * P;
		wA -= m_invIA * LA;

		vB += m_invMassB * P;
		wB += m_invIB * LB;
	}

	bA->m_linearVelocity = vA;
	bA->m_angularVelocity = wA;
	bB->m_linearVelocity = vB;
	bB->m_angularVelocity = wB;
}

bool b2WheelJoint::SolvePositionConstraints(float32 baumgarte)
{
	B2_NOT_USED(baumgarte);

	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 xA = bA->m_sweep.c;
	float32 angleA = bA->m_sweep.a;

	b2Vec2 xB = bB->m_sweep.c;
	float32 angleB = bB->m_sweep.a;

	b2Mat22 RA(angleA), RB(angleB);

	b2Vec2 rA = b2Mul(RA, m_localAnchorA - m_localCenterA);
	b2Vec2 rB = b2Mul(RB, m_localAnchorB - m_localCenterB);
	b2Vec2 d = xB + rB - xA - rA;

	b2Vec2 ay = b2Mul(RA, m_localYAxisA);

	float32 sAy = b2Cross(d + rA, ay);
	float32 sBy = b2Cross(rB, ay);

	float32 C = b2Dot(d, ay);

	float32 k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

	float32 impulse;
	if (k != 0.0f)
	{
		impulse = - C / k;
	}
	else
	{
		impulse = 0.0f;
	}

	b2Vec2 P = impulse * ay;
	float32 LA = impulse * sAy;
	float32 LB = impulse * sBy;

	xA -= m_invMassA * P;
	angleA -= m_invIA * LA;
	xB += m_invMassB * P;
	angleB += m_invIB * LB;

	// TODO_ERIN remove need for this.
	bA->m_sweep.c = xA;
	bA->m_sweep.a = angleA;
	bB->m_sweep.c = xB;
	bB->m_sweep.a = angleB;
	bA->SynchronizeTransform();
	bB->SynchronizeTransform();

	return b2Abs(C) <= b2_linearSlop;
}

b2Vec2 b2WheelJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2WheelJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2WheelJoint::GetReactionForce(float32 inv_dt) const
{
	return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

float32 b2WheelJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

float32 b2WheelJoint::GetJointTranslation() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 pA = bA->GetWorldPoint(m_localAnchorA);
	b2Vec2 pB = bB->GetWorldPoint(m_localAnchorB);
	b2Vec2 d = pB - pA;
	b2Vec2 axis = bA->GetWorldVector(m_localXAxisA);

	float32 translation = b2Dot(d, axis);
	return translation;
}

float32 b2WheelJoint::GetJointSpeed() const
{
	float32 wA = m_bodyA->m_angularVelocity;
	float32 wB = m_bodyB->m_angularVelocity;
	return wB - wA;
}

bool b2WheelJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b2WheelJoint::EnableMotor(bool flag)
{
	m_bodyA->SetAwake(true);
	m_bodyB->SetAwake(true);
	m_enableMotor = flag;
}

void b2WheelJoint::SetMotorSpeed(float32 speed)
{
	m_bodyA->SetAwake(true);
	m_bodyB->SetAwake(true);
	m_motorSpeed = speed;
}

void b2WheelJoint::SetMaxMotorTorque(float32 torque)
{
	m_bodyA->SetAwake(true);
	m_bodyB->SetAwake(true);
	m_maxMotorTorque = torque;
}

float32 b2WheelJoint::GetMotorTorque(float32 inv_dt) const
{
	return inv_dt * m_motorImpulse;
}





