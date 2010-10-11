/*
* Copyright (c) 2007-2010 Erin Catto http://www.gphysics.com
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

#include <Box2D/Dynamics/Joints/b2RopeJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>


// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

b2RopeJoint::b2RopeJoint(const b2RopeJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;

	m_maxLength = def->maxLength;

	m_mass = 0.0f;
	m_impulse = 0.0f;
	m_state = e_inactiveLimit;
	m_length = 0.0f;
}

void b2RopeJoint::InitVelocityConstraints(const b2TimeStep& step)
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	m_rA = b2Mul(bA->GetTransform().R, m_localAnchorA - bA->GetLocalCenter());
	m_rB = b2Mul(bB->GetTransform().R, m_localAnchorB - bB->GetLocalCenter());

	// Rope axis
	m_u = bB->m_sweep.c + m_rB - bA->m_sweep.c - m_rA;

	m_length = m_u.Length();

	float32 C = m_length - m_maxLength;
	if (C > 0.0f)
	{
		m_state = e_atUpperLimit;
	}
	else
	{
		m_state = e_inactiveLimit;
	}

	if (m_length > b2_linearSlop)
	{
		m_u *= 1.0f / m_length;
	}
	else
	{
		m_u.SetZero();
		m_mass = 0.0f;
		m_impulse = 0.0f;
		return;
	}

	// Compute effective mass.
	float32 crA = b2Cross(m_rA, m_u);
	float32 crB = b2Cross(m_rB, m_u);
	float32 invMass = bA->m_invMass + bA->m_invI * crA * crA + bB->m_invMass + bB->m_invI * crB * crB;

	m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

	if (step.warmStarting)
	{
		// Scale the impulse to support a variable time step.
		m_impulse *= step.dtRatio;

		b2Vec2 P = m_impulse * m_u;
		bA->m_linearVelocity -= bA->m_invMass * P;
		bA->m_angularVelocity -= bA->m_invI * b2Cross(m_rA, P);
		bB->m_linearVelocity += bB->m_invMass * P;
		bB->m_angularVelocity += bB->m_invI * b2Cross(m_rB, P);
	}
	else
	{
		m_impulse = 0.0f;
	}
}

void b2RopeJoint::SolveVelocityConstraints(const b2TimeStep& step)
{
	B2_NOT_USED(step);

	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	// Cdot = dot(u, v + cross(w, r))
	b2Vec2 vA = bA->m_linearVelocity + b2Cross(bA->m_angularVelocity, m_rA);
	b2Vec2 vB = bB->m_linearVelocity + b2Cross(bB->m_angularVelocity, m_rB);
	float32 C = m_length - m_maxLength;
	float32 Cdot = b2Dot(m_u, vB - vA);

	// Predictive constraint.
	if (C < 0.0f)
	{
		Cdot += step.inv_dt * C;
	}

	float32 impulse = -m_mass * Cdot;
	float32 oldImpulse = m_impulse;
	m_impulse = b2Min(0.0f, m_impulse + impulse);
	impulse = m_impulse - oldImpulse;

	b2Vec2 P = impulse * m_u;
	bA->m_linearVelocity -= bA->m_invMass * P;
	bA->m_angularVelocity -= bA->m_invI * b2Cross(m_rA, P);
	bB->m_linearVelocity += bB->m_invMass * P;
	bB->m_angularVelocity += bB->m_invI * b2Cross(m_rB, P);
}

bool b2RopeJoint::SolvePositionConstraints(float32 baumgarte)
{
	B2_NOT_USED(baumgarte);

	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 rA = b2Mul(bA->GetTransform().R, m_localAnchorA - bA->GetLocalCenter());
	b2Vec2 rB = b2Mul(bB->GetTransform().R, m_localAnchorB - bB->GetLocalCenter());

	b2Vec2 u = bB->m_sweep.c + rB - bA->m_sweep.c - rA;

	float32 length = u.Normalize();
	float32 C = length - m_maxLength;

	C = b2Clamp(C, 0.0f, b2_maxLinearCorrection);

	float32 impulse = -m_mass * C;
	b2Vec2 P = impulse * u;

	bA->m_sweep.c -= bA->m_invMass * P;
	bA->m_sweep.a -= bA->m_invI * b2Cross(rA, P);
	bB->m_sweep.c += bB->m_invMass * P;
	bB->m_sweep.a += bB->m_invI * b2Cross(rB, P);

	bA->SynchronizeTransform();
	bB->SynchronizeTransform();

	return length - m_maxLength < b2_linearSlop;
}

b2Vec2 b2RopeJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2RopeJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2RopeJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 F = (inv_dt * m_impulse) * m_u;
	return F;
}

float32 b2RopeJoint::GetReactionTorque(float32 inv_dt) const
{
	B2_NOT_USED(inv_dt);
	return 0.0f;
}

float32 b2RopeJoint::GetMaxLength() const
{
	return m_maxLength;
}

b2LimitState b2RopeJoint::GetLimitState() const
{
	return m_state;
}
