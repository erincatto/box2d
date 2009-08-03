#include "b2FixedJoint.h"
#include "../b2Body.h"
#include "../b2World.h"

void b2FixedJointDef::Initialize(b2Body* b1, b2Body* b2)
{
	body1 = b1;
	body2 = b2;
}

b2FixedJoint::b2FixedJoint(const b2FixedJointDef* def)
	: b2Joint(def)
{
	// Get bodies
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Get initial delta position and angle
	m_dp = b2MulT(b1->GetXForm().R, b2->GetXForm().position - b1->GetXForm().position);
	m_a = b2->GetAngle() - b1->GetAngle();
	m_R0 = b2MulT(b1->GetXForm().R, b2->GetXForm().R);

	// Reset accumulators
	m_lambda_a = 0.0f;
	m_lambda_p.Set(0.0f, 0.0f);
	m_lambda_p_a = 0.0f;
}

void b2FixedJoint::InitVelocityConstraints(const b2TimeStep& step)
{
	// Get bodies
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Get d for this step
	m_d = m_dp - b1->m_sweep.localCenter + b2Mul(m_R0, b2->m_sweep.localCenter);

	// Calculate effective mass for angle constraint
	float32 invMass = b1->m_invMass + b2->m_invMass;
	b2Assert(invMass > B2_FLT_EPSILON);
	m_mass = 1.0f / invMass;

	// Calculate effective inertia for angle constraint
	float32 invInertia = b1->m_invI + b2->m_invI;
	b2Assert(invInertia > B2_FLT_EPSILON);
	m_inertia = 1.0f / invInertia;

	if (step.warmStarting)
	{
		// Take results of previous frame for angular constraint
		b1->m_angularVelocity -= b1->m_invI * m_lambda_a;
		b2->m_angularVelocity += b2->m_invI * m_lambda_a;

		// Take results of previous frame for position constraint
		float32 s = sinf(b1->m_sweep.a), c = cosf(b1->m_sweep.a);
		b2Vec2 A(-s * m_d.x - c * m_d.y, c * m_d.x - s * m_d.y);
		b1->m_linearVelocity -= b1->m_invMass * m_lambda_p;
		b1->m_angularVelocity -= b1->m_invI * b2Dot(m_lambda_p, A);
		b2->m_linearVelocity += b2->m_invMass * m_lambda_p;
	}
	else
	{
		// Reset accumulators
		m_lambda_a = 0.0f;
		m_lambda_p.Set(0.0f, 0.0f);
		m_lambda_p_a = 0.0f;
	}
}

void b2FixedJoint::SolveVelocityConstraints(const b2TimeStep& step)
{
	B2_NOT_USED(step);

	// Get bodies
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Angle constraint: w2 - w1 = 0
	float32 Cdot_a = b2->m_angularVelocity - b1->m_angularVelocity;
	float32 lambda_a = -m_inertia * Cdot_a;
	m_lambda_a += lambda_a;
	b1->m_angularVelocity -= b1->m_invI * lambda_a;
	b2->m_angularVelocity += b2->m_invI * lambda_a;

	// Position constraint: v2 - v1 - d/dt R(a1) d = v2 - v1 - A w1 = 0
	float32 s = sinf(b1->m_sweep.a), c = cosf(b1->m_sweep.a);
	b2Vec2 A(-s * m_d.x - c * m_d.y, c * m_d.x - s * m_d.y);
	b2Vec2 Cdot_p = b2->m_linearVelocity - b1->m_linearVelocity - b1->m_angularVelocity * A;
	b2Vec2 mc_p(1.0f / (b1->m_invMass + b1->m_invI * A.x * A.x + b2->m_invMass), 1.0f / (b1->m_invMass + b1->m_invI * A.y * A.y + b2->m_invMass));
	b2Vec2 lambda_p(-mc_p.x * Cdot_p.x, -mc_p.y * Cdot_p.y);
	m_lambda_p += lambda_p;
	float32 lambda_p_a = b2Dot(A, lambda_p);
	m_lambda_p_a += lambda_p_a;
	b1->m_linearVelocity -= b1->m_invMass * lambda_p;
	b1->m_angularVelocity -= b1->m_invI * lambda_p_a;
	b2->m_linearVelocity += b2->m_invMass * lambda_p;
}

bool b2FixedJoint::SolvePositionConstraints(float32 baumgarte)
{
	B2_NOT_USED(baumgarte);

	// Get bodies
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Angle constraint: a2 - a1 - a0 = 0
	float32 C_a = b2->m_sweep.a - b1->m_sweep.a - m_a;
	float32 lambda_a = -m_inertia * C_a;
	b1->m_sweep.a -= b1->m_invI * lambda_a;
	b2->m_sweep.a += b2->m_invI * lambda_a;

	// Position constraint: x2 - x1 - R(a1) d = 0
	float32 s = sinf(b1->m_sweep.a), c = cosf(b1->m_sweep.a);
	b2Vec2 Rd(c * m_d.x - s * m_d.y, s * m_d.x + c * m_d.y);
	b2Vec2 C_p = b2->m_sweep.c - b1->m_sweep.c - Rd;
	b2Vec2 lambda_p = -m_mass * C_p;
	b1->m_sweep.c -= b1->m_invMass * lambda_p;
	b2->m_sweep.c += b2->m_invMass * lambda_p;

	// Push the changes to the transforms
	b1->SynchronizeTransform();
	b2->SynchronizeTransform();

	// Constraint is satisfied if all constraint equations are nearly zero
	return b2Abs(C_p.x) < b2_linearSlop && b2Abs(C_p.y) < b2_linearSlop && b2Abs(C_a) < b2_linearSlop;
}

b2Vec2 b2FixedJoint::GetAnchor1() const 
{ 
	// Return arbitrary position (we have to implement this abstract virtual function)
	return m_body1->GetWorldCenter();
}

b2Vec2 b2FixedJoint::GetAnchor2() const 
{ 
	// Return arbitrary position (we have to implement this abstract virtual function)
	return m_body2->GetWorldCenter(); 
}

b2Vec2 b2FixedJoint::GetReactionForce(float32 inv_dt) const
{
	return inv_dt * m_lambda_p;
}

float32 b2FixedJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * (m_lambda_a + m_lambda_p_a);
}
