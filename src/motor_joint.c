// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "solver_set.h"
#include "physics_world.h"

// needed for dll export
#include "box2d/box2d.h"

void b2MotorJoint_SetMaxForce( b2JointId jointId, float maxForce )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.maxForce = b2MaxFloat( 0.0f, maxForce );
}

float b2MotorJoint_GetMaxForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.maxForce;
}

void b2MotorJoint_SetMaxTorque( b2JointId jointId, float maxTorque )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.maxTorque = b2MaxFloat( 0.0f, maxTorque );
}

float b2MotorJoint_GetMaxTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.maxTorque;
}

void b2MotorJoint_SetCorrectionFactor( b2JointId jointId, float correctionFactor )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.correctionFactor = b2ClampFloat( correctionFactor, 0.0f, 1.0f );
}

float b2MotorJoint_GetCorrectionFactor( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.correctionFactor;
}

b2Vec2 b2GetMotorJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->motorJoint.linearImpulse );
	return force;
}

float b2GetMotorJointTorque( b2World* world, b2JointSim* base )
{
	return world->inv_h * base->motorJoint.angularImpulse;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2PrepareMotorJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_motorJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2BodyArray_Get( &world->bodies, idA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );

	b2SolverSet* setA = b2SolverSetArray_Get( &world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2SolverSetArray_Get( &world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2BodySimArray_Get( &setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2BodySimArray_Get( &setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float iA = bodySimA->invInertia;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2MotorJoint* joint = &base->motorJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	b2Vec2 rA = joint->frameA.p;
	b2Vec2 rB = joint->frameB.p;

	b2Mat22 K;
	K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.cy.x = K.cx.y;
	K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	joint->linearMass = b2GetInverse22( K );

	float ka = iA + iB;
	joint->angularMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartMotorJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_motorJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	b2MotorJoint* joint = &base->motorJoint;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
	stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + joint->angularImpulse );
	stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
	stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + joint->angularImpulse );
}

void b2SolveMotorJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_motorJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2MotorJoint* joint = &base->motorJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// angular constraint
	{
		b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		b2Rot relQ = b2InvMulRot( qA, qB );

		float jointAngle = b2Rot_GetAngle( relQ );
		float angularBias = context->inv_h * joint->correctionFactor * jointAngle;

		float Cdot = wB - wA;
		float impulse = -joint->angularMass * ( Cdot + angularBias );

		float oldImpulse = joint->angularImpulse;
		float maxImpulse = context->h * joint->maxTorque;
		joint->angularImpulse = b2ClampFloat( joint->angularImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint->angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// linear constraint
	{
		b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
		b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

		b2Vec2 ds = b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), b2Sub( rB, rA ) );
		b2Vec2 linearSeparation = b2Add( joint->deltaCenter, ds );
		b2Vec2 linearBias = b2MulSV( context->inv_h * joint->correctionFactor, linearSeparation );

		b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
		b2Vec2 b = b2MulMV( joint->linearMass, b2Add( Cdot, linearBias ) );
		b2Vec2 impulse = { -b.x, -b.y };

		b2Vec2 oldImpulse = joint->linearImpulse;
		float maxImpulse = context->h * joint->maxForce;
		joint->linearImpulse = b2Add( joint->linearImpulse, impulse );

		if ( b2LengthSquared( joint->linearImpulse ) > maxImpulse * maxImpulse )
		{
			joint->linearImpulse = b2Normalize( joint->linearImpulse );
			joint->linearImpulse.x *= maxImpulse;
			joint->linearImpulse.y *= maxImpulse;
		}

		impulse = b2Sub( joint->linearImpulse, oldImpulse );

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2DumpMotorJoint()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2MotorJointDef jd;\n");
	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", m_referenceAngle);
	b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
	b2Dump("  jd.damping = %.9g;\n", m_damping);
	b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
#endif
