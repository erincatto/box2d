// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box2d/box2d.h"

void b2MotorJoint_SetLinearVelocity( b2JointId jointId, b2Vec2 velocity )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.linearVelocity = velocity;
}

b2Vec2 b2MotorJoint_GetLinearVelocity( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.linearVelocity;
}

void b2MotorJoint_SetAngularVelocity( b2JointId jointId, float velocity )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.angularVelocity = velocity;
}

float b2MotorJoint_GetAngularVelocity( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.angularVelocity;
}

void b2MotorJoint_SetMaxVelocityTorque( b2JointId jointId, float maxTorque )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.maxVelocityTorque = maxTorque;
}

float b2MotorJoint_GetMaxVelocityTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.maxVelocityTorque;
}

void b2MotorJoint_SetMaxVelocityForce( b2JointId jointId, float maxForce )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.maxVelocityForce = maxForce;
}

float b2MotorJoint_GetMaxVelocityForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.maxVelocityForce;
}

void b2MotorJoint_SetLinearHertz( b2JointId jointId, float hertz )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.linearHertz = hertz;
}

float b2MotorJoint_GetLinearHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.linearHertz;
}

void b2MotorJoint_SetLinearDampingRatio( b2JointId jointId, float damping )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.linearDampingRatio = damping;
}

float b2MotorJoint_GetLinearDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.linearDampingRatio;
}

void b2MotorJoint_SetAngularHertz( b2JointId jointId, float hertz )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.angularHertz = hertz;
}

float b2MotorJoint_GetAngularHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.angularHertz;
}

void b2MotorJoint_SetAngularDampingRatio( b2JointId jointId, float damping )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.angularDampingRatio = damping;
}

float b2MotorJoint_GetAngularDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.angularDampingRatio;
}

void b2MotorJoint_SetMaxSpringForce( b2JointId jointId, float maxForce )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.maxSpringForce = b2MaxFloat( 0.0f, maxForce );
}

float b2MotorJoint_GetMaxSpringForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.maxSpringForce;
}

void b2MotorJoint_SetMaxSpringTorque( b2JointId jointId, float maxTorque )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	joint->motorJoint.maxSpringTorque = b2MaxFloat( 0.0f, maxTorque );
}

float b2MotorJoint_GetMaxSpringTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_motorJoint );
	return joint->motorJoint.maxSpringTorque;
}

b2Vec2 b2GetMotorJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, b2Add( base->motorJoint.linearVelocityImpulse, base->motorJoint.linearSpringImpulse ) );
	return force;
}

float b2GetMotorJointTorque( b2World* world, b2JointSim* base )
{
	return world->inv_h * ( base->motorJoint.angularVelocityImpulse + base->motorJoint.angularSpringImpulse );
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

	joint->linearSpring = b2MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );
	joint->angularSpring = b2MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );

	b2Mat22 kl;
	kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	kl.cy.x = kl.cx.y;
	kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	joint->linearMass = b2GetInverse22( kl );

	float ka = iA + iB;
	joint->angularMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->linearVelocityImpulse = b2Vec2_zero;
		joint->angularVelocityImpulse = 0.0f;
		joint->linearSpringImpulse = b2Vec2_zero;
		joint->angularSpringImpulse = 0.0f;
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

	b2Vec2 linearImpulse = b2Add( joint->linearVelocityImpulse, joint->linearSpringImpulse );
	float angularImpulse = joint->angularVelocityImpulse + joint->angularSpringImpulse;

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, linearImpulse );
		stateA->angularVelocity -= iA * ( b2Cross( rA, linearImpulse ) + angularImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, linearImpulse );
		stateB->angularVelocity += iB * ( b2Cross( rB, linearImpulse ) + angularImpulse );
	}
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

	// angular spring
	if ( joint->maxSpringTorque > 0.0f && joint->angularHertz > 0.0f )
	{
		b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		b2Rot relQ = b2InvMulRot( qA, qB );

		float c = b2Rot_GetAngle( relQ );
		float bias = joint->angularSpring.biasRate * c;
		float massScale = joint->angularSpring.massScale;
		float impulseScale = joint->angularSpring.impulseScale;

		float cdot = wB - wA;

		float maxImpulse = context->h * joint->maxSpringTorque;
		float oldImpulse = joint->angularSpringImpulse;
		float impulse = -massScale * joint->angularMass * ( cdot + bias ) - impulseScale * oldImpulse;
		joint->angularSpringImpulse = b2ClampFloat( oldImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint->angularSpringImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// angular velocity
	if ( joint->maxVelocityTorque > 0.0 )
	{
		float cdot = wB - wA - joint->angularVelocity;
		float impulse = -joint->angularMass * cdot;

		float maxImpulse = context->h * joint->maxVelocityTorque;
		float oldImpulse = joint->angularVelocityImpulse;
		joint->angularVelocityImpulse = b2ClampFloat( oldImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint->angularVelocityImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	// linear spring
	if ( joint->maxSpringForce > 0.0f && joint->linearHertz > 0.0f )
	{
		b2Vec2 dcA = stateA->deltaPosition;
		b2Vec2 dcB = stateB->deltaPosition;
		b2Vec2 c = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

		b2Vec2 bias = b2MulSV( joint->linearSpring.biasRate, c );
		float massScale = joint->linearSpring.massScale;
		float impulseScale = joint->linearSpring.impulseScale;

		b2Vec2 cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
		cdot = b2Add( cdot, bias );

		// Updating the effective mass here may be overkill
		b2Mat22 kl;
		kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		kl.cy.x = kl.cx.y;
		kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		joint->linearMass = b2GetInverse22( kl );

		b2Vec2 b = b2MulMV( joint->linearMass, cdot );

		b2Vec2 oldImpulse = joint->linearSpringImpulse;
		b2Vec2 impulse = {
			-massScale * b.x - impulseScale * oldImpulse.x,
			-massScale * b.y - impulseScale * oldImpulse.y,
		};

		float maxImpulse = context->h * joint->maxSpringForce;
		joint->linearSpringImpulse = b2Add( joint->linearSpringImpulse, impulse );

		if ( b2LengthSquared( joint->linearSpringImpulse ) > maxImpulse * maxImpulse )
		{
			joint->linearSpringImpulse = b2Normalize( joint->linearSpringImpulse );
			joint->linearSpringImpulse.x *= maxImpulse;
			joint->linearSpringImpulse.y *= maxImpulse;
		}

		impulse = b2Sub( joint->linearSpringImpulse, oldImpulse );

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}

	// linear velocity
	if ( joint->maxVelocityForce > 0.0f )
	{
		b2Vec2 cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
		cdot = b2Sub( cdot, joint->linearVelocity );
		b2Vec2 b = b2MulMV( joint->linearMass, cdot );
		b2Vec2 impulse = { -b.x, -b.y };

		b2Vec2 oldImpulse = joint->linearVelocityImpulse;
		float maxImpulse = context->h * joint->maxVelocityForce;
		joint->linearVelocityImpulse = b2Add( joint->linearVelocityImpulse, impulse );

		if ( b2LengthSquared( joint->linearVelocityImpulse ) > maxImpulse * maxImpulse )
		{
			joint->linearVelocityImpulse = b2Normalize( joint->linearVelocityImpulse );
			joint->linearVelocityImpulse.x *= maxImpulse;
			joint->linearVelocityImpulse.y *= maxImpulse;
		}

		impulse = b2Sub( joint->linearVelocityImpulse, oldImpulse );

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
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
