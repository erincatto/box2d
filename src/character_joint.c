// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "joint.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box2d/box2d.h"

void b2CharacterJoint_SetLinearHertz( b2JointId jointId, float hertz )
{
	// b2World* world = b2GetWorld( jointId.world0 );
	// B2_REC( world, CharacterJointSetLinearHertz, jointId, hertz );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_characterJoint );
	joint->characterJoint.linearHertz = hertz;
}

float b2CharacterJoint_GetLinearHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_characterJoint );
	return joint->characterJoint.linearHertz;
}

void b2CharacterJoint_SetLinearDampingRatio( b2JointId jointId, float damping )
{
	// b2World* world = b2GetWorld( jointId.world0 );
	// B2_REC( world, CharacterJointSetLinearDampingRatio, jointId, damping );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_characterJoint );
	joint->characterJoint.linearDampingRatio = damping;
}

float b2CharacterJoint_GetLinearDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_characterJoint );
	return joint->characterJoint.linearDampingRatio;
}

void b2CharacterJoint_SetMaxSpringForce( b2JointId jointId, float maxForce )
{
	// b2World* world = b2GetWorld( jointId.world0 );
	// B2_REC( world, CharacterJointSetMaxSpringForce, jointId, maxForce );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_characterJoint );
	joint->characterJoint.maxSpringForce = b2MaxFloat( 0.0f, maxForce );
}

float b2CharacterJoint_GetMaxSpringForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_characterJoint );
	return joint->characterJoint.maxSpringForce;
}

b2Vec2 b2GetCharacterJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->characterJoint.linearSpringImpulse );
	return force;
}

float b2GetCharacterJointTorque( b2World* world, b2JointSim* base )
{
	B2_UNUSED( world, base );
	return 0.0f;
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

void b2PrepareCharacterJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_characterJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2Array_Get( world->bodies, idA );
	b2Body* bodyB = b2Array_Get( world->bodies, idB );

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );

	b2SolverSet* setA = b2Array_Get( world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2Array_Get( setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2Array_Get( setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float iA = bodySimA->invInertia;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2CharacterJoint* joint = &base->characterJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );

	b2Vec2 rA = joint->frameA.p;
	b2Vec2 rB = joint->frameB.p;

	joint->linearSpring = b2MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );

	b2Mat22 kl;
	kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	kl.cy.x = kl.cx.y;
	kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	joint->linearMass = b2GetInverse22( kl );

	if ( context->enableWarmStarting == false )
	{
		joint->linearSpringImpulse = b2Vec2_zero;
	}
}

void b2WarmStartCharacterJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_characterJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	b2CharacterJoint* joint = &base->characterJoint;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );
	b2Vec2 linearImpulse = joint->linearSpringImpulse;

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, linearImpulse );
		stateA->angularVelocity -= iA * b2Cross( rA, linearImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, linearImpulse );
		stateB->angularVelocity += iB * b2Cross( rB, linearImpulse );
	}
}

void b2SolveCharacterJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_characterJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2CharacterJoint* joint = &base->characterJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

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
		vB = b2MulAdd( vB, mB, impulse );
	}

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = vA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = vB;
	}
}
