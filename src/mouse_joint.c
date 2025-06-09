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

void b2MouseJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	base->mouseJoint.hertz = hertz;
}

float b2MouseJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	return base->mouseJoint.hertz;
}

void b2MouseJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	base->mouseJoint.dampingRatio = dampingRatio;
}

float b2MouseJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	return base->mouseJoint.dampingRatio;
}

void b2MouseJoint_SetMaxForce( b2JointId jointId, float maxForce )
{
	B2_ASSERT( b2IsValidFloat( maxForce ) && maxForce >= 0.0f );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	base->mouseJoint.maxForce = maxForce;
}

float b2MouseJoint_GetMaxForce( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	return base->mouseJoint.maxForce;
}

b2Vec2 b2GetMouseJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->mouseJoint.linearImpulse );
	return force;
}

float b2GetMouseJointTorque( b2World* world, b2JointSim* base )
{
	return world->inv_h * base->mouseJoint.angularImpulse;
}

void b2PrepareMouseJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_mouseJoint );

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

	b2MouseJoint* joint = &base->mouseJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	joint->linearSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	float angularHertz = 0.5f;
	float angularDampingRatio = 0.1f;
	joint->angularSoftness = b2MakeSoft( angularHertz, angularDampingRatio, context->h );

	b2Vec2 rA = joint->frameA.p;
	b2Vec2 rB = joint->frameB.p;

	// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//   = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//     [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
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

void b2WarmStartMouseJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_mouseJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	b2MouseJoint* joint = &base->mouseJoint;

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

void b2SolveMouseJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_mouseJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2MouseJoint* joint = &base->mouseJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// Softness with no bias to reduce rotation speed
	{
		float massScale = joint->angularSoftness.massScale;
		float impulseScale = joint->angularSoftness.impulseScale;

		float Cdot = wB - wA;
		float impulse = -massScale * joint->angularMass * Cdot - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	float maxImpulse = joint->maxForce * context->h;

	{
		b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
		b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

		b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );

		b2Vec2 dcA = stateA->deltaPosition;
		b2Vec2 dcB = stateB->deltaPosition;
		b2Vec2 C = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );
		b2Vec2 bias = b2MulSV( joint->linearSoftness.biasRate, C );

		float massScale = joint->linearSoftness.massScale;
		float impulseScale = joint->linearSoftness.impulseScale;

		b2Vec2 b = b2MulMV( joint->linearMass, b2Add( Cdot, bias ) );

		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;

		b2Vec2 oldImpulse = joint->linearImpulse;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;

		float lengthSquared = b2LengthSquared( joint->linearImpulse );
		if ( lengthSquared > maxImpulse * maxImpulse )
		{
			joint->linearImpulse = b2MulSV( maxImpulse, b2Normalize( joint->linearImpulse ) );
		}

		impulse.x = joint->linearImpulse.x - oldImpulse.x;
		impulse.y = joint->linearImpulse.y - oldImpulse.y;

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
