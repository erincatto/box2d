// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "solver_set.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

void b2MouseJoint_SetTarget( b2JointId jointId, b2Vec2 target )
{
	B2_ASSERT( b2IsValidVec2( target ) );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	base->mouseJoint.targetA = target;
}

b2Vec2 b2MouseJoint_GetTarget( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_mouseJoint );
	return base->mouseJoint.targetA;
}

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

	// chase body id to the solver set where the body lives
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );

	B2_ASSERT( bodyB->setIndex == b2_awakeSet );
	b2SolverSet* setB = b2SolverSetArray_Get( &world->solverSets, bodyB->setIndex );

	int localIndexB = bodyB->localIndex;
	b2BodySim* bodySimB = b2BodySimArray_Get( &setB->bodySims, localIndexB );

	base->invMassB = bodySimB->invMass;
	base->invIB = bodySimB->invInertia;

	b2MouseJoint* joint = &base->mouseJoint;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;
	joint->anchorB = b2RotateVector( bodySimB->transform.q, b2Sub( base->localOriginAnchorB, bodySimB->localCenter ) );

	joint->linearSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	float angularHertz = 0.5f;
	float angularDampingRatio = 0.1f;
	joint->angularSoftness = b2MakeSoft( angularHertz, angularDampingRatio, context->h );

	b2Vec2 rB = joint->anchorB;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//   = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//     [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	b2Mat22 K;
	K.cx.x = mB + iB * rB.y * rB.y;
	K.cx.y = -iB * rB.x * rB.y;
	K.cy.x = K.cx.y;
	K.cy.y = mB + iB * rB.x * rB.x;

	joint->linearMass = b2GetInverse22( K );
	joint->deltaCenter = b2Sub( bodySimB->center, joint->targetA );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartMouseJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_mouseJoint );

	float mB = base->invMassB;
	float iB = base->invIB;

	b2MouseJoint* joint = &base->mouseJoint;

	b2BodyState* stateB = context->states + joint->indexB;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	b2Rot dqB = stateB->deltaRotation;
	b2Vec2 rB = b2RotateVector( dqB, joint->anchorB );

	vB = b2MulAdd( vB, mB, joint->linearImpulse );
	wB += iB * ( b2Cross( rB, joint->linearImpulse ) + joint->angularImpulse );

	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

void b2SolveMouseJoint( b2JointSim* base, b2StepContext* context )
{
	float mB = base->invMassB;
	float iB = base->invIB;

	b2MouseJoint* joint = &base->mouseJoint;
	b2BodyState* stateB = context->states + joint->indexB;

	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// Softness with no bias to reduce rotation speed
	{
		float massScale = joint->angularSoftness.massScale;
		float impulseScale = joint->angularSoftness.impulseScale;

		float impulse = iB > 0.0f ? -wB / iB : 0.0f;
		impulse = massScale * impulse - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;

		wB += iB * impulse;
	}

	float maxImpulse = joint->maxForce * context->h;

	{
		b2Rot dqB = stateB->deltaRotation;
		b2Vec2 rB = b2RotateVector( dqB, joint->anchorB );
		b2Vec2 Cdot = b2Add( vB, b2CrossSV( wB, rB ) );

		b2Vec2 separation = b2Add( b2Add( stateB->deltaPosition, rB ), joint->deltaCenter );
		b2Vec2 bias = b2MulSV( joint->linearSoftness.biasRate, separation );

		float massScale = joint->linearSoftness.massScale;
		float impulseScale = joint->linearSoftness.impulseScale;

		b2Vec2 b = b2MulMV( joint->linearMass, b2Add( Cdot, bias ) );

		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;

		b2Vec2 oldImpulse = joint->linearImpulse;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;

		float mag = b2Length( joint->linearImpulse );
		if ( mag > maxImpulse )
		{
			joint->linearImpulse = b2MulSV( maxImpulse, b2Normalize( joint->linearImpulse ) );
		}

		impulse.x = joint->linearImpulse.x - oldImpulse.x;
		impulse.y = joint->linearImpulse.y - oldImpulse.y;

		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}

	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}
