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

void b2MoverJoint_SetLinearVelocity( b2JointId jointId, b2Vec2 velocity )
{
	b2World* world = b2GetWorld( jointId.world0 );
	B2_REC( world, MoverJointSetLinearVelocity, jointId, velocity );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_moverJoint );
	joint->moverJoint.linearVelocity = velocity;
}

b2Vec2 b2MoverJoint_GetLinearVelocity( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_moverJoint );
	return joint->moverJoint.linearVelocity;
}

void b2MoverJoint_SetMaxVelocityForce( b2JointId jointId, b2Vec2 maxForce )
{
	b2World* world = b2GetWorld( jointId.world0 );
	B2_REC( world, MoverJointSetMaxVelocityForce, jointId, maxForce );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_moverJoint );
	joint->moverJoint.maxVelocityForce = maxForce;
}

b2Vec2 b2MoverJoint_GetMaxVelocityForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_moverJoint );
	return joint->moverJoint.maxVelocityForce;
}

b2Vec2 b2GetMoverJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->moverJoint.linearVelocityImpulse );
	return force;
}

void b2PrepareMoverJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_moverJoint );

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
	float mB = bodySimB->invMass;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = 0.0f;
	base->invIB = 0.0f;

	b2MoverJoint* joint = &base->moverJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	float k = mA + mB;
	joint->linearMass = k > 0.0f ? 1.0f / k : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->linearVelocityImpulse = b2Vec2_zero;
	}
}

void b2WarmStartMoverJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_moverJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;

	b2MoverJoint* joint = &base->moverJoint;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearVelocityImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearVelocityImpulse );
	}
}

void b2SolveMoverJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_moverJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2MoverJoint* joint = &base->moverJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	b2Vec2 vB = stateB->linearVelocity;

	// linear velocity
	if ( joint->maxVelocityForce.x > 0.0f || joint->maxVelocityForce.y > 0.0f )
	{
		b2Vec2 cdot = b2Sub( vB, vA );
		cdot = b2Sub( cdot, joint->linearVelocity );
		b2Vec2 b = b2MulSV( joint->linearMass, cdot );
		b2Vec2 impulse = { -b.x, -b.y };

		b2Vec2 oldImpulse = joint->linearVelocityImpulse;
		joint->linearVelocityImpulse = b2Add( joint->linearVelocityImpulse, impulse );
		b2Vec2 maxImpulse = b2MulSV( context->h, joint->maxVelocityForce );

		// clamp x and y separately
		joint->linearVelocityImpulse = b2Clamp( joint->linearVelocityImpulse, b2Neg( maxImpulse ), maxImpulse );

		impulse = b2Sub( joint->linearVelocityImpulse, oldImpulse );
		vA = b2MulSub( vA, mA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
	}
	else
	{
		joint->linearVelocityImpulse = b2Vec2_zero;
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
