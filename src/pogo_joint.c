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

static const b2Vec2 b2_pogoAxis = { 0.0f, 1.0f };

void b2PogoJoint_SetRestLength( b2JointId jointId, float length )
{
	b2World* world = b2GetWorld( jointId.world0 );
	B2_REC( world, PogoJointSetRestLength, jointId, length );
	
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	joint->pogoJoint.restLength = length;
}

float b2PogoJoint_GetRestLength( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	return joint->pogoJoint.restLength;
}

float b2PogoJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	return joint->pogoJoint.hertz;
}

void b2PogoJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	 b2World* world = b2GetWorld( jointId.world0 );
	 B2_REC( world, PogoJointSetSpringHertz, jointId, hertz );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	joint->pogoJoint.hertz = hertz;
}

void b2PogoJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	 b2World* world = b2GetWorld( jointId.world0 );
	 B2_REC( world, PogoJointSetSpringDampingRatio, jointId, dampingRatio );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	joint->pogoJoint.dampingRatio = dampingRatio;
}

float b2PogoJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	return joint->pogoJoint.dampingRatio;
}

float b2PogoJoint_GetLength( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_pogoJoint );

	// Relative to body A so the difference stays in float precision far from the origin
	b2WorldTransform wxfA = b2GetBodyTransform( world, jointSim->bodyIdA );
	b2Transform transformA = b2ToRelativeTransform( wxfA, wxfA.p );
	b2Transform transformB = b2ToRelativeTransform( b2GetBodyTransform( world, jointSim->bodyIdB ), wxfA.p );

	b2Vec2 axis = b2RotateVector( jointSim->localFrameB.q, b2_pogoAxis );
	axis = b2RotateVector( transformB.q, axis );
	b2Vec2 pA = b2TransformPoint( transformA, jointSim->localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, jointSim->localFrameB.p );
	b2Vec2 d = b2Sub( pB, pA );
	float length = b2Dot( d, axis );
	return length;
}

float b2PogoJoint_GetImpulse( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	return joint->pogoJoint.impulse;
}

float b2PogoJoint_GetVelocity( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	return joint->pogoJoint.velocity;
}

b2Vec2 b2GetPogoJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h * base->pogoJoint.impulse, base->pogoJoint.normal );
	return force;
}

void b2PreparePogoJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_pogoJoint );

	b2World* world = context->world;

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

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

	b2PogoJoint* joint = &base->pogoJoint;
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

	// compute effective mass
	float crA = b2Cross( rA, joint->normal );
	float crB = b2Cross( rB, joint->normal );
	float k = mA + mB + iA * crA * crA + iB * crB * crB;
	joint->linearMass = k > 0.0f ? 1.0f / k : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->impulse = 0.0f;
	}
}

void b2WarmStartPogoJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_pogoJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	b2PogoJoint* joint = &base->pogoJoint;

	if ( joint->hertz == 0.0f )
	{
		joint->impulse = 0.0f;
		return;
	}

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Vec2 linearImpulse = b2MulSV( joint->impulse, joint->normal );

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

void b2SolvePogoJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_UNUSED( useBias );

	B2_ASSERT( base->type == b2_pogoJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2PogoJoint* joint = &base->pogoJoint;
	if ( joint->hertz == 0.0f )
	{
		joint->impulse = 0.0f;
		return;
	}

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );


	float bias = 0.0f;
	if ( useBias )
	{
		// This is a custom pogo position correction designed to avoid feeding velocity into
		// the mover body. Otherwise the mover can make huge jumps going up stairs. This still
		// works with continuous collision.
		// However, this position delta is not observer contact forces and can lead to tunnelling.
		b2Vec2 dcA = stateA->deltaPosition;
		b2Vec2 dcB = stateB->deltaPosition;
		b2Vec2 d = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

		// The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
		b2Vec2 pogoAxis = b2RotateVector( joint->frameB.q, b2_pogoAxis );
		float c = b2Dot( pogoAxis, d ) - joint->restLength;

		// This could be divided by dot(joint->normal, pogoAxis) to account for the constraint
		// direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
		joint->velocity = b2SpringDamper( joint->hertz, joint->dampingRatio, c, joint->velocity, context->h );
		bias = -joint->velocity;
	}

	b2Vec2 vr = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	float cdot = b2Dot( joint->normal, vr );

	float maxTensionImpulse = context->h * joint->maxTensionForce;
	float maxCompressionImpulse = context->h * joint->maxCompressionForce;
	float oldImpulse = joint->impulse;
	float impulse = -joint->linearMass * (cdot + bias);
	joint->impulse = b2ClampFloat( joint->impulse + impulse, -maxTensionImpulse, maxCompressionImpulse );
	impulse = joint->impulse - oldImpulse;

	b2Vec2 P = b2MulSV( impulse, joint->normal );
	vA = b2MulSub( vA, mA, P );
	wA -= iA * b2Cross( rA, P );
	vB = b2MulAdd( vB, mB, P );
	wB += iB * b2Cross( rB, P );

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
