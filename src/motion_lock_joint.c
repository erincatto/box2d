// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box2d/box2d.h"

void b2MotionLockJoint_EnableLinearLock( b2JointId jointId, int axis, bool flag )
{
	B2_CHECK_INPUT( 0 <= axis && axis < 2 );

	b2World* world = b2GetWorld( jointId.world0 );
	B2_REC( world, MotionLockJointEnableLinearLock, jointId, axis, flag );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_motionLockJoint );
	b2MotionLockJoint* joint = &base->motionLockJoint;
	if ( flag != joint->lockLinear[axis] )
	{
		joint->lockLinear[axis] = flag;
		( &joint->linearImpulse.x )[axis] = 0.0f;
	}
}

bool b2MotionLockJoint_IsLinearLockEnabled( b2JointId jointId, int axis )
{
	B2_CHECK_INPUT_RETURN( 0 <= axis && axis < 2, false );

	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_motionLockJoint );
	return base->motionLockJoint.lockLinear[axis];
}

b2Vec2 b2GetMotionLockJointForce( b2World* world, b2JointSim* base )
{
	b2WorldTransform transformA = b2GetBodyTransform( world, base->bodyIdA );
	b2Vec2 force = b2MulSV( world->inv_h, base->motionLockJoint.linearImpulse );
	force = b2RotateVector( base->localFrameA.q, force );
	return b2RotateVector( transformA.q, force );
}

static inline b2Vec2 b2GetMotionLockSeparation( const b2MotionLockJoint* joint, const b2BodyState* stateA,
												const b2BodyState* stateB, b2Vec2 rA, b2Vec2 rB )
{
	b2Vec2 dpA = b2Add( stateA->deltaPosition, b2Sub( rA, joint->frameA.p ) );
	b2Vec2 dpB = b2Add( stateB->deltaPosition, b2Sub( rB, joint->frameB.p ) );
	return b2Add( joint->separation, b2Sub( dpB, dpA ) );
}

void b2PrepareMotionLockJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_motionLockJoint );

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

	float iA = bodySimA->invInertia;
	float iB = bodySimB->invInertia;

	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = iA;
	base->invIB = iB;

	b2MotionLockJoint* joint = &base->motionLockJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// todo comment: the anchor difference is taken in world precision so a lock far from the
	// center of body A keeps its accuracy in double precision
	b2Pos posA = b2OffsetPos( bodySimA->center, joint->frameA.p );
	b2Pos posB = b2OffsetPos( bodySimB->center, joint->frameB.p );
	joint->separation = b2SubPos( posB, posA );

	bool anchorAtCenterB = joint->frameB.p.x == 0.0f && joint->frameB.p.y == 0.0f;
	joint->diagonal = iA == 0.0f && ( iB == 0.0f || anchorAtCenterB );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
	}
}

void b2WarmStartMotionLockJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_motionLockJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	b2BodyState dummyState = b2_identityBodyState;

	b2MotionLockJoint* joint = &base->motionLockJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );
	b2Vec2 d = b2GetMotionLockSeparation( joint, stateA, stateB, rA, rB );

	b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
	b2Vec2 P = b2RotateVector( qA, joint->linearImpulse );

	float LA = b2Cross( b2Add( rA, d ), P );
	float LB = b2Cross( rB, P );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, P );
		stateA->angularVelocity -= iA * LA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, P );
		stateB->angularVelocity += iB * LB;
	}
}

void b2SolveMotionLockJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_motionLockJoint );

	b2MotionLockJoint* joint = &base->motionLockJoint;

	int axisIndices[2];
	int count = 0;
	for ( int i = 0; i < 2; ++i )
	{
		if ( joint->lockLinear[i] )
		{
			axisIndices[count] = i;
			count += 1;
		}
	}

	if ( count == 0 )
	{
		return;
	}

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	b2BodyState dummyState = b2_identityBodyState;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	float massScale = 1.0f;
	float impulseScale = 0.0f;
	float biasRate = 0.0f;
	if ( useBias )
	{
		massScale = base->constraintSoftness.massScale;
		impulseScale = base->constraintSoftness.impulseScale;
		biasRate = base->constraintSoftness.biasRate;
	}

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );
	b2Vec2 d = b2GetMotionLockSeparation( joint, stateA, stateB, rA, rB );
	b2Vec2 rAd = b2Add( rA, d );

	b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
	b2Vec2 allAxes[2] = { b2Rot_GetXAxis( qA ), b2Rot_GetYAxis( qA ) };

	b2Vec2 vRel = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rAd ) ) );

	b2Vec2 axes[2];
	float sA[2];
	float sB[2];
	float rhs[2];
	float* oldImpulses = &joint->linearImpulse.x;
	float impulses[2] = { 0.0f, 0.0f };

	for ( int i = 0; i < count; ++i )
	{
		b2Vec2 axis = allAxes[axisIndices[i]];
		axes[i] = axis;
		sA[i] = b2Cross( rAd, axis );
		sB[i] = b2Cross( rB, axis );
		rhs[i] = b2Dot( vRel, axis ) + biasRate * b2Dot( d, axis );
	}

	float m = mA + mB;

	if ( joint->diagonal )
	{
		float invM = m > 0.0f ? 1.0f / m : 0.0f;
		for ( int i = 0; i < count; ++i )
		{
			impulses[i] = -massScale * invM * rhs[i] - impulseScale * oldImpulses[axisIndices[i]];
		}
	}
	else
	{
		float solution[2] = { 0.0f, 0.0f };
		if ( count == 1 )
		{
			float k = m + iA * sA[0] * sA[0] + iB * sB[0] * sB[0];
			solution[0] = k > 0.0f ? rhs[0] / k : 0.0f;
		}
		else
		{
			float k11 = m + iA * sA[0] * sA[0] + iB * sB[0] * sB[0];
			float k12 = iA * sA[0] * sA[1] + iB * sB[0] * sB[1];
			float k22 = m + iA * sA[1] * sA[1] + iB * sB[1] * sB[1];
			b2Mat22 k = { { k11, k12 }, { k12, k22 } };
			b2Vec2 s = b2Solve22( k, (b2Vec2){ rhs[0], rhs[1] } );
			solution[0] = s.x;
			solution[1] = s.y;
		}

		for ( int i = 0; i < count; ++i )
		{
			impulses[i] = -massScale * solution[i] - impulseScale * oldImpulses[axisIndices[i]];
		}
	}

	b2Vec2 P = b2Vec2_zero;
	float LA = 0.0f;
	float LB = 0.0f;
	for ( int i = 0; i < count; ++i )
	{
		oldImpulses[axisIndices[i]] += impulses[i];
		P = b2MulAdd( P, impulses[i], axes[i] );
		LA += impulses[i] * sA[i];
		LB += impulses[i] * sB[i];
	}

	vA = b2MulSub( vA, mA, P );
	wA -= iA * LA;
	vB = b2MulAdd( vB, mB, P );
	wB += iB * LB;

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

void b2DrawMotionLockJoint( b2DebugDraw* draw, b2JointSim* base, b2WorldTransform transformA, b2WorldTransform transformB,
							float drawScale )
{
	B2_ASSERT( base->type == b2_motionLockJoint );

	b2MotionLockJoint* joint = &base->motionLockJoint;

	b2WorldTransform frameA = b2MulWorldTransforms( transformA, base->localFrameA );
	b2WorldTransform frameB = b2MulWorldTransforms( transformB, base->localFrameB );

	float length = 0.25f * drawScale;
	b2Vec2 axes[2] = { b2Rot_GetXAxis( frameA.q ), b2Rot_GetYAxis( frameA.q ) };
	b2HexColor colors[2] = { b2_colorRed, b2_colorGreen };

	for ( int i = 0; i < 2; ++i )
	{
		if ( joint->lockLinear[i] == false )
		{
			continue;
		}

		b2Vec2 offset = b2MulSV( length, axes[i] );
		draw->DrawLineFcn( b2OffsetPos( frameA.p, b2Neg( offset ) ), b2OffsetPos( frameA.p, offset ), colors[i],
						   draw->context );
	}

	draw->DrawLineFcn( frameA.p, frameB.p, b2_colorDarkSeaGreen, draw->context );
}
