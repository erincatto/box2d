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
	// b2World* world = b2GetWorld( jointId.world0 );
	// B2_REC( world, PogoJointSetLinearHertz, jointId, hertz );
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
	// b2World* world = b2GetWorld( jointId.world0 );
	// B2_REC( world, PogoJointSetLinearHertz, jointId, hertz );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	joint->pogoJoint.hertz = hertz;
}

void b2PogoJoint_SetSpringDampingRatio( b2JointId jointId, float damping )
{
	// b2World* world = b2GetWorld( jointId.world0 );
	// B2_REC( world, PogoJointSetLinearDampingRatio, jointId, damping );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	joint->pogoJoint.dampingRatio = damping;
}

float b2PogoJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	return joint->pogoJoint.dampingRatio;
}

b2Vec2 b2GetPogoJointForce( b2World* world, b2JointSim* base )
{
	b2WorldTransform worldTransformB = b2GetBodyTransform( world, base->bodyIdB );
	b2Rot qB = b2MulRot( worldTransformB.q, base->localFrameB.q );
	b2Vec2 axis = b2RotateVector( qB, b2_pogoAxis );
	b2Vec2 force = b2MulSV( world->inv_h * base->pogoJoint.impulse, axis );
	return force;
}

#if 0

typedef struct CastResult
{
	int bodyIdB;
	int hitBodyId;
	b2Pos point;
	b2Vec2 normal;
	float fraction;
	bool hit;
} CastContext;

static float CastCallback( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	CastContext* result = context;
	b2BodyId bodyId = b2Shape_GetBody( shapeId );
	if ( bodyId.index1 == result->bodyIdB + 1 )
	{
		return -1.0f;
	}

	result->hitBodyId = bodyId.index1 - 1;
	result->point = point;
	result->normal = normal;
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

void b2UpdatePogo( b2World* world, b2JointSim* base )
{
	B2_ASSERT( base->type == b2_pogoJoint );
	b2PogoJoint* pogoJoint = &base->pogoJoint;

	int idB = base->bodyIdB;

	b2Body* bodyB = b2Array_Get( world->bodies, idB );
	B2_ASSERT( bodyB->setIndex == b2_awakeSet );

	b2SolverSet* setB = b2Array_Get( world->solverSets, bodyB->setIndex );
	int localIndexB = bodyB->localIndex;
	b2BodySim* bodySimB = b2Array_Get( setB->bodySims, localIndexB );
	b2Vec2 axis = b2RotateVector( base->localFrameB.q, b2_pogoAxis );
	axis = b2RotateVector( bodySimB->transform.q, axis );

	b2Vec2 offset = b2RotateVector( bodySimB->transform.q, base->localFrameB.p );
	b2Pos origin = b2OffsetPos( bodySimB->transform.p, offset );

	b2WorldId worldId = {
		.index1 = world->worldId + 1,
		.generation = world->generation,
	};

	b2Vec2 perp = b2LeftPerp( axis );
	b2ShapeProxy proxy = { 0 };
	proxy.points[0] = b2MulSV( -0.5f * pogoJoint->width, perp );
	proxy.points[1] = b2MulSV( 0.5f * pogoJoint->width, perp );
	proxy.count = 2;
	proxy.radius = 0.0f;

	b2Vec2 translation = b2MulSV( -pogoJoint->restLength, axis );

	b2QueryFilter filter = b2DefaultQueryFilter();
	CastContext context = { 0 };
	context.bodyIdB = idB;
	b2World_CastShape( worldId, origin, &proxy, translation, filter, CastCallback, &context );

	if ( context.hit == true )
	{
		// todo this is broken. There is no island connection.
		B2_ASSERT( context.hitBodyId != idB );
		base->bodyIdA = context.hitBodyId;
		b2Pos pogoBottom = b2OffsetPos( origin, b2MulSV( context.fraction, translation ) );
		b2WorldTransform worldTransformA = b2GetBodyTransform( world, context.hitBodyId );
		base->localFrameA.p = b2InvTransformWorldPoint( worldTransformA, pogoBottom );
	}
	else
	{
		base->bodyIdA = B2_NULL_INDEX;
		pogoJoint->impulse = 0.0f;
	}
}
#endif

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

	joint->spring = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	b2Vec2 axis = b2RotateVector( joint->frameB.q, b2_pogoAxis );

	// compute effective mass
	float crA = b2Cross( rA, axis );
	float crB = b2Cross( rB, axis );
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

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Vec2 axis = b2RotateVector( joint->frameB.q, b2_pogoAxis );
	b2Vec2 linearImpulse = b2MulSV( joint->impulse, axis );

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

void b2SolvePogoJoint( b2JointSim* base, b2StepContext* context )
{
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

	b2Vec2 axis = b2RotateVector( joint->frameB.q, b2_pogoAxis );
	axis = b2RotateVector( stateB->deltaRotation, axis );

	b2Vec2 dcA = stateA->deltaPosition;
	b2Vec2 dcB = stateB->deltaPosition;
	b2Vec2 d = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );
	float c = b2Dot( axis, d ) - joint->restLength;

	float bias = joint->spring.biasRate * c;
	float massScale = joint->spring.massScale;
	float impulseScale = joint->spring.impulseScale;

	b2Vec2 vr = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	float cdot = b2Dot( axis, vr );

	float oldImpulse = joint->impulse;
	float impulse = -massScale * joint->linearMass * ( cdot + bias ) - impulseScale * oldImpulse;
	joint->impulse = joint->impulse + impulse;
	impulse = joint->impulse - oldImpulse;

	b2Vec2 P = b2MulSV( impulse, axis );
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

void b2DrawPogoJoint( b2DebugDraw* draw, b2JointSim* base, b2WorldTransform transformA, b2WorldTransform transformB )
{
	B2_ASSERT( base->type == b2_pogoJoint );

	// b2PogoJoint* joint = &base->pogoJoint;

	b2WorldTransform frameA = b2MulWorldTransforms( transformA, base->localFrameA );
	b2WorldTransform frameB = b2MulWorldTransforms( transformB, base->localFrameB );

	draw->DrawLineFcn( frameA.p, frameB.p, b2_colorDimGray, draw->context );
}
