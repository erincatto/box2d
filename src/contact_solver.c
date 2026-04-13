// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"

#include <stddef.h>

void b2PrepareContactConstraints( b2StepContext* context, int* contactIds, b2ContactConstraint* constraints, int count )
{
	b2World* world = context->world;
	b2Body* bodies = world->bodies.data;
	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	for ( int i = 0; i < count; ++i )
	{
		b2Contact* contactSim = b2ContactArray_Get( &world->contacts, contactIds[i] );
		b2Body* bodyA = bodies + contactSim->edges[0].bodyId;
		b2Body* bodyB = bodies + contactSim->edges[1].bodyId;
		int indexA = bodyA->stateIndex;
		int indexB = bodyB->stateIndex;

		const b2Manifold* manifold = &contactSim->manifold;
		int pointCount = manifold->pointCount;
		B2_ASSERT( 0 < pointCount && pointCount <= 2 );

		b2ContactConstraint* constraint = constraints + i;
		constraint->stateIndexA = indexA;
		constraint->stateIndexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contactSim->friction;
		constraint->restitution = contactSim->restitution;
		constraint->rollingResistance = contactSim->rollingResistance;
		constraint->rollingImpulse = warmStartScale * manifold->rollingImpulse;
		constraint->tangentSpeed = contactSim->tangentSpeed;
		constraint->pointCount = pointCount;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		float mA = bodyA->invMass;
		float iA = bodyA->invInertia;

		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;
		float mB = bodyB->invMass;
		float iB = bodyB->invInertia;

		{
			float k = iA + iB;
			constraint->rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
		}

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( normal );

		for ( int j = 0; j < pointCount; ++j )
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ContactConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = warmStartScale * mp->normalImpulse;
			cp->tangentImpulse = warmStartScale * mp->tangentImpulse;
			cp->totalNormalImpulse = 0.0f;

			b2Vec2 rA = mp->anchorA;
			b2Vec2 rB = mp->anchorB;
			cp->anchorA = rA;
			cp->anchorB = rB;
			cp->baseSeparation = mp->separation - b2Dot( b2Sub( rB, rA ), normal );

			float rnA = b2Cross( rA, normal );
			float rnB = b2Cross( rB, normal );
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			float rtA = b2Cross( rA, tangent );
			float rtB = b2Cross( rB, tangent );
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			cp->relativeVelocity = b2Dot( normal, b2Sub( vrB, vrA ) );
		}
	}
}

void b2WarmStartContactConstraints( b2StepContext* context, b2ContactConstraint* constraints, int count )
{
	b2BodyState* states = context->states;
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < count; ++i )
	{
		if ( i + 1 < count )
		{
			b2ContactConstraint* next = constraints + ( i + 1 );
			if ( next->stateIndexA != B2_NULL_INDEX )
				b2Prefetch( states + next->stateIndexA );
			if ( next->stateIndexB != B2_NULL_INDEX )
				b2Prefetch( states + next->stateIndexB );
		}

		b2ContactConstraint* constraint = constraints + i;

		int indexA = constraint->stateIndexA;
		int indexB = constraint->stateIndexB;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;

		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		float mA = stateA->invMass;
		float iA = stateA->invInertia;

		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		float mB = stateB->invMass;
		float iB = stateB->invInertia;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( normal );
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			b2Vec2 P = b2Add( b2MulSV( cp->normalImpulse, normal ), b2MulSV( cp->tangentImpulse, tangent ) );
			cp->totalNormalImpulse += cp->normalImpulse;

			wA -= iA * b2Cross( rA, P );
			vA = b2MulAdd( vA, -mA, P );
			wB += iB * b2Cross( rB, P );
			vB = b2MulAdd( vB, mB, P );
		}

		wA -= iA * constraint->rollingImpulse;
		wB += iB * constraint->rollingImpulse;

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
}

void b2SolveContactConstraints( b2StepContext* context, b2ContactConstraint* constraints, int count, float inv_h,
								float contactSpeed, bool useBias )
{
	b2BodyState dummyState = b2_identityBodyState;

	b2Softness contactSoftness = context->contactSoftness;
	b2Softness staticSoftness = context->staticSoftness;
	b2BodyState* states = context->states;

	for ( int i = 0; i < count; ++i )
	{
		if ( i + 1 < count )
		{
			b2ContactConstraint* next = constraints + ( i + 1 );
			if ( next->stateIndexA != B2_NULL_INDEX )
				b2Prefetch( states + next->stateIndexA );
			if ( next->stateIndexB != B2_NULL_INDEX )
				b2Prefetch( states + next->stateIndexB );
		}

		b2ContactConstraint* constraint = constraints + i;

		int indexA = constraint->stateIndexA;
		int indexB = constraint->stateIndexB;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		float mA = stateA->invMass;
		float iA = stateA->invInertia;
		b2Rot dqA = stateA->deltaRotation;

		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		float mB = stateB->invMass;
		float iB = stateB->invInertia;
		b2Rot dqB = stateB->deltaRotation;

		b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( normal );
		float friction = constraint->friction;
		b2Softness softness = ( indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX ) ? staticSoftness : contactSoftness;

		int pointCount = constraint->pointCount;
		float totalNormalImpulse = 0.0f;

		// Non-penetration
		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, rB ), b2RotateVector( dqA, rA ) ) );
			float s = cp->baseSeparation + b2Dot( ds, normal );

			float velocityBias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( s > 0.0f )
			{
				velocityBias = s * inv_h;
			}
			else if ( useBias )
			{
				velocityBias = b2MaxFloat( softness.massScale * softness.biasRate * s, -contactSpeed );
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			float impulse = -cp->normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp->normalImpulse;
			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;
			cp->totalNormalImpulse += impulse;

			totalNormalImpulse += newImpulse;

			b2Vec2 P = b2MulSV( impulse, normal );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		// Friction
		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			float vt = b2Dot( b2Sub( vrB, vrA ), tangent ) - constraint->tangentSpeed;

			float impulse = cp->tangentMass * ( -vt );
			float maxFriction = friction * cp->normalImpulse;
			float newImpulse = b2ClampFloat( cp->tangentImpulse + impulse, -maxFriction, maxFriction );
			impulse = newImpulse - cp->tangentImpulse;
			cp->tangentImpulse = newImpulse;

			b2Vec2 P = b2MulSV( impulse, tangent );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		// Rolling resistance
		if ( constraint->rollingResistance > 0.0f )
		{
			float deltaLambda = -constraint->rollingMass * ( wB - wA );
			float lambda = constraint->rollingImpulse;
			float maxLambda = constraint->rollingResistance * totalNormalImpulse;
			constraint->rollingImpulse = b2ClampFloat( lambda + deltaLambda, -maxLambda, maxLambda );
			deltaLambda = constraint->rollingImpulse - lambda;

			wA -= iA * deltaLambda;
			wB += iB * deltaLambda;
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
}

void b2ApplyContactRestitution( b2StepContext* context, b2ContactConstraint* constraints, int count, float threshold )
{
	b2BodyState* states = context->states;
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < count; ++i )
	{
		if ( i + 1 < count )
		{
			b2ContactConstraint* next = constraints + ( i + 1 );
			if ( next->stateIndexA != B2_NULL_INDEX )
				b2Prefetch( states + next->stateIndexA );
			if ( next->stateIndexB != B2_NULL_INDEX )
				b2Prefetch( states + next->stateIndexB );
		}

		b2ContactConstraint* constraint = constraints + i;
		float restitution = constraint->restitution;
		if ( restitution == 0.0f )
		{
			continue;
		}

		int indexA = constraint->stateIndexA;
		int indexB = constraint->stateIndexB;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		float mA = stateA->invMass;
		float iA = stateA->invInertia;

		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		float mB = stateB->invMass;
		float iB = stateB->invInertia;

		b2Vec2 normal = constraint->normal;
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			if ( cp->relativeVelocity > -threshold || cp->totalNormalImpulse == 0.0f )
			{
				continue;
			}

			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			float impulse = -cp->normalMass * ( vn + restitution * cp->relativeVelocity );
			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;
			cp->totalNormalImpulse += impulse;

			b2Vec2 P = b2MulSV( impulse, normal );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
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
}

void b2StoreContactImpulses( b2World* world, int* contactIds, b2ContactConstraint* constraints, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		const b2ContactConstraint* constraint = constraints + i;
		b2Contact* contact = b2ContactArray_Get( &world->contacts, contactIds[i] );
		b2Manifold* manifold = &contact->manifold;
		int pointCount = manifold->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
			manifold->points[j].totalNormalImpulse = constraint->points[j].totalNormalImpulse;
			manifold->points[j].normalVelocity = constraint->points[j].relativeVelocity;
		}

		manifold->rollingImpulse = constraint->rollingImpulse;
	}
}
