// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "platform.h"
#include "solver_set.h"

#include <stddef.h>

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

void b2PrepareContacts_Overflow( b2StepContext* context )
{
	b2TracyCZoneNC( prepare_overflow_contact, "Prepare Overflow Contact", b2_colorYellow, true );

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + B2_OVERFLOW_INDEX;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2ContactSim* contacts = color->contactSims.data;
	b2BodyState* states = context->states;

#if B2_ENABLE_VALIDATION
	b2Body* bodies = world->bodies.data;
#endif

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b2Softness contactSoftness = context->contactSoftness;
	b2Softness staticSoftness = context->staticSoftness;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;
	bool anyRestitution = false;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactSim* contactSim = contacts + i;

		b2Manifold* manifold = &contactSim->manifold;
		int pointCount = manifold->pointCount;

		B2_ASSERT( 0 < pointCount && pointCount <= 2 );

		int indexA = b2DecodeAwakeIndex( contactSim->encodedBodySimA );
		int indexB = b2DecodeAwakeIndex( contactSim->encodedBodySimB );

#if B2_ENABLE_VALIDATION
		b2Body* bodyA = bodies + contactSim->bodyIdA;
		B2_ASSERT( contactSim->encodedBodySimA == b2EncodeBodySimIndex( bodyA ) );

		b2Body* bodyB = bodies + contactSim->bodyIdB;
		B2_ASSERT( contactSim->encodedBodySimB == b2EncodeBodySimIndex( bodyB ) );
#endif

		b2ContactConstraint* constraint = constraints + i;

		// 0 is null
		constraint->indexA = indexA + 1;
		constraint->indexB = indexB + 1;
		constraint->normal = manifold->normal;
		constraint->friction = contactSim->friction;
		constraint->restitution = contactSim->restitution;
		constraint->rollingResistance = contactSim->rollingResistance;
		constraint->rollingImpulse = warmStartScale * manifold->rollingImpulse;
		constraint->tangentSpeed = contactSim->tangentSpeed;
		constraint->pointCount = pointCount;

		bool haveRestitution = contactSim->restitution > 0.0f;
		bool hitEvents = ( contactSim->simFlags & b2_simEnableHitEvent ) != 0;
		bool sampleVelocity = haveRestitution || hitEvents;
		anyRestitution = anyRestitution || haveRestitution;

		b2Vec2 vA = b2Vec2_zero;
		float wA = 0.0f;
		b2Vec2 vB = b2Vec2_zero;
		float wB = 0.0f;

		if ( sampleVelocity )
		{
			if ( indexA != B2_NULL_INDEX )
			{
				vA = states[indexA].linearVelocity;
				wA = states[indexA].angularVelocity;
			}

			if ( indexB != B2_NULL_INDEX )
			{
				vB = states[indexB].linearVelocity;
				wB = states[indexB].angularVelocity;
			}
		}

		float mA = contactSim->invMassA;
		float iA = contactSim->invIA;
		float mB = contactSim->invMassB;
		float iB = contactSim->invIB;

		if ( indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX )
		{
			constraint->softness = staticSoftness;
		}
		else
		{
			constraint->softness = contactSoftness;
		}

		// copy mass into constraint to avoid cache misses during sub-stepping
		constraint->invMassA = mA;
		constraint->invIA = iA;
		constraint->invMassB = mB;
		constraint->invIB = iB;

		{
			float k = iA + iB;
			constraint->rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
		}

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( constraint->normal );

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ManifoldPoint* mp = manifold->points + j;
			b2ContactConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = warmStartScale * mp->normalImpulse;
			cp->tangentImpulse = warmStartScale * mp->tangentImpulse;
			cp->totalNormalImpulse = 0.0f;
			cp->restitutionImpulse = 0.0f;

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

			// Only compute the normal velocity terms if needed.
			if ( sampleVelocity )
			{
				b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
				b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
				float vn = b2Dot( normal, b2Sub( vrB, vrA ) );

				cp->relativeVelocity = vn;
				mp->normalVelocity = vn;
			}
			else
			{
				cp->relativeVelocity = 0.0f;
				mp->normalVelocity = 0.0f;
			}
		}
	}

	if ( anyRestitution )
	{
		// Check if set first? Store per task context and OR?
		b2AtomicStoreInt( &context->anyRestitution, 1 );
	}

	b2TracyCZoneEnd( prepare_overflow_contact );
}

void b2WarmStartContacts_Overflow( b2StepContext* context )
{
	b2TracyCZoneNC( warmstart_overflow_contact, "WarmStart Overflow Contact", b2_colorDarkOrange, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + B2_OVERFLOW_INDEX;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;

		int indexA = constraint->indexA - 1;
		int indexB = constraint->indexB - 1;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;

		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;

		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( constraint->normal );
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchors
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

	b2TracyCZoneEnd( warmstart_overflow_contact );
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
void b2PushContacts_Overflow( b2StepContext* context )
{
	b2TracyCZoneNC( push_contact, "Push Overflow Contact", b2_colorAliceBlue, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + B2_OVERFLOW_INDEX;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

	float inv_h = context->inv_h;
	const float contactSpeed = context->world->contactSpeed;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;
		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		int indexA = constraint->indexA - 1;
		int indexB = constraint->indexB - 1;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		b2Rot dqA = stateA->deltaRotation;

		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		b2Rot dqB = stateB->deltaRotation;

		b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		b2Vec2 normal = constraint->normal;
		b2Softness softness = constraint->softness;
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchor points
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, rB ), b2RotateVector( dqA, rA ) ) );
			float s = cp->baseSeparation + b2Dot( ds, normal );

			float velocityBias;
			float massScale;
			float impulseScale;
			if ( s > 0.0f )
			{
				// speculative bias is positive
				velocityBias = s * inv_h;
				massScale = 1.0f;
				impulseScale = 0.0f;
			}
			else
			{
				// overlap bias is negative
				velocityBias = b2MaxFloat( softness.massScale * softness.biasRate * s, -contactSpeed );
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			// relative normal velocity at contact
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			// incremental normal impulse
			float impulse = -cp->normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp->normalImpulse;

			// clamp the accumulated impulse
			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;
			cp->totalNormalImpulse += impulse;

			// apply normal impulse
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

	b2TracyCZoneEnd( push_contact );
}

// Solve contacts: normal, friction and rolling resistance.
void b2SolveContacts_Overflow( b2StepContext* context )
{
	b2TracyCZoneNC( solve_contact, "Solve Overflow Contact", b2_colorAliceBlue, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + B2_OVERFLOW_INDEX;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

	float inv_h = context->inv_h;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;
		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		int indexA = constraint->indexA - 1;
		int indexB = constraint->indexB - 1;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		b2Rot dqA = stateA->deltaRotation;

		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		b2Rot dqB = stateB->deltaRotation;

		b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( normal );
		float friction = constraint->friction;

		int pointCount = constraint->pointCount;
		float totalNormalImpulse = 0.0f;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchor points
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, rB ), b2RotateVector( dqA, rA ) ) );
			float s = cp->baseSeparation + b2Dot( ds, normal );

			// speculative bias, zero when overlapped
			float velocityBias = s > 0.0f ? s * inv_h : 0.0f;

			// relative normal velocity at contact
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			// incremental normal impulse
			float impulse = -cp->normalMass * ( vn + velocityBias );

			// clamp the accumulated impulse
			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;
			cp->totalNormalImpulse += impulse;

			// This is used for rolling resistance
			totalNormalImpulse += newImpulse;

			// apply normal impulse
			b2Vec2 P = b2MulSV( impulse, normal );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );

			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		// Rolling resistance
		{
			float deltaLambda = -constraint->rollingMass * ( wB - wA );
			float lambda = constraint->rollingImpulse;
			float maxLambda = constraint->rollingResistance * totalNormalImpulse;
			constraint->rollingImpulse = b2ClampFloat( lambda + deltaLambda, -maxLambda, maxLambda );
			deltaLambda = constraint->rollingImpulse - lambda;

			wA -= iA * deltaLambda;
			wB += iB * deltaLambda;
		}

		// Friction
		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchor points
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// relative tangent velocity at contact
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );

			// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			//    = dot(vrB - vrA, tangent) - (sA + sB)

			float vt = b2Dot( b2Sub( vrB, vrA ), tangent ) - constraint->tangentSpeed;

			// incremental tangent impulse
			float impulse = cp->tangentMass * ( -vt );

			// clamp the accumulated force
			float maxFriction = friction * cp->normalImpulse;
			float newImpulse = b2ClampFloat( cp->tangentImpulse + impulse, -maxFriction, maxFriction );
			impulse = newImpulse - cp->tangentImpulse;
			cp->tangentImpulse = newImpulse;

			// apply tangent impulse
			b2Vec2 P = b2MulSV( impulse, tangent );
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

	b2TracyCZoneEnd( solve_contact );
}

void b2ApplyRestitution_Overflow( b2StepContext* context )
{
	b2TracyCZoneNC( restitution_overflow, "Restitution Overflow", b2_colorViolet, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + B2_OVERFLOW_INDEX;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

	float threshold = world->restitutionThreshold;
	float inv_h = context->inv_h;
	bool propagate = world->enableRestitutionPropagation;

	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;
		if ( propagate == false && constraint->restitution == 0.0f )
		{
			continue;
		}

		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		int indexA = constraint->indexA - 1;
		int indexB = constraint->indexB - 1;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		b2Rot dqA = stateA->deltaRotation;

		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		b2Rot dqB = stateB->deltaRotation;

		b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		b2Vec2 normal = constraint->normal;
		float restitution = constraint->restitution;
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// The total normal impulse is 0 for speculative points.
			float compressionImpulse = cp->totalNormalImpulse - cp->restitutionImpulse;
			bool armed = restitution > 0.0f && cp->relativeVelocity < -threshold && compressionImpulse > 0.0f;

			float velocityBias;
			if ( armed )
			{
				velocityBias = restitution * cp->relativeVelocity;
			}
			else
			{
				b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, rB ), b2RotateVector( dqA, rA ) ) );
				float s = cp->baseSeparation + b2Dot( ds, normal );

				velocityBias = s > 0.0f ? s * inv_h : 0.0f;
			}

			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			float impulse = -cp->normalMass * ( vn + velocityBias );

			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;

			float approachImpulse = b2MinFloat( b2MaxFloat( -cp->normalMass * vn, 0.0f ), b2MaxFloat( impulse, 0.0f ) );

			if ( armed )
			{
				// Poisson kinetic restitution guarantees no energy gain.
				float allowance = restitution * ( compressionImpulse + approachImpulse ) - cp->restitutionImpulse;
				impulse = b2MinFloat( impulse, approachImpulse + b2MaxFloat( allowance, 0.0f ) );
			}

			cp->normalImpulse += impulse;
			cp->restitutionImpulse += impulse - approachImpulse;
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

	b2TracyCZoneEnd( restitution_overflow );
}

void b2StoreImpulses_Overflow( b2StepContext* context )
{
	b2TracyCZoneNC( store_impulses, "Store", b2_colorFireBrick, true );

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + B2_OVERFLOW_INDEX;
	b2ContactConstraint* constraints = color->overflowConstraints;
	b2ContactSim* contactSims = color->contactSims.data;
	b2TaskContext* taskContext = world->taskContexts.data + 0;
	b2BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	int contactCount = color->contactSims.count;
	float negHitThreshold = -world->hitEventThreshold;
	bool hasHitEvents = taskContext->hasHitEvents;

	for ( int i = 0; i < contactCount; ++i )
	{
		const b2ContactConstraint* constraint = constraints + i;
		b2ContactSim* contactSim = contactSims + i;
		b2Manifold* manifold = &contactSim->manifold;
		int pointCount = manifold->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ManifoldPoint* mp = manifold->points + j;
			mp->normalImpulse = constraint->points[j].normalImpulse;
			mp->tangentImpulse = constraint->points[j].tangentImpulse;
			mp->totalNormalImpulse = constraint->points[j].totalNormalImpulse;
		}

		if ( ( contactSim->simFlags & b2_simEnableHitEvent ) != 0 )
		{
			for ( int j = 0; j < contactSim->manifold.pointCount; ++j )
			{
				b2ManifoldPoint* mp = manifold->points + j;

				// Need to check total impulse because the point may be speculative and not colliding
				if ( mp->normalVelocity < negHitThreshold && mp->totalNormalImpulse > 0.0f )
				{
					b2SetBit( hitEventBitSet, contactSim->contactId );
					hasHitEvents = true;
					break;
				}
			}
		}

		manifold->rollingImpulse = constraint->rollingImpulse;
	}

	taskContext->hasHitEvents = hasHitEvents;

	b2TracyCZoneEnd( store_impulses );
}

void b2PrepareContacts_Wide( b2SolverBlock block, b2StepContext* context )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b2PrepareContacts_WideW8( block, context );
		return;
	}
#endif

	b2PrepareContacts_WideW4( block, context );
}

void b2WarmStartContacts_Wide( b2SolverBlock block, b2StepContext* context )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b2WarmStartContacts_WideW8( block, context );
		return;
	}
#endif

	b2WarmStartContacts_WideW4( block, context );
}

void b2PushContacts_Wide( b2SolverBlock block, b2StepContext* context )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b2PushContacts_WideW8( block, context );
		return;
	}
#endif

	b2PushContacts_WideW4( block, context );
}

void b2SolveContacts_Wide( b2SolverBlock block, b2StepContext* context )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b2SolveContacts_WideW8( block, context );
		return;
	}
#endif

	b2SolveContacts_WideW4( block, context );
}

void b2ApplyRestitution_Wide( b2SolverBlock block, b2StepContext* context )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b2ApplyRestitution_WideW8( block, context );
		return;
	}
#endif

	b2ApplyRestitution_WideW4( block, context );
}

void b2StoreImpulses_Wide( b2SolverBlock block, b2StepContext* context, int workerIndex )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( context->world->simdWidth == 8 )
	{
		b2StoreImpulses_WideW8( block, context, workerIndex );
		return;
	}
#endif

	b2StoreImpulses_WideW4( block, context, workerIndex );
}

int b2GetWideContactConstraintByteCount( int simdWidth )
{
#if defined( B2_SIMD_HAS_WIDTH_8 )
	if ( simdWidth == 8 )
	{
		return b2GetWideContactConstraintByteCountW8();
	}
#else
	B2_UNUSED( simdWidth );
#endif

	return b2GetWideContactConstraintByteCountW4();
}
