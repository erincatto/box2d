// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "solver_set.h"
#include "world.h"
#include "x86/avx2.h"
#include "x86/fma.h"

// Soft contact constraints with sub-stepping support
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)

void b2PrepareOverflowContacts( b2StepContext* context )
{
	b2TracyCZoneNC( prepare_overflow_contact, "Prepare Overflow Contact", b2_colorYellow, true );

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + b2_overflowIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contacts.count;
	b2ContactSim* contacts = color->contacts.data;
	b2BodyState* awakeStates = context->states;

#if B2_VALIDATE
	b2Body* bodies = world->bodyArray;
#endif

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b2Softness contactSoftness = context->contactSoftness;
	b2Softness staticSoftness = context->staticSoftness;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactSim* contactSim = contacts + i;

		const b2Manifold* manifold = &contactSim->manifold;
		int pointCount = manifold->pointCount;

		B2_ASSERT( 0 < pointCount && pointCount <= 2 );

		int indexA = contactSim->bodySimIndexA;
		int indexB = contactSim->bodySimIndexB;

#if B2_VALIDATE
		b2Body* bodyA = bodies + contactSim->bodyIdA;
		int validIndexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
		B2_ASSERT( indexA == validIndexA );

		b2Body* bodyB = bodies + contactSim->bodyIdB;
		int validIndexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;
		B2_ASSERT( indexB == validIndexB );
#endif

		b2ContactConstraint* constraint = constraints + i;
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contactSim->friction;
		constraint->restitution = contactSim->restitution;
		constraint->pointCount = pointCount;

		b2Vec2 vA = b2Vec2_zero;
		float wA = 0.0f;
		float mA = contactSim->invMassA;
		float iA = contactSim->invIA;
		if ( indexA != B2_NULL_INDEX )
		{
			b2BodyState* stateA = awakeStates + indexA;
			vA = stateA->linearVelocity;
			wA = stateA->angularVelocity;
		}

		b2Vec2 vB = b2Vec2_zero;
		float wB = 0.0f;
		float mB = contactSim->invMassB;
		float iB = contactSim->invIB;
		if ( indexB != B2_NULL_INDEX )
		{
			b2BodyState* stateB = awakeStates + indexB;
			vB = stateB->linearVelocity;
			wB = stateB->angularVelocity;
		}

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

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( constraint->normal );

		for ( int j = 0; j < pointCount; ++j )
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ContactConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = warmStartScale * mp->normalImpulse;
			cp->tangentImpulse = warmStartScale * mp->tangentImpulse;
			cp->maxNormalImpulse = 0.0f;

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

			// Save relative velocity for restitution
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			cp->relativeVelocity = b2Dot( normal, b2Sub( vrB, vrA ) );
		}
	}

	b2TracyCZoneEnd( prepare_overflow_contact );
}

void b2WarmStartOverflowContacts( b2StepContext* context )
{
	b2TracyCZoneNC( warmstart_overflow_contact, "WarmStart Overflow Contact", b2_colorDarkOrange, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + b2_overflowIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contacts.count;
	b2SolverSet* awakeSet = context->world->solverSetArray + b2_awakeSet;
	b2BodyState* states = awakeSet->states.data;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		const b2ContactConstraint* constraint = constraints + i;

		int indexA = constraint->indexA;
		int indexB = constraint->indexB;

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
			const b2ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchors
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			b2Vec2 P = b2Add( b2MulSV( cp->normalImpulse, normal ), b2MulSV( cp->tangentImpulse, tangent ) );
			wA -= iA * b2Cross( rA, P );
			vA = b2MulAdd( vA, -mA, P );
			wB += iB * b2Cross( rB, P );
			vB = b2MulAdd( vB, mB, P );
		}

		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}

	b2TracyCZoneEnd( warmstart_overflow_contact );
}

void b2SolveOverflowContacts( b2StepContext* context, bool useBias )
{
	b2TracyCZoneNC( solve_contact, "Solve Contact", b2_colorAliceBlue, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + b2_overflowIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contacts.count;
	b2SolverSet* awakeSet = context->world->solverSetArray + b2_awakeSet;
	b2BodyState* states = awakeSet->states.data;

	float inv_h = context->inv_h;
	const float pushout = context->world->contactPushoutVelocity;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;
		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		b2BodyState* stateA = constraint->indexA == B2_NULL_INDEX ? &dummyState : states + constraint->indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		b2Rot dqA = stateA->deltaRotation;

		b2BodyState* stateB = constraint->indexB == B2_NULL_INDEX ? &dummyState : states + constraint->indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		b2Rot dqB = stateB->deltaRotation;

		b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp( normal );
		float friction = constraint->friction;
		b2Softness softness = constraint->softness;

		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// compute current separation
			// todo this is subject to round-off error if the anchor is far from the body center of mass
			// todo for example a large world with a single static body and many offset shapes
			b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, cp->anchorB ), b2RotateVector( dqA, cp->anchorA ) ) );
			float s = b2Dot( ds, normal ) + cp->baseSeparation;

			float velocityBias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( s > 0.0f )
			{
				// speculative bias
				velocityBias = s * inv_h;
			}
			else if ( useBias )
			{
				velocityBias = b2MaxFloat( softness.biasRate * s, -pushout );
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			// fixed anchor points
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// relative normal velocity at contact
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			// incremental normal impulse
			float impulse = -cp->normalMass * massScale * ( vn + velocityBias ) - impulseScale * cp->normalImpulse;

			// clamp the accumulated impulse
			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;
			cp->maxNormalImpulse = b2MaxFloat( cp->maxNormalImpulse, impulse );

			// apply normal impulse
			b2Vec2 P = b2MulSV( impulse, normal );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );

			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// fixed anchor points
			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// relative tangent velocity at contact
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			float vt = b2Dot( b2Sub( vrB, vrA ), tangent );

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

		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}

	b2TracyCZoneEnd( solve_contact );
}

void b2ApplyOverflowRestitution( b2StepContext* context )
{
	b2TracyCZoneNC( overflow_resitution, "Overflow Restitution", b2_colorViolet, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + b2_overflowIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contacts.count;
	b2SolverSet* awakeSet = context->world->solverSetArray + b2_awakeSet;
	b2BodyState* states = awakeSet->states.data;

	float threshold = context->world->restitutionThreshold;

	// dummy state to represent a static body
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;

		float restitution = constraint->restitution;
		if ( restitution == 0.0f )
		{
			continue;
		}

		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		b2BodyState* stateA = constraint->indexA == B2_NULL_INDEX ? &dummyState : states + constraint->indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;

		b2BodyState* stateB = constraint->indexB == B2_NULL_INDEX ? &dummyState : states + constraint->indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;

		b2Vec2 normal = constraint->normal;
		int pointCount = constraint->pointCount;

		// it is possible to get more accurate restitution by iterating
		// this only makes a difference if there are two contact points
		// for (int iter = 0; iter < 10; ++iter)
		{
			for ( int j = 0; j < pointCount; ++j )
			{
				b2ContactConstraintPoint* cp = constraint->points + j;

				// if the normal impulse is zero then there was no collision
				// this skips speculative contact points that didn't generate an impulse
				// The max normal impulse is used in case there was a collision that moved away within the sub-step process
				if ( cp->relativeVelocity > -threshold || cp->maxNormalImpulse == 0.0f )
				{
					continue;
				}

				// fixed anchor points
				b2Vec2 rA = cp->anchorA;
				b2Vec2 rB = cp->anchorB;

				// relative normal velocity at contact
				b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
				b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
				float vn = b2Dot( b2Sub( vrB, vrA ), normal );

				// compute normal impulse
				float impulse = -cp->normalMass * ( vn + restitution * cp->relativeVelocity );

				// clamp the accumulated impulse
				// todo should this be stored?
				float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
				impulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;
				cp->maxNormalImpulse = b2MaxFloat( cp->maxNormalImpulse, impulse );

				// apply contact impulse
				b2Vec2 P = b2MulSV( impulse, normal );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}
		}

		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}

	b2TracyCZoneEnd( overflow_resitution );
}

void b2StoreOverflowImpulses( b2StepContext* context )
{
	b2TracyCZoneNC( store_impulses, "Store", b2_colorFirebrick, true );

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + b2_overflowIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	b2ContactSim* contacts = color->contacts.data;
	int contactCount = color->contacts.count;

	// float hitEventThreshold = context->world->hitEventThreshold;

	for ( int i = 0; i < contactCount; ++i )
	{
		const b2ContactConstraint* constraint = constraints + i;
		b2ContactSim* contact = contacts + i;
		b2Manifold* manifold = &contact->manifold;
		int pointCount = manifold->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
			manifold->points[j].maxNormalImpulse = constraint->points[j].maxNormalImpulse;
			manifold->points[j].normalVelocity = constraint->points[j].relativeVelocity;
		}
	}

	b2TracyCZoneEnd( store_impulses );
}

// SIMD WIP
#define add( a, b ) simde_mm256_add_ps( ( a ), ( b ) )
#define sub( a, b ) simde_mm256_sub_ps( ( a ), ( b ) )
#define mul( a, b ) simde_mm256_mul_ps( ( a ), ( b ) )

// todo SIMDE implementation of simde_mm256_fnmadd_ps is slow if FMA is not available
// #define muladd(a, b, c) simde_mm256_fmadd_ps(b, c, a)
// #define mulsub(a, b, c) simde_mm256_fnmadd_ps(b, c, a)

#define muladd( a, b, c ) simde_mm256_add_ps( ( a ), simde_mm256_mul_ps( ( b ), ( c ) ) )
#define mulsub( a, b, c ) simde_mm256_sub_ps( ( a ), simde_mm256_mul_ps( ( b ), ( c ) ) )

static inline b2FloatW b2DotW( b2Vec2W a, b2Vec2W b )
{
	return add( mul( a.X, b.X ), mul( a.Y, b.Y ) );
}

static inline b2FloatW b2CrossW( b2Vec2W a, b2Vec2W b )
{
	return sub( mul( a.X, b.Y ), mul( a.Y, b.X ) );
}

static inline b2Vec2W b2RotateVectorW( b2RotW q, b2Vec2W v )
{
	return ( b2Vec2W ){ sub( mul( q.C, v.X ), mul( q.S, v.Y ) ), add( mul( q.S, v.X ), mul( q.C, v.Y ) ) };
}

// wide version of b2BodyState
typedef struct b2SimdBody
{
	b2Vec2W v;
	b2FloatW w;
	b2FloatW flags;
	b2Vec2W dp;
	b2RotW dq;
} b2SimdBody;

// This is a load and 8x8 transpose
static b2SimdBody b2GatherBodies( const b2BodyState* restrict states, int* restrict indices )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );
	// static const b2BodyState b2_identityBodyState = {{0.0f, 0.0f}, 0.0f, 0, {0.0f, 0.0f}, {1.0f, 0.0f}};
	b2FloatW identity = simde_mm256_setr_ps( 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 1.0f, 0.0f );
	b2FloatW b0 = indices[0] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[0] ) );
	b2FloatW b1 = indices[1] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[1] ) );
	b2FloatW b2 = indices[2] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[2] ) );
	b2FloatW b3 = indices[3] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[3] ) );
	b2FloatW b4 = indices[4] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[4] ) );
	b2FloatW b5 = indices[5] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[5] ) );
	b2FloatW b6 = indices[6] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[6] ) );
	b2FloatW b7 = indices[7] == B2_NULL_INDEX ? identity : simde_mm256_load_ps( (float*)( states + indices[7] ) );

	b2FloatW t0 = simde_mm256_unpacklo_ps( b0, b1 );
	b2FloatW t1 = simde_mm256_unpackhi_ps( b0, b1 );
	b2FloatW t2 = simde_mm256_unpacklo_ps( b2, b3 );
	b2FloatW t3 = simde_mm256_unpackhi_ps( b2, b3 );
	b2FloatW t4 = simde_mm256_unpacklo_ps( b4, b5 );
	b2FloatW t5 = simde_mm256_unpackhi_ps( b4, b5 );
	b2FloatW t6 = simde_mm256_unpacklo_ps( b6, b7 );
	b2FloatW t7 = simde_mm256_unpackhi_ps( b6, b7 );
	b2FloatW tt0 = simde_mm256_shuffle_ps( t0, t2, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt1 = simde_mm256_shuffle_ps( t0, t2, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt2 = simde_mm256_shuffle_ps( t1, t3, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt3 = simde_mm256_shuffle_ps( t1, t3, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt4 = simde_mm256_shuffle_ps( t4, t6, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt5 = simde_mm256_shuffle_ps( t4, t6, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt6 = simde_mm256_shuffle_ps( t5, t7, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt7 = simde_mm256_shuffle_ps( t5, t7, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );

	b2SimdBody simdBody;
	simdBody.v.X = simde_mm256_permute2f128_ps( tt0, tt4, 0x20 );
	simdBody.v.Y = simde_mm256_permute2f128_ps( tt1, tt5, 0x20 );
	simdBody.w = simde_mm256_permute2f128_ps( tt2, tt6, 0x20 );
	simdBody.flags = simde_mm256_permute2f128_ps( tt3, tt7, 0x20 );
	simdBody.dp.X = simde_mm256_permute2f128_ps( tt0, tt4, 0x31 );
	simdBody.dp.Y = simde_mm256_permute2f128_ps( tt1, tt5, 0x31 );
	simdBody.dq.C = simde_mm256_permute2f128_ps( tt2, tt6, 0x31 );
	simdBody.dq.S = simde_mm256_permute2f128_ps( tt3, tt7, 0x31 );
	return simdBody;
}

// This writes everything back to the solver bodies but only the velocities change
static void b2ScatterBodies( b2BodyState* restrict states, int* restrict indices, const b2SimdBody* restrict simdBody )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );
	b2FloatW t0 = simde_mm256_unpacklo_ps( simdBody->v.X, simdBody->v.Y );
	b2FloatW t1 = simde_mm256_unpackhi_ps( simdBody->v.X, simdBody->v.Y );
	b2FloatW t2 = simde_mm256_unpacklo_ps( simdBody->w, simdBody->flags );
	b2FloatW t3 = simde_mm256_unpackhi_ps( simdBody->w, simdBody->flags );
	b2FloatW t4 = simde_mm256_unpacklo_ps( simdBody->dp.X, simdBody->dp.Y );
	b2FloatW t5 = simde_mm256_unpackhi_ps( simdBody->dp.X, simdBody->dp.Y );
	b2FloatW t6 = simde_mm256_unpacklo_ps( simdBody->dq.C, simdBody->dq.S );
	b2FloatW t7 = simde_mm256_unpackhi_ps( simdBody->dq.C, simdBody->dq.S );
	b2FloatW tt0 = simde_mm256_shuffle_ps( t0, t2, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt1 = simde_mm256_shuffle_ps( t0, t2, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt2 = simde_mm256_shuffle_ps( t1, t3, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt3 = simde_mm256_shuffle_ps( t1, t3, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt4 = simde_mm256_shuffle_ps( t4, t6, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt5 = simde_mm256_shuffle_ps( t4, t6, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt6 = simde_mm256_shuffle_ps( t5, t7, SIMDE_MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt7 = simde_mm256_shuffle_ps( t5, t7, SIMDE_MM_SHUFFLE( 3, 2, 3, 2 ) );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if ( indices[0] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[0] ), simde_mm256_permute2f128_ps( tt0, tt4, 0x20 ) );
	if ( indices[1] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[1] ), simde_mm256_permute2f128_ps( tt1, tt5, 0x20 ) );
	if ( indices[2] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[2] ), simde_mm256_permute2f128_ps( tt2, tt6, 0x20 ) );
	if ( indices[3] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[3] ), simde_mm256_permute2f128_ps( tt3, tt7, 0x20 ) );
	if ( indices[4] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[4] ), simde_mm256_permute2f128_ps( tt0, tt4, 0x31 ) );
	if ( indices[5] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[5] ), simde_mm256_permute2f128_ps( tt1, tt5, 0x31 ) );
	if ( indices[6] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[6] ), simde_mm256_permute2f128_ps( tt2, tt6, 0x31 ) );
	if ( indices[7] != B2_NULL_INDEX )
		simde_mm256_store_ps( (float*)( states + indices[7] ), simde_mm256_permute2f128_ps( tt3, tt7, 0x31 ) );
}

void b2PrepareContactsTask( int startIndex, int endIndex, b2StepContext* context )
{
	b2TracyCZoneNC( prepare_contact, "Prepare Contact", b2_colorYellow, true );
	b2World* world = context->world;
	b2ContactSim** contacts = context->contacts;
	b2ContactConstraintSIMD* constraints = context->simdContactConstraints;
	b2BodyState* awakeStates = context->states;
#if B2_VALIDATE
	b2Body* bodies = world->bodyArray;
#endif

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b2Softness contactSoftness = context->contactSoftness;
	b2Softness staticSoftness = context->staticSoftness;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* constraint = constraints + i;

		for ( int j = 0; j < 8; ++j )
		{
			b2ContactSim* contactSim = contacts[8 * i + j];

			if ( contactSim != NULL )
			{
				const b2Manifold* manifold = &contactSim->manifold;

				int indexA = contactSim->bodySimIndexA;
				int indexB = contactSim->bodySimIndexB;

#if B2_VALIDATE
				b2Body* bodyA = bodies + contactSim->bodyIdA;
				int validIndexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
				b2Body* bodyB = bodies + contactSim->bodyIdB;
				int validIndexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;

				B2_ASSERT( indexA == validIndexA );
				B2_ASSERT( indexB == validIndexB );
#endif
				constraint->indexA[j] = indexA;
				constraint->indexB[j] = indexB;

				b2Vec2 vA = b2Vec2_zero;
				float wA = 0.0f;
				float mA = contactSim->invMassA;
				float iA = contactSim->invIA;
				if ( indexA != B2_NULL_INDEX )
				{
					b2BodyState* stateA = awakeStates + indexA;
					vA = stateA->linearVelocity;
					wA = stateA->angularVelocity;
				}

				b2Vec2 vB = b2Vec2_zero;
				float wB = 0.0f;
				float mB = contactSim->invMassB;
				float iB = contactSim->invIB;
				if ( indexB != B2_NULL_INDEX )
				{
					b2BodyState* stateB = awakeStates + indexB;
					vB = stateB->linearVelocity;
					wB = stateB->angularVelocity;
				}

				( (float*)&constraint->invMassA )[j] = mA;
				( (float*)&constraint->invMassB )[j] = mB;
				( (float*)&constraint->invIA )[j] = iA;
				( (float*)&constraint->invIB )[j] = iB;

				b2Softness soft = ( indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX ) ? staticSoftness : contactSoftness;

				b2Vec2 normal = manifold->normal;
				( (float*)&constraint->normal.X )[j] = normal.x;
				( (float*)&constraint->normal.Y )[j] = normal.y;

				( (float*)&constraint->friction )[j] = contactSim->friction;
				( (float*)&constraint->restitution )[j] = contactSim->restitution;
				( (float*)&constraint->biasRate )[j] = soft.biasRate;
				( (float*)&constraint->massScale )[j] = soft.massScale;
				( (float*)&constraint->impulseScale )[j] = soft.impulseScale;

				b2Vec2 tangent = b2RightPerp( normal );

				{
					const b2ManifoldPoint* mp = manifold->points + 0;

					b2Vec2 rA = mp->anchorA;
					b2Vec2 rB = mp->anchorB;

					( (float*)&constraint->anchorA1.X )[j] = rA.x;
					( (float*)&constraint->anchorA1.Y )[j] = rA.y;
					( (float*)&constraint->anchorB1.X )[j] = rB.x;
					( (float*)&constraint->anchorB1.Y )[j] = rB.y;

					( (float*)&constraint->baseSeparation1 )[j] = mp->separation - b2Dot( b2Sub( rB, rA ), normal );

					( (float*)&constraint->normalImpulse1 )[j] = warmStartScale * mp->normalImpulse;
					( (float*)&constraint->tangentImpulse1 )[j] = warmStartScale * mp->tangentImpulse;
					( (float*)&constraint->maxNormalImpulse1 )[j] = 0.0f;

					float rnA = b2Cross( rA, normal );
					float rnB = b2Cross( rB, normal );
					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					( (float*)&constraint->normalMass1 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					float rtA = b2Cross( rA, tangent );
					float rtB = b2Cross( rB, tangent );
					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					( (float*)&constraint->tangentMass1 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// relative velocity for restitution
					b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
					b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
					( (float*)&constraint->relativeVelocity1 )[j] = b2Dot( normal, b2Sub( vrB, vrA ) );
				}

				int pointCount = manifold->pointCount;
				B2_ASSERT( 0 < pointCount && pointCount <= 2 );

				if ( pointCount == 2 )
				{
					const b2ManifoldPoint* mp = manifold->points + 1;

					b2Vec2 rA = mp->anchorA;
					b2Vec2 rB = mp->anchorB;

					( (float*)&constraint->anchorA2.X )[j] = rA.x;
					( (float*)&constraint->anchorA2.Y )[j] = rA.y;
					( (float*)&constraint->anchorB2.X )[j] = rB.x;
					( (float*)&constraint->anchorB2.Y )[j] = rB.y;

					( (float*)&constraint->baseSeparation2 )[j] = mp->separation - b2Dot( b2Sub( rB, rA ), normal );

					( (float*)&constraint->normalImpulse2 )[j] = warmStartScale * mp->normalImpulse;
					( (float*)&constraint->tangentImpulse2 )[j] = warmStartScale * mp->tangentImpulse;
					( (float*)&constraint->maxNormalImpulse2 )[j] = 0.0f;

					float rnA = b2Cross( rA, normal );
					float rnB = b2Cross( rB, normal );
					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					( (float*)&constraint->normalMass2 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					float rtA = b2Cross( rA, tangent );
					float rtB = b2Cross( rB, tangent );
					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					( (float*)&constraint->tangentMass2 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// relative velocity for restitution
					b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
					b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
					( (float*)&constraint->relativeVelocity2 )[j] = b2Dot( normal, b2Sub( vrB, vrA ) );
				}
				else
				{
					// dummy data that has no effect
					( (float*)&constraint->baseSeparation2 )[j] = 0.0f;
					( (float*)&constraint->normalImpulse2 )[j] = 0.0f;
					( (float*)&constraint->tangentImpulse2 )[j] = 0.0f;
					( (float*)&constraint->maxNormalImpulse2 )[j] = 0.0f;
					( (float*)&constraint->anchorA2.X )[j] = 0.0f;
					( (float*)&constraint->anchorA2.Y )[j] = 0.0f;
					( (float*)&constraint->anchorB2.X )[j] = 0.0f;
					( (float*)&constraint->anchorB2.Y )[j] = 0.0f;
					( (float*)&constraint->normalMass2 )[j] = 0.0f;
					( (float*)&constraint->tangentMass2 )[j] = 0.0f;
					( (float*)&constraint->relativeVelocity2 )[j] = 0.0f;
				}
			}
			else
			{
				// SIMD remainder
				constraint->indexA[j] = B2_NULL_INDEX;
				constraint->indexB[j] = B2_NULL_INDEX;
				( (float*)&constraint->friction )[j] = 0.0f;
				( (float*)&constraint->restitution )[j] = 0.0f;
				( (float*)&constraint->biasRate )[j] = 0.0f;
				( (float*)&constraint->massScale )[j] = 0.0f;
				( (float*)&constraint->impulseScale )[j] = 0.0f;
				( (float*)&constraint->normal.X )[j] = 0.0f;
				( (float*)&constraint->normal.Y )[j] = 0.0f;

				( (float*)&constraint->baseSeparation1 )[j] = 0.0f;
				( (float*)&constraint->normalImpulse1 )[j] = 0.0f;
				( (float*)&constraint->tangentImpulse1 )[j] = 0.0f;
				( (float*)&constraint->maxNormalImpulse1 )[j] = 0.0f;
				( (float*)&constraint->anchorA1.X )[j] = 0.0f;
				( (float*)&constraint->anchorA1.Y )[j] = 0.0f;
				( (float*)&constraint->anchorB1.X )[j] = 0.0f;
				( (float*)&constraint->anchorB1.Y )[j] = 0.0f;
				( (float*)&constraint->normalMass1 )[j] = 0.0f;
				( (float*)&constraint->tangentMass1 )[j] = 0.0f;
				( (float*)&constraint->relativeVelocity1 )[j] = 0.0f;

				( (float*)&constraint->baseSeparation2 )[j] = 0.0f;
				( (float*)&constraint->normalImpulse2 )[j] = 0.0f;
				( (float*)&constraint->tangentImpulse2 )[j] = 0.0f;
				( (float*)&constraint->maxNormalImpulse2 )[j] = 0.0f;
				( (float*)&constraint->anchorA2.X )[j] = 0.0f;
				( (float*)&constraint->anchorA2.Y )[j] = 0.0f;
				( (float*)&constraint->anchorB2.X )[j] = 0.0f;
				( (float*)&constraint->anchorB2.Y )[j] = 0.0f;
				( (float*)&constraint->normalMass2 )[j] = 0.0f;
				( (float*)&constraint->tangentMass2 )[j] = 0.0f;
				( (float*)&constraint->relativeVelocity2 )[j] = 0.0f;
			}
		}
	}

	b2TracyCZoneEnd( prepare_contact );
}

void b2WarmStartContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex )
{
	b2TracyCZoneNC( warm_start_contact, "Warm Start", b2_colorGreen, true );

	b2BodyState* states = context->states;
	b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;
		b2SimdBody bA = b2GatherBodies( states, c->indexA );
		b2SimdBody bB = b2GatherBodies( states, c->indexB );

		b2FloatW tangentX = c->normal.Y;
		b2FloatW tangentY = sub( simde_mm256_setzero_ps(), c->normal.X );

		{
			// fixed anchors
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			b2Vec2W P;
			P.X = add( mul( c->normalImpulse1, c->normal.X ), mul( c->tangentImpulse1, tangentX ) );
			P.Y = add( mul( c->normalImpulse1, c->normal.Y ), mul( c->tangentImpulse1, tangentY ) );
			bA.w = mulsub( bA.w, c->invIA, b2CrossW( rA, P ) );
			bA.v.X = mulsub( bA.v.X, c->invMassA, P.X );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, P.Y );
			bB.w = muladd( bB.w, c->invIB, b2CrossW( rB, P ) );
			bB.v.X = muladd( bB.v.X, c->invMassB, P.X );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, P.Y );
		}

		{
			// fixed anchors
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			b2Vec2W P;
			P.X = add( mul( c->normalImpulse2, c->normal.X ), mul( c->tangentImpulse2, tangentX ) );
			P.Y = add( mul( c->normalImpulse2, c->normal.Y ), mul( c->tangentImpulse2, tangentY ) );
			bA.w = mulsub( bA.w, c->invIA, b2CrossW( rA, P ) );
			bA.v.X = mulsub( bA.v.X, c->invMassA, P.X );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, P.Y );
			bB.w = muladd( bB.w, c->invIB, b2CrossW( rB, P ) );
			bB.v.X = muladd( bB.v.X, c->invMassB, P.X );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, P.Y );
		}

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

	b2TracyCZoneEnd( warm_start_contact );
}

void b2SolveContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias )
{
	b2TracyCZoneNC( solve_contact, "Solve Contact", b2_colorAliceBlue, true );

	b2BodyState* states = context->states;
	b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints;
	b2FloatW inv_h = simde_mm256_set1_ps( context->inv_h );
	b2FloatW minBiasVel = simde_mm256_set1_ps( -context->world->contactPushoutVelocity );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;

		b2SimdBody bA = b2GatherBodies( states, c->indexA );
		b2SimdBody bB = b2GatherBodies( states, c->indexB );

		b2FloatW biasRate, massScale, impulseScale;
		if ( useBias )
		{
			biasRate = c->biasRate;
			massScale = c->massScale;
			impulseScale = c->impulseScale;
		}
		else
		{
			biasRate = simde_mm256_setzero_ps();
			massScale = simde_mm256_set1_ps( 1.0f );
			impulseScale = simde_mm256_setzero_ps();
		}

		b2Vec2W dp = { sub( bB.dp.X, bA.dp.X ), sub( bB.dp.Y, bA.dp.Y ) };

		// point1 non-penetration constraint
		{
			// moving anchors for current separation
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA1 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB1 );

			// compute current separation
			// todo this is subject to round-off error if the anchor is far from the body center of mass
			// todo for example a large world with a single static body and many offset shapes
			b2Vec2W ds = { add( dp.X, sub( rsB.X, rsA.X ) ), add( dp.Y, sub( rsB.Y, rsA.Y ) ) };
			b2FloatW s = add( b2DotW( c->normal, ds ), c->baseSeparation1 );

			b2FloatW test = simde_mm256_cmp_ps( s, simde_mm256_setzero_ps(), SIMDE_CMP_GT_OQ );
			b2FloatW specBias = mul( s, inv_h );
			b2FloatW softBias = simde_mm256_max_ps( mul( biasRate, s ), minBiasVel );

			// #todo slow on SSE2
			b2FloatW bias = simde_mm256_blendv_ps( softBias, specBias, test );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = sub( sub( bB.v.X, mul( bB.w, rB.Y ) ), sub( bA.v.X, mul( bA.w, rA.Y ) ) );
			b2FloatW dvy = sub( add( bB.v.Y, mul( bB.w, rB.X ) ), add( bA.v.Y, mul( bA.w, rA.X ) ) );
			b2FloatW vn = add( mul( dvx, c->normal.X ), mul( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse =
				add( mul( c->normalMass1, mul( massScale, add( vn, bias ) ) ), mul( impulseScale, c->normalImpulse1 ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = simde_mm256_max_ps( sub( c->normalImpulse1, negImpulse ), simde_mm256_setzero_ps() );
			b2FloatW impulse = sub( newImpulse, c->normalImpulse1 );
			c->normalImpulse1 = newImpulse;
			c->maxNormalImpulse1 = simde_mm256_max_ps( c->maxNormalImpulse1, newImpulse );

			// Apply contact impulse
			b2FloatW Px = mul( impulse, c->normal.X );
			b2FloatW Py = mul( impulse, c->normal.Y );

			bA.v.X = mulsub( bA.v.X, c->invMassA, Px );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, Py );
			bA.w = mulsub( bA.w, c->invIA, sub( mul( rA.X, Py ), mul( rA.Y, Px ) ) );

			bB.v.X = muladd( bB.v.X, c->invMassB, Px );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, Py );
			bB.w = muladd( bB.w, c->invIB, sub( mul( rB.X, Py ), mul( rB.Y, Px ) ) );
		}

		// second point non-penetration constraint
		{
			// moving anchors for current separation
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA2 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB2 );

			// compute current separation
			b2Vec2W ds = { add( dp.X, sub( rsB.X, rsA.X ) ), add( dp.Y, sub( rsB.Y, rsA.Y ) ) };
			b2FloatW s = add( b2DotW( c->normal, ds ), c->baseSeparation2 );

			b2FloatW test = simde_mm256_cmp_ps( s, simde_mm256_setzero_ps(), SIMDE_CMP_GT_OQ );
			b2FloatW specBias = mul( s, inv_h );
			b2FloatW softBias = simde_mm256_max_ps( mul( biasRate, s ), minBiasVel );

			// #todo slow on SSE2
			b2FloatW bias = simde_mm256_blendv_ps( softBias, specBias, test );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = sub( sub( bB.v.X, mul( bB.w, rB.Y ) ), sub( bA.v.X, mul( bA.w, rA.Y ) ) );
			b2FloatW dvy = sub( add( bB.v.Y, mul( bB.w, rB.X ) ), add( bA.v.Y, mul( bA.w, rA.X ) ) );
			b2FloatW vn = add( mul( dvx, c->normal.X ), mul( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse =
				add( mul( c->normalMass2, mul( massScale, add( vn, bias ) ) ), mul( impulseScale, c->normalImpulse2 ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = simde_mm256_max_ps( sub( c->normalImpulse2, negImpulse ), simde_mm256_setzero_ps() );
			b2FloatW impulse = sub( newImpulse, c->normalImpulse2 );
			c->normalImpulse2 = newImpulse;
			c->maxNormalImpulse2 = simde_mm256_max_ps( c->maxNormalImpulse2, newImpulse );

			// Apply contact impulse
			b2FloatW Px = mul( impulse, c->normal.X );
			b2FloatW Py = mul( impulse, c->normal.Y );

			bA.v.X = mulsub( bA.v.X, c->invMassA, Px );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, Py );
			bA.w = mulsub( bA.w, c->invIA, sub( mul( rA.X, Py ), mul( rA.Y, Px ) ) );

			bB.v.X = muladd( bB.v.X, c->invMassB, Px );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, Py );
			bB.w = muladd( bB.w, c->invIB, sub( mul( rB.X, Py ), mul( rB.Y, Px ) ) );
		}

		b2FloatW tangentX = c->normal.Y;
		b2FloatW tangentY = sub( simde_mm256_setzero_ps(), c->normal.X );

		// point 1 friction constraint
		{
			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = sub( sub( bB.v.X, mul( bB.w, rB.Y ) ), sub( bA.v.X, mul( bA.w, rA.Y ) ) );
			b2FloatW dvy = sub( add( bB.v.Y, mul( bB.w, rB.X ) ), add( bA.v.Y, mul( bA.w, rA.X ) ) );
			b2FloatW vt = add( mul( dvx, tangentX ), mul( dvy, tangentY ) );

			// Compute tangent force
			b2FloatW negImpulse = mul( c->tangentMass1, vt );

			// Clamp the accumulated force
			b2FloatW maxFriction = mul( c->friction, c->normalImpulse1 );
			b2FloatW newImpulse = sub( c->tangentImpulse1, negImpulse );
			newImpulse =
				simde_mm256_max_ps( sub( simde_mm256_setzero_ps(), maxFriction ), simde_mm256_min_ps( newImpulse, maxFriction ) );
			b2FloatW impulse = sub( newImpulse, c->tangentImpulse1 );
			c->tangentImpulse1 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = mul( impulse, tangentX );
			b2FloatW Py = mul( impulse, tangentY );

			bA.v.X = mulsub( bA.v.X, c->invMassA, Px );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, Py );
			bA.w = mulsub( bA.w, c->invIA, sub( mul( rA.X, Py ), mul( rA.Y, Px ) ) );

			bB.v.X = muladd( bB.v.X, c->invMassB, Px );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, Py );
			bB.w = muladd( bB.w, c->invIB, sub( mul( rB.X, Py ), mul( rB.Y, Px ) ) );
		}

		// second point friction constraint
		{
			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = sub( sub( bB.v.X, mul( bB.w, rB.Y ) ), sub( bA.v.X, mul( bA.w, rA.Y ) ) );
			b2FloatW dvy = sub( add( bB.v.Y, mul( bB.w, rB.X ) ), add( bA.v.Y, mul( bA.w, rA.X ) ) );
			b2FloatW vt = add( mul( dvx, tangentX ), mul( dvy, tangentY ) );

			// Compute tangent force
			b2FloatW negImpulse = mul( c->tangentMass2, vt );

			// Clamp the accumulated force
			b2FloatW maxFriction = mul( c->friction, c->normalImpulse2 );
			b2FloatW newImpulse = sub( c->tangentImpulse2, negImpulse );
			newImpulse =
				simde_mm256_max_ps( sub( simde_mm256_setzero_ps(), maxFriction ), simde_mm256_min_ps( newImpulse, maxFriction ) );
			b2FloatW impulse = sub( newImpulse, c->tangentImpulse2 );
			c->tangentImpulse2 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = mul( impulse, tangentX );
			b2FloatW Py = mul( impulse, tangentY );

			bA.v.X = mulsub( bA.v.X, c->invMassA, Px );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, Py );
			bA.w = mulsub( bA.w, c->invIA, sub( mul( rA.X, Py ), mul( rA.Y, Px ) ) );

			bB.v.X = muladd( bB.v.X, c->invMassB, Px );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, Py );
			bB.w = muladd( bB.w, c->invIB, sub( mul( rB.X, Py ), mul( rB.Y, Px ) ) );
		}

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

	b2TracyCZoneEnd( solve_contact );
}

void b2ApplyRestitutionTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex )
{
	b2TracyCZoneNC( restitution, "Restitution", b2_colorDodgerBlue, true );

	b2BodyState* states = context->states;
	b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints;
	b2FloatW threshold = simde_mm256_set1_ps( context->world->restitutionThreshold );
	b2FloatW zero = simde_mm256_setzero_ps();

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;

		b2SimdBody bA = b2GatherBodies( states, c->indexA );
		b2SimdBody bB = b2GatherBodies( states, c->indexB );

		// first point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			b2FloatW test1 = simde_mm256_cmp_ps( add( c->relativeVelocity1, threshold ), zero, SIMDE_CMP_GT_OQ );
			b2FloatW test2 = simde_mm256_cmp_ps( c->maxNormalImpulse1, zero, SIMDE_CMP_EQ_OQ );
			b2FloatW test = simde_mm256_or_ps( test1, test2 );

			// todo slow on SSE2
			b2FloatW mass = simde_mm256_blendv_ps( c->normalMass1, zero, test );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = sub( sub( bB.v.X, mul( bB.w, rB.Y ) ), sub( bA.v.X, mul( bA.w, rA.Y ) ) );
			b2FloatW dvy = sub( add( bB.v.Y, mul( bB.w, rB.X ) ), add( bA.v.Y, mul( bA.w, rA.X ) ) );
			b2FloatW vn = add( mul( dvx, c->normal.X ), mul( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = mul( mass, add( vn, mul( c->restitution, c->relativeVelocity1 ) ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = simde_mm256_max_ps( sub( c->normalImpulse1, negImpulse ), simde_mm256_setzero_ps() );
			b2FloatW impulse = sub( newImpulse, c->normalImpulse1 );
			c->normalImpulse1 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = mul( impulse, c->normal.X );
			b2FloatW Py = mul( impulse, c->normal.Y );

			bA.v.X = mulsub( bA.v.X, c->invMassA, Px );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, Py );
			bA.w = mulsub( bA.w, c->invIA, sub( mul( rA.X, Py ), mul( rA.Y, Px ) ) );

			bB.v.X = muladd( bB.v.X, c->invMassB, Px );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, Py );
			bB.w = muladd( bB.w, c->invIB, sub( mul( rB.X, Py ), mul( rB.Y, Px ) ) );
		}

		// second point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			b2FloatW test1 = simde_mm256_cmp_ps( add( c->relativeVelocity2, threshold ), zero, SIMDE_CMP_GT_OQ );
			b2FloatW test2 = simde_mm256_cmp_ps( c->maxNormalImpulse2, zero, SIMDE_CMP_EQ_OQ );
			b2FloatW test = simde_mm256_or_ps( test1, test2 );

			// todo slow on SSE2
			b2FloatW mass = simde_mm256_blendv_ps( c->normalMass2, zero, test );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = sub( sub( bB.v.X, mul( bB.w, rB.Y ) ), sub( bA.v.X, mul( bA.w, rA.Y ) ) );
			b2FloatW dvy = sub( add( bB.v.Y, mul( bB.w, rB.X ) ), add( bA.v.Y, mul( bA.w, rA.X ) ) );
			b2FloatW vn = add( mul( dvx, c->normal.X ), mul( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = mul( mass, add( vn, mul( c->restitution, c->relativeVelocity2 ) ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = simde_mm256_max_ps( sub( c->normalImpulse2, negImpulse ), simde_mm256_setzero_ps() );
			b2FloatW impulse = sub( newImpulse, c->normalImpulse2 );
			c->normalImpulse2 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = mul( impulse, c->normal.X );
			b2FloatW Py = mul( impulse, c->normal.Y );

			bA.v.X = mulsub( bA.v.X, c->invMassA, Px );
			bA.v.Y = mulsub( bA.v.Y, c->invMassA, Py );
			bA.w = mulsub( bA.w, c->invIA, sub( mul( rA.X, Py ), mul( rA.Y, Px ) ) );

			bB.v.X = muladd( bB.v.X, c->invMassB, Px );
			bB.v.Y = muladd( bB.v.Y, c->invMassB, Py );
			bB.w = muladd( bB.w, c->invIB, sub( mul( rB.X, Py ), mul( rB.Y, Px ) ) );
		}

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

	b2TracyCZoneEnd( restitution );
}

void b2StoreImpulsesTask( int startIndex, int endIndex, b2StepContext* context )
{
	b2TracyCZoneNC( store_impulses, "Store", b2_colorFirebrick, true );

	b2ContactSim** contacts = context->contacts;
	const b2ContactConstraintSIMD* constraints = context->simdContactConstraints;

	b2Manifold dummy = { 0 };

	for ( int i = startIndex; i < endIndex; ++i )
	{
		const b2ContactConstraintSIMD* c = constraints + i;
		const float* normalImpulse1 = (float*)&c->normalImpulse1;
		const float* normalImpulse2 = (float*)&c->normalImpulse2;
		const float* tangentImpulse1 = (float*)&c->tangentImpulse1;
		const float* tangentImpulse2 = (float*)&c->tangentImpulse2;
		const float* maxNormalImpulse1 = (float*)&c->maxNormalImpulse1;
		const float* maxNormalImpulse2 = (float*)&c->maxNormalImpulse2;
		const float* normalVelocity1 = (float*)&c->relativeVelocity1;
		const float* normalVelocity2 = (float*)&c->relativeVelocity2;

		int base = 8 * i;
		b2Manifold* m0 = contacts[base + 0] == NULL ? &dummy : &contacts[base + 0]->manifold;
		b2Manifold* m1 = contacts[base + 1] == NULL ? &dummy : &contacts[base + 1]->manifold;
		b2Manifold* m2 = contacts[base + 2] == NULL ? &dummy : &contacts[base + 2]->manifold;
		b2Manifold* m3 = contacts[base + 3] == NULL ? &dummy : &contacts[base + 3]->manifold;
		b2Manifold* m4 = contacts[base + 4] == NULL ? &dummy : &contacts[base + 4]->manifold;
		b2Manifold* m5 = contacts[base + 5] == NULL ? &dummy : &contacts[base + 5]->manifold;
		b2Manifold* m6 = contacts[base + 6] == NULL ? &dummy : &contacts[base + 6]->manifold;
		b2Manifold* m7 = contacts[base + 7] == NULL ? &dummy : &contacts[base + 7]->manifold;

		m0->points[0].normalImpulse = normalImpulse1[0];
		m0->points[0].tangentImpulse = tangentImpulse1[0];
		m0->points[0].maxNormalImpulse = maxNormalImpulse1[0];
		m0->points[0].normalVelocity = normalVelocity1[0];

		m0->points[1].normalImpulse = normalImpulse2[0];
		m0->points[1].tangentImpulse = tangentImpulse2[0];
		m0->points[1].maxNormalImpulse = maxNormalImpulse2[0];
		m0->points[1].normalVelocity = normalVelocity2[0];

		m1->points[0].normalImpulse = normalImpulse1[1];
		m1->points[0].tangentImpulse = tangentImpulse1[1];
		m1->points[0].maxNormalImpulse = maxNormalImpulse1[1];
		m1->points[0].normalVelocity = normalVelocity1[1];

		m1->points[1].normalImpulse = normalImpulse2[1];
		m1->points[1].tangentImpulse = tangentImpulse2[1];
		m1->points[1].maxNormalImpulse = maxNormalImpulse2[1];
		m1->points[1].normalVelocity = normalVelocity2[1];

		m2->points[0].normalImpulse = normalImpulse1[2];
		m2->points[0].tangentImpulse = tangentImpulse1[2];
		m2->points[0].maxNormalImpulse = maxNormalImpulse1[2];
		m2->points[0].normalVelocity = normalVelocity1[2];

		m2->points[1].normalImpulse = normalImpulse2[2];
		m2->points[1].tangentImpulse = tangentImpulse2[2];
		m2->points[1].maxNormalImpulse = maxNormalImpulse2[2];
		m2->points[1].normalVelocity = normalVelocity2[2];

		m3->points[0].normalImpulse = normalImpulse1[3];
		m3->points[0].tangentImpulse = tangentImpulse1[3];
		m3->points[0].maxNormalImpulse = maxNormalImpulse1[3];
		m3->points[0].normalVelocity = normalVelocity1[3];

		m3->points[1].normalImpulse = normalImpulse2[3];
		m3->points[1].tangentImpulse = tangentImpulse2[3];
		m3->points[1].maxNormalImpulse = maxNormalImpulse2[3];
		m3->points[1].normalVelocity = normalVelocity2[3];

		m4->points[0].normalImpulse = normalImpulse1[4];
		m4->points[0].tangentImpulse = tangentImpulse1[4];
		m4->points[0].maxNormalImpulse = maxNormalImpulse1[4];
		m4->points[0].normalVelocity = normalVelocity1[4];

		m4->points[1].normalImpulse = normalImpulse2[4];
		m4->points[1].tangentImpulse = tangentImpulse2[4];
		m4->points[1].maxNormalImpulse = maxNormalImpulse2[4];
		m4->points[1].normalVelocity = normalVelocity2[4];

		m5->points[0].normalImpulse = normalImpulse1[5];
		m5->points[0].tangentImpulse = tangentImpulse1[5];
		m5->points[0].maxNormalImpulse = maxNormalImpulse1[5];
		m5->points[0].normalVelocity = normalVelocity1[5];

		m5->points[1].normalImpulse = normalImpulse2[5];
		m5->points[1].tangentImpulse = tangentImpulse2[5];
		m5->points[1].maxNormalImpulse = maxNormalImpulse2[5];
		m5->points[1].normalVelocity = normalVelocity2[5];

		m6->points[0].normalImpulse = normalImpulse1[6];
		m6->points[0].tangentImpulse = tangentImpulse1[6];
		m6->points[0].maxNormalImpulse = maxNormalImpulse1[6];
		m6->points[0].normalVelocity = normalVelocity1[6];

		m6->points[1].normalImpulse = normalImpulse2[6];
		m6->points[1].tangentImpulse = tangentImpulse2[6];
		m6->points[1].maxNormalImpulse = maxNormalImpulse2[6];
		m6->points[1].normalVelocity = normalVelocity2[6];

		m7->points[0].normalImpulse = normalImpulse1[7];
		m7->points[0].tangentImpulse = tangentImpulse1[7];
		m7->points[0].maxNormalImpulse = maxNormalImpulse1[7];
		m7->points[0].normalVelocity = normalVelocity1[7];

		m7->points[1].normalImpulse = normalImpulse2[7];
		m7->points[1].tangentImpulse = tangentImpulse2[7];
		m7->points[1].maxNormalImpulse = maxNormalImpulse2[7];
		m7->points[1].normalVelocity = normalVelocity2[7];
	}

	b2TracyCZoneEnd( store_impulses );
}
