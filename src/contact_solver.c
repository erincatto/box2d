// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "solver_set.h"
#include "world.h"

#include <stddef.h>

void b2PrepareOverflowContacts( b2StepContext* context )
{
	b2TracyCZoneNC( prepare_overflow_contact, "Prepare Overflow Contact", b2_colorYellow, true );

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + b2_overflowIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2ContactSim* contacts = color->contactSims.data;
	b2BodyState* awakeStates = context->states;

#if B2_VALIDATE
	b2Body* bodies = world->bodies.data;
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
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

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
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

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
			// this is subject to round-off error if the anchor is far from the body center of mass
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
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

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
	b2ContactSim* contacts = color->contactSims.data;
	int contactCount = color->contactSims.count;

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

#if defined( B2_SIMD_AVX2 )

#include <immintrin.h>

// wide float holds 8 numbers
typedef __m256 b2FloatW;

#elif defined( B2_SIMD_NEON )

#include <arm_neon.h>

// wide float holds 4 numbers
typedef float32x4_t b2FloatW;

#elif defined( B2_SIMD_SSE2 )

#include <emmintrin.h>

// wide float holds 4 numbers
typedef __m128 b2FloatW;

#else

// scalar math
typedef struct b2FloatW
{
	float x, y, z, w;
} b2FloatW;

#endif

// Wide vec2
typedef struct b2Vec2W
{
	b2FloatW X, Y;
} b2Vec2W;

// Wide rotation
typedef struct b2RotW
{
	b2FloatW C, S;
} b2RotW;

#if defined( B2_SIMD_AVX2 )

static inline b2FloatW b2ZeroW()
{
	return _mm256_setzero_ps();
}

static inline b2FloatW b2SplatW( float scalar )
{
	return _mm256_set1_ps( scalar );
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return _mm256_add_ps( a, b );
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return _mm256_sub_ps( a, b );
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return _mm256_mul_ps( a, b );
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	// FMA can be emulated: https://github.com/lattera/glibc/blob/master/sysdeps/ieee754/dbl-64/s_fmaf.c#L34
	// return _mm256_fmadd_ps( b, c, a );
	return _mm256_add_ps( _mm256_mul_ps( b, c ), a );
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	// return _mm256_fnmadd_ps(b, c, a);
	return _mm256_sub_ps( a, _mm256_mul_ps( b, c ) );
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	return _mm256_min_ps( a, b );
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	return _mm256_max_ps( a, b );
}

static inline b2FloatW b2OrW( b2FloatW a, b2FloatW b )
{
	return _mm256_or_ps( a, b );
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	return _mm256_cmp_ps( a, b, _CMP_GT_OQ );
}

static inline b2FloatW b2EqualsW( b2FloatW a, b2FloatW b )
{
	return _mm256_cmp_ps( a, b, _CMP_EQ_OQ );
}

// component-wise returns mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	return _mm256_blendv_ps( a, b, mask );
}

#elif defined( B2_SIMD_NEON )

static inline b2FloatW b2ZeroW()
{
	return vdupq_n_f32( 0.0f );
}

static inline b2FloatW b2SplatW( float scalar )
{
	return vdupq_n_f32( scalar );
}

static inline b2FloatW b2SetW( float a, float b, float c, float d )
{
	float32_t array[4] = { a, b, c, d };
	return vld1q_f32( array );
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return vaddq_f32( a, b );
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return vsubq_f32( a, b );
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return vmulq_f32( a, b );
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return vmlaq_f32( a, b, c );
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return vmlsq_f32( a, b, c );
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	return vminq_f32( a, b );
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	return vmaxq_f32( a, b );
}

static inline b2FloatW b2OrW( b2FloatW a, b2FloatW b )
{
	return vreinterpretq_f32_u32( vorrq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	return vreinterpretq_f32_u32( vcgtq_f32( a, b ) );
}

static inline b2FloatW b2EqualsW( b2FloatW a, b2FloatW b )
{
	return vreinterpretq_f32_u32( vceqq_f32( a, b ) );
}

// component-wise returns mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	uint32x4_t mask32 = vreinterpretq_u32_f32( mask );
	return vbslq_f32( mask32, b, a );
}

static inline b2FloatW b2LoadW( const float32_t* data )
{
	return vld1q_f32( data );
}

static inline void b2StoreW( float32_t* data, b2FloatW a )
{
	return vst1q_f32( data, a );
}

static inline b2FloatW b2UnpackLoW( b2FloatW a, b2FloatW b )
{
#if defined( __aarch64__ )
	return vzip1q_f32( a, b );
#else
	float32x2_t a1 = vget_low_f32( a );
	float32x2_t b1 = vget_low_f32( b );
	float32x2x2_t result = vzip_f32( a1, b1 );
	return vcombine_f32( result.val[0], result.val[1] );
#endif
}

static inline b2FloatW b2UnpackHiW( b2FloatW a, b2FloatW b )
{
#if defined( __aarch64__ )
	return vzip2q_f32( a, b );
#else
	float32x2_t a1 = vget_high_f32( a );
	float32x2_t b1 = vget_high_f32( b );
	float32x2x2_t result = vzip_f32( a1, b1 );
	return vcombine_f32( result.val[0], result.val[1] );
#endif
}

#elif defined( B2_SIMD_SSE2 )

static inline b2FloatW b2ZeroW()
{
	return _mm_setzero_ps();
}

static inline b2FloatW b2SplatW( float scalar )
{
	return _mm_set1_ps( scalar );
}

static inline b2FloatW b2SetW( float a, float b, float c, float d )
{
	return _mm_setr_ps( a, b, c, d );
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return _mm_add_ps( a, b );
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return _mm_sub_ps( a, b );
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return _mm_mul_ps( a, b );
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return _mm_add_ps( a, _mm_mul_ps( b, c ) );
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return _mm_sub_ps( a, _mm_mul_ps( b, c ) );
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	return _mm_min_ps( a, b );
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	return _mm_max_ps( a, b );
}

static inline b2FloatW b2OrW( b2FloatW a, b2FloatW b )
{
	return _mm_or_ps( a, b );
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	return _mm_cmpgt_ps( a, b );
}

static inline b2FloatW b2EqualsW( b2FloatW a, b2FloatW b )
{
	return _mm_cmpeq_ps( a, b );
}

// component-wise returns mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	return _mm_or_ps( _mm_and_ps( mask, b ), _mm_andnot_ps( mask, a ) );
}

static inline b2FloatW b2LoadW( const float* data )
{
	return _mm_load_ps( data );
}

static inline void b2StoreW( float* data, b2FloatW a )
{
	_mm_store_ps( data, a );
}

static inline b2FloatW b2UnpackLoW( b2FloatW a, b2FloatW b )
{
	return _mm_unpacklo_ps( a, b );
}

static inline b2FloatW b2UnpackHiW( b2FloatW a, b2FloatW b )
{
	return _mm_unpackhi_ps( a, b );
}

#else

static inline b2FloatW b2ZeroW()
{
	return ( b2FloatW ){ 0.0f, 0.0f, 0.0f, 0.0f };
}

static inline b2FloatW b2SplatW( float scalar )
{
	return ( b2FloatW ){ scalar, scalar, scalar, scalar };
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return ( b2FloatW ){ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return ( b2FloatW ){ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return ( b2FloatW ){ a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w };
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return ( b2FloatW ){ a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w };
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return ( b2FloatW ){ a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w };
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	return r;
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

static inline b2FloatW b2OrW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x != 0.0f || b.x != 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f || b.y != 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f || b.z != 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f || b.w != 0.0f ? 1.0f : 0.0f;
	return r;
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x > b.x ? 1.0f : 0.0f;
	r.y = a.y > b.y ? 1.0f : 0.0f;
	r.z = a.z > b.z ? 1.0f : 0.0f;
	r.w = a.w > b.w ? 1.0f : 0.0f;
	return r;
}

static inline b2FloatW b2EqualsW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x == b.x ? 1.0f : 0.0f;
	r.y = a.y == b.y ? 1.0f : 0.0f;
	r.z = a.z == b.z ? 1.0f : 0.0f;
	r.w = a.w == b.w ? 1.0f : 0.0f;
	return r;
}

// component-wise returns mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	b2FloatW r;
	r.x = mask.x != 0.0f ? b.x : a.x;
	r.y = mask.y != 0.0f ? b.y : a.y;
	r.z = mask.z != 0.0f ? b.z : a.z;
	r.w = mask.w != 0.0f ? b.w : a.w;
	return r;
}

#endif

static inline b2FloatW b2DotW( b2Vec2W a, b2Vec2W b )
{
	return b2AddW( b2MulW( a.X, b.X ), b2MulW( a.Y, b.Y ) );
}

static inline b2FloatW b2CrossW( b2Vec2W a, b2Vec2W b )
{
	return b2SubW( b2MulW( a.X, b.Y ), b2MulW( a.Y, b.X ) );
}

static inline b2Vec2W b2RotateVectorW( b2RotW q, b2Vec2W v )
{
	return ( b2Vec2W ){ b2SubW( b2MulW( q.C, v.X ), b2MulW( q.S, v.Y ) ), b2AddW( b2MulW( q.S, v.X ), b2MulW( q.C, v.Y ) ) };
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

typedef struct b2ContactConstraintSIMD
{
	int indexA[B2_SIMD_WIDTH];
	int indexB[B2_SIMD_WIDTH];

	b2FloatW invMassA, invMassB;
	b2FloatW invIA, invIB;
	b2Vec2W normal;
	b2FloatW friction;
	b2FloatW biasRate;
	b2FloatW massScale;
	b2FloatW impulseScale;
	b2Vec2W anchorA1, anchorB1;
	b2FloatW normalMass1, tangentMass1;
	b2FloatW baseSeparation1;
	b2FloatW normalImpulse1;
	b2FloatW maxNormalImpulse1;
	b2FloatW tangentImpulse1;
	b2Vec2W anchorA2, anchorB2;
	b2FloatW baseSeparation2;
	b2FloatW normalImpulse2;
	b2FloatW maxNormalImpulse2;
	b2FloatW tangentImpulse2;
	b2FloatW normalMass2, tangentMass2;
	b2FloatW restitution;
	b2FloatW relativeVelocity1, relativeVelocity2;
} b2ContactConstraintSIMD;

int b2GetContactConstraintSIMDByteCount( void )
{
	return sizeof( b2ContactConstraintSIMD );
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

// Custom gather/scatter for each SIMD type
#if defined( B2_SIMD_AVX2 )

// This is a load and 8x8 transpose
static b2SimdBody b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );
	// b2BodyState b2_identityBodyState = {{0.0f, 0.0f}, 0.0f, 0, {0.0f, 0.0f}, {1.0f, 0.0f}};
	b2FloatW identity = _mm256_setr_ps( 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 1.0f, 0.0f );
	b2FloatW b0 = indices[0] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[0] ) );
	b2FloatW b1 = indices[1] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[1] ) );
	b2FloatW b2 = indices[2] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[2] ) );
	b2FloatW b3 = indices[3] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[3] ) );
	b2FloatW b4 = indices[4] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[4] ) );
	b2FloatW b5 = indices[5] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[5] ) );
	b2FloatW b6 = indices[6] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[6] ) );
	b2FloatW b7 = indices[7] == B2_NULL_INDEX ? identity : _mm256_load_ps( (float*)( states + indices[7] ) );

	b2FloatW t0 = _mm256_unpacklo_ps( b0, b1 );
	b2FloatW t1 = _mm256_unpackhi_ps( b0, b1 );
	b2FloatW t2 = _mm256_unpacklo_ps( b2, b3 );
	b2FloatW t3 = _mm256_unpackhi_ps( b2, b3 );
	b2FloatW t4 = _mm256_unpacklo_ps( b4, b5 );
	b2FloatW t5 = _mm256_unpackhi_ps( b4, b5 );
	b2FloatW t6 = _mm256_unpacklo_ps( b6, b7 );
	b2FloatW t7 = _mm256_unpackhi_ps( b6, b7 );
	b2FloatW tt0 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt1 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt2 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt3 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt4 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt5 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt6 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt7 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

	b2SimdBody simdBody;
	simdBody.v.X = _mm256_permute2f128_ps( tt0, tt4, 0x20 );
	simdBody.v.Y = _mm256_permute2f128_ps( tt1, tt5, 0x20 );
	simdBody.w = _mm256_permute2f128_ps( tt2, tt6, 0x20 );
	simdBody.flags = _mm256_permute2f128_ps( tt3, tt7, 0x20 );
	simdBody.dp.X = _mm256_permute2f128_ps( tt0, tt4, 0x31 );
	simdBody.dp.Y = _mm256_permute2f128_ps( tt1, tt5, 0x31 );
	simdBody.dq.C = _mm256_permute2f128_ps( tt2, tt6, 0x31 );
	simdBody.dq.S = _mm256_permute2f128_ps( tt3, tt7, 0x31 );
	return simdBody;
}

// This writes everything back to the solver bodies but only the velocities change
static void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices, const b2SimdBody* B2_RESTRICT simdBody )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );
	b2FloatW t0 = _mm256_unpacklo_ps( simdBody->v.X, simdBody->v.Y );
	b2FloatW t1 = _mm256_unpackhi_ps( simdBody->v.X, simdBody->v.Y );
	b2FloatW t2 = _mm256_unpacklo_ps( simdBody->w, simdBody->flags );
	b2FloatW t3 = _mm256_unpackhi_ps( simdBody->w, simdBody->flags );
	b2FloatW t4 = _mm256_unpacklo_ps( simdBody->dp.X, simdBody->dp.Y );
	b2FloatW t5 = _mm256_unpackhi_ps( simdBody->dp.X, simdBody->dp.Y );
	b2FloatW t6 = _mm256_unpacklo_ps( simdBody->dq.C, simdBody->dq.S );
	b2FloatW t7 = _mm256_unpackhi_ps( simdBody->dq.C, simdBody->dq.S );
	b2FloatW tt0 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt1 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt2 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt3 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt4 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt5 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt6 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt7 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if ( indices[0] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[0] ), _mm256_permute2f128_ps( tt0, tt4, 0x20 ) );
	if ( indices[1] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[1] ), _mm256_permute2f128_ps( tt1, tt5, 0x20 ) );
	if ( indices[2] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[2] ), _mm256_permute2f128_ps( tt2, tt6, 0x20 ) );
	if ( indices[3] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[3] ), _mm256_permute2f128_ps( tt3, tt7, 0x20 ) );
	if ( indices[4] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[4] ), _mm256_permute2f128_ps( tt0, tt4, 0x31 ) );
	if ( indices[5] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[5] ), _mm256_permute2f128_ps( tt1, tt5, 0x31 ) );
	if ( indices[6] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[6] ), _mm256_permute2f128_ps( tt2, tt6, 0x31 ) );
	if ( indices[7] != B2_NULL_INDEX )
		_mm256_store_ps( (float*)( states + indices[7] ), _mm256_permute2f128_ps( tt3, tt7, 0x31 ) );
}

#elif defined( B2_SIMD_NEON )

// This is a load and transpose
static b2SimdBody b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );

	// [vx vy w flags]
	b2FloatW identityA = b2ZeroW();

	// [dpx dpy dqc dqs]

	b2FloatW identityB = b2SetW( 0.0f, 0.0f, 1.0f, 0.0f );

	b2FloatW b1a = indices[0] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[0] ) + 0 );
	b2FloatW b1b = indices[0] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[0] ) + 4 );
	b2FloatW b2a = indices[1] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[1] ) + 0 );
	b2FloatW b2b = indices[1] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[1] ) + 4 );
	b2FloatW b3a = indices[2] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[2] ) + 0 );
	b2FloatW b3b = indices[2] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[2] ) + 4 );
	b2FloatW b4a = indices[3] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[3] ) + 0 );
	b2FloatW b4b = indices[3] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[3] ) + 4 );

	// [vx1 vx3 vy1 vy3]
	b2FloatW t1a = b2UnpackLoW( b1a, b3a );

	// [vx2 vx4 vy2 vy4]
	b2FloatW t2a = b2UnpackLoW( b2a, b4a );

	// [w1 w3 f1 f3]
	b2FloatW t3a = b2UnpackHiW( b1a, b3a );

	// [w2 w4 f2 f4]
	b2FloatW t4a = b2UnpackHiW( b2a, b4a );

	b2SimdBody simdBody;
	simdBody.v.X = b2UnpackLoW( t1a, t2a );
	simdBody.v.Y = b2UnpackHiW( t1a, t2a );
	simdBody.w = b2UnpackLoW( t3a, t4a );
	simdBody.flags = b2UnpackHiW( t3a, t4a );

	b2FloatW t1b = b2UnpackLoW( b1b, b3b );
	b2FloatW t2b = b2UnpackLoW( b2b, b4b );
	b2FloatW t3b = b2UnpackHiW( b1b, b3b );
	b2FloatW t4b = b2UnpackHiW( b2b, b4b );

	simdBody.dp.X = b2UnpackLoW( t1b, t2b );
	simdBody.dp.Y = b2UnpackHiW( t1b, t2b );
	simdBody.dq.C = b2UnpackLoW( t3b, t4b );
	simdBody.dq.S = b2UnpackHiW( t3b, t4b );

	return simdBody;
}

// This writes only the velocities back to the solver bodies
// https://developer.arm.com/documentation/102107a/0100/Floating-point-4x4-matrix-transposition
static void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices, const b2SimdBody* B2_RESTRICT simdBody )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );

	//	b2FloatW x = b2SetW(0.0f, 1.0f, 2.0f, 3.0f);
	//	b2FloatW y = b2SetW(4.0f, 5.0f, 6.0f, 7.0f);
	//	b2FloatW z = b2SetW(8.0f, 9.0f, 10.0f, 11.0f);
	//	b2FloatW w = b2SetW(12.0f, 13.0f, 14.0f, 15.0f);
	//
	//	float32x4x2_t rr1 = vtrnq_f32( x, y );
	//	float32x4x2_t rr2 = vtrnq_f32( z, w );
	//
	//	float32x4_t b1 = vcombine_f32(vget_low_f32(rr1.val[0]), vget_low_f32(rr2.val[0]));
	//	float32x4_t b2 = vcombine_f32(vget_low_f32(rr1.val[1]), vget_low_f32(rr2.val[1]));
	//	float32x4_t b3 = vcombine_f32(vget_high_f32(rr1.val[0]), vget_high_f32(rr2.val[0]));
	//	float32x4_t b4 = vcombine_f32(vget_high_f32(rr1.val[1]), vget_high_f32(rr2.val[1]));

	// transpose
	float32x4x2_t r1 = vtrnq_f32( simdBody->v.X, simdBody->v.Y );
	float32x4x2_t r2 = vtrnq_f32( simdBody->w, simdBody->flags );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if ( indices[0] != B2_NULL_INDEX )
	{
		float32x4_t body1 = vcombine_f32( vget_low_f32( r1.val[0] ), vget_low_f32( r2.val[0] ) );
		b2StoreW( (float*)( states + indices[0] ), body1 );
	}

	if ( indices[1] != B2_NULL_INDEX )
	{
		float32x4_t body2 = vcombine_f32( vget_low_f32( r1.val[1] ), vget_low_f32( r2.val[1] ) );
		b2StoreW( (float*)( states + indices[1] ), body2 );
	}

	if ( indices[2] != B2_NULL_INDEX )
	{
		float32x4_t body3 = vcombine_f32( vget_high_f32( r1.val[0] ), vget_high_f32( r2.val[0] ) );
		b2StoreW( (float*)( states + indices[2] ), body3 );
	}

	if ( indices[3] != B2_NULL_INDEX )
	{
		float32x4_t body4 = vcombine_f32( vget_high_f32( r1.val[1] ), vget_high_f32( r2.val[1] ) );
		b2StoreW( (float*)( states + indices[3] ), body4 );
	}
}

#elif defined( B2_SIMD_SSE2 )

// This is a load and transpose
static b2SimdBody b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );

	// [vx vy w flags]
	b2FloatW identityA = b2ZeroW();

	// [dpx dpy dqc dqs]
	b2FloatW identityB = b2SetW( 0.0f, 0.0f, 1.0f, 0.0f );

	b2FloatW b1a = indices[0] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[0] ) + 0 );
	b2FloatW b1b = indices[0] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[0] ) + 4 );
	b2FloatW b2a = indices[1] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[1] ) + 0 );
	b2FloatW b2b = indices[1] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[1] ) + 4 );
	b2FloatW b3a = indices[2] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[2] ) + 0 );
	b2FloatW b3b = indices[2] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[2] ) + 4 );
	b2FloatW b4a = indices[3] == B2_NULL_INDEX ? identityA : b2LoadW( (float*)( states + indices[3] ) + 0 );
	b2FloatW b4b = indices[3] == B2_NULL_INDEX ? identityB : b2LoadW( (float*)( states + indices[3] ) + 4 );

	// [vx1 vx3 vy1 vy3]
	b2FloatW t1a = b2UnpackLoW( b1a, b3a );

	// [vx2 vx4 vy2 vy4]
	b2FloatW t2a = b2UnpackLoW( b2a, b4a );

	// [w1 w3 f1 f3]
	b2FloatW t3a = b2UnpackHiW( b1a, b3a );

	// [w2 w4 f2 f4]
	b2FloatW t4a = b2UnpackHiW( b2a, b4a );

	b2SimdBody simdBody;
	simdBody.v.X = b2UnpackLoW( t1a, t2a );
	simdBody.v.Y = b2UnpackHiW( t1a, t2a );
	simdBody.w = b2UnpackLoW( t3a, t4a );
	simdBody.flags = b2UnpackHiW( t3a, t4a );

	b2FloatW t1b = b2UnpackLoW( b1b, b3b );
	b2FloatW t2b = b2UnpackLoW( b2b, b4b );
	b2FloatW t3b = b2UnpackHiW( b1b, b3b );
	b2FloatW t4b = b2UnpackHiW( b2b, b4b );

	simdBody.dp.X = b2UnpackLoW( t1b, t2b );
	simdBody.dp.Y = b2UnpackHiW( t1b, t2b );
	simdBody.dq.C = b2UnpackLoW( t3b, t4b );
	simdBody.dq.S = b2UnpackHiW( t3b, t4b );

	return simdBody;
}

// This writes only the velocities back to the solver bodies
static void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices, const b2SimdBody* B2_RESTRICT simdBody )
{
	_Static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );

	// [vx1 vy1 vx2 vy2]
	b2FloatW t1 = b2UnpackLoW( simdBody->v.X, simdBody->v.Y );
	// [vx3 vy3 vx4 vy4]
	b2FloatW t2 = b2UnpackHiW( simdBody->v.X, simdBody->v.Y );
	// [w1 f1 w2 f2]
	b2FloatW t3 = b2UnpackLoW( simdBody->w, simdBody->flags );
	// [w3 f3 w4 f4]
	b2FloatW t4 = b2UnpackHiW( simdBody->w, simdBody->flags );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if ( indices[0] != B2_NULL_INDEX )
	{
		// [t1.x t1.y t3.x t3.y]
		b2StoreW( (float*)( states + indices[0] ), _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) ) );
	}

	if ( indices[1] != B2_NULL_INDEX )
	{
		// [t1.z t1.w t3.z t3.w]
		b2StoreW( (float*)( states + indices[1] ), _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) ) );
	}

	if ( indices[2] != B2_NULL_INDEX )
	{
		// [t2.x t2.y t4.x t4.y]
		b2StoreW( (float*)( states + indices[2] ), _mm_shuffle_ps( t2, t4, _MM_SHUFFLE( 1, 0, 1, 0 ) ) );
	}

	if ( indices[3] != B2_NULL_INDEX )
	{
		// [t2.z t2.w t4.z t4.w]
		b2StoreW( (float*)( states + indices[3] ), _mm_shuffle_ps( t2, t4, _MM_SHUFFLE( 3, 2, 3, 2 ) ) );
	}
}

#else

// This is a load and transpose
static b2SimdBody b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	b2BodyState identity = b2_identityBodyState;

	b2BodyState s1 = indices[0] == B2_NULL_INDEX ? identity : states[indices[0]];
	b2BodyState s2 = indices[1] == B2_NULL_INDEX ? identity : states[indices[1]];
	b2BodyState s3 = indices[2] == B2_NULL_INDEX ? identity : states[indices[2]];
	b2BodyState s4 = indices[3] == B2_NULL_INDEX ? identity : states[indices[3]];

	b2SimdBody simdBody;
	simdBody.v.X = ( b2FloatW ){ s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x };
	simdBody.v.Y = ( b2FloatW ){ s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y };
	simdBody.w = ( b2FloatW ){ s1.angularVelocity, s2.angularVelocity, s3.angularVelocity, s4.angularVelocity };
	simdBody.flags = ( b2FloatW ){ (float)s1.flags, (float)s2.flags, (float)s3.flags, (float)s4.flags };
	simdBody.dp.X = ( b2FloatW ){ s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x };
	simdBody.dp.Y = ( b2FloatW ){ s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y };
	simdBody.dq.C = ( b2FloatW ){ s1.deltaRotation.c, s2.deltaRotation.c, s3.deltaRotation.c, s4.deltaRotation.c };
	simdBody.dq.S = ( b2FloatW ){ s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s };

	return simdBody;
}

// This writes only the velocities back to the solver bodies
static void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices, const b2SimdBody* B2_RESTRICT simdBody )
{
	if ( indices[0] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[0];
		state->linearVelocity.x = simdBody->v.X.x;
		state->linearVelocity.y = simdBody->v.Y.x;
		state->angularVelocity = simdBody->w.x;
	}

	if ( indices[1] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[1];
		state->linearVelocity.x = simdBody->v.X.y;
		state->linearVelocity.y = simdBody->v.Y.y;
		state->angularVelocity = simdBody->w.y;
	}

	if ( indices[2] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[2];
		state->linearVelocity.x = simdBody->v.X.z;
		state->linearVelocity.y = simdBody->v.Y.z;
		state->angularVelocity = simdBody->w.z;
	}

	if ( indices[3] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[3];
		state->linearVelocity.x = simdBody->v.X.w;
		state->linearVelocity.y = simdBody->v.Y.w;
		state->angularVelocity = simdBody->w.w;
	}
}

#endif

void b2PrepareContactsTask( int startIndex, int endIndex, b2StepContext* context )
{
	b2TracyCZoneNC( prepare_contact, "Prepare Contact", b2_colorYellow, true );
	b2World* world = context->world;
	b2ContactSim** contacts = context->contacts;
	b2ContactConstraintSIMD* constraints = context->simdContactConstraints;
	b2BodyState* awakeStates = context->states;
#if B2_VALIDATE
	b2Body* bodies = world->bodies.data;
#endif

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b2Softness contactSoftness = context->contactSoftness;
	b2Softness staticSoftness = context->staticSoftness;

	float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* constraint = constraints + i;

		for ( int j = 0; j < B2_SIMD_WIDTH; ++j )
		{
			b2ContactSim* contactSim = contacts[B2_SIMD_WIDTH * i + j];

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

				( (float*)&constraint->invMassA )[j] = 0.0f;
				( (float*)&constraint->invMassB )[j] = 0.0f;
				( (float*)&constraint->invIA )[j] = 0.0f;
				( (float*)&constraint->invIB )[j] = 0.0f;

				( (float*)&constraint->normal.X )[j] = 0.0f;
				( (float*)&constraint->normal.Y )[j] = 0.0f;
				( (float*)&constraint->friction )[j] = 0.0f;
				( (float*)&constraint->biasRate )[j] = 0.0f;
				( (float*)&constraint->massScale )[j] = 0.0f;
				( (float*)&constraint->impulseScale )[j] = 0.0f;

				( (float*)&constraint->anchorA1.X )[j] = 0.0f;
				( (float*)&constraint->anchorA1.Y )[j] = 0.0f;
				( (float*)&constraint->anchorB1.X )[j] = 0.0f;
				( (float*)&constraint->anchorB1.Y )[j] = 0.0f;
				( (float*)&constraint->baseSeparation1 )[j] = 0.0f;
				( (float*)&constraint->normalImpulse1 )[j] = 0.0f;
				( (float*)&constraint->tangentImpulse1 )[j] = 0.0f;
				( (float*)&constraint->maxNormalImpulse1 )[j] = 0.0f;
				( (float*)&constraint->normalMass1 )[j] = 0.0f;
				( (float*)&constraint->tangentMass1 )[j] = 0.0f;

				( (float*)&constraint->anchorA2.X )[j] = 0.0f;
				( (float*)&constraint->anchorA2.Y )[j] = 0.0f;
				( (float*)&constraint->anchorB2.X )[j] = 0.0f;
				( (float*)&constraint->anchorB2.Y )[j] = 0.0f;
				( (float*)&constraint->baseSeparation2 )[j] = 0.0f;
				( (float*)&constraint->normalImpulse2 )[j] = 0.0f;
				( (float*)&constraint->tangentImpulse2 )[j] = 0.0f;
				( (float*)&constraint->maxNormalImpulse2 )[j] = 0.0f;
				( (float*)&constraint->normalMass2 )[j] = 0.0f;
				( (float*)&constraint->tangentMass2 )[j] = 0.0f;

				( (float*)&constraint->restitution )[j] = 0.0f;
				( (float*)&constraint->relativeVelocity1 )[j] = 0.0f;
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
		b2FloatW tangentY = b2SubW( b2ZeroW(), c->normal.X );

		{
			// fixed anchors
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			b2Vec2W P;
			P.X = b2AddW( b2MulW( c->normalImpulse1, c->normal.X ), b2MulW( c->tangentImpulse1, tangentX ) );
			P.Y = b2AddW( b2MulW( c->normalImpulse1, c->normal.Y ), b2MulW( c->tangentImpulse1, tangentY ) );
			bA.w = b2MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, P.X );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, P.Y );
			bB.w = b2MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, P.X );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, P.Y );
		}

		{
			// fixed anchors
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			b2Vec2W P;
			P.X = b2AddW( b2MulW( c->normalImpulse2, c->normal.X ), b2MulW( c->tangentImpulse2, tangentX ) );
			P.Y = b2AddW( b2MulW( c->normalImpulse2, c->normal.Y ), b2MulW( c->tangentImpulse2, tangentY ) );
			bA.w = b2MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, P.X );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, P.Y );
			bB.w = b2MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, P.X );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, P.Y );
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
	b2FloatW inv_h = b2SplatW( context->inv_h );
	b2FloatW minBiasVel = b2SplatW( -context->world->contactPushoutVelocity );

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
			biasRate = b2ZeroW();
			massScale = b2SplatW( 1.0f );
			impulseScale = b2ZeroW();
		}

		b2Vec2W dp = { b2SubW( bB.dp.X, bA.dp.X ), b2SubW( bB.dp.Y, bA.dp.Y ) };

		// point1 non-penetration constraint
		{
			// moving anchors for current separation
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA1 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB1 );

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( c->normal, ds ), c->baseSeparation1 );

			// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
			b2FloatW mask = b2GreaterThanW( s, b2ZeroW() );
			b2FloatW specBias = b2MulW( s, inv_h );
			b2FloatW softBias = b2MaxW( b2MulW( biasRate, s ), minBiasVel );
			b2FloatW bias = b2BlendW( softBias, specBias, mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2AddW( b2MulW( c->normalMass1, b2MulW( massScale, b2AddW( vn, bias ) ) ),
										  b2MulW( impulseScale, c->normalImpulse1 ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse1, negImpulse ), b2ZeroW() );
			b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse1 );
			c->normalImpulse1 = newImpulse;
			c->maxNormalImpulse1 = b2MaxW( c->maxNormalImpulse1, newImpulse );

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, c->normal.X );
			b2FloatW Py = b2MulW( impulse, c->normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// second point non-penetration constraint
		{
			// moving anchors for current separation
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA2 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB2 );

			// compute current separation
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( c->normal, ds ), c->baseSeparation2 );

			b2FloatW mask = b2GreaterThanW( s, b2ZeroW() );
			b2FloatW specBias = b2MulW( s, inv_h );
			b2FloatW softBias = b2MaxW( b2MulW( biasRate, s ), minBiasVel );
			b2FloatW bias = b2BlendW( softBias, specBias, mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2AddW( b2MulW( c->normalMass2, b2MulW( massScale, b2AddW( vn, bias ) ) ),
										  b2MulW( impulseScale, c->normalImpulse2 ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse2, negImpulse ), b2ZeroW() );
			b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse2 );
			c->normalImpulse2 = newImpulse;
			c->maxNormalImpulse2 = b2MaxW( c->maxNormalImpulse2, newImpulse );

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, c->normal.X );
			b2FloatW Py = b2MulW( impulse, c->normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		b2FloatW tangentX = c->normal.Y;
		b2FloatW tangentY = b2SubW( b2ZeroW(), c->normal.X );

		// point 1 friction constraint
		{
			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vt = b2AddW( b2MulW( dvx, tangentX ), b2MulW( dvy, tangentY ) );

			// Compute tangent force
			b2FloatW negImpulse = b2MulW( c->tangentMass1, vt );

			// Clamp the accumulated force
			b2FloatW maxFriction = b2MulW( c->friction, c->normalImpulse1 );
			b2FloatW newImpulse = b2SubW( c->tangentImpulse1, negImpulse );
			newImpulse = b2MaxW( b2SubW( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			b2FloatW impulse = b2SubW( newImpulse, c->tangentImpulse1 );
			c->tangentImpulse1 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, tangentX );
			b2FloatW Py = b2MulW( impulse, tangentY );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// second point friction constraint
		{
			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vt = b2AddW( b2MulW( dvx, tangentX ), b2MulW( dvy, tangentY ) );

			// Compute tangent force
			b2FloatW negImpulse = b2MulW( c->tangentMass2, vt );

			// Clamp the accumulated force
			b2FloatW maxFriction = b2MulW( c->friction, c->normalImpulse2 );
			b2FloatW newImpulse = b2SubW( c->tangentImpulse2, negImpulse );
			newImpulse = b2MaxW( b2SubW( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			b2FloatW impulse = b2SubW( newImpulse, c->tangentImpulse2 );
			c->tangentImpulse2 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, tangentX );
			b2FloatW Py = b2MulW( impulse, tangentY );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
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
	b2FloatW threshold = b2SplatW( context->world->restitutionThreshold );
	b2FloatW zero = b2ZeroW();

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;

		b2SimdBody bA = b2GatherBodies( states, c->indexA );
		b2SimdBody bB = b2GatherBodies( states, c->indexB );

		// first point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			b2FloatW mask1 = b2GreaterThanW( b2AddW( c->relativeVelocity1, threshold ), zero );
			b2FloatW mask2 = b2EqualsW( c->maxNormalImpulse1, zero );
			b2FloatW mask = b2OrW( mask1, mask2 );
			b2FloatW mass = b2BlendW( c->normalMass1, zero, mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2MulW( mass, b2AddW( vn, b2MulW( c->restitution, c->relativeVelocity1 ) ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse1, negImpulse ), b2ZeroW() );
			b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse1 );
			c->normalImpulse1 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, c->normal.X );
			b2FloatW Py = b2MulW( impulse, c->normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// second point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			b2FloatW mask1 = b2GreaterThanW( b2AddW( c->relativeVelocity2, threshold ), zero );
			b2FloatW mask2 = b2EqualsW( c->maxNormalImpulse2, zero );
			b2FloatW mask = b2OrW( mask1, mask2 );
			b2FloatW mass = b2BlendW( c->normalMass2, zero, mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2MulW( mass, b2AddW( vn, b2MulW( c->restitution, c->relativeVelocity2 ) ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse2, negImpulse ), b2ZeroW() );
			b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse2 );
			c->normalImpulse2 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, c->normal.X );
			b2FloatW Py = b2MulW( impulse, c->normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

	b2TracyCZoneEnd( restitution );
}

#if B2_SIMD_WIDTH == 8

// todo try making an inner loop on B2_SIMD_WIDTH to have a single implementation of this function
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

#else

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

		int base = 4 * i;
		b2Manifold* m0 = contacts[base + 0] == NULL ? &dummy : &contacts[base + 0]->manifold;
		b2Manifold* m1 = contacts[base + 1] == NULL ? &dummy : &contacts[base + 1]->manifold;
		b2Manifold* m2 = contacts[base + 2] == NULL ? &dummy : &contacts[base + 2]->manifold;
		b2Manifold* m3 = contacts[base + 3] == NULL ? &dummy : &contacts[base + 3]->manifold;

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
	}

	b2TracyCZoneEnd( store_impulses );
}

#endif
