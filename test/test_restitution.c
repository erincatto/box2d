// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "physics_world.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEP_COUNT 4

// A two point landing converges its Gauss-Seidel coupling in four stage iterations and the default
// is two, so the tests that read the coefficient or the spin of a flat landing pin the count
#define RESTITUTION_ITERATIONS 4

// Impact speed shared by every scenario that measures a coefficient. Well above the default
// restitution threshold so the bounce is always armed.
#define IMPACT_SPEED 5.0f

// Each subtest prints all of its measurements before asserting, and the battery runs every subtest
// so one bad scenario does not hide the rest.
#define RUN_MEASUREMENT( T )                                                                                                     \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( T() != 0 )                                                                                                          \
		{                                                                                                                        \
			printf( "  subtest failed: " #T "\n" );                                                                              \
			failureCount += 1;                                                                                                   \
		}                                                                                                                        \
		else                                                                                                                     \
		{                                                                                                                        \
			printf( "  subtest passed: " #T "\n" );                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static b2WorldId MakeWorld( float gravityY )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, gravityY };
	worldDef.enableSleep = false;
	return b2CreateWorld( &worldDef );
}

static b2WorldId MakeWorldIterations( float gravityY, int restitutionIterations )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, gravityY };
	worldDef.enableSleep = false;
	worldDef.restitutionIterations = restitutionIterations;
	return b2CreateWorld( &worldDef );
}

// Ground with its top surface at y = 0
static void MakeGround( b2WorldId worldId, float restitution )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = (b2Pos){ 0.0f, -1.0f };
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.0f;
	shapeDef.material.restitution = restitution;
	b2Polygon box = b2MakeBox( 40.0f, 1.0f );
	b2CreatePolygonShape( groundId, &shapeDef, &box );
}

static b2BodyId MakeBall( b2WorldId worldId, float x, float y, float velocityY, float restitution )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ x, y };
	bodyDef.linearVelocity = (b2Vec2){ 0.0f, velocityY };
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.0f;
	shapeDef.material.restitution = restitution;
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2CreateCircleShape( bodyId, &shapeDef, &circle );
	return bodyId;
}

// Two free circles closing head on with no gravity. The coefficient is the ratio of relative normal
// speeds at the contact, which is the mass independent definition.
static float MeasureHeadOn( float restitution, float densityB, float* momentumError )
{
	b2WorldId worldId = MakeWorld( 0.0f );

	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = 0.0f;
	shapeDef.material.restitution = restitution;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ -1.0f, 0.0f };
	bodyDef.linearVelocity = (b2Vec2){ 0.5f * IMPACT_SPEED, 0.0f };
	b2BodyId idA = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( idA, &shapeDef, &circle );

	bodyDef.position = (b2Pos){ 1.0f, 0.0f };
	bodyDef.linearVelocity = (b2Vec2){ -0.5f * IMPACT_SPEED, 0.0f };
	b2BodyId idB = b2CreateBody( worldId, &bodyDef );
	shapeDef.density = densityB;
	b2CreateCircleShape( idB, &shapeDef, &circle );

	float massA = b2Body_GetMass( idA );
	float massB = b2Body_GetMass( idB );
	float momentum0 = massA * b2Body_GetLinearVelocity( idA ).x + massB * b2Body_GetLinearVelocity( idB ).x;

	for ( int i = 0; i < 120; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	float vA = b2Body_GetLinearVelocity( idA ).x;
	float vB = b2Body_GetLinearVelocity( idB ).x;
	float momentum1 = massA * vA + massB * vB;

	b2DestroyWorld( worldId );

	*momentumError = ( momentum1 - momentum0 ) / ( massA + massB );
	return ( vB - vA ) / IMPACT_SPEED;
}

// Ball driven into static ground with no gravity so the measurement carries no gravity bias. The
// gap shifts where inside the time step the impact lands.
static float MeasureGroundBounce( float restitution, float gap )
{
	b2WorldId worldId = MakeWorld( 0.0f );
	MakeGround( worldId, 0.0f );

	b2BodyId ballId = MakeBall( worldId, 0.0f, 0.5f + gap, -IMPACT_SPEED, restitution );

	for ( int i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	float speed = b2Body_GetLinearVelocity( ballId ).y;
	b2DestroyWorld( worldId );
	return speed / IMPACT_SPEED;
}

// Impactor driven onto a column of resting balls pinned against the ground. The support balls are
// dead so only the top contact can bounce. A supported target has infinite effective mass along the
// normal, so the coefficient must not depend on the column height.
static float MeasureSupportedBounce( float restitution, int supportCount, bool propagate )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, 0.0f };
	worldDef.enableSleep = false;
	worldDef.enableRestitutionPropagation = propagate;
	worldDef.restitutionIterations = RESTITUTION_ITERATIONS;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	MakeGround( worldId, 0.0f );

	for ( int i = 0; i < supportCount; ++i )
	{
		MakeBall( worldId, 0.0f, 0.5f + 1.0f * i, 0.0f, 0.0f );
	}

	float y = 0.5f + 1.0f * supportCount + 0.5f * IMPACT_SPEED * TIME_STEP;
	b2BodyId ballId = MakeBall( worldId, 0.0f, y, -IMPACT_SPEED, restitution );

	for ( int i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	float speed = b2Body_GetLinearVelocity( ballId ).y;
	b2DestroyWorld( worldId );
	return speed / IMPACT_SPEED;
}

// Flat box landing on both corners at once with no gravity
static float MeasureFlatBounce( float restitution, int subStepCount, float* spin )
{
	b2WorldId worldId = MakeWorldIterations( 0.0f, RESTITUTION_ITERATIONS );
	MakeGround( worldId, 0.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, 0.25f + 0.5f * IMPACT_SPEED * TIME_STEP };
	bodyDef.linearVelocity = (b2Vec2){ 0.0f, -IMPACT_SPEED };
	b2BodyId boxId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = 0.0f;
	shapeDef.material.restitution = restitution;
	b2Polygon box = b2MakeBox( 1.0f, 0.25f );
	b2CreatePolygonShape( boxId, &shapeDef, &box );

	for ( int i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, TIME_STEP, subStepCount );
	}

	float speed = b2Body_GetLinearVelocity( boxId ).y;
	*spin = b2Body_GetAngularVelocity( boxId );
	b2DestroyWorld( worldId );
	return speed / IMPACT_SPEED;
}

// Perfectly elastic ball dropped onto a segment, the shape the Restitution sample uses
static int MeasureDrop( float dropHeight, float* apexes, int capacity )
{
	b2WorldId worldId = MakeWorld( -10.0f );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShape = b2DefaultShapeDef();
	b2Segment segment = { { -60.0f, 0.0f }, { 60.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &groundShape, &segment );

	b2BodyId ballId = MakeBall( worldId, 0.0f, dropHeight, 0.0f, 1.0f );

	int apexCount = 0;
	float previousSpeed = 0.0f;

	for ( int i = 0; i < 4000 && apexCount < capacity; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		float speed = b2Body_GetLinearVelocity( ballId ).y;
		if ( previousSpeed > 0.0f && speed <= 0.0f )
		{
			apexes[apexCount] = (float)b2Body_GetPosition( ballId ).y;
			apexCount += 1;
		}
		previousSpeed = speed;
	}

	b2DestroyWorld( worldId );
	return apexCount;
}

static int HeadOnTest( void )
{
	static const float restitutions[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
	static const float densities[] = { 1.0f, 10.0f, 100.0f };
	const float tolerance = 0.02f;

	int failed = 0;

	for ( int i = 0; i < ARRAY_COUNT( densities ); ++i )
	{
		float worstError = 0.0f;
		float worstMomentum = 0.0f;

		for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
		{
			float momentumError = 0.0f;
			float measured = MeasureHeadOn( restitutions[j], densities[i], &momentumError );

			float error = b2AbsFloat( measured - restitutions[j] );
			if ( error > worstError )
			{
				worstError = error;
			}

			if ( b2AbsFloat( momentumError ) > b2AbsFloat( worstMomentum ) )
			{
				worstMomentum = momentumError;
			}

			if ( error > tolerance )
			{
				printf( "    head on density %.0f e %.2f -> %.4f\n", densities[i], restitutions[j], measured );
				failed = 1;
			}
		}

		printf( "    head on density %6.1f worst error %.4f momentum drift %.2e\n", densities[i], worstError, worstMomentum );

		if ( b2AbsFloat( worstMomentum ) > 1.0e-4f )
		{
			failed = 1;
		}
	}

	return failed;
}

static int PhaseTest( void )
{
	static const float restitutions[] = { 0.25f, 0.5f, 0.9f };
	const int sampleCount = 16;
	const float tolerance = 0.05f;

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
	{
		float minimum = FLT_MAX;
		float maximum = -FLT_MAX;
		float sum = 0.0f;

		for ( int i = 0; i < sampleCount; ++i )
		{
			// Sweep the gap across one step of travel so the impact lands at every phase
			float gap = ( (float)i / (float)sampleCount ) * IMPACT_SPEED * TIME_STEP;
			float measured = MeasureGroundBounce( restitutions[j], gap );
			minimum = b2MinFloat( minimum, measured );
			maximum = b2MaxFloat( maximum, measured );
			sum += measured;
		}

		float mean = sum / sampleCount;
		float spread = maximum - minimum;
		printf( "    phase e %.2f -> mean %.4f spread %.4f [%.4f, %.4f]\n", restitutions[j], mean, spread, minimum, maximum );

		if ( spread > tolerance || b2AbsFloat( mean - restitutions[j] ) > tolerance )
		{
			failed = 1;
		}
	}

	return failed;
}

// By default the restitution stage sweeps only the contacts that carry restitution, so the ball
// pushes the column down and the ground under it gets no say until the next step: e = 0.9 gives
// about 0.52 on one support. With propagation enabled the stage sweeps every contact, the ground
// contact supplies the reaction within the stage and e = 0.9 gives 0.85 on one support and 0.73
// on three at the pinned four iterations (0.71 and 0.65 at the default two), decaying with depth
// because the column needs an iteration per level. Propagation costs a normal pass over every
// contact per iteration, hence the option. Both modes are gated: never above the free coefficient,
// which would be energy from nowhere, and never collapsed.
static int SupportedTest( void )
{
	static const float restitutions[] = { 0.5f, 0.9f };

	int failed = 0;

	for ( int propagate = 0; propagate <= 1; ++propagate )
	{
		float floor = propagate ? 0.75f : 0.5f;

		for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
		{
			for ( int n = 0; n <= 3; ++n )
			{
				float measured = MeasureSupportedBounce( restitutions[j], n, propagate == 1 );
				printf( "    supported e %.2f supports %d propagation %d -> %.4f\n", restitutions[j], n, propagate, measured );

				if ( measured > restitutions[j] + 0.01f || measured < floor * restitutions[j] )
				{
					failed = 1;
				}
			}
		}
	}

	return failed;
}

// A symmetric two point landing. The stage solves the two points in sequence, so a single sweep
// leaves the box spinning: the first impulse tilts it and the second point sees a larger closing
// speed. Four stage iterations converge the coupling and leave a small residual, two leave the box
// with 0.3 rad/s. The tolerance admits that residual and rejects the single sweep behavior.
static int TwoPointTest( void )
{
	static const float restitutions[] = { 0.5f, 0.9f };
	static const int subStepCounts[] = { 4, 8 };
	const float speedTolerance = 0.01f;
	const float spinTolerance = 0.03f;

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
	{
		for ( int k = 0; k < ARRAY_COUNT( subStepCounts ); ++k )
		{
			float spin = 0.0f;
			float measured = MeasureFlatBounce( restitutions[j], subStepCounts[k], &spin );
			printf( "    two point e %.2f substeps %d -> %.4f spin %.4f\n", restitutions[j], subStepCounts[k], measured, spin );

			// Only the shipping sub step count is a gate. The wider count is reported so a
			// convergence problem can be told apart from a formulation problem.
			if ( subStepCounts[k] != SUB_STEP_COUNT )
			{
				continue;
			}

			if ( b2AbsFloat( measured - restitutions[j] ) > speedTolerance || b2AbsFloat( spin ) > spinTolerance )
			{
				failed = 1;
			}
		}
	}

	return failed;
}

// A flat box carrying spin. Both corners stay in contact and the constraints are linear in the body
// velocity, so perfect restitution reverses linear and angular velocity. Four stage iterations
// bring both within a few hundredths. One left a residual an order of magnitude larger.
static int SpinTest( void )
{
	static const float spins[] = { 0.0f, 0.5f, 1.0f, 2.0f };

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( spins ); ++j )
	{
		b2WorldId worldId = MakeWorldIterations( 0.0f, RESTITUTION_ITERATIONS );
		MakeGround( worldId, 0.0f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0f, 0.25f + 0.5f * IMPACT_SPEED * TIME_STEP };
		bodyDef.linearVelocity = (b2Vec2){ 0.0f, -IMPACT_SPEED };
		bodyDef.angularVelocity = spins[j];
		b2BodyId boxId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 1.0f;
		b2Polygon box = b2MakeBox( 1.0f, 0.25f );
		b2CreatePolygonShape( boxId, &shapeDef, &box );

		for ( int i = 0; i < 60; ++i )
		{
			b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		float speed = b2Body_GetLinearVelocity( boxId ).y;
		float spin = b2Body_GetAngularVelocity( boxId );
		b2DestroyWorld( worldId );

		printf( "    spin in %+.2f -> vy %+.4f (want %+.4f)  w %+.4f (want %+.4f)\n", spins[j], speed, IMPACT_SPEED, spin,
				-spins[j] );

		if ( b2AbsFloat( speed - IMPACT_SPEED ) > 0.05f || b2AbsFloat( spin + spins[j] ) > 0.05f )
		{
			failed = 1;
		}
	}

	return failed;
}

// Perfectly elastic ball under gravity. Only heights where continuous collision engages are used.
// Continuous collision lands the ball on the surface, so the bounce is armed from the true impact
// speed and the apex holds. Slower drops resolve the impact inside the overlap and the penetration
// recovery adds height, by design. The 10 m case is the long horizon read: the per manifold energy
// budget crept up 0.05% a bounce there while 20 m and 40 m looked clean.
static int DropTest( void )
{
	static const float heights[] = { 40.0f, 20.0f, 10.0f };

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( heights ); ++j )
	{
		float apexes[6] = { 0 };
		int apexCount = MeasureDrop( heights[j], apexes, ARRAY_COUNT( apexes ) );

		printf( "    drop %5.1f ->", heights[j] );
		for ( int i = 0; i < apexCount; ++i )
		{
			printf( " %8.3f", apexes[i] );
		}
		printf( "\n" );

		if ( apexCount < ARRAY_COUNT( apexes ) )
		{
			failed = 1;
			continue;
		}

		float highest = apexes[0];
		for ( int i = 1; i < apexCount; ++i )
		{
			highest = b2MaxFloat( highest, apexes[i] );
		}

		// A later apex above the first, or a first apex above the drop, is energy from nowhere
		if ( highest > 1.001f * apexes[0] || apexes[0] > 1.001f * heights[j] || apexes[apexCount - 1] < 0.8f * apexes[0] )
		{
			failed = 1;
		}
	}

	return failed;
}

// Mirrors the SingleBoxRestitution sample: square box, perfectly elastic, dropped flat onto a
// segment with an aggressive continuous safety factor so it lands square on the surface. A flat
// landing solves two coplanar points in sequence, and one sweep over-delivers: the first impulse
// tilts the box, the second point sees a larger closing speed and the sum overshoots the rigid body
// answer, leaving spin that tilts the box for the next landing. Four stage iterations converge the
// coupling, at the default two the plank leaves with 0.13 rad/s and tumbles on its second bounce.
// The bounds below reject the single sweep behavior and any apex above the drop.
static int SingleBoxTest( void )
{
	b2WorldId worldId = MakeWorldIterations( -10.0f, RESTITUTION_ITERATIONS );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShape = b2DefaultShapeDef();
	b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &groundShape, &segment );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.restitution = 1.0f;
	shapeDef.material.friction = 0.0f;
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );

	const float dropHeight = 10.0f;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, dropHeight };
	bodyDef.safetyFactor = 0.01f;
	b2BodyId boxId = b2CreateBody( worldId, &bodyDef );
	b2CreatePolygonShape( boxId, &shapeDef, &box );

	float firstSpin = 0.0f;
	float apexes[4] = { 0 };
	float spins[4] = { 0 };
	int apexCount = 0;
	float previousSpeed = 0.0f;

	// A nearly elastic bounce from 10 m takes about 165 steps, so budget for four of them
	for ( int i = 0; i < 1200 && apexCount < ARRAY_COUNT( apexes ); ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		float speed = b2Body_GetLinearVelocity( boxId ).y;

		if ( apexCount == 0 && previousSpeed <= 0.0f && speed > 0.0f )
		{
			firstSpin = b2AbsFloat( b2Body_GetAngularVelocity( boxId ) );
		}

		if ( previousSpeed > 0.0f && speed <= 0.0f )
		{
			apexes[apexCount] = (float)b2Body_GetPosition( boxId ).y;
			spins[apexCount] = b2AbsFloat( b2Body_GetAngularVelocity( boxId ) );
			apexCount += 1;
		}
		previousSpeed = speed;
	}

	b2DestroyWorld( worldId );

	printf( "    single box first bounce spin %.4f apexes", firstSpin );
	for ( int i = 0; i < apexCount; ++i )
	{
		printf( " %7.3f (spin %5.2f)", apexes[i], spins[i] );
	}
	printf( "\n" );

	ENSURE( apexCount == ARRAY_COUNT( apexes ) );

	int failed = 0;

	if ( firstSpin > 0.01f )
	{
		failed = 1;
	}

	for ( int i = 0; i < apexCount; ++i )
	{
		if ( apexes[i] < 0.95f * dropHeight || apexes[i] > 1.001f * dropHeight )
		{
			failed = 1;
		}
	}

	return failed;
}

static int ThresholdTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	float threshold = worldDef.restitutionThreshold;

	static const float scales[] = { 0.5f, 0.9f, 1.5f, 4.0f };

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( scales ); ++j )
	{
		float speed = scales[j] * threshold;

		b2WorldId worldId = MakeWorld( 0.0f );
		MakeGround( worldId, 0.0f );
		b2BodyId ballId = MakeBall( worldId, 0.0f, 0.5f + 0.5f * speed * TIME_STEP, -speed, 1.0f );

		for ( int i = 0; i < 60; ++i )
		{
			b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		float ratio = b2Body_GetLinearVelocity( ballId ).y / speed;
		b2DestroyWorld( worldId );

		printf( "    threshold %.2fx -> %.4f\n", scales[j], ratio );

		if ( scales[j] < 1.0f && ratio > 0.05f )
		{
			failed = 1;
		}

		if ( scales[j] > 1.0f && ratio < 0.9f )
		{
			failed = 1;
		}
	}

	return failed;
}

// High restitution must not wake a settled stack back up
static int RestingTest( void )
{
	b2WorldId worldId = MakeWorld( -10.0f );
	MakeGround( worldId, 0.9f );

	const int boxCount = 10;
	b2BodyId boxIds[10];

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = 0.6f;
	shapeDef.material.restitution = 0.9f;
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );

	for ( int i = 0; i < boxCount; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0f, 0.5f + 1.0f * i };
		boxIds[i] = b2CreateBody( worldId, &bodyDef );
		b2CreatePolygonShape( boxIds[i], &shapeDef, &box );
	}

	for ( int i = 0; i < 120; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	float settled = (float)b2Body_GetPosition( boxIds[boxCount - 1] ).y;
	float drift = 0.0f;
	float peakSpeed = 0.0f;

	for ( int i = 0; i < 300; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		float y = (float)b2Body_GetPosition( boxIds[boxCount - 1] ).y;
		drift = b2MaxFloat( drift, b2AbsFloat( y - settled ) );

		for ( int j = 0; j < boxCount; ++j )
		{
			peakSpeed = b2MaxFloat( peakSpeed, b2Length( b2Body_GetLinearVelocity( boxIds[j] ) ) );
		}
	}

	b2DestroyWorld( worldId );

	printf( "    resting drift %.5f peak speed %.5f\n", drift, peakSpeed );

	ENSURE( drift < 0.01f );
	ENSURE( peakSpeed < 0.05f );
	return 0;
}

// The manifold point normal velocity is published only for contacts that enabled hit events, and it
// is not a stale value otherwise, it is zero, so a reader can tell "not measured" from "measured
// zero" by whether the shape enables the events. Restitution no longer reads it at all, so this is
// the only thing keeping the field alive.
static float MeasureFirstTouchNormalVelocity( bool enableHitEvents )
{
	b2WorldId worldId = MakeWorld( 0.0f );
	MakeGround( worldId, 0.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, 0.6f };
	b2BodyId ballId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.0f;
	shapeDef.material.restitution = 0.0f;
	shapeDef.enableHitEvents = enableHitEvents;
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2CreateCircleShape( ballId, &shapeDef, &circle );

	float normalVelocity = FLT_MAX;

	// Hold the closing speed so the sampled value does not depend on which step the pair is created
	for ( int i = 0; i < 20; ++i )
	{
		b2Body_SetLinearVelocity( ballId, (b2Vec2){ 0.0f, -IMPACT_SPEED } );
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		b2ContactData contactData;
		if ( b2Body_GetContactData( ballId, &contactData, 1 ) == 1 && contactData.manifold.pointCount > 0 )
		{
			normalVelocity = contactData.manifold.points[0].normalVelocity;
			break;
		}
	}

	b2DestroyWorld( worldId );
	return normalVelocity;
}

static int NormalVelocityTest( void )
{
	float quiet = MeasureFirstTouchNormalVelocity( false );
	float published = MeasureFirstTouchNormalVelocity( true );

	printf( "    normal velocity hit events off %.4f on %.4f (closing at %.4f)\n", quiet, published, -IMPACT_SPEED );

	int failed = 0;

	if ( quiet != 0.0f )
	{
		failed = 1;
	}

	if ( published > -0.8f * IMPACT_SPEED )
	{
		failed = 1;
	}

	return failed;
}

// Mechanical energy of a body as symplectic Euler conserves it. The integrator updates velocity
// before position, so the quantity that is exactly constant in free flight is not kinetic plus
// potential but that plus half a sub-step of the gravity power, m * dot(v, g) * h / 2. Plain
// kinetic plus potential sampled at step boundaries jumps by about g * v * h at every bounce and
// drifts down by g^2 h^2 / 2 each sub-step, which would hide a real gain or fake one. With this
// measure a contact that never does positive work leaves the energy flat to round-off.
static float MeasureEnergy( b2BodyId bodyId, b2WorldId worldId )
{
	b2MassData massData = b2Body_GetMassData( bodyId );
	b2Vec2 v = b2Body_GetLinearVelocity( bodyId );
	float w = b2Body_GetAngularVelocity( bodyId );
	b2Vec2 gravity = b2MulSV( b2Body_GetGravityScale( bodyId ), b2World_GetGravity( worldId ) );
	float h = TIME_STEP / SUB_STEP_COUNT;

	float kinetic = 0.5f * massData.mass * b2Dot( v, v ) + 0.5f * massData.rotationalInertia * w * w;
	float potential = -massData.mass * b2Dot( gravity, b2ToVec2( b2Body_GetWorldCenter( bodyId ) ) );
	float phase = 0.5f * h * massData.mass * b2Dot( v, gravity );

	return kinetic + potential + phase;
}

typedef struct CornerResult
{
	float spin;
	float expectedSpin;
	float energyRatio;
} CornerResult;

// Box dropped tilted so one corner lands first, with continuous collision so it lands on the surface.
// The manifold still carries the high corner as a speculative point, which is what stops a spinning
// box from sweeping a corner through the ground, but a point that far out must not be armed: when it
// was, the box landed as if flat and came off with no spin. Newton's law at the single touching
// corner gives the expected spin.
static CornerResult MeasureCornerLanding( float angle )
{
	b2WorldId worldId = MakeWorld( -10.0f );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShape = b2DefaultShapeDef();
	b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &groundShape, &segment );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.restitution = 1.0f;
	shapeDef.material.friction = 0.0f;
	shapeDef.enableHitEvents = true;
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, 5.0f };
	bodyDef.rotation = b2MakeRot( angle );
	bodyDef.safetyFactor = 0.01f;
	b2BodyId boxId = b2CreateBody( worldId, &bodyDef );
	b2CreatePolygonShape( boxId, &shapeDef, &box );

	b2MassData massData = b2Body_GetMassData( boxId );
	float startEnergy = MeasureEnergy( boxId, worldId );

	CornerResult result = { 0 };

	for ( int i = 0; i < 400; ++i )
	{
		float speedBefore = b2Body_GetLinearVelocity( boxId ).y;
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		b2ContactEvents events = b2World_GetContactEvents( worldId );
		if ( events.hitCount > 0 )
		{
			result.spin = b2Body_GetAngularVelocity( boxId );

			// The box falls without spin, so the corner offset is the rotated box corner and the
			// approach speed is the fall speed. Perfectly elastic single point impulse.
			b2Vec2 r = b2RotateVector( b2MakeRot( angle ), (b2Vec2){ 0.5f, -0.5f } );
			float rn = b2Cross( r, (b2Vec2){ 0.0f, 1.0f } );
			float k = 1.0f / massData.mass + rn * rn / massData.rotationalInertia;
			float impulse = 2.0f * -speedBefore / k;
			result.expectedSpin = rn * impulse / massData.rotationalInertia;
			break;
		}
	}

	// Leave the floor, then read the energy once no contact remains
	for ( int i = 0; i < 400; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		b2ContactData contactData;
		if ( b2Body_GetContactData( boxId, &contactData, 1 ) == 0 && b2Body_GetLinearVelocity( boxId ).y < 0.0f )
		{
			result.energyRatio = MeasureEnergy( boxId, worldId ) / startEnergy;
			break;
		}
	}

	b2DestroyWorld( worldId );

	printf( "    corner landing angle %+.2f -> spin %+.3f (rigid %+.3f), energy after rebound %.4f\n", angle, result.spin,
			result.expectedSpin, result.energyRatio );

	return result;
}

// At the wider tilt the high corner stays clear of the floor for the rest of the step, so the first
// impact is the single point law to a percent and the rebound carries the energy. At the narrower
// tilt the high corner swings down and lands inside the same step. It is caught by the speculative
// constraint, which is not armed, so that secondary impact is inelastic and the rebound loses
// energy. The gate there only asks for significant spin, which the flat landing bug had at zero,
// and that the pair of impacts never beats the single point law.
static int CornerTest( void )
{
	CornerResult wide = MeasureCornerLanding( -0.4f );
	CornerResult narrow = MeasureCornerLanding( -0.2f );

	int failed = 0;

	if ( b2AbsFloat( wide.spin - wide.expectedSpin ) > 0.02f * wide.expectedSpin )
	{
		failed = 1;
	}

	if ( wide.energyRatio < 0.97f || wide.energyRatio > 1.01f )
	{
		failed = 1;
	}

	if ( narrow.spin < 0.4f * narrow.expectedSpin || narrow.spin > 1.02f * narrow.expectedSpin )
	{
		failed = 1;
	}

	if ( narrow.energyRatio > 1.01f )
	{
		failed = 1;
	}

	return failed;
}

typedef struct OvershootResult
{
	float firstApex;
	float firstEnergyRatio;
	float peakEnergyRatio;
	float speedRatio;
	int bounceCount;
} OvershootResult;

// A unit box dropped flat on a floor narrower than itself, perfectly elastic, so the two contact
// points sit inboard of the box corners. A perfectly elastic bounce cannot come back higher than it
// was dropped from and cannot gain energy.
static OvershootResult MeasureOvershoot( bool continuous )
{
	b2WorldId worldId = MakeWorldIterations( -10.0f, RESTITUTION_ITERATIONS );

	b2BodyDef floorDef = b2DefaultBodyDef();
	floorDef.position = (b2Pos){ 0.0f, -0.25f };
	b2BodyId floorId = b2CreateBody( worldId, &floorDef );

	b2ShapeDef floorShape = b2DefaultShapeDef();
	b2Polygon floor = b2MakeBox( 0.375f, 0.25f );
	b2CreatePolygonShape( floorId, &floorShape, &floor );

	const float dropHeight = 10.0f;

	b2BodyDef boxDef = b2DefaultBodyDef();
	boxDef.type = b2_dynamicBody;
	boxDef.position = (b2Pos){ 0.0f, dropHeight };
	if ( continuous )
	{
		boxDef.safetyFactor = 0.1f;
	}
	b2BodyId boxId = b2CreateBody( worldId, &boxDef );

	b2ShapeDef boxShape = b2DefaultShapeDef();
	boxShape.material.restitution = 1.0f;
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( boxId, &boxShape, &box );

	OvershootResult result = { 0 };

	float startEnergy = MeasureEnergy( boxId, worldId );
	float impactSpeed = 0.0f;
	float reboundSpeed = 0.0f;
	float bounceHeight = 0.0f;
	float previousSpeed = 0.0f;

	for ( int i = 0; i < 600 && result.bounceCount < 2; ++i )
	{
		float before = b2Body_GetLinearVelocity( boxId ).y;
		float heightBefore = (float)b2Body_GetPosition( boxId ).y;
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		float speed = b2Body_GetLinearVelocity( boxId ).y;

		if ( reboundSpeed == 0.0f && before < 0.0f && speed > 0.0f )
		{
			impactSpeed = -before;
			reboundSpeed = speed;
			bounceHeight = heightBefore;
		}

		if ( reboundSpeed == 0.0f )
		{
			continue;
		}

		// Every step, not just at the apexes, because the impact step itself is where any gain appears
		float ratio = MeasureEnergy( boxId, worldId ) / startEnergy;
		if ( ratio > result.peakEnergyRatio )
		{
			result.peakEnergyRatio = ratio;
		}

		if ( previousSpeed > 0.0f && speed <= 0.0f )
		{
			if ( result.bounceCount == 0 )
			{
				result.firstApex = (float)b2Body_GetPosition( boxId ).y;
				result.firstEnergyRatio = ratio;
			}

			result.bounceCount += 1;
		}

		previousSpeed = speed;
	}

	b2DestroyWorld( worldId );

	result.speedRatio = reboundSpeed / impactSpeed;

	printf( "    overshoot continuous %d apex %.4f of %.4f at y %.4f, speed ratio %.4f, energy %.4f then %.4f\n",
			continuous ? 1 : 0, result.firstApex, dropHeight, bounceHeight, result.speedRatio, result.firstEnergyRatio,
			result.peakEnergyRatio );

	return result;
}

// Two runs, because two unrelated effects were tangled together here.
//
// The impulse must never return more speed than it received. That is the restitution invariant and it
// is checked in both runs; the unconverged multi point solve failed it.
//
// Everything else depends on whether continuous collision engages. Without it the box moves 0.23 m
// in the step before contact, just under the 0.25 m trigger, so the first manifold appears with the
// box already deep. It bounces from down there and climbs back through that depth for free, which is
// worth a fraction of a metre of apex and a little energy, and none of it is restitution. With
// continuous collision the box lands on the surface and both the apex and the energy come in under
// where they started. The safety factor here is the supported answer for a body whose elastic
// accuracy matters.
//
// The discrete run is only gated on its first bounce. Later bounces there are a genuine blow-up, not
// a tolerance question: the free height tilts the box and the next landing is a deep corner impact.
// Lowering the safety factor is the fix; the discrete run is kept to pin what happens when it is
// left alone, and because its speed ratio is what caught the real bug.
static int OvershootTest( void )
{
	const float dropHeight = 10.0f;

	OvershootResult discrete = MeasureOvershoot( false );
	OvershootResult continuous = MeasureOvershoot( true );

	int failed = 0;

	if ( discrete.speedRatio > 1.0f || continuous.speedRatio > 1.0f )
	{
		failed = 1;
	}

	if ( continuous.firstApex > dropHeight || continuous.peakEnergyRatio > 1.001f )
	{
		failed = 1;
	}

	if ( discrete.firstApex > dropHeight + 0.15f || discrete.firstEnergyRatio > 1.02f )
	{
		failed = 1;
	}

	if ( discrete.bounceCount < 2 || continuous.bounceCount < 2 )
	{
		failed = 1;
	}

	return failed;
}

typedef struct ImpulseResult
{
	float worstError;
	float approachSpeed;
	float firstSeparation;
	float bounceImpulse;
	int contactSteps;
	int toiSteps;
	int toiImpulseSteps;
} ImpulseResult;

// Ball dropped under gravity with the contact impulse read back every step. Nothing else touches the
// ball, so once gravity is taken out the change in momentum over a step is the impulse the contact
// applied, and that is what the total normal impulse must report. A time of impact step is left out
// of the balance: no contact was solved on it, and the sweep hands back the gravity of the time it
// cut short.
static ImpulseResult MeasureDropImpulse( float restitution, float dropHeight )
{
	b2WorldId worldId = MakeWorld( -10.0f );
	MakeGround( worldId, 0.0f );

	b2BodyId ballId = MakeBall( worldId, 0.0f, 0.5f + dropHeight, 0.0f, restitution );
	b2World* world = b2GetWorldFromId( worldId );

	float mass = b2Body_GetMass( ballId );
	float gravityY = b2World_GetGravity( worldId ).y;

	ImpulseResult result = { 0 };
	bool touched = false;
	bool bouncing = false;

	// Enough for the longest fall and the bounce that follows it
	for ( int i = 0; i < 240; ++i )
	{
		float speedBefore = b2Body_GetLinearVelocity( ballId ).y;
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		float speedAfter = b2Body_GetLinearVelocity( ballId ).y;

		float measured = 0.0f;
		float separation = 0.0f;
		b2ContactData contactData[4];
		int contactCount = b2Body_GetContactData( ballId, contactData, ARRAY_COUNT( contactData ) );
		for ( int c = 0; c < contactCount; ++c )
		{
			const b2Manifold* manifold = &contactData[c].manifold;
			for ( int p = 0; p < manifold->pointCount; ++p )
			{
				measured += manifold->points[p].totalNormalImpulse;
				separation = manifold->points[p].separation;
			}
		}

		// Transient body flags are cleared and rewritten every step
		b2Body* ball = b2GetBodyFullId( world, ballId );
		if ( ball->flags & b2_hadTimeOfImpact )
		{
			result.toiSteps += 1;
			if ( measured != 0.0f )
			{
				result.toiImpulseSteps += 1;
			}
			continue;
		}

		float expected = mass * ( speedAfter - speedBefore ) - mass * gravityY * TIME_STEP;
		result.worstError = b2MaxFloat( result.worstError, b2AbsFloat( measured - expected ) );

		if ( contactCount > 0 )
		{
			if ( touched == false )
			{
				touched = true;
				bouncing = true;
				result.approachSpeed = -speedBefore;
				result.firstSeparation = separation;
			}

			if ( bouncing )
			{
				result.bounceImpulse += measured;
				result.contactSteps += 1;
			}
		}
		else
		{
			bouncing = false;
		}
	}

	b2DestroyWorld( worldId );
	return result;
}

// Heights on both sides of the continuous collision threshold, so the impulse is checked for a ball
// that lands inside the overlap and for one the sweep sets down on the surface. The threshold is
// derived from the body so the split survives a change to the default safety factor.
//
// The step balance is the real gate. The bounce total is bracketed as well so the restitution sweep
// means something: the contact has to reverse the approach at the coefficient and may carry the
// weight for at most as long as it lasted. The stage runs after the sub-steps, so the reversal
// lands a step late and the ball can come in up to a step of gravity under it.
static int ImpulseTest( void )
{
	static const float restitutions[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
	static const float heights[] = { 1.0f, 5.0f, 20.0f, 45.0f };

	float mass;
	float fastSpeed;
	{
		b2WorldId worldId = MakeWorld( -10.0f );
		b2BodyId ballId = MakeBall( worldId, 0.0f, 0.5f, 0.0f, 0.0f );
		mass = b2Body_GetMass( ballId );
		fastSpeed = b2Body_GetSafetyFactor( ballId ) * b2Body_GetMinExtent( ballId ) / TIME_STEP;
		b2DestroyWorld( worldId );
	}

	const float weightImpulse = mass * 10.0f * TIME_STEP;

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( heights ); ++j )
	{
		float impactSpeed = sqrtf( 20.0f * heights[j] );
		bool expectToi = impactSpeed > fastSpeed;

		// Float noise in the solver velocities scales with the impact speed, and the ball is heavy
		float tolerance = 1e-5f * mass * ( 10.0f + impactSpeed );

		for ( int k = 0; k < ARRAY_COUNT( restitutions ); ++k )
		{
			ImpulseResult result = MeasureDropImpulse( restitutions[k], heights[j] );

			float reversal = ( 1.0f + restitutions[k] ) * mass * result.approachSpeed;
			float lower = reversal - weightImpulse;
			float upper = reversal + weightImpulse * result.contactSteps;

			printf( "    impulse drop %4.1f e %.2f toi %d at %+.4f -> worst step error %.1e, bounce %.1f in [%.1f, %.1f] over %d "
					"steps\n",
					heights[j], restitutions[k], result.toiSteps, result.firstSeparation, result.worstError, result.bounceImpulse,
					lower, upper, result.contactSteps );

			if ( result.worstError > tolerance )
			{
				failed = 1;
			}

			if ( result.contactSteps == 0 || result.toiImpulseSteps > 0 )
			{
				failed = 1;
			}

			float slack = result.contactSteps * tolerance;
			if ( result.bounceImpulse < lower - slack || result.bounceImpulse > upper + slack )
			{
				failed = 1;
			}

			if ( ( result.toiSteps > 0 ) != expectToi )
			{
				printf( "    continuous collision %s at %.1f m/s (threshold %.1f m/s)\n", expectToi ? "expected" : "unexpected",
						impactSpeed, fastSpeed );
				failed = 1;
			}
		}
	}

	return failed;
}

typedef struct EnergyScene
{
	const char* name;
	float safetyFactor;
	float friction;
	float angle;
	float spin;
	float height;
	float velocityY;
	float halfWidth;
	float halfHeight;
	bool bullet;
	bool stack;
	int stepCount;
} EnergyScene;

// Peak energy over the run relative to the start, for every dynamic body in the scene. Perfectly
// elastic and the energy may still only go down: friction and the soft contact position correction
// dissipate, and a bounce may return at most what the contact absorbed.
static float MeasurePeakEnergy( const EnergyScene* scene )
{
	b2WorldId worldId = MakeWorld( -10.0f );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShape = b2DefaultShapeDef();
	groundShape.material.friction = scene->friction;
	b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &groundShape, &segment );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = scene->friction;
	shapeDef.material.restitution = 1.0f;
	b2Polygon box = b2MakeBox( scene->halfWidth, scene->halfHeight );

	b2BodyId bodyIds[4];
	int bodyCount = 0;

	if ( scene->stack )
	{
		// Resting box under the dropped one, like the TwoBoxRestitution sample
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0f, scene->halfHeight };
		bodyDef.safetyFactor = scene->safetyFactor;
		bodyIds[bodyCount] = b2CreateBody( worldId, &bodyDef );
		b2CreatePolygonShape( bodyIds[bodyCount], &shapeDef, &box );
		bodyCount += 1;
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0f, scene->height };
		bodyDef.rotation = b2MakeRot( scene->angle );
		bodyDef.angularVelocity = scene->spin;
		bodyDef.linearVelocity = (b2Vec2){ 0.0f, scene->velocityY };
		bodyDef.safetyFactor = scene->safetyFactor;
		bodyDef.isBullet = scene->bullet;
		bodyIds[bodyCount] = b2CreateBody( worldId, &bodyDef );
		b2CreatePolygonShape( bodyIds[bodyCount], &shapeDef, &box );
		bodyCount += 1;
	}

	float start = 0.0f;
	for ( int k = 0; k < bodyCount; ++k )
	{
		start += MeasureEnergy( bodyIds[k], worldId );
	}

	float peak = start;
	int peakStep = -1;

	for ( int i = 0; i < scene->stepCount; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		float energy = 0.0f;
		for ( int k = 0; k < bodyCount; ++k )
		{
			energy += MeasureEnergy( bodyIds[k], worldId );
		}

		if ( energy > peak )
		{
			peak = energy;
			peakStep = i;
		}
	}

	b2DestroyWorld( worldId );

	float ratio = peak / start;
	printf( "    energy %-24s peak %.5f at step %d\n", scene->name, ratio, peakStep );
	return ratio;
}

// Perfectly elastic material with friction, dropped every way that used to gain energy: a spinning
// corner landing, tumbling, a flat landing with spin, a box falling on a resting box, and a tall box
// landing flat on its narrow end. Energy must never rise above the start. This is the load bearing
// property of restitution. Accuracy of the coefficient and freedom from spin are secondary.
//
// The tall boxes guard the stage iteration count. Their contact points sit close together while the
// inertia is long, so the off diagonal of the two point normal mass is large and one sweep
// overshoots: the narrowest gains 1.7% frictionless and 1.0% with friction at a single sweep and is
// flat from two on.
static int EnergyTest( void )
{
	static const EnergyScene scenes[] = {
		{ "spinning corner", 0.01f, 0.6f, -1.43f, 7.9f, 0.7f, -1.9f, 0.5f, 0.5f, false, false, 300 },
		{ "spinning corner fast", 0.01f, 0.6f, -0.8f, 12.0f, 3.0f, -8.0f, 0.5f, 0.5f, false, false, 300 },
		{ "tumble", 0.01f, 0.6f, -0.3f, 3.0f, 8.0f, 0.0f, 0.5f, 0.5f, false, false, 600 },
		{ "tumble discrete", 0.5f, 0.6f, -0.3f, 3.0f, 8.0f, 0.0f, 0.5f, 0.5f, false, false, 600 },
		{ "flat with spin", 0.01f, 0.6f, 0.0f, 2.0f, 2.0f, -5.0f, 0.5f, 0.5f, false, false, 300 },
		{ "flat frictionless", 0.01f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.5f, 0.5f, false, false, 1200 },
		{ "corner", 0.01f, 0.6f, -0.2f, 0.0f, 5.0f, 0.0f, 0.5f, 0.5f, false, false, 600 },
		{ "two box", 0.02f, 0.6f, 0.0f, 0.0f, 4.0f, 0.0f, 0.5f, 0.5f, true, true, 1500 },
		{ "two box discrete", 0.5f, 0.6f, 0.0f, 0.0f, 4.0f, 0.0f, 0.5f, 0.5f, false, true, 1500 },
		{ "tall flat frictionless", 0.01f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.5f, 1.0f, false, false, 1200 },
		{ "tall flat friction", 0.01f, 0.6f, 0.0f, 0.0f, 10.0f, 0.0f, 0.5f, 1.0f, false, false, 1200 },
		{ "narrow flat frictionless", 0.01f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.25f, 1.0f, false, false, 1200 },
		{ "narrow flat friction", 0.01f, 0.6f, 0.0f, 0.0f, 10.0f, 0.0f, 0.25f, 1.0f, false, false, 1200 },
	};

	// The soft contact position correction lifts a body that lands inside the ground, which is
	// potential energy from nowhere at any restitution. Continuous collision keeps landings on the
	// surface, so the tolerance is round-off. The discrete scenes land deep and are allowed the lift.
	const float tolerance = 1.0e-3f;
	const float discreteTolerance = 2.0e-2f;

	int failed = 0;

	for ( int i = 0; i < ARRAY_COUNT( scenes ); ++i )
	{
		float ratio = MeasurePeakEnergy( scenes + i );
		float limit = scenes[i].safetyFactor < 0.1f ? tolerance : discreteTolerance;
		if ( ratio > 1.0f + limit )
		{
			failed = 1;
		}
	}

	return failed;
}

static uint64_t RunWorkerScene( int workerCount )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	worldDef.enableSleep = false;
	worldDef.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	MakeGround( worldId, 0.6f );

	for ( int i = 0; i < 20; ++i )
	{
		MakeBall( worldId, -10.0f + 1.05f * i, 3.0f + 0.13f * i, 0.0f, 0.6f );
	}

	for ( int i = 0; i < 200; ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	uint64_t hash = b2World_GetStateHash( worldId );
	b2DestroyWorld( worldId );
	return hash;
}

// The sampled approach speed and the Poisson accumulator are per manifold point state, so they must
// survive the split across workers
static int WorkerParityTest( void )
{
	uint64_t hash0 = RunWorkerScene( 0 );
	uint64_t hash1 = RunWorkerScene( 1 );
	uint64_t hash4 = RunWorkerScene( 4 );

	printf( "    worker hashes 0x%016llx 0x%016llx 0x%016llx\n", (unsigned long long)hash0, (unsigned long long)hash1,
			(unsigned long long)hash4 );

	ENSURE( hash0 == hash1 );
	ENSURE( hash0 == hash4 );
	return 0;
}

int RestitutionTest( void )
{
	int failureCount = 0;

	RUN_MEASUREMENT( HeadOnTest );
	RUN_MEASUREMENT( PhaseTest );
	RUN_MEASUREMENT( SupportedTest );
	RUN_MEASUREMENT( TwoPointTest );
	RUN_MEASUREMENT( SpinTest );
	RUN_MEASUREMENT( DropTest );
	RUN_MEASUREMENT( SingleBoxTest );
	RUN_MEASUREMENT( ThresholdTest );
	RUN_MEASUREMENT( RestingTest );
	RUN_MEASUREMENT( NormalVelocityTest );
	RUN_MEASUREMENT( OvershootTest );
	RUN_MEASUREMENT( CornerTest );
	RUN_MEASUREMENT( EnergyTest );
	RUN_MEASUREMENT( ImpulseTest );
	RUN_MEASUREMENT( WorkerParityTest );

	return failureCount > 0 ? 1 : 0;
}
