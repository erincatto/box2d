// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <stdio.h>

#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEP_COUNT 4

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
static float MeasureSupportedBounce( float restitution, int supportCount )
{
	b2WorldId worldId = MakeWorld( 0.0f );
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
	b2WorldId worldId = MakeWorld( 0.0f );
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

// The bounce is solved as a constraint alongside the support contacts, so the column can supply the
// reaction. A terminal restitution pass has nothing after it to do that and measured about 0.6 for
// e = 0.9, which the tolerance is chosen to reject.
static int SupportedTest( void )
{
	static const float restitutions[] = { 0.5f, 0.9f };
	const float tolerance = 0.2f;

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
	{
		for ( int n = 0; n <= 3; ++n )
		{
			float measured = MeasureSupportedBounce( restitutions[j], n );
			printf( "    supported e %.2f supports %d -> %.4f\n", restitutions[j], n, measured );

			if ( b2AbsFloat( measured - restitutions[j] ) > tolerance )
			{
				failed = 1;
			}

			// Rebounding faster than the impact is energy from nowhere, whatever the coefficient
			if ( measured > 1.01f )
			{
				failed = 1;
			}
		}
	}

	return failed;
}

// A symmetric two point landing. The two points are solved in sequence within a relax pass and the
// bounce retires once both points separate, so a small residual spin is expected. The tolerance
// admits that residual and rejects the gross asymmetry of one point taking the whole bounce.
static int TwoPointTest( void )
{
	static const float restitutions[] = { 0.5f, 0.9f };
	static const int subStepCounts[] = { 4, 8 };
	const float speedTolerance = 0.1f;
	const float spinTolerance = 0.25f;

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
// velocity, so perfect restitution reverses linear and angular velocity. The single relax pass
// leaves a residual well inside the tolerance.
static int SpinTest( void )
{
	static const float spins[] = { 0.0f, 0.5f, 1.0f, 2.0f };

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( spins ); ++j )
	{
		b2WorldId worldId = MakeWorld( 0.0f );
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

		if ( b2AbsFloat( speed - IMPACT_SPEED ) > 0.25f || b2AbsFloat( spin + spins[j] ) > 0.25f )
		{
			failed = 1;
		}
	}

	return failed;
}

// Perfectly elastic ball under gravity. Only heights where continuous collision engages are used.
// Continuous collision lands the ball on the surface, so the bounce is armed from the true impact
// speed and the apex holds. Slower drops resolve the impact inside the overlap and the penetration
// recovery adds height, by design. See the restitution notes in the contact solver prepare stage.
static int DropTest( void )
{
	static const float heights[] = { 40.0f, 20.0f };

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

		if ( highest > 1.02f * apexes[0] || apexes[apexCount - 1] < 0.8f * apexes[0] )
		{
			failed = 1;
		}
	}

	return failed;
}

// Mirrors the SingleRestitution sample: square box, perfectly elastic, dropped flat onto a segment
// with an aggressive continuous safety factor so it lands square on the surface. Only the first
// bounce is a gate. The residual spin from that landing tilts the box for the next one, and a corner
// first landing under the per point impact law sheds energy into rotation, so later apexes fall off.
static int SingleBoxTest( void )
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
	int apexCount = 0;
	float previousSpeed = 0.0f;

	// A nearly elastic bounce from 10 m takes about 165 steps, so budget for four of them
	for ( int i = 0; i < 1200 && apexCount < ARRAY_COUNT( apexes ); ++i )
	{
		b2World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		float speed = b2Body_GetLinearVelocity( boxId ).y;

		if ( apexCount == 0 && previousSpeed <= 0.0f && speed > 0.0f )
		{
			firstSpin = b2Body_GetAngularVelocity( boxId );
		}

		if ( previousSpeed > 0.0f && speed <= 0.0f )
		{
			apexes[apexCount] = (float)b2Body_GetPosition( boxId ).y;
			apexCount += 1;
		}
		previousSpeed = speed;
	}

	b2DestroyWorld( worldId );

	printf( "    single box first bounce spin %+.4f apexes", firstSpin );
	for ( int i = 0; i < apexCount; ++i )
	{
		printf( " %7.3f", apexes[i] );
	}
	printf( "\n" );

	ENSURE( apexCount == ARRAY_COUNT( apexes ) );

	int failed = 0;

	if ( apexes[0] < 0.97f * dropHeight || apexes[0] > 1.02f * dropHeight )
	{
		failed = 1;
	}

	if ( b2AbsFloat( firstSpin ) > 0.25f )
	{
		failed = 1;
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

// The armed bounce is per manifold point state, so it must survive the split across workers
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
	RUN_MEASUREMENT( WorkerParityTest );

	return failureCount > 0 ? 1 : 0;
}
