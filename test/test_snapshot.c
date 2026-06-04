// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world_snapshot.h"

#include "physics_world.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <stdio.h>
#include <string.h>

// Build a scene that exercises every heap-bearing container:
// - ground (static box)
// - main stack of dynamic boxes (settling stack)
// - several joint types (revolute, prismatic, distance, weld)
// - one chain shape
// - one sensor shape overlapping a moving body
// - isolated second stack let to sleep before the snapshot
//
// Returns the worldId and, via out-params, a body that stays dynamic so the
// sensor has something to overlap with.
static b2WorldId BuildScene( int workerCount )
{
	b2WorldDef def = b2DefaultWorldDef();
	def.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &def );

	// Ground
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.position = (b2Vec2){ 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bd );

		b2Polygon groundBox = b2MakeBox( 40.0f, 1.0f );
		b2ShapeDef sd = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &sd, &groundBox );
	}

	// Main stack: 8 dynamic boxes
	b2BodyId stackTop = b2_nullBodyId;
	{
		b2ShapeDef sd = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		for ( int i = 0; i < 8; ++i )
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = (b2Vec2){ 0.0f, 0.5f + (float)i * 1.1f };
			b2BodyId bodyId = b2CreateBody( worldId, &bd );
			b2CreatePolygonShape( bodyId, &sd, &box );
			stackTop = bodyId;
		}
	}

	// Joint bodies: two dynamic bodies, one for each pair of joints
	b2BodyDef jbDef = b2DefaultBodyDef();
	jbDef.type = b2_dynamicBody;
	jbDef.position = (b2Vec2){ 5.0f, 2.0f };
	b2BodyId jbA = b2CreateBody( worldId, &jbDef );
	jbDef.position = (b2Vec2){ 7.0f, 2.0f };
	b2BodyId jbB = b2CreateBody( worldId, &jbDef );
	jbDef.position = (b2Vec2){ 9.0f, 2.0f };
	b2BodyId jbC = b2CreateBody( worldId, &jbDef );
	jbDef.position = (b2Vec2){ 11.0f, 2.0f };
	b2BodyId jbD = b2CreateBody( worldId, &jbDef );

	b2Polygon jbox = b2MakeBox( 0.3f, 0.3f );
	b2ShapeDef jsd = b2DefaultShapeDef();
	b2CreatePolygonShape( jbA, &jsd, &jbox );
	b2CreatePolygonShape( jbB, &jsd, &jbox );
	b2CreatePolygonShape( jbC, &jsd, &jbox );
	b2CreatePolygonShape( jbD, &jsd, &jbox );

	// Revolute joint (mirrors determinism.c idiom)
	{
		b2RevoluteJointDef rd = b2DefaultRevoluteJointDef();
		rd.enableLimit = true;
		rd.lowerAngle = -0.1f * B2_PI;
		rd.upperAngle = 0.2f * B2_PI;
		rd.enableSpring = true;
		rd.hertz = 1.0f;
		rd.dampingRatio = 1.0f;
		rd.enableMotor = true;
		rd.maxMotorTorque = 0.5f;
		rd.base.bodyIdA = jbA;
		rd.base.bodyIdB = jbB;
		rd.base.localFrameA.p = (b2Vec2){ 0.3f, 0.0f };
		rd.base.localFrameB.p = (b2Vec2){ -0.3f, 0.0f };
		b2CreateRevoluteJoint( worldId, &rd );
	}

	// Prismatic joint
	{
		b2PrismaticJointDef pd = b2DefaultPrismaticJointDef();
		pd.enableLimit = true;
		pd.lowerTranslation = -0.5f;
		pd.upperTranslation = 0.5f;
		pd.base.bodyIdA = jbB;
		pd.base.bodyIdB = jbC;
		pd.base.localFrameA.p = (b2Vec2){ 0.3f, 0.0f };
		pd.base.localFrameB.p = (b2Vec2){ -0.3f, 0.0f };
		b2CreatePrismaticJoint( worldId, &pd );
	}

	// Distance joint
	{
		b2DistanceJointDef dd = b2DefaultDistanceJointDef();
		dd.length = 2.0f;
		dd.base.bodyIdA = jbC;
		dd.base.bodyIdB = jbD;
		dd.base.localFrameA.p = (b2Vec2){ 0.3f, 0.0f };
		dd.base.localFrameB.p = (b2Vec2){ -0.3f, 0.0f };
		b2CreateDistanceJoint( worldId, &dd );
	}

	// Weld joint
	{
		b2WeldJointDef wd = b2DefaultWeldJointDef();
		wd.linearHertz = 5.0f;
		wd.linearDampingRatio = 0.7f;
		wd.base.bodyIdA = jbD;
		wd.base.bodyIdB = stackTop;
		wd.base.localFrameA.p = (b2Vec2){ 0.3f, 0.0f };
		wd.base.localFrameB.p = (b2Vec2){ 0.0f, 0.0f };
		b2CreateWeldJoint( worldId, &wd );
	}

	// Chain shape on a static body
	{
		b2BodyDef cbd = b2DefaultBodyDef();
		cbd.position = (b2Vec2){ -10.0f, 0.0f };
		b2BodyId chainBodyId = b2CreateBody( worldId, &cbd );

		b2Vec2 chainPoints[5] = {
			{ -4.0f, 0.0f }, { -2.0f, 0.0f }, { 0.0f, 0.0f }, { 2.0f, 0.0f }, { 4.0f, 2.0f }
		};
		b2SurfaceMaterial chainMat = b2DefaultSurfaceMaterial();
		chainMat.friction = 0.4f;
		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = chainPoints;
		chainDef.count = 5;
		chainDef.materials = &chainMat;
		chainDef.materialCount = 1;
		chainDef.isLoop = false;
		b2ChainId chainId = b2CreateChain( chainBodyId, &chainDef );
		(void)chainId;
	}

	// Sensor on a static body, overlapping the scene area
	{
		b2BodyDef sbd = b2DefaultBodyDef();
		sbd.position = (b2Vec2){ 0.0f, 5.0f };
		b2BodyId sensorBodyId = b2CreateBody( worldId, &sbd );

		b2Polygon sensorBox = b2MakeBox( 3.0f, 3.0f );
		b2ShapeDef sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		b2CreatePolygonShape( sensorBodyId, &sensorDef, &sensorBox );
	}

	// Isolated second stack far from the main scene — will go to sleep independently
	{
		b2ShapeDef sd = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		for ( int i = 0; i < 6; ++i )
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = (b2Vec2){ 40.0f, 0.5f + (float)i * 1.1f };
			b2BodyId bodyId = b2CreateBody( worldId, &bd );
			b2CreatePolygonShape( bodyId, &sd, &box );
		}
	}

	return worldId;
}

// Step until both the main scene and the isolated stack have settled to sleep.
// Returns the step count used. Mirrors the settle-detect idiom from determinism.c.
static int StepUntilSleep( b2WorldId worldId )
{
	float dt = 1.0f / 60.0f;
	int subSteps = 4;
	int maxSteps = 500;

	for ( int step = 0; step < maxSteps; ++step )
	{
		b2World_Step( worldId, dt, subSteps );

		int awake = b2World_GetAwakeBodyCount( worldId );
		if ( awake == 0 )
		{
			return step + 1;
		}
	}
	return maxSteps;
}

int SnapshotTest( void )
{
	// Phase 1: build and settle worldA
	b2WorldId worldAId = BuildScene( 1 );
	StepUntilSleep( worldAId );

	b2World* worldA = b2GetWorldFromId( worldAId );

	// The sleeping-set path (sets beyond the initial 3) must be exercised
	ENSURE( worldA->solverSets.count > 3 );

	// Serialize worldA
	b2RecBuffer buf;
	buf.data = NULL;
	buf.capacity = 0;
	buf.size = 0;
	b2SerializeWorld( worldA, &buf );
	ENSURE( buf.size > 0 );

	// Phase 2: deserialize into worldB (same worker count)
	b2WorldId worldBId = b2DeserializeWorld( buf.data, buf.size, 1 );
	ENSURE( b2World_IsValid( worldBId ) );

	b2World* worldB = b2GetWorldFromId( worldBId );

	// Immediate hash check — A and B must be identical
	uint64_t hashA0 = b2HashWorldState( worldA );
	uint64_t hashB0 = b2HashWorldState( worldB );
	ENSURE( hashA0 == hashB0 );

	uint64_t deepA0 = b2HashWorldStateDeep( worldA );
	uint64_t deepB0 = b2HashWorldStateDeep( worldB );
	ENSURE( deepA0 == deepB0 );

	// Phase 3: lockstep worldA vs worldB for 120 steps, assert both hashes match each step
	float dt = 1.0f / 60.0f;
	int subSteps = 4;
	for ( int step = 0; step < 120; ++step )
	{
		b2World_Step( worldAId, dt, subSteps );
		b2World_Step( worldBId, dt, subSteps );

		uint64_t sA = b2HashWorldState( worldA );
		uint64_t sB = b2HashWorldState( worldB );
		if ( sA != sB )
		{
			printf( "shallow hash mismatch at lockstep step %d (A=%llu B=%llu)\n", step,
					(unsigned long long)sA, (unsigned long long)sB );
			ENSURE( false );
		}

		uint64_t dA = b2HashWorldStateDeep( worldA );
		uint64_t dB = b2HashWorldStateDeep( worldB );
		if ( dA != dB )
		{
			printf( "deep hash mismatch at lockstep step %d (A=%llu B=%llu)\n", step,
					(unsigned long long)dA, (unsigned long long)dB );
			ENSURE( false );
		}
	}

	// Phase 4: restore is worker-count independent. Snapshot worldA at its current state,
	// rebuild it at one and four workers from the same bytes, then lockstep the two.
	b2RecBufFree( &buf );
	buf = (b2RecBuffer){ 0 };
	b2SerializeWorld( worldA, &buf );

	b2WorldId worldA1Id = b2DeserializeWorld( buf.data, buf.size, 1 );
	ENSURE( b2World_IsValid( worldA1Id ) );
	b2World* worldA1 = b2GetWorldFromId( worldA1Id );

	b2WorldId worldCId = b2DeserializeWorld( buf.data, buf.size, 4 );
	ENSURE( b2World_IsValid( worldCId ) );
	b2World* worldC = b2GetWorldFromId( worldCId );

	ENSURE( b2HashWorldState( worldA1 ) == b2HashWorldState( worldC ) );

	for ( int step = 0; step < 120; ++step )
	{
		b2World_Step( worldA1Id, dt, subSteps );
		b2World_Step( worldCId, dt, subSteps );

		uint64_t sA1 = b2HashWorldState( worldA1 );
		uint64_t sC = b2HashWorldState( worldC );
		if ( sA1 != sC )
		{
			printf( "one vs four worker hash mismatch at step %d (1w=%llu 4w=%llu)\n", step,
					(unsigned long long)sA1, (unsigned long long)sC );
			ENSURE( false );
		}
	}

	// Clean up
	b2DestroyWorld( worldAId );
	b2DestroyWorld( worldBId );
	b2DestroyWorld( worldA1Id );
	b2DestroyWorld( worldCId );

	b2RecBufFree( &buf );

	return 0;
}
