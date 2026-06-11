// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "test_macros.h"
#include "world_snapshot.h"

#include "core.h"
#include "physics_world.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* s_snapPath = "test_snapshot_midstream.b2rec";

// Ids held across a snapshot to prove they keep resolving after an in-place restore
typedef struct SnapshotIds
{
	b2BodyId body;
	b2ShapeId shape;
	b2JointId joint;
	b2ChainId chain;
} SnapshotIds;

// Build a scene that exercises every heap-bearing container:
// - ground (static box)
// - main stack of dynamic boxes (settling stack)
// - several joint types (revolute, prismatic, distance, weld)
// - one chain shape
// - one sensor shape overlapping a moving body
// - isolated second stack let to sleep before the snapshot
//
// Returns the worldId. When outIds is non-NULL it also returns one id of each kind
// (body, shape, joint, chain) so a caller can check they survive an in-place restore.
static b2WorldId BuildScene( int workerCount, SnapshotIds* outIds )
{
	b2JointId heldJoint = b2_nullJointId;
	b2ChainId heldChain = b2_nullChainId;

	b2WorldDef def = b2DefaultWorldDef();
	def.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &def );

	// Ground
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.position = b2MakePosition( (b2Vec2){ 0.0f, -1.0f } );
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
			bd.position = b2MakePosition( (b2Vec2){ 0.0f, 0.5f + (float)i * 1.1f } );
			b2BodyId bodyId = b2CreateBody( worldId, &bd );
			b2CreatePolygonShape( bodyId, &sd, &box );
			stackTop = bodyId;
		}
	}

	// Joint bodies: two dynamic bodies, one for each pair of joints
	b2BodyDef jbDef = b2DefaultBodyDef();
	jbDef.type = b2_dynamicBody;
	jbDef.position = b2MakePosition( (b2Vec2){ 5.0f, 2.0f } );
	b2BodyId jbA = b2CreateBody( worldId, &jbDef );
	jbDef.position = b2MakePosition( (b2Vec2){ 7.0f, 2.0f } );
	b2BodyId jbB = b2CreateBody( worldId, &jbDef );
	jbDef.position = b2MakePosition( (b2Vec2){ 9.0f, 2.0f } );
	b2BodyId jbC = b2CreateBody( worldId, &jbDef );
	jbDef.position = b2MakePosition( (b2Vec2){ 11.0f, 2.0f } );
	b2BodyId jbD = b2CreateBody( worldId, &jbDef );

	b2Polygon jbox = b2MakeBox( 0.3f, 0.3f );
	b2ShapeDef jsd = b2DefaultShapeDef();
	b2ShapeId heldShape = b2CreatePolygonShape( jbA, &jsd, &jbox );
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
		heldJoint = b2CreateRevoluteJoint( worldId, &rd );
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
		cbd.position = b2MakePosition( (b2Vec2){ -10.0f, 0.0f } );
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
		heldChain = b2CreateChain( chainBodyId, &chainDef );
	}

	// Sensor on a static body, overlapping the scene area
	{
		b2BodyDef sbd = b2DefaultBodyDef();
		sbd.position = b2MakePosition( (b2Vec2){ 0.0f, 5.0f } );
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
			bd.position = b2MakePosition( (b2Vec2){ 40.0f, 0.5f + (float)i * 1.1f } );
			b2BodyId bodyId = b2CreateBody( worldId, &bd );
			b2CreatePolygonShape( bodyId, &sd, &box );
		}
	}

	if ( outIds != NULL )
	{
		outIds->body = stackTop;
		outIds->shape = heldShape;
		outIds->joint = heldJoint;
		outIds->chain = heldChain;
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
	b2WorldId worldAId = BuildScene( 1, NULL );
	StepUntilSleep( worldAId );

	b2World* worldA = b2GetWorldFromId( worldAId );

	// The sleeping-set path (sets beyond the initial 3) must be exercised
	ENSURE( worldA->solverSets.count > 3 );

	// Serialize worldA
	b2RecBuffer buf = { 0 };
	b2SerializeWorld( worldA, &buf );
	ENSURE( buf.size > 0 );

	// Phase 2: deserialize into worldB (same worker count)
	b2WorldId worldBId = b2CreateWorldFromSnapshot( buf.data, buf.size, 1 );
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

	b2WorldId worldA1Id = b2CreateWorldFromSnapshot( buf.data, buf.size, 1 );
	ENSURE( b2World_IsValid( worldA1Id ) );
	b2World* worldA1 = b2GetWorldFromId( worldA1Id );

	b2WorldId worldCId = b2CreateWorldFromSnapshot( buf.data, buf.size, 4 );
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

	// Phase 5: in-place restore keeps held ids working and rolls the world back exactly
	SnapshotIds ids;
	b2WorldId rId = BuildScene( 1, &ids );
	StepUntilSleep( rId );
	b2World* rWorld = b2GetWorldFromId( rId );

	// Producer: size query, then fill a caller-owned buffer
	int imageSize = b2World_Snapshot( rId, NULL, 0 );
	ENSURE( imageSize > 0 );
	uint8_t* image = b2Alloc( imageSize );
	int written = b2World_Snapshot( rId, image, imageSize );
	ENSURE( written == imageSize );

	uint64_t snapHash = b2HashWorldStateDeep( rWorld );

	// Diverge from the snapshot: push a held body and add a body that did not exist at the snapshot
	b2Body_SetLinearVelocity( ids.body, (b2Vec2){ 3.0f, 6.0f } );
	b2Body_SetAwake( ids.body, true );
	b2BodyDef postDef = b2DefaultBodyDef();
	postDef.type = b2_dynamicBody;
	postDef.position = b2MakePosition( (b2Vec2){ 20.0f, 20.0f } );
	b2BodyId postBody = b2CreateBody( rId, &postDef );
	for ( int step = 0; step < 30; ++step )
	{
		b2World_Step( rId, dt, subSteps );
	}
	ENSURE( b2HashWorldStateDeep( rWorld ) != snapHash );

	ENSURE( b2World_Restore( rId, image, imageSize ) );

	// Whole-world state rolled back to the snapshot instant
	ENSURE( b2HashWorldStateDeep( rWorld ) == snapHash );

	// The world id and every id held at the snapshot instant resolve again
	ENSURE( b2World_IsValid( rId ) );
	ENSURE( b2Body_IsValid( ids.body ) );
	ENSURE( b2Shape_IsValid( ids.shape ) );
	ENSURE( b2Joint_IsValid( ids.joint ) );
	ENSURE( b2Chain_IsValid( ids.chain ) );

	// An id minted after the snapshot is rejected, not aliased onto a different object
	ENSURE( b2Body_IsValid( postBody ) == false );

	// Phase 6: a rejected image leaves the world untouched
	uint64_t preBadHash = b2HashWorldStateDeep( rWorld );
	ENSURE( b2World_Restore( rId, NULL, 0 ) == false );
	uint8_t* corrupt = b2Alloc( imageSize );
	memcpy( corrupt, image, imageSize );
	corrupt[0] ^= 0xFF; // break the magic
	ENSURE( b2World_Restore( rId, corrupt, imageSize ) == false );
	ENSURE( b2HashWorldStateDeep( rWorld ) == preBadHash );
	b2Free( corrupt, imageSize );

	// Repeated in-place restore over the chain/sensor/island heap must not leak
	for ( int i = 0; i < 3; ++i )
	{
		ENSURE( b2World_Restore( rId, image, imageSize ) );
	}
	ENSURE( b2HashWorldStateDeep( rWorld ) == snapHash );

	// Phase 7: restore is worker-count independent. Restore the one-worker image into a
	// four-worker world and lockstep it against the same image loaded fresh at one worker.
	b2WorldDef def4 = b2DefaultWorldDef();
	def4.workerCount = 4;
	b2WorldId sId = b2CreateWorld( &def4 );
	ENSURE( b2World_Restore( sId, image, imageSize ) );
	b2World* sWorld = b2GetWorldFromId( sId );

	b2WorldId freshId = b2CreateWorldFromSnapshot( image, imageSize, 1 );
	ENSURE( b2World_IsValid( freshId ) );
	b2World* freshWorld = b2GetWorldFromId( freshId );

	ENSURE( b2HashWorldState( sWorld ) == b2HashWorldState( freshWorld ) );
	for ( int step = 0; step < 120; ++step )
	{
		b2World_Step( sId, dt, subSteps );
		b2World_Step( freshId, dt, subSteps );
		if ( b2HashWorldState( sWorld ) != b2HashWorldState( freshWorld ) )
		{
			printf( "in-place vs fresh hash mismatch at step %d\n", step );
			ENSURE( false );
		}
	}

	b2Free( image, imageSize );

	// Phase 8: mid-stream recording into a file, then deterministic replay. b2World_StartRecording
	// writes a snapshot of the live world, the file then continues with the hook log. Replay passing
	// proves the deserialized snapshot reproduced the live world for every recorded step.
	{
		b2WorldId wId = BuildScene( 1, NULL );

		// Snapshot mid-motion so the recorded tail exercises moving bodies, not a settled world
		for ( int step = 0; step < 20; ++step )
		{
			b2World_Step( wId, dt, subSteps );
		}

		b2Recording* rec = b2CreateRecording( 0 );
		b2World_StartRecording( wId, rec );
		for ( int step = 0; step < 60; ++step )
		{
			b2World_Step( wId, dt, subSteps );
		}
		b2World_StopRecording( wId );
		b2DestroyWorld( wId );

		const uint8_t* recData = b2Recording_GetData( rec );
		int recSize = b2Recording_GetSize( rec );
		ENSURE( b2ValidateReplay( recData, recSize, 0 ) );
		ENSURE( b2ValidateReplay( recData, recSize, 4 ) );

		// File round-trip: save the buffer, load it back, and replay the loaded copy
		ENSURE( b2SaveRecordingToFile( rec, s_snapPath ) );
		b2Recording* loaded = b2LoadRecordingFromFile( s_snapPath );
		ENSURE( loaded != NULL );
		ENSURE( b2ValidateReplay( b2Recording_GetData( loaded ), b2Recording_GetSize( loaded ), 0 ) );
		b2DestroyRecording( loaded );

		// The player opens the recording and the replay world id is stable across a restart
		b2RecPlayer* player = b2RecPlayer_Create( recData, recSize, 0 );
		ENSURE( player != NULL );
		b2WorldId pid0 = b2RecPlayer_GetWorldId( player );

		int frames = 0;
		while ( b2RecPlayer_StepFrame( player ) )
		{
			frames += 1;
		}
		ENSURE( frames == 60 );
		ENSURE( b2RecPlayer_HasDiverged( player ) == false );

		b2RecPlayer_Restart( player );
		b2WorldId pid1 = b2RecPlayer_GetWorldId( player );
		ENSURE( pid0.index1 == pid1.index1 && pid0.generation == pid1.generation );
		ENSURE( b2RecPlayer_GetFrame( player ) == 0 );

		int frames2 = 0;
		while ( b2RecPlayer_StepFrame( player ) )
		{
			frames2 += 1;
		}
		ENSURE( frames2 == 60 );
		ENSURE( b2RecPlayer_HasDiverged( player ) == false );

		b2RecPlayer_Destroy( player );
		b2DestroyRecording( rec );
	}

	// Phase 9: snapshot-equals-real. A world built from a mid-stream snapshot must reproduce the
	// origin world's future step for step, not merely be internally self-consistent.
	{
		b2WorldId wId = BuildScene( 1, NULL );

		// Snapshot mid-motion so the compared tail has real dynamics
		for ( int step = 0; step < 20; ++step )
		{
			b2World_Step( wId, dt, subSteps );
		}

		int snapSize = b2World_Snapshot( wId, NULL, 0 );
		uint8_t* snap = b2Alloc( snapSize );
		b2World_Snapshot( wId, snap, snapSize );

		b2World* origin = b2GetWorldFromId( wId );

		enum
		{
			tailSteps = 90
		};
		uint64_t realTail[tailSteps];
		for ( int step = 0; step < tailSteps; ++step )
		{
			b2World_Step( wId, dt, subSteps );
			realTail[step] = b2HashWorldState( origin );
		}

		b2WorldId cId = b2CreateWorldFromSnapshot( snap, snapSize, 1 );
		ENSURE( b2World_IsValid( cId ) );
		b2World* clone = b2GetWorldFromId( cId );
		for ( int step = 0; step < tailSteps; ++step )
		{
			b2World_Step( cId, dt, subSteps );
			if ( realTail[step] != b2HashWorldState( clone ) )
			{
				printf( "snapshot-equals-real mismatch at tail step %d\n", step );
				ENSURE( false );
			}
		}

		b2Free( snap, snapSize );
		b2DestroyWorld( wId );
		b2DestroyWorld( cId );
	}

	// Phase 10: a recording started before the first step also restarts in place, with a stable
	// replay world id
	{
		b2WorldDef wd = b2DefaultWorldDef();
		wd.workerCount = 1;
		b2WorldId wId = b2CreateWorld( &wd );

		b2Recording* rec = b2CreateRecording( 0 );
		b2World_StartRecording( wId, rec );

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = b2MakePosition( (b2Vec2){ 0.0f, 8.0f } );
		b2BodyId fallingBody = b2CreateBody( wId, &bd );
		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		b2ShapeDef sdef = b2DefaultShapeDef();
		b2CreatePolygonShape( fallingBody, &sdef, &box );

		for ( int step = 0; step < 30; ++step )
		{
			b2World_Step( wId, dt, subSteps );
		}
		b2World_StopRecording( wId );
		b2DestroyWorld( wId );

		b2RecPlayer* player = b2RecPlayer_Create( b2Recording_GetData( rec ), b2Recording_GetSize( rec ), 0 );
		ENSURE( player != NULL );
		b2WorldId pid0 = b2RecPlayer_GetWorldId( player );
		for ( int i = 0; i < 5; ++i )
		{
			b2RecPlayer_StepFrame( player );
		}
		b2RecPlayer_Restart( player );
		b2WorldId pid1 = b2RecPlayer_GetWorldId( player );
		ENSURE( pid0.index1 == pid1.index1 && pid0.generation == pid1.generation );
		ENSURE( b2RecPlayer_GetFrame( player ) == 0 );
		b2RecPlayer_Destroy( player );
		b2DestroyRecording( rec );
	}

	remove( s_snapPath );

	// Clean up
	b2DestroyWorld( worldAId );
	b2DestroyWorld( worldBId );
	b2DestroyWorld( worldA1Id );
	b2DestroyWorld( worldCId );
	b2DestroyWorld( rId );
	b2DestroyWorld( sId );
	b2DestroyWorld( freshId );

	b2RecBufFree( &buf );

	return 0;
}
