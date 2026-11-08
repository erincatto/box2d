// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "determinism.h"
#include "physics_world.h"
#include "platform.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/types.h"

#include <stdio.h>
#include <string.h>

#define MAX_BATCH_TASKS ( B2_MAX_WORKERS + 2 )

typedef enum TaskOrder
{
	e_forward,
	e_reverse,
} TaskOrder;

// Runs every batch on the calling thread. Honoring the deferred flag holds the tree rebuild back until
// right before the Continue that joins it, which catches a join list placed on the wrong batch.
typedef struct SerialHost
{
	TaskOrder order;
	bool honorDeferred;
	int batchCount;
	int deferredBatch;
	int joinBatch;
	int deferredOnlyBatchCount;
	int maxBatchSize;
	const b2Task* heldTask;
} SerialHost;

static void RunSerialBatch( SerialHost* host, b2TaskBatch batch )
{
	host->maxBatchSize = batch.count > host->maxBatchSize ? batch.count : host->maxBatchSize;

	int deferredCount = 0;
	for ( int k = 0; k < batch.count; ++k )
	{
		int i = host->order == e_forward ? k : batch.count - 1 - k;
		const b2Task* task = batch.tasks + i;
		if ( task->deferred )
		{
			deferredCount += 1;
			host->deferredBatch = host->batchCount;
			if ( host->honorDeferred )
			{
				host->heldTask = task;
				continue;
			}
		}

		task->fcn( task->context );
	}

	if ( deferredCount == batch.count )
	{
		host->deferredOnlyBatchCount += 1;
	}

	if ( batch.joinCount > 0 )
	{
		host->joinBatch = host->batchCount;
		if ( host->heldTask != NULL )
		{
			host->heldTask->fcn( host->heldTask->context );
			host->heldTask = NULL;
		}
	}

	host->batchCount += 1;
}

static int SerialPhasedStep( SerialHost* host, b2WorldId worldId, float timeStep, int subStepCount )
{
	host->batchCount = 0;
	host->deferredBatch = -1;
	host->joinBatch = -1;
	host->heldTask = NULL;

	b2TaskBatch batch = b2World_BeginStep( worldId, timeStep, subStepCount );
	while ( batch.count > 0 )
	{
		ENSURE( batch.count <= MAX_BATCH_TASKS );
		ENSURE( batch.joinCount == 0 || batch.joinCount == 1 );
		RunSerialBatch( host, batch );
		batch = b2World_ContinueStep( worldId );
	}

	ENSURE( host->heldTask == NULL );
	ENSURE( ( host->deferredBatch == -1 ) == ( host->joinBatch == -1 ) );
	ENSURE( host->deferredBatch <= host->joinBatch );
	return 0;
}

// A pool of threads draining a queue that only grows. Slots are never reused, so producers and
// consumers need no reset handshake.
#define POOL_SLOT_CAPACITY ( 1 << 18 )
#define POOL_MAX_THREADS 8

typedef struct PoolSlot
{
	b2TaskCallbackFcn* fcn;
	void* context;
	b2AtomicInt* counter;
} PoolSlot;

typedef struct Pool
{
	PoolSlot* slots;
	b2AtomicInt published;
	b2AtomicInt claimed;
	b2AtomicInt quit;
	b2Thread* threads[POOL_MAX_THREADS];
	int threadCount;
} Pool;

static bool PoolRunOne( Pool* pool )
{
	int claimed = b2AtomicLoadInt( &pool->claimed );
	int published = b2AtomicLoadInt( &pool->published );
	if ( claimed >= published )
	{
		return false;
	}

	if ( b2AtomicCompareExchangeInt( &pool->claimed, claimed, claimed + 1 ) == false )
	{
		return true;
	}

	PoolSlot* slot = pool->slots + claimed;
	slot->fcn( slot->context );
	if ( slot->counter != NULL )
	{
		b2AtomicFetchAddInt( slot->counter, -1 );
	}

	return true;
}

static void PoolThreadMain( void* context )
{
	Pool* pool = context;
	while ( b2AtomicLoadInt( &pool->quit ) == 0 )
	{
		if ( PoolRunOne( pool ) == false )
		{
			b2Yield();
		}
	}
}

static void PoolPush( Pool* pool, b2TaskCallbackFcn* fcn, void* context, b2AtomicInt* counter )
{
	int index = b2AtomicLoadInt( &pool->published );
	B2_ASSERT( index < POOL_SLOT_CAPACITY );
	pool->slots[index] = (PoolSlot){ fcn, context, counter };
	b2AtomicStoreInt( &pool->published, index + 1 );
}

static void PoolHelpUntilZero( Pool* pool, b2AtomicInt* counter )
{
	while ( b2AtomicLoadInt( counter ) > 0 )
	{
		if ( PoolRunOne( pool ) == false )
		{
			b2Pause();
		}
	}
}

static void CreatePool( Pool* pool, int threadCount )
{
	memset( pool, 0, sizeof( Pool ) );
	pool->slots = b2Alloc( POOL_SLOT_CAPACITY * sizeof( PoolSlot ) );
	pool->threadCount = threadCount;
	for ( int i = 0; i < threadCount; ++i )
	{
		pool->threads[i] = b2CreateThread( PoolThreadMain, pool, "step_test" );
	}
}

static void DestroyPool( Pool* pool )
{
	b2AtomicStoreInt( &pool->quit, 1 );
	for ( int i = 0; i < pool->threadCount; ++i )
	{
		b2JoinThread( pool->threads[i] );
	}
	b2Free( pool->slots, POOL_SLOT_CAPACITY * sizeof( PoolSlot ) );
}

// Pull host on the pool. The rebuild stays in flight across Continue calls until a batch joins it.
static void PooledPhasedStep( Pool* pool, b2WorldId worldId, float timeStep, int subStepCount )
{
	b2AtomicInt batchCounter;
	b2AtomicInt treeCounter;
	b2AtomicStoreInt( &treeCounter, 0 );

	b2TaskBatch batch = b2World_BeginStep( worldId, timeStep, subStepCount );
	while ( batch.count > 0 )
	{
		int nonDeferred = 0;
		for ( int i = 0; i < batch.count; ++i )
		{
			nonDeferred += batch.tasks[i].deferred ? 0 : 1;
		}

		b2AtomicStoreInt( &batchCounter, nonDeferred );

		for ( int i = 0; i < batch.count; ++i )
		{
			const b2Task* task = batch.tasks + i;
			if ( task->deferred )
			{
				b2AtomicStoreInt( &treeCounter, 1 );
				PoolPush( pool, task->fcn, task->context, &treeCounter );
			}
			else
			{
				PoolPush( pool, task->fcn, task->context, &batchCounter );
			}
		}

		PoolHelpUntilZero( pool, &batchCounter );

		if ( batch.joinCount > 0 )
		{
			PoolHelpUntilZero( pool, &treeCounter );
		}

		batch = b2World_ContinueStep( worldId );
	}

	B2_ASSERT( b2AtomicLoadInt( &treeCounter ) == 0 );
}

static void* EnqueueOnlyTask( b2TaskCallbackFcn* task, void* taskContext, void* userContext )
{
	Pool* pool = userContext;
	PoolPush( pool, task, taskContext, NULL );
	return pool;
}

typedef enum HostKind
{
	e_syncHost,
	e_serialHost,
	e_pooledHost,
} HostKind;

typedef struct HostConfig
{
	HostKind kind;
	TaskOrder order;
	bool honorDeferred;
	int workerCount;
	int threadCount;
} HostConfig;

static int RunFallingHinges( HostConfig config )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = config.workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	Pool pool;
	if ( config.kind == e_pooledHost )
	{
		CreatePool( &pool, config.threadCount - 1 );
	}

	SerialHost serialHost = { .order = config.order, .honorDeferred = config.honorDeferred };

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;
	int stepLimit = 1000;
	for ( int i = 0; i < stepLimit; ++i )
	{
		int subStepCount = 4;
		if ( config.kind == e_serialHost )
		{
			int result = SerialPhasedStep( &serialHost, worldId, timeStep, subStepCount );
			ENSURE( result == 0 );
		}
		else if ( config.kind == e_pooledHost )
		{
			PooledPhasedStep( &pool, worldId, timeStep, subStepCount );
		}
		else
		{
			b2World_Step( worldId, timeStep, subStepCount );
		}

		if ( UpdateFallingHinges( worldId, &data ) )
		{
			break;
		}
	}

	if ( config.kind != e_syncHost )
	{
		ENSURE( b2GetWorldFromId( worldId )->scheduler == NULL );
	}

	ENSURE( serialHost.maxBatchSize <= config.workerCount + 1 );

	b2DestroyWorld( worldId );

	if ( config.kind == e_pooledHost )
	{
		DestroyPool( &pool );
	}

	if ( data.sleepStep != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH )
	{
		printf( "  host=%d order=%d deferred=%d workers=%d threads=%d sleepStep=%d hash=0x%08X\n", config.kind, config.order,
				config.honorDeferred ? 1 : 0, config.workerCount, config.threadCount, data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

static int SerialPullTest( void )
{
	int workerCounts[] = { 1, 4, 8 };
	for ( int i = 0; i < 3; ++i )
	{
		HostConfig config = { .kind = e_serialHost, .order = e_forward, .workerCount = workerCounts[i] };
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

// The first solver task to run must orchestrate whatever its slot, or a serial host strands a spinner
static int ReverseOrderTest( void )
{
	int workerCounts[] = { 4, 8 };
	for ( int i = 0; i < 2; ++i )
	{
		HostConfig config = { .kind = e_serialHost, .order = e_reverse, .workerCount = workerCounts[i] };
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

static int DeferredJoinTest( void )
{
	int workerCounts[] = { 1, 4 };
	for ( int i = 0; i < 2; ++i )
	{
		HostConfig config = {
			.kind = e_serialHost, .order = e_forward, .honorDeferred = true, .workerCount = workerCounts[i] };
		ENSURE( RunFallingHinges( config ) == 0 );

		config.order = e_reverse;
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

static int PooledPullTest( void )
{
	int pairs[][2] = { { 1, 1 }, { 2, 2 }, { 4, 4 }, { 8, 8 }, { 8, 2 }, { 2, 8 } };
	for ( int i = 0; i < 6; ++i )
	{
		HostConfig config = { .kind = e_pooledHost, .workerCount = pairs[i][0], .threadCount = pairs[i][1] };
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

static int EnqueueOnlyTest( void )
{
	int workerCounts[] = { 2, 4 };
	for ( int i = 0; i < 2; ++i )
	{
		Pool pool;
		CreatePool( &pool, 3 );

		b2WorldDef worldDef = b2DefaultWorldDef();
		worldDef.workerCount = workerCounts[i];
		worldDef.enqueueTask = EnqueueOnlyTask;
		worldDef.finishTask = NULL;
		worldDef.userTaskContext = &pool;
		b2WorldId worldId = b2CreateWorld( &worldDef );

		FallingHingeData data = CreateFallingHinges( worldId );

		float timeStep = 1.0f / 60.0f;
		for ( int step = 0; step < 1000; ++step )
		{
			b2World_Step( worldId, timeStep, 4 );
			if ( UpdateFallingHinges( worldId, &data ) )
			{
				break;
			}
		}

		ENSURE( b2GetWorldFromId( worldId )->scheduler == NULL );

		b2DestroyWorld( worldId );
		DestroyPool( &pool );

		ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
		ENSURE( data.hash == EXPECTED_HASH );

		DestroyFallingHinges( &data );
	}

	return 0;
}

static int AlternateTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 4;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );
	SerialHost host = { .order = e_reverse, .honorDeferred = true };

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 1000; ++i )
	{
		if ( ( i & 1 ) == 0 )
		{
			b2World_Step( worldId, timeStep, 4 );
		}
		else
		{
			ENSURE( SerialPhasedStep( &host, worldId, timeStep, 4 ) == 0 );
		}

		if ( UpdateFallingHinges( worldId, &data ) )
		{
			break;
		}
	}

	b2DestroyWorld( worldId );

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

static int EmptyWorldTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 4;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	SerialHost host = { .order = e_forward };
	ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
	ENSURE( host.batchCount == 0 );
	ENSURE( host.deferredBatch == -1 );

	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );
	ENSURE( host.batchCount == 0 );

	b2DestroyWorld( worldId );
	return 0;
}

static b2BodyId CreateBall( b2WorldId worldId, b2Pos position )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = position;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2CreateCircleShape( bodyId, &shapeDef, &circle );
	return bodyId;
}

// The rebuild must join on the solve batch normally and on the collide batch when solve is skipped
static int JoinPlacementTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 4;

	// A lone ball, no contacts. A zero step skips solve, so the collide batch is the rebuild alone.
	{
		b2WorldId worldId = b2CreateWorld( &worldDef );
		b2BodyId bodyId = CreateBall( worldId, (b2Pos){ 0.0f, 4.0f } );

		SerialHost host = { .order = e_forward, .honorDeferred = true };
		ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );
		ENSURE( host.deferredBatch >= 0 );
		ENSURE( host.joinBatch == host.deferredBatch );
		ENSURE( host.deferredOnlyBatchCount == 1 );

		b2Pos p = b2Body_GetPosition( bodyId );
		ENSURE( p.x == 0.0f && p.y == 4.0f );

		b2DestroyWorld( worldId );
	}

	// Touching balls on the ground with a zero step: the collide batch has contacts and the join
	{
		b2WorldId worldId = b2CreateWorld( &worldDef );

		b2BodyDef groundDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &groundDef );
		b2ShapeDef groundShapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &groundShapeDef, &segment );

		for ( int i = 0; i < 8; ++i )
		{
			CreateBall( worldId, (b2Pos){ -4.0f + 1.0f * i, 0.5f } );
		}

		SerialHost host = { .order = e_forward, .honorDeferred = true };
		ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );
		ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );

		// Moving step: the join lands on a later batch than the rebuild
		for ( int i = 0; i < 4; ++i )
		{
			ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
			if ( host.deferredBatch >= 0 )
			{
				ENSURE( host.joinBatch > host.deferredBatch );
			}
		}

		b2DestroyWorld( worldId );
	}

	return 0;
}

// A scene with bullets, a sensor, joints and sleeping islands, checked against the synchronous step
// every step by state hash.
static b2WorldId CreateMixedScene( int workerCount )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &groundShapeDef, &segment );

	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;
	b2Polygon sensorBox = b2MakeOffsetBox( 4.0f, 2.0f, (b2Vec2){ 10.0f, 2.0f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &sensorDef, &sensorBox );

	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef boxDef = b2DefaultShapeDef();
	boxDef.enableSensorEvents = true;
	boxDef.enableHitEvents = true;

	for ( int row = 0; row < 10; ++row )
	{
		for ( int col = 0; col < 10 - row; ++col )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = (b2Pos){ -5.0f + col * 1.05f + row * 0.525f, 0.5f + row * 1.0f };
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &boxDef, &box );
		}
	}

	b2BodyId prevId = groundId;
	for ( int i = 0; i < 6; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 12.0f + i * 1.0f, 8.0f };
		b2BodyId linkId = b2CreateBody( worldId, &bodyDef );
		b2Polygon link = b2MakeBox( 0.5f, 0.125f );
		b2CreatePolygonShape( linkId, &boxDef, &link );

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = prevId;
		jointDef.base.bodyIdB = linkId;
		jointDef.base.localFrameA.p = prevId.index1 == groundId.index1 ? (b2Vec2){ 11.5f, 8.0f } : (b2Vec2){ 0.5f, 0.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };
		b2CreateRevoluteJoint( worldId, &jointDef );
		prevId = linkId;
	}

	for ( int i = 0; i < 4; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = (b2Pos){ -30.0f, 1.0f + 2.0f * i };
		bodyDef.linearVelocity = (b2Vec2){ 200.0f, 0.0f };
		b2BodyId bulletId = b2CreateBody( worldId, &bodyDef );
		b2ShapeDef bulletDef = b2DefaultShapeDef();
		bulletDef.enableSensorEvents = true;
		b2Circle circle = { { 0.0f, 0.0f }, 0.1f };
		b2CreateCircleShape( bulletId, &bulletDef, &circle );
	}

	return worldId;
}

static int MixedSceneTest( void )
{
	int workerCounts[] = { 1, 4, 8 };
	for ( int w = 0; w < 3; ++w )
	{
		int workerCount = workerCounts[w];
		b2WorldId syncId = CreateMixedScene( workerCount );
		b2WorldId serialId = CreateMixedScene( workerCount );
		b2WorldId pooledId = CreateMixedScene( workerCount );

		Pool pool;
		CreatePool( &pool, 3 );
		SerialHost host = { .order = e_reverse, .honorDeferred = true };

		int sensorBeginTotal = 0;
		int hitTotal = 0;
		int maxBatchCount = 0;
		float timeStep = 1.0f / 60.0f;
		for ( int i = 0; i < 400; ++i )
		{
			b2World_Step( syncId, timeStep, 4 );
			ENSURE( SerialPhasedStep( &host, serialId, timeStep, 4 ) == 0 );
			PooledPhasedStep( &pool, pooledId, timeStep, 4 );

			uint64_t syncHash = b2World_GetStateHash( syncId );
			ENSURE( b2World_GetStateHash( serialId ) == syncHash );
			ENSURE( b2World_GetStateHash( pooledId ) == syncHash );

			b2SensorEvents syncSensors = b2World_GetSensorEvents( syncId );
			b2SensorEvents serialSensors = b2World_GetSensorEvents( serialId );
			ENSURE( syncSensors.beginCount == serialSensors.beginCount );
			ENSURE( syncSensors.endCount == serialSensors.endCount );

			b2ContactEvents syncContacts = b2World_GetContactEvents( syncId );
			b2ContactEvents serialContacts = b2World_GetContactEvents( serialId );
			ENSURE( syncContacts.beginCount == serialContacts.beginCount );
			ENSURE( syncContacts.hitCount == serialContacts.hitCount );

			b2BodyEvents syncBodies = b2World_GetBodyEvents( syncId );
			b2BodyEvents serialBodies = b2World_GetBodyEvents( serialId );
			ENSURE( syncBodies.moveCount == serialBodies.moveCount );

			sensorBeginTotal += serialSensors.beginCount;
			hitTotal += serialContacts.hitCount;
			maxBatchCount = host.batchCount > maxBatchCount ? host.batchCount : maxBatchCount;
		}

		ENSURE( sensorBeginTotal > 0 );
		ENSURE( hitTotal > 0 );
		ENSURE( maxBatchCount == 7 );
		ENSURE( b2World_GetAwakeBodyCount( serialId ) < 55 + 6 + 4 );

		ENSURE( b2World_GetAwakeBodyCount( syncId ) == b2World_GetAwakeBodyCount( serialId ) );
		ENSURE( b2World_GetCounters( serialId ).taskCount > 0 );

		DestroyPool( &pool );
		b2DestroyWorld( syncId );
		b2DestroyWorld( serialId );
		b2DestroyWorld( pooledId );
	}

	return 0;
}

static int PhasedRecordingTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2Recording* recording = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, recording );

	FallingHingeData data = CreateFallingHinges( worldId );

	SerialHost host = { .order = e_reverse, .honorDeferred = true };
	for ( int i = 0; i < 120; ++i )
	{
		ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
		UpdateFallingHinges( worldId, &data );
	}

	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );
	DestroyFallingHinges( &data );

	const uint8_t* bytes = b2Recording_GetData( recording );
	int size = b2Recording_GetSize( recording );
	ENSURE( b2ValidateRecording( bytes, size, 0 ) );
	ENSURE( b2ValidateRecording( bytes, size, 4 ) );

	b2DestroyRecording( recording );
	return 0;
}

int StepTest( void )
{
	RUN_SUBTEST( SerialPullTest );
	RUN_SUBTEST( ReverseOrderTest );
	RUN_SUBTEST( DeferredJoinTest );
	RUN_SUBTEST( PooledPullTest );
	RUN_SUBTEST( EnqueueOnlyTest );
	RUN_SUBTEST( AlternateTest );
	RUN_SUBTEST( EmptyWorldTest );
	RUN_SUBTEST( JoinPlacementTest );
	RUN_SUBTEST( MixedSceneTest );
	RUN_SUBTEST( PhasedRecordingTest );

	return 0;
}
