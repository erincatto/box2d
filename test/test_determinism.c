// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "TaskScheduler_c.h"
#include "test_macros.h"

#include "box2d/base.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef BOX2D_PROFILE
	#include <tracy/TracyC.h>
#else
	#define TracyCFrameMark
#endif

enum
{
	e_columns = 10,
	e_rows = 10,
	e_count = e_columns * e_rows,
	e_maxTasks = 128,
};

b2Vec2 finalPositions[2][e_count];
b2Rot finalRotations[2][e_count];

typedef struct TaskData
{
	b2TaskCallback* box2dTask;
	void* box2dContext;
} TaskData;

enkiTaskScheduler* scheduler;
enkiTaskSet* tasks[e_maxTasks];
TaskData taskData[e_maxTasks];
int taskCount;

static void ExecuteRangeTask( uint32_t start, uint32_t end, uint32_t threadIndex, void* context )
{
	TaskData* data = context;
	data->box2dTask( start, end, threadIndex, data->box2dContext );
}

static void* EnqueueTask( b2TaskCallback* box2dTask, int itemCount, int minRange, void* box2dContext, void* userContext )
{
	MAYBE_UNUSED( userContext );

	if ( taskCount < e_maxTasks )
	{
		enkiTaskSet* task = tasks[taskCount];
		TaskData* data = taskData + taskCount;
		data->box2dTask = box2dTask;
		data->box2dContext = box2dContext;

		struct enkiParamsTaskSet params;
		params.minRange = minRange;
		params.setSize = itemCount;
		params.pArgs = data;
		params.priority = 0;

		enkiSetParamsTaskSet( task, params );
		enkiAddTaskSet( scheduler, task );

		++taskCount;

		return task;
	}
	else
	{
		box2dTask( 0, itemCount, 0, box2dContext );

		return NULL;
	}
}

static void FinishTask( void* userTask, void* userContext )
{
	MAYBE_UNUSED( userContext );

	enkiTaskSet* task = userTask;
	enkiWaitForTaskSet( scheduler, task );
}

// todo_erin move this to shared
static void TiltedStacks( int testIndex, int workerCount )
{
	scheduler = enkiNewTaskScheduler();
	struct enkiTaskSchedulerConfig config = enkiGetTaskSchedulerConfig( scheduler );
	config.numTaskThreadsToCreate = workerCount - 1;
	enkiInitTaskSchedulerWithConfig( scheduler, config );

	for ( int i = 0; i < e_maxTasks; ++i )
	{
		tasks[i] = enkiCreateTaskSet( scheduler, ExecuteRangeTask );
	}

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.enqueueTask = EnqueueTask;
	worldDef.finishTask = FinishTask;
	worldDef.workerCount = workerCount;
	worldDef.enableSleep = false;

	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyId bodies[e_count];

	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.position = ( b2Vec2 ){ 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bd );

		b2Polygon box = b2MakeBox( 1000.0f, 1.0f );
		b2ShapeDef sd = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &sd, &box );
	}

	b2Polygon box = b2MakeRoundedBox( 0.45f, 0.45f, 0.05f );
	b2ShapeDef sd = b2DefaultShapeDef();
	sd.density = 1.0f;
	sd.friction = 0.3f;

	float offset = 0.2f;
	float dx = 5.0f;
	float xroot = -0.5f * dx * ( e_columns - 1.0f );

	for ( int j = 0; j < e_columns; ++j )
	{
		float x = xroot + j * dx;

		for ( int i = 0; i < e_rows; ++i )
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;

			int n = j * e_rows + i;

			bd.position = ( b2Vec2 ){ x + offset * i, 0.5f + 1.0f * i };
			b2BodyId bodyId = b2CreateBody( worldId, &bd );
			bodies[n] = bodyId;

			b2CreatePolygonShape( bodyId, &sd, &box );
		}
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 3;

	for ( int i = 0; i < 100; ++i )
	{
		b2World_Step( worldId, timeStep, subStepCount );
		taskCount = 0;
		TracyCFrameMark;
	}

	for ( int i = 0; i < e_count; ++i )
	{
		finalPositions[testIndex][i] = b2Body_GetPosition( bodies[i] );
		finalRotations[testIndex][i] = b2Body_GetRotation( bodies[i] );
	}

	b2DestroyWorld( worldId );

	for ( int i = 0; i < e_maxTasks; ++i )
	{
		enkiDeleteTaskSet( scheduler, tasks[i] );
	}

	enkiDeleteTaskScheduler( scheduler );
}

// Test multithreaded determinism.
static int MultithreadingTest( void )
{
	// Test 1 : 4 threads
	TiltedStacks( 0, 4 );

	// Test 2 : 1 thread
	TiltedStacks( 1, 1 );

	// Both runs should produce identical results
	for ( int i = 0; i < e_count; ++i )
	{
		b2Vec2 p1 = finalPositions[0][i];
		b2Vec2 p2 = finalPositions[1][i];
		b2Rot rot1 = finalRotations[0][i];
		b2Rot rot2 = finalRotations[1][i];

		ENSURE( p1.x == p2.x );
		ENSURE( p1.y == p2.y );
		ENSURE( rot1.c == rot2.c );
		ENSURE( rot1.s == rot2.s );
	}

	return 0;
}

// Test cross platform determinism based on the FallingHinges sample.
static int CrossPlatformTest(void)
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Vec2){ 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 20.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	int columnCount = 4;
	int rowCount = 30;
	int bodyCount = rowCount * columnCount;

	b2BodyId* bodies = calloc( bodyCount, sizeof( b2BodyId ) );

	float h = 0.25f;
	float r = 0.1f * h;
	b2Polygon box = b2MakeRoundedBox( h - r, h - r, r );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.friction = 0.3f;

	float offset = 0.4f * h;
	float dx = 10.0f * h;
	float xroot = -0.5f * dx * ( columnCount - 1.0f );

	b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
	jointDef.enableLimit = true;
	jointDef.lowerAngle = -0.1f * B2_PI;
	jointDef.upperAngle = 0.2f * B2_PI;
	jointDef.enableSpring = true;
	jointDef.hertz = 0.5f;
	jointDef.dampingRatio = 0.5f;
	jointDef.localAnchorA = (b2Vec2){ h, h };
	jointDef.localAnchorB = (b2Vec2){ offset, -h };
	jointDef.drawSize = 0.1f;

	int bodyIndex = 0;

	for ( int j = 0; j < columnCount; ++j )
	{
		float x = xroot + j * dx;

		b2BodyId prevBodyId = b2_nullBodyId;

		for ( int i = 0; i < rowCount; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			bodyDef.position.x = x + offset * i;
			bodyDef.position.y = h + 2.0f * h * i;

			// this tests the deterministic cosine and sine functions
			bodyDef.rotation = b2MakeRot( 0.1f * i - 1.0f );

			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			if ( ( i & 1 ) == 0 )
			{
				prevBodyId = bodyId;
			}
			else
			{
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				b2CreateRevoluteJoint( worldId, &jointDef );
				prevBodyId = b2_nullBodyId;
			}

			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			assert( bodyIndex < bodyCount );
			bodies[bodyIndex] = bodyId;

			bodyIndex += 1;
		}
	}

	assert( bodyIndex == bodyCount );

	uint32_t hash = 0;
	int sleepStep = -1;
	float timeStep = 1.0f / 60.0f;

	int stepCount = 0;
	int maxSteps = 500;
	while ( stepCount < maxSteps )
	{
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		if ( hash == 0 )
		{
			b2BodyEvents bodyEvents = b2World_GetBodyEvents( worldId );

			if ( bodyEvents.moveCount == 0 )
			{
				int awakeCount = b2World_GetAwakeBodyCount( worldId );
				ENSURE( awakeCount == 0 );

				hash = B2_HASH_INIT;
				for ( int i = 0; i < bodyCount; ++i )
				{
					b2Transform xf = b2Body_GetTransform( bodies[i] );
					hash = b2Hash( hash, (uint8_t*)( &xf ), sizeof( b2Transform ) );
				}

				sleepStep = stepCount;
				printf( "step = %d, hash = 0x%08x\n", sleepStep, hash );

				break;
			}
		}

		stepCount += 1;
	}

	ENSURE( stepCount < maxSteps );
	ENSURE( sleepStep == 263 );
	ENSURE( hash == 0x7de58fbe );

	free( bodies );

	b2DestroyWorld( worldId );

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( CrossPlatformTest );

	return 0;
}
