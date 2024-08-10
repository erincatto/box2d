// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "TaskScheduler_c.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

#include <stdio.h>

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

void ExecuteRangeTask( uint32_t start, uint32_t end, uint32_t threadIndex, void* context )
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

void TiltedStacks( int testIndex, int workerCount )
{
	scheduler = enkiNewTaskScheduler();
	struct enkiTaskSchedulerConfig config = enkiGetTaskSchedulerConfig( scheduler );
	config.numTaskThreadsToCreate = workerCount - 1;
	enkiInitTaskSchedulerWithConfig( scheduler, config );

	for ( int i = 0; i < e_maxTasks; ++i )
	{
		tasks[i] = enkiCreateTaskSet( scheduler, ExecuteRangeTask );
	}

	// Define the gravity vector.
	b2Vec2 gravity = { 0.0f, -10.0f };

	// Construct a world object, which will hold and simulate the rigid bodies.
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = gravity;
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
int DeterminismTest( void )
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
