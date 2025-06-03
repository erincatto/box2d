// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "TaskScheduler_c.h"
#include "determinism.h"
#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/types.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef BOX2D_PROFILE
#include <tracy/TracyC.h>
#else
#define TracyCFrameMark
#endif

#define EXPECTED_SLEEP_STEP 288
#define EXPECTED_HASH 0x35467e1e

enum
{
	e_maxTasks = 128,
};

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

	box2dTask( 0, itemCount, 0, box2dContext );
	return NULL;
}

static void FinishTask( void* userTask, void* userContext )
{
	MAYBE_UNUSED( userContext );

	enkiTaskSet* task = userTask;
	enkiWaitForTaskSet( scheduler, task );
}

static int SingleMultithreadingTest( int workerCount )
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

	b2WorldId worldId = b2CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;

	bool done = false;
	while ( done == false )
	{
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateFallingHinges( worldId, &data );
	}

	b2DestroyWorld( worldId );

	for ( int i = 0; i < e_maxTasks; ++i )
	{
		enkiDeleteTaskSet( scheduler, tasks[i] );
	}

	enkiDeleteTaskScheduler( scheduler );

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

// Test multithreaded determinism.
static int MultithreadingTest( void )
{
	for ( int workerCount = 1; workerCount < 6; ++workerCount )
	{
		int result = SingleMultithreadingTest( workerCount );
		ENSURE( result == 0 );
	}

	return 0;
}

// Test cross-platform determinism.
static int CrossPlatformTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;

	bool done = false;
	while ( done == false )
	{
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateFallingHinges( worldId, &data );
	}

	ENSURE( data.sleepStep == EXPECTED_SLEEP_STEP );
	ENSURE( data.hash == EXPECTED_HASH );

	DestroyFallingHinges( &data );

	b2DestroyWorld( worldId );

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( CrossPlatformTest );

	return 0;
}
