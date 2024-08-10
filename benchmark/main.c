// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "TaskScheduler_c.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN64 )
	#include <windows.h>
#elif defined( __APPLE__ )
	#include <unistd.h>
#elif defined( __linux__ )
	#include <unistd.h>
#endif

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )
#define MAYBE_UNUSED( x ) ( (void)( x ) )

typedef b2WorldId CreateBenchmarkFcn( b2WorldDef* worldDef );
extern b2WorldId JointGrid( b2WorldDef* worldDef );
extern b2WorldId LargePyramid( b2WorldDef* worldDef );
extern b2WorldId ManyPyramids( b2WorldDef* worldDef );
extern b2WorldId Smash( b2WorldDef* worldDef );
extern b2WorldId Tumbler( b2WorldDef* worldDef );

typedef struct Benchmark
{
	const char* name;
	CreateBenchmarkFcn* createFcn;
	int stepCount;
} Benchmark;

#define MAX_TASKS 128
#define THREAD_LIMIT 32

typedef struct TaskData
{
	b2TaskCallback* box2dTask;
	void* box2dContext;
} TaskData;

enkiTaskScheduler* scheduler;
enkiTaskSet* tasks[MAX_TASKS];
TaskData taskData[MAX_TASKS];
int taskCount;

int GetNumberOfCores()
{
#if defined( _WIN64 )
	SYSTEM_INFO sysinfo;
	GetSystemInfo( &sysinfo );
	return sysinfo.dwNumberOfProcessors;
#elif defined( __APPLE__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#elif defined( __linux__ )
	(int)return sysconf( _SC_NPROCESSORS_ONLN );
#else
	return 1;
#endif
}

void ExecuteRangeTask( uint32_t start, uint32_t end, uint32_t threadIndex, void* context )
{
	TaskData* data = context;
	data->box2dTask( start, end, threadIndex, data->box2dContext );
}

static void* EnqueueTask( b2TaskCallback* box2dTask, int itemCount, int minRange, void* box2dContext, void* userContext )
{
	MAYBE_UNUSED( userContext );

	if ( taskCount < MAX_TASKS )
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
		printf( "MAX_TASKS exceeded!!!\n" );
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

int main( int argc, char** argv )
{
	int maxThreadCount = GetNumberOfCores();
	int runCount = 4;
	b2Counters counters = { 0 };
	bool enableContinuous = true;

	assert( maxThreadCount <= THREAD_LIMIT );

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		if ( strncmp( arg, "-t=", 3 ) == 0 )
		{
			int threadCount = atoi( arg + 3 );
			maxThreadCount = b2MinInt( maxThreadCount, threadCount );
		}
		else if ( strcmp( arg, "-h" ) == 0 )
		{
			printf( "Usage\n"
					"-t=<thread count>: the maximum number of threads to use\n" );
		}
	}

	Benchmark benchmarks[] = {
		{ "joint_grid", JointGrid, 500 },
		{ "large_pyramid", LargePyramid, 500 },
		{ "many_pyramids", ManyPyramids, 200 },
		{ "smash", Smash, 300 },
		{ "tumbler", Tumbler, 750 },
	};

	int benchmarkCount = ARRAY_COUNT( benchmarks );

	printf( "Starting Box2D benchmarks\n" );
	printf( "======================================\n" );

	for ( int benchmarkIndex = 0; benchmarkIndex < benchmarkCount; ++benchmarkIndex )
	{
#ifdef NDEBUG
		int stepCount = benchmarks[benchmarkIndex].stepCount;
#else
		int stepCount = 10;
#endif

		bool countersAcquired = false;

		printf( "benchmark: %s, steps = %d\n", benchmarks[benchmarkIndex].name, stepCount );

		float maxFps[THREAD_LIMIT] = { 0 };

		for ( int threadCount = 1; threadCount <= maxThreadCount; ++threadCount )
		{
			printf( "thread count: %d\n", threadCount );

			for ( int runIndex = 0; runIndex < runCount; ++runIndex )
			{
				scheduler = enkiNewTaskScheduler();
				struct enkiTaskSchedulerConfig config = enkiGetTaskSchedulerConfig( scheduler );
				config.numTaskThreadsToCreate = threadCount - 1;
				enkiInitTaskSchedulerWithConfig( scheduler, config );

				for ( int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex )
				{
					tasks[taskIndex] = enkiCreateTaskSet( scheduler, ExecuteRangeTask );
				}

				b2WorldDef worldDef = b2DefaultWorldDef();
				worldDef.enableSleep = false;
				worldDef.enableContinous = enableContinuous;
				worldDef.enqueueTask = EnqueueTask;
				worldDef.finishTask = FinishTask;
				worldDef.workerCount = threadCount;

				b2WorldId worldId = benchmarks[benchmarkIndex].createFcn( &worldDef );

				float timeStep = 1.0f / 60.0f;
				int subStepCount = 4;

				// Initial step can be expensive and skew benchmark
				b2World_Step( worldId, timeStep, subStepCount );

				b2Timer timer = b2CreateTimer();

				for ( int step = 0; step < stepCount; ++step )
				{
					b2World_Step( worldId, timeStep, subStepCount );
					taskCount = 0;
				}

				float ms = b2GetMilliseconds( &timer );
				float fps = 1000.0f * stepCount / ms;
				printf( "run %d : %g (ms), %g (fps)\n", runIndex, ms, fps );

				maxFps[threadCount - 1] = b2MaxFloat( maxFps[threadCount - 1], fps );

				if ( countersAcquired == false )
				{
					counters = b2World_GetCounters( worldId );
					countersAcquired = true;
				}

				b2DestroyWorld( worldId );

				for ( int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex )
				{
					enkiDeleteTaskSet( scheduler, tasks[taskIndex] );
					tasks[taskIndex] = NULL;
					taskData[taskIndex] = ( TaskData ){ 0 };
				}

				enkiDeleteTaskScheduler( scheduler );
				scheduler = NULL;
			}
		}

		printf( "body %d / shape %d / contact %d / joint %d / stack %d\n\n", counters.bodyCount, counters.shapeCount,
				counters.contactCount, counters.jointCount, counters.stackUsed );

		char fileName[64] = { 0 };
		snprintf( fileName, 64, "%s.csv", benchmarks[benchmarkIndex].name );
		FILE* file = fopen( fileName, "w" );
		if ( file == NULL )
		{
			continue;
		}

		fprintf( file, "threads,fps\n" );
		for ( int threadCount = 1; threadCount <= maxThreadCount; ++threadCount )
		{
			fprintf( file, "%d,%g\n", threadCount, maxFps[threadCount - 1] );
		}

		fclose( file );
	}

	printf( "======================================\n" );
	printf( "All Box2D benchmarks complete!\n" );

	return 0;
}
