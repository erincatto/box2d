// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "TaskScheduler_c.h"
#include "benchmarks.h"

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

typedef void CreateFcn( b2WorldId worldId );
typedef float StepFcn( b2WorldId worldId, int stepCount );

typedef struct Benchmark
{
	const char* name;
	CreateFcn* createFcn;
	StepFcn* stepFcn;
	int totalStepCount;
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
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#elif defined( __EMSCRIPTEN__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
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

static void MinProfile( b2Profile* p1, const b2Profile* p2 )
{
	p1->step = b2MinFloat( p1->step, p2->step );
	p1->pairs = b2MinFloat( p1->pairs, p2->pairs );
	p1->collide = b2MinFloat( p1->collide, p2->collide );
	p1->solveConstraints = b2MinFloat( p1->solveConstraints, p2->solveConstraints );
	p1->transforms = b2MinFloat( p1->transforms, p2->transforms );
	p1->refit = b2MinFloat( p1->refit, p2->refit );
	p1->sleepIslands = b2MinFloat( p1->sleepIslands, p2->sleepIslands );
}

// Box2D benchmark application. On Windows I recommend running this in an administrator command prompt. Don't use Windows
// Terminal. Or use affinity. [0x01 0x02 0x04 0x08 0x10 0x20 0x40 0x80]
// Examples:
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=8
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 -b=3 -r=20 -s
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 -b=3 -r=1 -nc -s

int main( int argc, char** argv )
{
	Benchmark benchmarks[] = {
		{ "joint_grid", CreateJointGrid, NULL, 500 },
		{ "large_pyramid", CreateLargePyramid, NULL, 500 },
		{ "many_pyramids", CreateManyPyramids, NULL, 200 },
		{ "rain", CreateRain, StepRain, 1000 },
		{ "smash", CreateSmash, NULL, 300 },
		{ "spinner", CreateSpinner, StepSpinner, 1400 },
		{ "tumbler", CreateTumbler, NULL, 750 },
	};

	int benchmarkCount = ARRAY_COUNT( benchmarks );

	int maxSteps = benchmarks[0].totalStepCount;
	for ( int i = 1; i < benchmarkCount; ++i )
	{
		maxSteps = b2MaxInt( maxSteps, benchmarks[i].totalStepCount );
	}

	b2Profile maxProfile = {
		.step = FLT_MAX,
		.pairs = FLT_MAX,
		.collide = FLT_MAX,
		.solve = FLT_MAX,
		.mergeIslands = FLT_MAX,
		.prepareStages = FLT_MAX,
		.solveConstraints = FLT_MAX,
		.prepareConstraints = FLT_MAX,
		.integrateVelocities = FLT_MAX,
		.warmStart = FLT_MAX,
		.solveImpulses = FLT_MAX,
		.integratePositions = FLT_MAX,
		.relaxImpulses = FLT_MAX,
		.applyRestitution = FLT_MAX,
		.storeImpulses = FLT_MAX,
		.splitIslands = FLT_MAX,
		.transforms = FLT_MAX,
		.hitEvents = FLT_MAX,
		.refit = FLT_MAX,
		.bullets = FLT_MAX,
		.sleepIslands = FLT_MAX,
	};

	b2Profile* profiles = malloc( maxSteps * sizeof( b2Profile ) );
	for ( int i = 0; i < maxSteps; ++i )
	{
		profiles[i] = maxProfile;
	}

	float* stepResults = malloc( maxSteps * sizeof( float ) );
	memset( stepResults, 0, maxSteps * sizeof( float ) );

	int maxThreadCount = GetNumberOfCores();
	int runCount = 4;
	int singleBenchmark = -1;
	int singleWorkerCount = -1;
	b2Counters counters = { 0 };
	bool enableContinuous = true;
	bool recordStepTimes = false;

	assert( maxThreadCount <= THREAD_LIMIT );

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		if ( strncmp( arg, "-t=", 3 ) == 0 )
		{
			int threadCount = atoi( arg + 3 );
			maxThreadCount = b2ClampInt( threadCount, 1, maxThreadCount );
		}
		else if ( strncmp( arg, "-b=", 3 ) == 0 )
		{
			singleBenchmark = atoi( arg + 3 );
			singleBenchmark = b2ClampInt( singleBenchmark, 0, benchmarkCount - 1 );
		}
		else if ( strncmp( arg, "-w=", 3 ) == 0 )
		{
			singleWorkerCount = atoi( arg + 3 );
		}
		else if ( strncmp( arg, "-r=", 3 ) == 0 )
		{
			runCount = b2ClampInt( atoi( arg + 3 ), 1, 1000 );
		}
		else if ( strncmp( arg, "-nc", 3 ) == 0 )
		{
			enableContinuous = false;
			printf( "Continuous disabled\n" );
		}
		else if ( strncmp( arg, "-s", 3 ) == 0 )
		{
			recordStepTimes = true;
		}
		else if ( strcmp( arg, "-h" ) == 0 )
		{
			printf( "Usage\n"
					"-t=<integer>: the maximum number of threads to use\n"
					"-b=<integer>: run a single benchmark\n"
					"-w=<integer>: run a single worker count\n"
					"-r=<integer>: number of repeats (default is 4)\n"
					"-s: record step times\n" );
			exit( 0 );
		}
	}

	if ( singleWorkerCount != -1 )
	{
		singleWorkerCount = b2ClampInt( singleWorkerCount, 1, maxThreadCount );
	}

	printf( "Starting Box2D benchmarks\n" );
	printf( "======================================\n" );

	for ( int benchmarkIndex = 0; benchmarkIndex < benchmarkCount; ++benchmarkIndex )
	{
		if ( singleBenchmark != -1 && benchmarkIndex != singleBenchmark )
		{
			continue;
		}

#ifdef NDEBUG
		int stepCount = benchmarks[benchmarkIndex].totalStepCount;
#else
		int stepCount = 10;
#endif

		Benchmark* benchmark = benchmarks + benchmarkIndex;

		bool countersAcquired = false;

		printf( "benchmark: %s, steps = %d\n", benchmarks[benchmarkIndex].name, stepCount );

		float minTime[THREAD_LIMIT] = { 0 };

		for ( int threadCount = 1; threadCount <= maxThreadCount; ++threadCount )
		{
			if ( singleWorkerCount != -1 && singleWorkerCount != threadCount )
			{
				continue;
			}

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
				worldDef.enableContinuous = enableContinuous;
				worldDef.enqueueTask = EnqueueTask;
				worldDef.finishTask = FinishTask;
				worldDef.workerCount = threadCount;
				b2WorldId worldId = b2CreateWorld( &worldDef );

				benchmark->createFcn( worldId );

				float timeStep = 1.0f / 60.0f;
				int subStepCount = 4;

				// Initial step can be expensive and skew benchmark
				if ( benchmark->stepFcn != NULL )
				{
					stepResults[0] = benchmark->stepFcn( worldId, 0 );
				}

				assert( stepCount <= maxSteps );

				b2World_Step( worldId, timeStep, subStepCount );

				b2Profile profile = b2World_GetProfile( worldId );
				MinProfile( profiles + 0, &profile );

				taskCount = 0;

				uint64_t ticks = b2GetTicks();

				for ( int stepIndex = 1; stepIndex < stepCount; ++stepIndex )
				{
					if ( benchmark->stepFcn != NULL )
					{
						stepResults[stepIndex] = benchmark->stepFcn( worldId, stepIndex );
					}

					b2World_Step( worldId, timeStep, subStepCount );
					taskCount = 0;

					profile = b2World_GetProfile( worldId );
					MinProfile( profiles + stepIndex, &profile );
				}

				float ms = b2GetMilliseconds( ticks );
				printf( "run %d : %g (ms)\n", runIndex, ms );

				if (runIndex == 0)
				{
					minTime[threadCount - 1] = ms ;
				}
				else
				{
					minTime[threadCount - 1] = b2MinFloat( minTime[threadCount - 1], ms );
				}

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

			if ( recordStepTimes )
			{
				char fileName[64] = { 0 };
				snprintf( fileName, 64, "%s_t%d.dat", benchmarks[benchmarkIndex].name, threadCount );
				FILE* file = fopen( fileName, "w" );
				if ( file == NULL )
				{
					continue;
				}

				for ( int stepIndex = 0; stepIndex < stepCount; ++stepIndex )
				{
					b2Profile p = profiles[stepIndex];
					fprintf( file, "%g %g %g %g %g %g %g\n", p.step, p.pairs, p.collide, p.solveConstraints, p.transforms, p.refit, p.sleepIslands );
				}

				fclose( file );
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

		fprintf( file, "threads,ms\n" );
		for ( int threadIndex = 1; threadIndex <= maxThreadCount; ++threadIndex )
		{
			fprintf( file, "%d,%g\n", threadIndex, minTime[threadIndex - 1] );
		}

		fclose( file );
	}

	printf( "======================================\n" );
	printf( "All Box2D benchmarks complete!\n" );

	free( profiles );
	free( stepResults );

	return 0;
}
