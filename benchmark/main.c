// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "benchmarks.h"
#include "utils.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef TRACY_ENABLE
#include <tracy/TracyC.h>
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

static void MinProfile( b2Profile* p1, const b2Profile* p2 )
{
	p1->step = b2MinFloat( p1->step, p2->step );
	p1->pairs = b2MinFloat( p1->pairs, p2->pairs );
	p1->collide = b2MinFloat( p1->collide, p2->collide );
	p1->constraints = b2MinFloat( p1->constraints, p2->constraints );
	p1->transforms = b2MinFloat( p1->transforms, p2->transforms );
	p1->refit = b2MinFloat( p1->refit, p2->refit );
	p1->sleepIslands = b2MinFloat( p1->sleepIslands, p2->sleepIslands );
}

// Match a name=value option in either its short or long spelling. Returns the value or NULL.
static const char* MatchValue( const char* arg, const char* shortName, const char* longName )
{
	size_t n = strlen( shortName );
	if ( strncmp( arg, shortName, n ) == 0 )
	{
		return arg + n;
	}
	n = strlen( longName );
	if ( strncmp( arg, longName, n ) == 0 )
	{
		return arg + n;
	}
	return NULL;
}

// Distinguish a numeric benchmark index from a name. Empty string is not numeric.
static bool IsAllDigits( const char* s )
{
	if ( *s == 0 )
	{
		return false;
	}
	for ( ; *s != 0; ++s )
	{
		if ( isdigit( (unsigned char)*s ) == 0 )
		{
			return false;
		}
	}
	return true;
}

static bool EqualsIgnoreCase( const char* a, const char* b )
{
	while ( *a != 0 && *b != 0 )
	{
		if ( tolower( (unsigned char)*a ) != tolower( (unsigned char)*b ) )
		{
			return false;
		}
		++a;
		++b;
	}
	return *a == *b;
}

static int FindBenchmark( const Benchmark* benchmarks, int count, const char* name )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( EqualsIgnoreCase( benchmarks[i].name, name ) )
		{
			return i;
		}
	}
	return -1;
}

static void PrintBenchmarks( const Benchmark* benchmarks, int count )
{
	printf( "Registered benchmarks:\n" );
	for ( int i = 0; i < count; ++i )
	{
		printf( "  %2d  %s\n", i, benchmarks[i].name );
	}
}

// Box2D benchmark application. On Windows it is important to use affinity avoid cross CCD
// usage or efficiency cores. Also on Windows create a power plan with Processor power management
// Min/Max of 99%. This prevents boosting and makes the benchmarks more repeatable.
// Affinity [0x01 0x02 0x04 0x08 0x10 0x20 0x40 0x80]

// Run all benchmarks with 1 to 8 threads.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=8

// Run all benchmarks with 4 workers only.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4

// List the registered benchmarks.
// .\build\bin\Release\benchmark.exe -l

// Run a single benchmark by name with 4 workers.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 --benchmark=many_pyramids -r=5

// Run benchmark 3 with 4 workers and repeat 20 times. Record the step times.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 -b=3 -r=20 -s
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=8 -b=7

// Run benchmark 3 with 4 workers and run once. Disable continuous collision. Record the step times.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 -b=3 -r=1 -nc -s

int main( int argc, char** argv )
{
#ifdef TRACY_ENABLE
	___tracy_startup_profiler();
#endif

	Benchmark benchmarks[] = {
		{ "compounds", CreateCompounds, NULL, 500 },
		{ "joint_grid", CreateJointGrid, NULL, 500 },
		{ "junkyard", CreateJunkyard, StepJunkyard, 800 },
		{ "large_pyramid", CreateLargePyramid, NULL, 500 },
		{ "many_pyramids", CreateManyPyramids, NULL, 200 },
		{ "rain", CreateRain, StepRain, 1000 },
		{ "smash", CreateSmash, NULL, 300 },
		{ "spinner", CreateSpinner, StepSpinner, 500 },
		{ "tumbler", CreateTumbler, NULL, 750 },
		{ "washer", CreateWasher, NULL, 500 },
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
		.solverSetup = FLT_MAX,
		.constraints = FLT_MAX,
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

	int maxThreadCount = b2MinInt(GetNumberOfCores(), B2_MAX_WORKERS);
	int runCount = 4;
	int singleBenchmark = -1;
	int singleWorkerCount = -1;
	b2Counters counters = { 0 };
	bool enableContinuous = true;
	bool recordStepTimes = false;

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		const char* value;
		if ( ( value = MatchValue( arg, "-t=", "--threads=" ) ) != NULL )
		{
			int threadCount = atoi( value );
			maxThreadCount = b2ClampInt( threadCount, 1, maxThreadCount );
		}
		else if ( ( value = MatchValue( arg, "-b=", "--benchmark=" ) ) != NULL )
		{
			// Accept an index for backward compatibility, otherwise an exact name
			if ( IsAllDigits( value ) )
			{
				singleBenchmark = b2ClampInt( atoi( value ), 0, benchmarkCount - 1 );
			}
			else
			{
				singleBenchmark = FindBenchmark( benchmarks, benchmarkCount, value );
				if ( singleBenchmark == -1 )
				{
					printf( "No benchmark named \"%s\"\n", value );
					PrintBenchmarks( benchmarks, benchmarkCount );
					exit( 1 );
				}
			}
		}
		else if ( ( value = MatchValue( arg, "-w=", "--workers=" ) ) != NULL )
		{
			singleWorkerCount = atoi( value );
		}
		else if ( ( value = MatchValue( arg, "-r=", "--repeats=" ) ) != NULL )
		{
			runCount = b2ClampInt( atoi( value ), 1, 1000 );
		}
		else if ( strcmp( arg, "-nc" ) == 0 || strcmp( arg, "--no-continuous" ) == 0 )
		{
			enableContinuous = false;
			printf( "Continuous disabled\n" );
		}
		else if ( strcmp( arg, "-s" ) == 0 || strcmp( arg, "--record-steps" ) == 0 )
		{
			recordStepTimes = true;
		}
		else if ( strcmp( arg, "-l" ) == 0 || strcmp( arg, "--list" ) == 0 )
		{
			PrintBenchmarks( benchmarks, benchmarkCount );
			exit( 0 );
		}
		else if ( strcmp( arg, "-h" ) == 0 || strcmp( arg, "--help" ) == 0 )
		{
			printf( "Usage\n"
					"-t, --threads=<integer>: the maximum number of threads to use\n"
					"-b, --benchmark=<index|name>: run a single benchmark by index or name\n"
					"-w, --workers=<integer>: run a single worker count\n"
					"-r, --repeats=<integer>: number of repeats (default is 4)\n"
					"-nc, --no-continuous: disable continuous collision\n"
					"-s, --record-steps: record step times\n"
					"-l, --list: list the registered benchmarks\n"
					"-h, --help: print this help\n" );
			exit( 0 );
		}
		else
		{
			printf( "Unknown option \"%s\", use -h for help\n", arg );
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

		float minTime[B2_MAX_WORKERS] = { 0 };

		for ( int threadCount = 1; threadCount <= maxThreadCount; ++threadCount )
		{
			if ( singleWorkerCount != -1 && singleWorkerCount != threadCount )
			{
				continue;
			}

			printf( "thread count: %d\n", threadCount );

			for ( int runIndex = 0; runIndex < runCount; ++runIndex )
			{
				b2WorldDef worldDef = b2DefaultWorldDef();
				worldDef.enableContinuous = enableContinuous;
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

				uint64_t ticks = b2GetTicks();

				for ( int stepIndex = 1; stepIndex < stepCount; ++stepIndex )
				{
					if ( benchmark->stepFcn != NULL )
					{
						stepResults[stepIndex] = benchmark->stepFcn( worldId, stepIndex );
					}

					b2World_Step( worldId, timeStep, subStepCount );
					profile = b2World_GetProfile( worldId );
					MinProfile( profiles + stepIndex, &profile );
				}

				float ms = b2GetMilliseconds( ticks );
				printf( "run %d : %g (ms)\n", runIndex, ms );

				if ( runIndex == 0 )
				{
					minTime[threadCount - 1] = ms;
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
					fprintf( file, "%g %g %g %g %g %g %g\n", p.step, p.pairs, p.collide, p.constraints, p.transforms,
							 p.refit, p.sleepIslands );
				}

				fclose( file );
			}
		}

		printf( "body %d / shape %d / contact %d / joint %d / stack %d\n", counters.bodyCount, counters.shapeCount,
				counters.contactCount, counters.jointCount, counters.stackUsed );
		printf( "color counts:" );
		for ( int c = 0; c < ARRAY_COUNT( counters.colorCounts ); ++c )
		{
			printf( " %d", counters.colorCounts[c] );
		}
		printf( "\n\n" );

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

#ifdef TRACY_ENABLE
	___tracy_shutdown_profiler();
#endif
	return 0;
}
