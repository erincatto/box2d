// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/base.h"

#include <stddef.h>

#if defined( _WIN32 )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

static double s_invFrequency = 0.0;

b2Timer b2CreateTimer( void )
{
	LARGE_INTEGER largeInteger;

	if ( s_invFrequency == 0.0 )
	{
		QueryPerformanceFrequency( &largeInteger );

		s_invFrequency = (double)largeInteger.QuadPart;
		if ( s_invFrequency > 0.0 )
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	QueryPerformanceCounter( &largeInteger );
	b2Timer timer;
	timer.start = largeInteger.QuadPart;
	return timer;
}

int64_t b2GetTicks( b2Timer* timer )
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter( &largeInteger );
	int64_t ticks = largeInteger.QuadPart;
	int64_t count = ticks - timer->start;
	timer->start = ticks;
	return count;
}

float b2GetMilliseconds( const b2Timer* timer )
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter( &largeInteger );
	int64_t count = largeInteger.QuadPart;
	float ms = (float)( s_invFrequency * ( count - timer->start ) );
	return ms;
}

float b2GetMillisecondsAndReset( b2Timer* timer )
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter( &largeInteger );
	int64_t count = largeInteger.QuadPart;
	float ms = (float)( s_invFrequency * ( count - timer->start ) );
	timer->start = count;
	return ms;
}

void b2SleepMilliseconds( int milliseconds )
{
	// also SwitchToThread()
	Sleep( (DWORD)milliseconds );
}

void b2Yield()
{
	SwitchToThread();
}

#elif defined( __linux__ ) || defined( __EMSCRIPTEN__ )

#include <sched.h>
#include <time.h>

// maybe try CLOCK_MONOTONIC_RAW
b2Timer b2CreateTimer( void )
{
	b2Timer timer;
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	timer.tv_sec = ts.tv_sec;
	timer.tv_nsec = ts.tv_nsec;
	return timer;
}

float b2GetMilliseconds( const b2Timer* timer )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	time_t start_sec = timer->tv_sec;
	long start_nsec = timer->tv_nsec;

	time_t sec_diff = ts.tv_sec - start_sec;
	long nsec_diff = ts.tv_nsec - start_nsec;

	return (float)( sec_diff * 1000.0 + nsec_diff / 1000000.0 );
}

float b2GetMillisecondsAndReset( b2Timer* timer )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	time_t start_sec = timer->tv_sec;
	long start_nsec = timer->tv_nsec;

	time_t sec_diff = ts.tv_sec - start_sec;
	long nsec_diff = ts.tv_nsec - start_nsec;

	timer->tv_sec = ts.tv_sec;
	timer->tv_nsec = ts.tv_nsec;

	return (float)( sec_diff * 1000.0 + nsec_diff / 1000000.0 );
}

void b2SleepMilliseconds( int milliseconds )
{
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = ( milliseconds % 1000 ) * 1000000;
	nanosleep( &ts, NULL );
}

void b2Yield()
{
	sched_yield();
}

#elif defined( __APPLE__ )

#include <mach/mach_time.h>
#include <sched.h>
#include <sys/time.h>

static double s_invFrequency = 0.0;

b2Timer b2CreateTimer( void )
{
	if (s_invFrequency == 0)
	{
		mach_timebase_info_data_t timebase;
		mach_timebase_info( &timebase );

		// convert to ns then to ms
		s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
	}

	uint64_t start = mach_absolute_time();
	b2Timer timer = { start };
	return timer;
}

float b2GetMilliseconds( const b2Timer* timer )
{
	uint64_t count = mach_absolute_time();
	float ms = (float)( s_invFrequency * ( count - timer->start ) );
	return ms;
}

float b2GetMillisecondsAndReset( b2Timer* timer )
{
	uint64_t count = mach_absolute_time();
	float ms = (float)( s_invFrequency * ( count - timer->start ) );
	timer->start = count;
	return ms;
}

void b2SleepMilliseconds( int milliseconds )
{
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = ( milliseconds % 1000 ) * 1000000;
	nanosleep( &ts, NULL );
}

void b2Yield()
{
	sched_yield();
}

#else

b2Timer b2CreateTimer( void )
{
	b2Timer timer = { 0 };
	return timer;
}

float b2GetMilliseconds( const b2Timer* timer )
{
	( (void)( timer ) );
	return 0.0f;
}

float b2GetMillisecondsAndReset( b2Timer* timer )
{
	( (void)( timer ) );
	return 0.0f;
}

void b2SleepMilliseconds( int milliseconds )
{
	( (void)( milliseconds ) );
}

void b2Yield()
{
}

#endif

// djb2 hash
// https://en.wikipedia.org/wiki/List_of_hash_functions
uint32_t b2Hash( uint32_t hash, const uint8_t* data, int count )
{
	uint32_t result = hash;
	for ( int i = 0; i < count; i++ )
	{
		result = ( result << 5 ) + result + data[i];
	}

	return result;
}
