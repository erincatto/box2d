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

#elif defined( __linux__ ) || defined( __APPLE__ ) || defined( __EMSCRIPTEN__ )

#include <sched.h>
#include <sys/time.h>
#include <time.h>

b2Timer b2CreateTimer( void )
{
	b2Timer timer;
	struct timeval t;
	gettimeofday( &t, 0 );
	timer.start_sec = t.tv_sec;
	timer.start_usec = t.tv_usec;
	return timer;
}

float b2GetMilliseconds( const b2Timer* timer )
{
	struct timeval t;
	gettimeofday( &t, 0 );
	time_t start_sec = timer->start_sec;
	suseconds_t start_usec = (suseconds_t)timer->start_usec;

	// http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
	if ( t.tv_usec < start_usec )
	{
		int nsec = ( start_usec - t.tv_usec ) / 1000000 + 1;
		start_usec -= 1000000 * nsec;
		start_sec += nsec;
	}

	if ( t.tv_usec - start_usec > 1000000 )
	{
		int nsec = ( t.tv_usec - start_usec ) / 1000000;
		start_usec += 1000000 * nsec;
		start_sec -= nsec;
	}
	return 1000.0f * ( t.tv_sec - start_sec ) + 0.001f * ( t.tv_usec - start_usec );
}

float b2GetMillisecondsAndReset( b2Timer* timer )
{
	struct timeval t;
	gettimeofday( &t, 0 );
	time_t start_sec = timer->start_sec;
	suseconds_t start_usec = (suseconds_t)timer->start_usec;

	// http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
	if ( t.tv_usec < start_usec )
	{
		int nsec = ( start_usec - t.tv_usec ) / 1000000 + 1;
		start_usec -= 1000000 * nsec;
		start_sec += nsec;
	}

	if ( t.tv_usec - start_usec > 1000000 )
	{
		int nsec = ( t.tv_usec - start_usec ) / 1000000;
		start_usec += 1000000 * nsec;
		start_sec -= nsec;
	}

	timer->start_sec = t.tv_sec;
	timer->start_usec = t.tv_usec;

	return 1000.0f * ( t.tv_sec - start_sec ) + 0.001f * ( t.tv_usec - start_usec );
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
