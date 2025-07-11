// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/base.h"

#include <stddef.h>

#if defined( _MSC_VER )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>

static double s_invFrequency = 0.0;

uint64_t b2GetTicks( void )
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter( &counter );
	return (uint64_t)counter.QuadPart;
}

float b2GetMilliseconds( uint64_t ticks )
{
	if ( s_invFrequency == 0.0 )
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );

		s_invFrequency = (double)frequency.QuadPart;
		if ( s_invFrequency > 0.0 )
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	uint64_t ticksNow = b2GetTicks();
	return (float)( s_invFrequency * ( ticksNow - ticks ) );
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	if ( s_invFrequency == 0.0 )
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );

		s_invFrequency = (double)frequency.QuadPart;
		if ( s_invFrequency > 0.0 )
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	uint64_t ticksNow = b2GetTicks();
	float ms = (float)( s_invFrequency * ( ticksNow - *ticks ) );
	*ticks = ticksNow;
	return ms;
}

void b2Yield( void )
{
	SwitchToThread();
}

typedef struct b2Mutex
{
	CRITICAL_SECTION cs;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = b2Alloc( sizeof( b2Mutex ) );
	InitializeCriticalSection( &m->cs );
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	DeleteCriticalSection( &m->cs );
	*m = (b2Mutex){ 0 };
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	EnterCriticalSection( &m->cs );
}

void b2UnlockMutex( b2Mutex* m )
{
	LeaveCriticalSection( &m->cs );
}

#elif defined( __linux__ ) || defined( __EMSCRIPTEN__ )

#include <sched.h>
#include <time.h>

uint64_t b2GetTicks( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

float b2GetMilliseconds( uint64_t ticks )
{
	uint64_t ticksNow = b2GetTicks();
	return (float)( ( ticksNow - ticks ) / 1000000.0 );
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	uint64_t ticksNow = b2GetTicks();
	float ms = (float)( ( ticksNow - *ticks ) / 1000000.0 );
	*ticks = ticksNow;
	return ms;
}

void b2Yield( void )
{
	sched_yield();
}

#include <pthread.h>
typedef struct b2Mutex
{
	pthread_mutex_t mtx;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = b2Alloc( sizeof( b2Mutex ) );
	pthread_mutex_init( &m->mtx, NULL );
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	pthread_mutex_destroy( &m->mtx );
	*m = (b2Mutex){ 0 };
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	pthread_mutex_lock( &m->mtx );
}

void b2UnlockMutex( b2Mutex* m )
{
	pthread_mutex_unlock( &m->mtx );
}

#elif defined( __APPLE__ )

#include <mach/mach_time.h>
#include <sched.h>
#include <sys/time.h>

static double s_invFrequency = 0.0;

uint64_t b2GetTicks( void )
{
	return mach_absolute_time();
}

float b2GetMilliseconds( uint64_t ticks )
{
	if ( s_invFrequency == 0 )
	{
		mach_timebase_info_data_t timebase;
		mach_timebase_info( &timebase );

		// convert to ns then to ms
		s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
	}

	uint64_t ticksNow = b2GetTicks();
	return (float)( s_invFrequency * ( ticksNow - ticks ) );
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	if ( s_invFrequency == 0 )
	{
		mach_timebase_info_data_t timebase;
		mach_timebase_info( &timebase );

		// convert to ns then to ms
		s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
	}

	uint64_t ticksNow = b2GetTicks();
	float ms = (float)( s_invFrequency * ( ticksNow - *ticks ) );
	*ticks = ticksNow;
	return ms;
}

void b2Yield( void )
{
	sched_yield();
}

#include <pthread.h>
typedef struct b2Mutex
{
	pthread_mutex_t mtx;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = b2Alloc( sizeof( b2Mutex ) );
	pthread_mutex_init( &m->mtx, NULL );
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	pthread_mutex_destroy( &m->mtx );
	*m = (b2Mutex){ 0 };
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	pthread_mutex_lock( &m->mtx );
}

void b2UnlockMutex( b2Mutex* m )
{
	pthread_mutex_unlock( &m->mtx );
}

#else

uint64_t b2GetTicks( void )
{
	return 0;
}

float b2GetMilliseconds( uint64_t ticks )
{
	( (void)( ticks ) );
	return 0.0f;
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	( (void)( ticks ) );
	return 0.0f;
}

void b2Yield( void )
{
}

typedef struct b2Mutex
{
	int dummy;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = b2Alloc( sizeof( b2Mutex ) );
	m->dummy = 42;
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	*m = (b2Mutex){ 0 };
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	(void)m;
}

void b2UnlockMutex( b2Mutex* m )
{
	(void)m;
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
