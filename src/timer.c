// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/base.h"

#include <stddef.h>

#if defined( _MSC_VER )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <Windows.h>
#include <limits.h>

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

typedef struct b2ConditionVariable
{
	CONDITION_VARIABLE cv;
} b2ConditionVariable;

b2ConditionVariable* b2CreateConditionVariable( void )
{
	b2ConditionVariable* cv = b2Alloc( sizeof( b2ConditionVariable ) );
	InitializeConditionVariable( &cv->cv );
	return cv;
}

void b2DestroyConditionVariable( b2ConditionVariable* cv )
{
	*cv = (b2ConditionVariable){ 0 };
	b2Free( cv, sizeof( b2ConditionVariable ) );
}

void b2WaitConditionVariable( b2ConditionVariable* cv, b2Mutex* m )
{
	SleepConditionVariableCS( &cv->cv, &m->cs, INFINITE );
}

void b2SignalConditionVariable( b2ConditionVariable* cv )
{
	WakeConditionVariable( &cv->cv );
}

void b2BroadcastConditionVariable( b2ConditionVariable* cv )
{
	WakeAllConditionVariable( &cv->cv );
}

typedef struct b2Semaphore
{
	HANDLE semaphore;
} b2Semaphore;

b2Semaphore* b2CreateSemaphore( int initCount )
{
	b2Semaphore* s = b2Alloc( sizeof( b2Semaphore ) );
	s->semaphore = CreateSemaphoreExW( NULL, initCount, INT_MAX, NULL, 0, SEMAPHORE_ALL_ACCESS );
	return s;
}

void b2DestroySemaphore( b2Semaphore* s )
{
	CloseHandle( s->semaphore );
	*s = (b2Semaphore){ 0 };
	b2Free( s, sizeof( b2Semaphore ) );
}

void b2WaitSemaphore( b2Semaphore* s )
{
	WaitForSingleObjectEx( s->semaphore, INFINITE, FALSE );
}

void b2SignalSemaphore( b2Semaphore* s )
{
	ReleaseSemaphore( s->semaphore, 1, NULL );
}

typedef struct b2Thread
{
	HANDLE thread;
	b2ThreadFunction* function;
	void* context;
} b2Thread;

static DWORD WINAPI b2ThreadStart( LPVOID param )
{
	b2Thread* t = (b2Thread*)param;
	t->function( t->context );
	return 0;
}

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context )
{
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->function = function;
	t->context = context;
	t->thread = CreateThread( NULL, 0, b2ThreadStart, t, 0, NULL );
	return t;
}

void b2JoinThread( b2Thread* t )
{
	WaitForSingleObject( t->thread, INFINITE );
	CloseHandle( t->thread );
	*t = (b2Thread){ 0 };
	b2Free( t, sizeof( b2Thread ) );
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

typedef struct b2ConditionVariable
{
	pthread_cond_t cond;
} b2ConditionVariable;

b2ConditionVariable* b2CreateConditionVariable( void )
{
	b2ConditionVariable* cv = b2Alloc( sizeof( b2ConditionVariable ) );
	pthread_cond_init( &cv->cond, NULL );
	return cv;
}

void b2DestroyConditionVariable( b2ConditionVariable* cv )
{
	pthread_cond_destroy( &cv->cond );
	*cv = (b2ConditionVariable){ 0 };
	b2Free( cv, sizeof( b2ConditionVariable ) );
}

void b2WaitConditionVariable( b2ConditionVariable* cv, b2Mutex* m )
{
	pthread_cond_wait( &cv->cond, &m->mtx );
}

void b2SignalConditionVariable( b2ConditionVariable* cv )
{
	pthread_cond_signal( &cv->cond );
}

void b2BroadcastConditionVariable( b2ConditionVariable* cv )
{
	pthread_cond_broadcast( &cv->cond );
}

#include <semaphore.h>

typedef struct b2Semaphore
{
	sem_t semaphore;
} b2Semaphore;

b2Semaphore* b2CreateSemaphore( int initCount )
{
	b2Semaphore* s = b2Alloc( sizeof( b2Semaphore ) );
	sem_init( &s->semaphore, 0, (unsigned int)initCount );
	return s;
}

void b2DestroySemaphore( b2Semaphore* s )
{
	sem_destroy( &s->semaphore );
	*s = (b2Semaphore){ 0 };
	b2Free( s, sizeof( b2Semaphore ) );
}

void b2WaitSemaphore( b2Semaphore* s )
{
	sem_wait( &s->semaphore );
}

void b2SignalSemaphore( b2Semaphore* s )
{
	sem_post( &s->semaphore );
}

typedef struct b2Thread
{
	pthread_t thread;
	b2ThreadFunction* function;
	void* context;
} b2Thread;

static void* b2ThreadStart( void* param )
{
	b2Thread* t = (b2Thread*)param;
	t->function( t->context );
	return NULL;
}

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context )
{
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->function = function;
	t->context = context;
	pthread_create( &t->thread, NULL, b2ThreadStart, t );
	return t;
}

void b2JoinThread( b2Thread* t )
{
	pthread_join( t->thread, NULL );
	*t = (b2Thread){ 0 };
	b2Free( t, sizeof( b2Thread ) );
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

typedef struct b2ConditionVariable
{
	pthread_cond_t cond;
} b2ConditionVariable;

b2ConditionVariable* b2CreateConditionVariable( void )
{
	b2ConditionVariable* cv = b2Alloc( sizeof( b2ConditionVariable ) );
	pthread_cond_init( &cv->cond, NULL );
	return cv;
}

void b2DestroyConditionVariable( b2ConditionVariable* cv )
{
	pthread_cond_destroy( &cv->cond );
	*cv = (b2ConditionVariable){ 0 };
	b2Free( cv, sizeof( b2ConditionVariable ) );
}

void b2WaitConditionVariable( b2ConditionVariable* cv, b2Mutex* m )
{
	pthread_cond_wait( &cv->cond, &m->mtx );
}

void b2SignalConditionVariable( b2ConditionVariable* cv )
{
	pthread_cond_signal( &cv->cond );
}

void b2BroadcastConditionVariable( b2ConditionVariable* cv )
{
	pthread_cond_broadcast( &cv->cond );
}

#include <dispatch/dispatch.h>

typedef struct b2Semaphore
{
	dispatch_semaphore_t semaphore;
} b2Semaphore;

b2Semaphore* b2CreateSemaphore( int initCount )
{
	b2Semaphore* s = b2Alloc( sizeof( b2Semaphore ) );
	s->semaphore = dispatch_semaphore_create( (long)initCount );
	return s;
}

void b2DestroySemaphore( b2Semaphore* s )
{
	dispatch_release( s->semaphore );
	*s = (b2Semaphore){ 0 };
	b2Free( s, sizeof( b2Semaphore ) );
}

void b2WaitSemaphore( b2Semaphore* s )
{
	dispatch_semaphore_wait( s->semaphore, DISPATCH_TIME_FOREVER );
}

void b2SignalSemaphore( b2Semaphore* s )
{
	dispatch_semaphore_signal( s->semaphore );
}

typedef struct b2Thread
{
	pthread_t thread;
	b2ThreadFunction* function;
	void* context;
} b2Thread;

static void* b2ThreadStart( void* param )
{
	b2Thread* t = (b2Thread*)param;
	t->function( t->context );
	return NULL;
}

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context )
{
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->function = function;
	t->context = context;
	pthread_create( &t->thread, NULL, b2ThreadStart, t );
	return t;
}

void b2JoinThread( b2Thread* t )
{
	pthread_join( t->thread, NULL );
	*t = (b2Thread){ 0 };
	b2Free( t, sizeof( b2Thread ) );
}

#else

// Fallbacks for unknown platforms

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

typedef struct b2ConditionVariable
{
	int dummy;
} b2ConditionVariable;

b2ConditionVariable* b2CreateConditionVariable( void )
{
	b2ConditionVariable* cv = b2Alloc( sizeof( b2ConditionVariable ) );
	cv->dummy = 42;
	return cv;
}

void b2DestroyConditionVariable( b2ConditionVariable* cv )
{
	*cv = (b2ConditionVariable){ 0 };
	b2Free( cv, sizeof( b2ConditionVariable ) );
}

void b2WaitConditionVariable( b2ConditionVariable* cv, b2Mutex* m )
{
	(void)cv;
	(void)m;
}

void b2SignalConditionVariable( b2ConditionVariable* cv )
{
	(void)cv;
}

void b2BroadcastConditionVariable( b2ConditionVariable* cv )
{
	(void)cv;
}

typedef struct b2Semaphore
{
	int dummy;
} b2Semaphore;

b2Semaphore* b2CreateSemaphore( int initCount )
{
	b2Semaphore* s = b2Alloc( sizeof( b2Semaphore ) );
	(void)initCount;
	s->dummy = 42;
	return s;
}

void b2DestroySemaphore( b2Semaphore* s )
{
	*s = (b2Semaphore){ 0 };
	b2Free( s, sizeof( b2Semaphore ) );
}

void b2WaitSemaphore( b2Semaphore* s )
{
	(void)s;
}

void b2SignalSemaphore( b2Semaphore* s )
{
	(void)s;
}

typedef struct b2Thread
{
	int dummy;
} b2Thread;

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context )
{
	function( context );
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->dummy = 42;
	return t;
}

void b2JoinThread( b2Thread* t )
{
	*t = (b2Thread){ 0 };
	b2Free( t, sizeof( b2Thread ) );
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
