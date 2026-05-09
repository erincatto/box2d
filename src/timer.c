// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// Required on Linux to expose pthread_setname_np. Must be defined before any
// system header is included.
#if defined( __linux__ ) && !defined( _GNU_SOURCE )
#define _GNU_SOURCE
#endif

#include "core.h"

#include "box2d/base.h"

#include <stddef.h>
#include <stdio.h>

#if defined( _WIN32 )

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
	char name[32];
} b2Thread;

typedef HRESULT( WINAPI* b2SetThreadDescriptionFn )( HANDLE, PCWSTR );

// SetThreadDescription exists on Windows 10 1607+. Resolve it dynamically so
// older Windows versions still link. Resolved once, cached for subsequent calls.
static void b2SetCurrentThreadName( const char* name )
{
	if ( name == NULL || name[0] == 0 )
	{
		return;
	}

	static b2SetThreadDescriptionFn pfn = NULL;
	static int resolved = 0;

	if ( resolved == 0 )
	{
		HMODULE kernel = GetModuleHandleW( L"kernel32.dll" );
		if ( kernel != NULL )
		{
			// MSVC /Wall warns C4191 and GCC/Clang -Wcast-function-type
			// warns on every FARPROC function-pointer cast. This is the
			// intended use of GetProcAddress, so suppress locally.
#if defined( __clang__ )
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-function-type"
#elif defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4191 )
#elif defined( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif
			pfn = (b2SetThreadDescriptionFn)GetProcAddress( kernel, "SetThreadDescription" );
#if defined( __clang__ )
#pragma clang diagnostic pop
#elif defined( _MSC_VER )
#pragma warning( pop )
#elif defined( __GNUC__ )
#pragma GCC diagnostic pop
#endif
		}
		resolved = 1;
	}

	if ( pfn == NULL )
	{
		return;
	}

	wchar_t wide[32];
	int n = MultiByteToWideChar( CP_UTF8, 0, name, -1, wide, (int)( sizeof( wide ) / sizeof( wide[0] ) ) );
	if ( n > 0 )
	{
		pfn( GetCurrentThread(), wide );
	}
}

static DWORD WINAPI b2ThreadStart( LPVOID param )
{
	b2Thread* t = (b2Thread*)param;
	b2SetCurrentThreadName( t->name );
	b2TracyCSetThreadName( t->name );
	t->function( t->context );
	return 0;
}

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context, const char* name )
{
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->function = function;
	t->context = context;
	if ( name != NULL )
	{
		snprintf( t->name, sizeof( t->name ), "%s", name );
	}
	else
	{
		t->name[0] = 0;
	}
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
	char name[32];
} b2Thread;

static void b2SetCurrentThreadName( const char* name )
{
	if ( name == NULL || name[0] == 0 )
	{
		return;
	}

#if defined( __linux__ )
	// Linux caps thread names at 15 chars + null terminator.
	char truncated[16];
	snprintf( truncated, sizeof( truncated ), "%s", name );
	pthread_setname_np( pthread_self(), truncated );
#else
	(void)name;
#endif
}

static void* b2ThreadStart( void* param )
{
	b2Thread* t = (b2Thread*)param;
	b2SetCurrentThreadName( t->name );
	b2TracyCSetThreadName( t->name );
	t->function( t->context );
	return NULL;
}

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context, const char* name )
{
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->function = function;
	t->context = context;
	if ( name != NULL )
	{
		snprintf( t->name, sizeof( t->name ), "%s", name );
	}
	else
	{
		t->name[0] = 0;
	}
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

#include <dispatch/dispatch.h>

typedef struct b2Semaphore
{
	dispatch_semaphore_t semaphore;
	int initialCount;
} b2Semaphore;

b2Semaphore* b2CreateSemaphore( int initCount )
{
	b2Semaphore* s = b2Alloc( sizeof( b2Semaphore ) );
	s->semaphore = dispatch_semaphore_create( (long)initCount );
	s->initialCount = initCount;
	return s;
}

void b2DestroySemaphore( b2Semaphore* s )
{
	// libdispatch aborts if the current count is less than the initial count at release time.
	// Pad with signals so the invariant always holds; no one is waiting at this point.
	for ( int i = 0; i < s->initialCount; ++i )
	{
		dispatch_semaphore_signal( s->semaphore );
	}
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
	char name[32];
} b2Thread;

// macOS pthread_setname_np takes only the name — it always names the calling thread.
static void b2SetCurrentThreadName( const char* name )
{
	if ( name == NULL || name[0] == 0 )
	{
		return;
	}
	pthread_setname_np( name );
}

static void* b2ThreadStart( void* param )
{
	b2Thread* t = (b2Thread*)param;
	b2SetCurrentThreadName( t->name );
	b2TracyCSetThreadName( t->name );
	t->function( t->context );
	return NULL;
}

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context, const char* name )
{
	b2Thread* t = b2Alloc( sizeof( b2Thread ) );
	t->function = function;
	t->context = context;
	if ( name != NULL )
	{
		snprintf( t->name, sizeof( t->name ), "%s", name );
	}
	else
	{
		t->name[0] = 0;
	}
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

b2Thread* b2CreateThread( b2ThreadFunction* function, void* context, const char* name )
{
	(void)name;
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
