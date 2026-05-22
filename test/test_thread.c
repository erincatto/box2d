// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "test_macros.h"

// b2Semaphore tests

static int SemaphoreCreateDestroyTest( void )
{
	b2Semaphore* s = b2CreateSemaphore( 0 );
	ENSURE( s != NULL );
	b2DestroySemaphore( s );
	return 0;
}

typedef struct SemData
{
	b2Semaphore* sem;
	int value;
} SemData;

static void SemWorker( void* context )
{
	SemData* data = (SemData*)context;
	data->value = 99;
	b2SignalSemaphore( data->sem );
}

static int SemaphoreSignalWaitTest( void )
{
	SemData data = { 0 };
	data.sem = b2CreateSemaphore( 0 );
	data.value = 0;

	b2Thread* thread = b2CreateThread( SemWorker, &data, "sem test" );
	b2WaitSemaphore( data.sem );

	ENSURE( data.value == 99 );

	b2JoinThread( thread );
	b2DestroySemaphore( data.sem );
	return 0;
}

static int SemaphoreInitialCountTest( void )
{
	// Semaphore initialized to 3 should allow 3 waits without blocking
	b2Semaphore* s = b2CreateSemaphore( 3 );

	// These should not block since count starts at 3
	b2WaitSemaphore( s );
	b2WaitSemaphore( s );
	b2WaitSemaphore( s );

	// Signal once so the next wait doesn't block
	b2SignalSemaphore( s );
	b2WaitSemaphore( s );

	b2DestroySemaphore( s );
	return 0;
}

// b2Thread tests

static int ThreadCreateJoinTest( void )
{
	SemData data = { 0 };
	data.sem = b2CreateSemaphore( 0 );
	data.value = 0;

	b2Thread* thread = b2CreateThread( SemWorker, &data, "join test" );
	b2JoinThread( thread );

	ENSURE( data.value == 99 );

	b2DestroySemaphore( data.sem );
	return 0;
}

typedef struct SumData
{
	b2Mutex* mutex;
	int sum;
} SumData;

static void SumWorker( void* context )
{
	SumData* data = (SumData*)context;
	for ( int i = 0; i < 1000; ++i )
	{
		b2LockMutex( data->mutex );
		data->sum += 1;
		b2UnlockMutex( data->mutex );
	}
}

static int ThreadMultipleTest( void )
{
	SumData data = { 0 };
	data.mutex = b2CreateMutex();
	data.sum = 0;

	enum { THREAD_COUNT = 4 };
	b2Thread* threads[THREAD_COUNT];
	for ( int i = 0; i < THREAD_COUNT; ++i )
	{
		char name[16];
		snprintf( name, sizeof( name ), "sum test %d", i );
		threads[i] = b2CreateThread( SumWorker, &data, name );
	}

	for ( int i = 0; i < THREAD_COUNT; ++i )
	{
		b2JoinThread( threads[i] );
	}

	ENSURE( data.sum == THREAD_COUNT * 1000 );

	b2DestroyMutex( data.mutex );
	return 0;
}

int ThreadTest( void )
{
	RUN_SUBTEST( SemaphoreCreateDestroyTest );
	RUN_SUBTEST( SemaphoreSignalWaitTest );
	RUN_SUBTEST( SemaphoreInitialCountTest );
	RUN_SUBTEST( ThreadCreateJoinTest );
	RUN_SUBTEST( ThreadMultipleTest );
	return 0;
}
