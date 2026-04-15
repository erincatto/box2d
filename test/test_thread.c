// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "test_macros.h"

// b2ConditionVariable tests

static int ConditionVariableCreateDestroyTest( void )
{
	b2ConditionVariable* cv = b2CreateConditionVariable();
	ENSURE( cv != NULL );
	b2DestroyConditionVariable( cv );
	return 0;
}

typedef struct CondVarData
{
	b2Mutex* mutex;
	b2ConditionVariable* cv;
	int value;
	bool ready;
} CondVarData;

static void CondVarWorker( void* context )
{
	CondVarData* data = (CondVarData*)context;
	b2LockMutex( data->mutex );
	data->value = 42;
	data->ready = true;
	b2SignalConditionVariable( data->cv );
	b2UnlockMutex( data->mutex );
}

static int ConditionVariableSignalTest( void )
{
	CondVarData data = { 0 };
	data.mutex = b2CreateMutex();
	data.cv = b2CreateConditionVariable();
	data.value = 0;
	data.ready = false;

	b2Thread* thread = b2CreateThread( CondVarWorker, &data );

	b2LockMutex( data.mutex );
	while ( data.ready == false )
	{
		b2WaitConditionVariable( data.cv, data.mutex );
	}
	b2UnlockMutex( data.mutex );

	ENSURE( data.value == 42 );

	b2JoinThread( thread );
	b2DestroyConditionVariable( data.cv );
	b2DestroyMutex( data.mutex );
	return 0;
}

typedef struct BroadcastData
{
	b2Mutex* mutex;
	b2ConditionVariable* cv;
	bool go;
	int counter;
} BroadcastData;

static void BroadcastWorker( void* context )
{
	BroadcastData* data = (BroadcastData*)context;
	b2LockMutex( data->mutex );
	while ( data->go == false )
	{
		b2WaitConditionVariable( data->cv, data->mutex );
	}
	data->counter += 1;
	b2UnlockMutex( data->mutex );
}

static int ConditionVariableBroadcastTest( void )
{
	BroadcastData data = { 0 };
	data.mutex = b2CreateMutex();
	data.cv = b2CreateConditionVariable();
	data.go = false;
	data.counter = 0;

	enum { THREAD_COUNT = 4 };
	b2Thread* threads[THREAD_COUNT];
	for ( int i = 0; i < THREAD_COUNT; ++i )
	{
		threads[i] = b2CreateThread( BroadcastWorker, &data );
	}

	b2LockMutex( data.mutex );
	data.go = true;
	b2BroadcastConditionVariable( data.cv );
	b2UnlockMutex( data.mutex );

	for ( int i = 0; i < THREAD_COUNT; ++i )
	{
		b2JoinThread( threads[i] );
	}

	ENSURE( data.counter == THREAD_COUNT );

	b2DestroyConditionVariable( data.cv );
	b2DestroyMutex( data.mutex );
	return 0;
}

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

	b2Thread* thread = b2CreateThread( SemWorker, &data );
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

	b2Thread* thread = b2CreateThread( SemWorker, &data );
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
		threads[i] = b2CreateThread( SumWorker, &data );
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
	RUN_SUBTEST( ConditionVariableCreateDestroyTest );
	RUN_SUBTEST( ConditionVariableSignalTest );
	RUN_SUBTEST( ConditionVariableBroadcastTest );
	RUN_SUBTEST( SemaphoreCreateDestroyTest );
	RUN_SUBTEST( SemaphoreSignalWaitTest );
	RUN_SUBTEST( SemaphoreInitialCountTest );
	RUN_SUBTEST( ThreadCreateJoinTest );
	RUN_SUBTEST( ThreadMultipleTest );
	return 0;
}
