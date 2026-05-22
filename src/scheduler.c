// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "atomic.h"
#include "core.h"
#include "scheduler.h"

#include "box2d/base.h"
#include "box2d/constants.h"

#include <stdio.h>
#include <string.h>

enum b2SchedulerTaskStatus
{
	b2_schedulerFree = 0,
	b2_schedulerPending = 1,
	b2_schedulerClaimed = 2,
	b2_schedulerComplete = 3,
};

typedef struct b2SchedulerTask
{
	b2TaskCallback* callback;
	void* taskContext;
	b2AtomicInt status;
} b2SchedulerTask;

typedef struct b2SchedulerWorkerContext
{
	struct b2Scheduler* scheduler;
	int threadIndex;
} b2SchedulerWorkerContext;

typedef struct b2Scheduler
{
	b2Thread* threads[B2_MAX_WORKERS];
	b2SchedulerWorkerContext workerContexts[B2_MAX_WORKERS];

	// total workers including main thread
	int workerCount;

	// threads created = workerCount - 1
	int threadCount;

	b2SchedulerTask tasks[B2_MAX_TASKS];
	b2AtomicInt nextSlot;

	b2Semaphore* taskSemaphore;
	b2AtomicInt shutdown;
} b2Scheduler;

// Try to claim and execute one pending task.
// Returns true if work was performed, false otherwise.
static bool b2SchedulerExecuteOne( b2Scheduler* scheduler )
{
	int taskCount = b2AtomicLoadInt( &scheduler->nextSlot );
	for ( int t = 0; t < taskCount; ++t )
	{
		b2SchedulerTask* task = scheduler->tasks + t;
		if ( b2AtomicLoadInt( &task->status ) != b2_schedulerPending )
		{
			continue;
		}

		if ( b2AtomicCompareExchangeInt( &task->status, b2_schedulerPending, b2_schedulerClaimed ) == false )
		{
			continue;
		}

		task->callback( task->taskContext );

		b2AtomicStoreInt( &task->status, b2_schedulerComplete );
		return true;
	}

	return false;
}

// Background worker thread entry point.
static void b2SchedulerWorkerMain( void* context )
{
	b2SchedulerWorkerContext* workerContext = context;
	b2Scheduler* scheduler = workerContext->scheduler;

	while ( true )
	{
		b2WaitSemaphore( scheduler->taskSemaphore );

		if ( b2AtomicLoadInt( &scheduler->shutdown ) != 0 )
		{
			break;
		}

		// Claim and execute all available work
		while ( b2SchedulerExecuteOne( scheduler ) )
		{
		}
	}
}

b2Scheduler* b2CreateScheduler( int workerCount )
{
	B2_ASSERT( 0 < workerCount && workerCount <= B2_MAX_WORKERS );

	b2Scheduler* scheduler = b2Alloc( sizeof( b2Scheduler ) );
	memset( scheduler, 0, sizeof( b2Scheduler ) );

	scheduler->workerCount = workerCount;
	int threadCount = workerCount - 1;
	scheduler->threadCount = threadCount;
	scheduler->taskSemaphore = b2CreateSemaphore( 0 );
	b2AtomicStoreInt( &scheduler->shutdown, 0 );
	b2AtomicStoreInt( &scheduler->nextSlot, 0 );

	// Background threads use indices 1..workerCount-1.
	// Main thread uses index 0.
	for ( int i = 0; i < threadCount; ++i )
	{
		scheduler->workerContexts[i].scheduler = scheduler;
		scheduler->workerContexts[i].threadIndex = i + 1;

		char name[16];
		snprintf( name, sizeof( name ), "box2d_worker_%02d", i + 1 );
		scheduler->threads[i] = b2CreateThread( b2SchedulerWorkerMain, scheduler->workerContexts + i, name );
	}

	return scheduler;
}

void b2DestroyScheduler( b2Scheduler* scheduler )
{
	b2AtomicStoreInt( &scheduler->shutdown, 1 );

	// Wake all background threads so they see the shutdown flag
	for ( int i = 0; i < scheduler->threadCount; ++i )
	{
		b2SignalSemaphore( scheduler->taskSemaphore );
	}

	for ( int i = 0; i < scheduler->threadCount; ++i )
	{
		b2JoinThread( scheduler->threads[i] );
		scheduler->threads[i] = NULL;
	}

	b2DestroySemaphore( scheduler->taskSemaphore );
	b2Free( scheduler, sizeof( b2Scheduler ) );
}

void b2ResetScheduler( b2Scheduler* scheduler )
{
	b2AtomicStoreInt( &scheduler->nextSlot, 0 );
}

void* b2SchedulerEnqueueTask( b2TaskCallback* task, void* taskContext, void* userContext )
{
	b2Scheduler* scheduler = userContext;

	int slot = b2AtomicFetchAddInt( &scheduler->nextSlot, 1 );
	B2_ASSERT( slot < B2_MAX_TASKS );

	b2SchedulerTask* schedulerTask = scheduler->tasks + slot;
	schedulerTask->callback = task;
	schedulerTask->taskContext = taskContext;

	// Memory fence: status must be published after callback and context are written
	b2AtomicStoreInt( &schedulerTask->status, b2_schedulerPending );

	// One wake per enqueue is enough: at most one worker picks up each task.
	b2SignalSemaphore( scheduler->taskSemaphore );

	return schedulerTask;
}

void b2SchedulerFinishTask( void* userTask, void* userContext )
{
	if ( userTask == NULL )
	{
		return;
	}

	b2Scheduler* scheduler = userContext;
	b2SchedulerTask* waitTask = userTask;

	// Main thread helps execute any available work while waiting for the
	// target task to complete. This keeps the main thread from idling when
	// background threads are busy on other tasks from the same phase.
	while ( b2AtomicLoadInt( &waitTask->status ) != b2_schedulerComplete )
	{
		if ( b2SchedulerExecuteOne( scheduler ) == false )
		{
			b2Yield();
		}
	}
}
