// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "parallel_for.h"

#include "core.h"
#include "physics_world.h"
#include "platform.h"
#include "solver.h"

#include "box2d/base.h"
#include "box2d/constants.h"

#include <stddef.h>

static void b2RunStepTask( void* taskContext )
{
	b2StepTask* task = taskContext;
	b2StepContext* stepContext = task->stepContext;

	task->fcn( task->context );

	if ( task->deferred )
	{
		if ( b2AtomicFetchAddInt( &stepContext->treePending, -1 ) == 1 && stepContext->signalSemaphores )
		{
			b2SignalSemaphore( stepContext->treeSemaphore );
		}
	}
	else
	{
		if ( b2AtomicFetchAddInt( &stepContext->pendingCount, -1 ) == 1 && stepContext->signalSemaphores )
		{
			b2SignalSemaphore( stepContext->batchSemaphore );
		}
	}
}

void b2ResetTaskBatch( b2StepContext* stepContext )
{
	stepContext->batchStart = 1;
	stepContext->batchCount = 0;
	stepContext->joinTree = false;
}

void b2AddTask( b2StepContext* stepContext, b2TaskCallbackFcn* fcn, void* context )
{
	int index = stepContext->batchStart + stepContext->batchCount;
	B2_ASSERT( 1 <= index && index < B2_STEP_TASK_CAPACITY );

	b2StepTask* stepTask = stepContext->stepTasks + index;
	stepTask->fcn = fcn;
	stepTask->context = context;
	stepTask->stepContext = stepContext;
	stepTask->deferred = false;

	stepContext->tasks[index] = (b2Task){
		.fcn = b2RunStepTask,
		.context = stepTask,
		.deferred = false,
	};

	stepContext->batchCount += 1;
	stepContext->taskCount += 1;
}

void b2StageDeferredTask( b2StepContext* stepContext, b2TaskCallbackFcn* fcn, void* context )
{
	B2_ASSERT( stepContext->treeStaged == false );

	b2StepTask* stepTask = stepContext->stepTasks + 0;
	stepTask->fcn = fcn;
	stepTask->context = context;
	stepTask->stepContext = stepContext;
	stepTask->deferred = true;

	stepContext->tasks[0] = (b2Task){
		.fcn = b2RunStepTask,
		.context = stepTask,
		.deferred = true,
	};

	stepContext->treeStaged = true;
	stepContext->taskCount += 1;
	b2AtomicStoreInt( &stepContext->treePending, 1 );
}

static void b2ParallelForTrampoline( void* taskContext )
{
	b2ParallelForTask* task = taskContext;
	b2ParallelForShared* shared = task->shared;
	int workerIndex = task->workerIndex;
	void* context = shared->context;
	b2ParallelForCallback* callback = shared->callback;

	int blockCount = shared->blockCount;
	int blockSize = shared->blockSize;
	int itemCount = shared->itemCount;

	for ( ;; )
	{
		int blockIndex = b2AtomicFetchAddInt( &shared->nextBlock, 1 );
		if ( blockIndex >= blockCount )
		{
			break;
		}

		int start = blockIndex * blockSize;
		int end = start + blockSize;
		if ( end > itemCount )
		{
			end = itemCount;
		}

		callback( start, end, workerIndex, context );
	}
}

void b2StageParallelFor( b2World* world, b2ParallelForCallback* callback, int itemCount, int minRange, void* context )
{
	if ( itemCount <= 0 )
	{
		return;
	}

	B2_ASSERT( minRange > 0 );

	int workerCount = world->workerCount;
	B2_ASSERT( 0 < workerCount && workerCount <= B2_MAX_WORKERS );

	// Target multiple blocks per worker to reduce thread stalls.
	// block size grows once items exceed maxBlockCount * minRange
	// so the block count stays bounded and per-block sync overhead stays low.
	int blocksPerWorker = 8;
	int maxBlockCount = blocksPerWorker * workerCount;

	int blockSize;
	int blockCount;
	if ( itemCount <= minRange * maxBlockCount )
	{
		blockSize = minRange;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	else
	{
		blockSize = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	B2_ASSERT( blockCount >= 1 );
	B2_ASSERT( blockSize * blockCount >= itemCount );

	// No point enqueueing more tasks than blocks.
	int taskCount = workerCount < blockCount ? workerCount : blockCount;

	b2StepContext* stepContext = world->stepContext;
	b2ParallelForShared* shared = &stepContext->parallelFor;
	shared->blockCount = blockCount;
	shared->blockSize = blockSize;
	shared->itemCount = itemCount;
	shared->callback = callback;
	shared->context = context;
	b2AtomicStoreInt( &shared->nextBlock, 0 );

	for ( int i = 0; i < taskCount; ++i )
	{
		b2ParallelForTask* task = stepContext->parallelForTasks + i;
		task->shared = shared;
		task->workerIndex = i;
		b2AddTask( stepContext, b2ParallelForTrampoline, task );
	}
}
