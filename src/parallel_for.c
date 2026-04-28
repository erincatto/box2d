// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "parallel_for.h"

#include "atomic.h"
#include "core.h"
#include "physics_world.h"

#include "box2d/base.h"
#include "box2d/constants.h"

#include <stddef.h>

// Shared state for one b2ParallelFor invocation. Workers race on nextBlock to
// claim work, so a slow chunk can't strand the other threads.
typedef struct b2ParallelForShared
{
	b2AtomicInt nextBlock;
	int blockCount;
	int blockSize;
	int itemCount;
	b2ParallelForCallback* callback;
	void* context;
} b2ParallelForShared;

typedef struct b2ParallelForTask
{
	b2ParallelForShared* shared;
	int workerIndex;
} b2ParallelForTask;

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

void b2ParallelFor( b2World* world, b2ParallelForCallback* callback, int itemCount, int minRange, void* context )
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
	int blocksPerWorker = 4;
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

	b2ParallelForShared shared;
	shared.blockCount = blockCount;
	shared.blockSize = blockSize;
	shared.itemCount = itemCount;
	shared.callback = callback;
	shared.context = context;
	b2AtomicStoreInt( &shared.nextBlock, 0 );

	b2ParallelForTask tasks[B2_MAX_WORKERS];
	void* handles[B2_MAX_WORKERS];
	for ( int i = 0; i < taskCount; ++i )
	{
		tasks[i].shared = &shared;
		tasks[i].workerIndex = i;

		if (world->taskCount < B2_MAX_TASKS)
		{
			handles[i] = world->enqueueTaskFcn( &b2ParallelForTrampoline, tasks + i, world->userTaskContext );
			world->taskCount += 1;
		}
		else
		{
			handles[i] = NULL;
			b2ParallelForTrampoline( tasks + i );
		}
	}

	for ( int i = 0; i < taskCount; ++i )
	{
		if ( handles[i] != NULL )
		{
			world->finishTaskFcn( handles[i], world->userTaskContext );
		}
	}
}
