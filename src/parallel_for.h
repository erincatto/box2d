// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;
typedef void b2TaskCallbackFcn( void* taskContext );

// Callback invoked by a staged parallel-for to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world->taskContexts.data + workerIndex).
typedef void b2ParallelForCallback( int startIndex, int endIndex, int workerIndex, void* context );

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

void b2ResetTaskBatch( b2StepContext* stepContext );
void b2AddTask( b2StepContext* stepContext, b2TaskCallbackFcn* fcn, void* context );
void b2StageDeferredTask( b2StepContext* stepContext, b2TaskCallbackFcn* fcn, void* context );

// Divide [0, itemCount) into blocks and add up to world->workerCount tasks to the current
// batch. Each task loops, atomically claiming the next unclaimed block until the range is
// drained. minRange is the minimum block size; block size grows once itemCount exceeds
// 8 * workerCount * minRange so block count stays bounded.
void b2StageParallelFor( b2World* world, b2ParallelForCallback* callback, int itemCount, int minRange, void* context );
