// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef struct b2World b2World;

// Callback invoked by b2ParallelFor to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world->taskContexts.data + workerIndex).
typedef void b2ParallelForCallback( int startIndex, int endIndex, int workerIndex, void* context );

// Divide [0, itemCount) into blocks and process them with cooperative claiming:
// up to world->workerCount tasks are enqueued, and each task loops, atomically
// claiming the next unclaimed block until the range is drained. Blocks the
// caller until all work is complete. minRange is the minimum block size; block
// size grows once itemCount exceeds 4 * workerCount * minRange so block count
// stays bounded.
void b2ParallelFor( b2World* world, b2ParallelForCallback* callback, int itemCount, int minRange, void* context );
