// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

B2_ARRAY_DECLARE( b2ArenaEntry, b2ArenaEntry );

typedef struct b2ArenaEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b2ArenaEntry;

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B2_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
typedef struct b2ArenaAllocator
{
	char* data;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b2ArenaEntryArray entries;
} b2ArenaAllocator;

b2ArenaAllocator b2CreateArenaAllocator( int capacity );
void b2DestroyArenaAllocator( b2ArenaAllocator* allocator );

void* b2AllocateArenaItem( b2ArenaAllocator* alloc, int size, const char* name );
void b2FreeArenaItem( b2ArenaAllocator* alloc, void* mem );

// Grow the arena based on usage
void b2GrowArena( b2ArenaAllocator* alloc );

int b2GetArenaCapacity( b2ArenaAllocator* alloc );
int b2GetArenaAllocation( b2ArenaAllocator* alloc );
int b2GetMaxArenaAllocation( b2ArenaAllocator* alloc );

B2_ARRAY_INLINE( b2ArenaEntry, b2ArenaEntry );
