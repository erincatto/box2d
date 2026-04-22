// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

#include <stdbool.h>

b2DeclareArray( b2StackEntry );

typedef struct b2StackEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b2StackEntry;

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B2_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
typedef struct b2Stack
{
	char* data;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b2Array( b2StackEntry ) entries;
} b2Stack;

b2Stack b2CreateStack( int capacity );
void b2DestroyStack( b2Stack* allocator );

void* b2StackAlloc( b2Stack* alloc, int size, const char* name );
void b2StackFree( b2Stack* alloc, void* mem );

// Grow the stack based on usage
void b2GrowStack( b2Stack* alloc );

int b2GetStackCapacity( b2Stack* alloc );
int b2GetStackAllocation( b2Stack* alloc );
int b2GetMaxStackAllocation( b2Stack* alloc );

