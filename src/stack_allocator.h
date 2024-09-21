// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

B2_ARRAY_DECLARE( b2StackEntry, b2StackEntry );

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B2_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
typedef struct b2StackAllocator
{
	char* data;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b2StackEntryArray entries;
} b2StackAllocator;

b2StackAllocator b2CreateStackAllocator( int capacity );
void b2DestroyStackAllocator( b2StackAllocator* allocator );

void* b2AllocateStackItem( b2StackAllocator* alloc, int size, const char* name );
void b2FreeStackItem( b2StackAllocator* alloc, void* mem );

// Grow the stack based on usage
void b2GrowStack( b2StackAllocator* alloc );

int b2GetStackCapacity( b2StackAllocator* alloc );
int b2GetStackAllocation( b2StackAllocator* alloc );
int b2GetMaxStackAllocation( b2StackAllocator* alloc );
