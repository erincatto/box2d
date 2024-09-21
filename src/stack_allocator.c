// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "stack_allocator.h"

#include "array.h"
#include "core.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct b2StackEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b2StackEntry;

B2_ARRAY_INLINE( b2StackEntry, b2StackEntry );
B2_ARRAY_SOURCE( b2StackEntry, b2StackEntry );

b2StackAllocator b2CreateStackAllocator( int capacity )
{
	B2_ASSERT( capacity >= 0 );
	b2StackAllocator allocator = { 0 };
	allocator.capacity = capacity;
	allocator.data = b2Alloc( capacity );
	allocator.allocation = 0;
	allocator.maxAllocation = 0;
	allocator.index = 0;
	allocator.entries = b2StackEntryArray_Create( 32 );
	return allocator;
}

void b2DestroyStackAllocator( b2StackAllocator* allocator )
{
	b2StackEntryArray_Destroy( &allocator->entries );
	b2Free( allocator->data, allocator->capacity );
}

void* b2AllocateStackItem( b2StackAllocator* alloc, int size, const char* name )
{
	// ensure allocation is 32 byte aligned to support 256-bit SIMD
	int size32 = ( ( size - 1 ) | 0x1F ) + 1;

	b2StackEntry entry;
	entry.size = size32;
	entry.name = name;
	if ( alloc->index + size32 > alloc->capacity )
	{
		// fall back to the heap (undesirable)
		entry.data = b2Alloc( size32 );
		entry.usedMalloc = true;

		B2_ASSERT( ( (uintptr_t)entry.data & 0x1F ) == 0 );
	}
	else
	{
		entry.data = alloc->data + alloc->index;
		entry.usedMalloc = false;
		alloc->index += size32;

		B2_ASSERT( ( (uintptr_t)entry.data & 0x1F ) == 0 );
	}

	alloc->allocation += size32;
	if ( alloc->allocation > alloc->maxAllocation )
	{
		alloc->maxAllocation = alloc->allocation;
	}

	b2StackEntryArray_Push( &alloc->entries, entry );
	return entry.data;
}

void b2FreeStackItem( b2StackAllocator* alloc, void* mem )
{
	int entryCount = alloc->entries.count;
	B2_ASSERT( entryCount > 0 );
	b2StackEntry* entry = alloc->entries.data + ( entryCount - 1 );
	B2_ASSERT( mem == entry->data );
	if ( entry->usedMalloc )
	{
		b2Free( mem, entry->size );
	}
	else
	{
		alloc->index -= entry->size;
	}
	alloc->allocation -= entry->size;
	b2StackEntryArray_Pop( &alloc->entries );
}

void b2GrowStack( b2StackAllocator* alloc )
{
	// Stack must not be in use
	B2_ASSERT( alloc->allocation == 0 );

	if ( alloc->maxAllocation > alloc->capacity )
	{
		b2Free( alloc->data, alloc->capacity );
		alloc->capacity = alloc->maxAllocation + alloc->maxAllocation / 2;
		alloc->data = b2Alloc( alloc->capacity );
	}
}

int b2GetStackCapacity( b2StackAllocator* alloc )
{
	return alloc->capacity;
}

int b2GetStackAllocation( b2StackAllocator* alloc )
{
	return alloc->allocation;
}

int b2GetMaxStackAllocation( b2StackAllocator* alloc )
{
	return alloc->maxAllocation;
}
