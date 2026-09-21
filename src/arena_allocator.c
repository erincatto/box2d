// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "arena_allocator.h"

#include "core.h"

#include <stdbool.h>
#include <stddef.h>

b2Stack b2CreateStack( int capacity )
{
	B2_ASSERT( capacity >= 0 );
	b2Stack allocator = { 0 };
	allocator.capacity = capacity;
	allocator.data = b2Alloc( capacity );
	allocator.allocation = 0;
	allocator.maxAllocation = 0;
	allocator.index = 0;
	b2Array_CreateN( allocator.entries, 32 );
	return allocator;
}

void b2DestroyStack( b2Stack* allocator )
{
	b2Array_Destroy( allocator->entries );
	b2Free( allocator->data, allocator->capacity );
}

void* b2StackAlloc( b2Stack* alloc, int size, const char* name )
{
	int alignedSize = ( ( size - 1 ) | ( B2_ALIGNMENT - 1 ) ) + 1;

	b2StackEntry entry;
	entry.size = alignedSize;
	entry.name = name;
	if ( alloc->index + alignedSize > alloc->capacity )
	{
		// fall back to the heap (undesirable)
		entry.data = b2Alloc( alignedSize );
		entry.usedMalloc = true;

		B2_ASSERT( ( (uintptr_t)entry.data & ( B2_ALIGNMENT - 1 ) ) == 0 );
	}
	else
	{
		entry.data = alloc->data + alloc->index;
		entry.usedMalloc = false;
		alloc->index += alignedSize;

		B2_ASSERT( ( (uintptr_t)entry.data & ( B2_ALIGNMENT - 1 ) ) == 0 );
	}

	alloc->allocation += alignedSize;
	if ( alloc->allocation > alloc->maxAllocation )
	{
		alloc->maxAllocation = alloc->allocation;
	}

	b2Array_Push( alloc->entries, entry );
	return entry.data;
}

void b2StackFree( b2Stack* alloc, void* mem )
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
	b2Array_Pop( alloc->entries );
}

void b2GrowStack( b2Stack* alloc )
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

int b2GetStackCapacity( b2Stack* alloc )
{
	return alloc->capacity;
}

int b2GetStackAllocation( b2Stack* alloc )
{
	return alloc->allocation;
}

int b2GetMaxStackAllocation( b2Stack* alloc )
{
	return alloc->maxAllocation;
}
