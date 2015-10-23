/*
* Copyright (c) 2014 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <Box2D/Common/b2TrackedBlock.h>
#include <stddef.h>
#include <stdint.h>
#include <new>

// Initialize this block with a reference to "this".
b2TrackedBlock::b2TrackedBlock()
{
	b2TrackedBlock** pointerToThis =
		(b2TrackedBlock**)((uint8*)GetMemory() - sizeof(b2TrackedBlock**));
	*pointerToThis = this;
}

/// Get the allocated memory associated with this block.
void* b2TrackedBlock::GetMemory() const
{
	// The size of data in this without padding.
	static const uint32 kSizeOfThisWithNoPadding =
		sizeof(*this) - sizeof(m_padding) + sizeof(b2TrackedBlock**);

	// Make sure b2_mallocAlignment is base2.
	b2Assert(((b2_mallocAlignment - 1) & b2_mallocAlignment) == 0);

	// Round the pointer following data in this to b2_mallocAlignment.
	uint8* const aligned = (uint8*)(
		((uintptr_t)this + kSizeOfThisWithNoPadding + b2_mallocAlignment - 1) &
		~((uintptr_t)b2_mallocAlignment - 1));
	// Verify offset doesn't overlap data in this.
	b2Assert((uintptr_t)aligned - (uintptr_t)this >= kSizeOfThisWithNoPadding);
	return aligned;
}

/// Allocate a b2TrackedBlock returning a pointer to memory of size
/// bytes that can be used by the caller.
void* b2TrackedBlock::Allocate(uint32 size)
{
	void* memory = (b2TrackedBlock*)b2Alloc(sizeof(b2TrackedBlock) +
											size);
	if (!memory)
	{
		return NULL;
	}
	return (new(memory) b2TrackedBlock)->GetMemory();
}

/// Get a b2TrackedBlock from a pointer to memory returned by
/// b2TrackedBlock::Allocate().
b2TrackedBlock* b2TrackedBlock::GetFromMemory(void *memory)
{
	uint8* const aligned = (uint8*)memory;
	b2Assert(memory);
	b2TrackedBlock **blockPtr = (b2TrackedBlock**)(aligned -
												   sizeof(b2TrackedBlock**));
	b2Assert(*blockPtr);
	return *blockPtr;
}

/// Free a block of memory returned by b2TrackedBlock::Allocate()
void b2TrackedBlock::Free(void *memory)
{
	Free(GetFromMemory(memory));
}

/// Free a b2TrackedBlock.
void b2TrackedBlock::Free(b2TrackedBlock *block)
{
	b2Assert(block);
	block->~b2TrackedBlock();
	b2Free(block);
}

/// Allocate a block of size bytes using b2TrackedBlock::Allocate().
void* b2TrackedBlockAllocator::Allocate(uint32 size)
{
	void *memory = b2TrackedBlock::Allocate(size);
	m_blocks.InsertBefore(b2TrackedBlock::GetFromMemory(memory));
	return memory;
}

/// Free a block returned by Allocate().
void b2TrackedBlockAllocator::Free(void *memory)
{
	b2TrackedBlock::Free(memory);
}

/// Free all allocated blocks.
void b2TrackedBlockAllocator::FreeAll()
{
	while (!m_blocks.IsEmpty())
	{
		b2TrackedBlock::Free(m_blocks.GetNext());
	}
}
