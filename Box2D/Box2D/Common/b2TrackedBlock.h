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
#ifndef B2_TRACKED_BLOCK_H
#define B2_TRACKED_BLOCK_H

#include <Box2D/Common/b2IntrusiveList.h>
#include <Box2D/Common/b2Settings.h>

/// Alignment (in bytes) of user memory associated with b2TrackedBlock.
const int32 b2_mallocAlignment = 32;

/// Allocated block of memory that can be tracked in a b2IntrusiveList.
class b2TrackedBlock : public b2TypedIntrusiveListNode<b2TrackedBlock>
{
private:
	// Initialize this block with a reference to "this".
	b2TrackedBlock();
	// Remove the block from the list.
	~b2TrackedBlock() { }

public:
	/// Get the allocated memory associated with this block.
	void* GetMemory() const;

private:
	// Padding required to align the pointer to user memory in the block
	// to b2_mallocAlignment.
	uint8 m_padding[b2_mallocAlignment + sizeof(b2TrackedBlock**)];

public:
	/// Allocate a b2TrackedBlock returning a pointer to memory of size
	/// bytes that can be used by the caller.
	static void* Allocate(uint32 size);

	/// Get a b2TrackedBlock from a pointer to memory returned by
	/// b2TrackedBlock::Allocate().
	static b2TrackedBlock* GetFromMemory(void *memory);

	/// Free a block of memory returned by b2TrackedBlock::Allocate()
	static void Free(void *memory);

	/// Free a b2TrackedBlock.
	static void Free(b2TrackedBlock *block);
};

/// Allocator of blocks which are tracked in a list.
class b2TrackedBlockAllocator
{
public:
	/// Initialize.
	b2TrackedBlockAllocator() {}
	/// Free all allocated blocks.
	~b2TrackedBlockAllocator()
	{
		FreeAll();
	}

	/// Allocate a block of size bytes using b2TrackedBlock::Allocate().
	void* Allocate(uint32 size);

	/// Free a block returned by Allocate().
	void Free(void *memory);

	/// Free all allocated blocks.
	void FreeAll();

	// Get the list of allocated blocks.
	const b2TypedIntrusiveListNode<b2TrackedBlock>& GetList() const
	{
		return m_blocks;
	}

private:
	b2TypedIntrusiveListNode<b2TrackedBlock> m_blocks;
};

#endif  // B2_TRACKED_BLOCK_H
