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
#include <list>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "gtest/gtest.h"
#include "Box2D/Box2D.h"
#include "Box2D/Common/b2IntrusiveList.h"
#include "Box2D/Common/b2TrackedBlock.h"

static const uint32 kAllocationSize = 64;

// Test b2TrackedBlock.
class TrackedBlockTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
		// Reset the allocation counter.
		b2SetNumAllocs(0);
	}

	virtual void TearDown()
	{
		// Verify all memory is deallocated.
		EXPECT_EQ(b2GetNumAllocs(), 0);
	}
};


// Test b2TrackedBlockAllocator.
class TrackedBlockAllocatorTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
		// Reset the allocation counter.
		b2SetNumAllocs(0);
	}

	virtual void TearDown()
	{
		// Verify all memory is deallocated.
		EXPECT_EQ(b2GetNumAllocs(), 0);
	}
};

// Allocate and free a block.
TEST_F(TrackedBlockTests, AllocateFree)
{
	EXPECT_EQ(b2GetNumAllocs(), 0);
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	EXPECT_EQ(b2GetNumAllocs(), 1);
	b2TrackedBlock::Free(mem);
	EXPECT_EQ(b2GetNumAllocs(), 0);
}

// Allocate a block, verify the alignment of the returned address and free it.
TEST_F(TrackedBlockTests, AllocateCheckAlignment)
{
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	EXPECT_EQ(0U, (uintptr_t)mem & (b2_mallocAlignment - 1));
	b2TrackedBlock::Free(mem);
}

// Allocate a block, clobber the user data and verify that it's possible to
// free the block afterwards.
TEST_F(TrackedBlockTests, AllocateClobber)
{
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	memset(mem, 0xde, kAllocationSize);
	b2TrackedBlock::Free(mem);
}

// Allocate a block and retrieve the b2TrackedBlock pointer from it.
TEST_F(TrackedBlockTests, AllocateGetBlock)
{
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	b2TrackedBlock *block = b2TrackedBlock::GetFromMemory(mem);
	EXPECT_TRUE(block != NULL);
	b2TrackedBlock::Free(mem);
}

// Allocate a block, retrieve the b2TrackedBlock pointer from it and finally
// verify GetMemory() returns a pointer to the original memory block.
TEST_F(TrackedBlockTests, AllocateGetMemory)
{
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	b2TrackedBlock *block = b2TrackedBlock::GetFromMemory(mem);
	ASSERT_TRUE(block != NULL);
	ASSERT_EQ(block->GetMemory(), mem);
	b2TrackedBlock::Free(mem);
}

// Allocate a block, retrieve the b2TrackedBlock pointer and free the
// block using b2TrackedBlock.
TEST_F(TrackedBlockTests, AllocateFreeTrackedBlock)
{
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	b2TrackedBlock *block = b2TrackedBlock::GetFromMemory(mem);
	ASSERT_TRUE(block != NULL);
	b2TrackedBlock::Free(block);
}

// Allocate a block, add to a list, free the block and verify it's removed
// from the list.
TEST_F(TrackedBlockTests, AllocateFreeRemoveFromList)
{
	b2TypedIntrusiveListNode<b2TrackedBlock> list;
	void *mem = b2TrackedBlock::Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	b2TrackedBlock *block = b2TrackedBlock::GetFromMemory(mem);
	ASSERT_TRUE(block != NULL);
	list.InsertBefore(block);
	EXPECT_EQ(block, list.GetNext());
	b2TrackedBlock::Free(mem);
	EXPECT_TRUE(list.IsEmpty());
}

// Allocate a block using a tracked block allocator.
TEST_F(TrackedBlockAllocatorTests, AllocateFree)
{
	b2TrackedBlockAllocator allocator;
	void *mem = allocator.Allocate(kAllocationSize);
	ASSERT_TRUE(mem != NULL);
	memset(mem, 0xde, kAllocationSize);
	allocator.Free(mem);
}

// Allocate 10 blocks using a tracked block allocator.
TEST_F(TrackedBlockAllocatorTests, AllocateFreeAll)
{
	b2TrackedBlockAllocator allocator;
	for (uint32 i = 0; i < 10; ++i)
	{
		void *mem = allocator.Allocate(kAllocationSize);
		ASSERT_TRUE(mem != NULL);
		memset(mem, 0xde, kAllocationSize);
	}
	allocator.FreeAll();
	EXPECT_EQ(b2GetNumAllocs(), 0);
}

// Allocate a block and verify the tracked block allocator frees it on
// destruction.
TEST_F(TrackedBlockAllocatorTests, AllocateFreeOnDestruction)
{
	{
		b2TrackedBlockAllocator allocator;
		void *mem = allocator.Allocate(kAllocationSize);
		ASSERT_TRUE(mem != NULL);
	}
	EXPECT_EQ(b2GetNumAllocs(), 0);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
