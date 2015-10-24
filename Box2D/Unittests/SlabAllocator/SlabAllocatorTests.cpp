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
#include "gtest/gtest.h"
#include "Box2D/Box2D.h"
#include "Box2D/Common/b2FreeList.h"
#include "Box2D/Common/b2IntrusiveList.h"
#include "Box2D/Common/b2SlabAllocator.h"

// Class used to test the slab allocator.
class SlabItem : public b2TypedIntrusiveListNode<SlabItem>
{
public:
	// Initialize the value associated with the item to 0.
	SlabItem() : m_value(0) { }
	// Remove the item (m_node) from a list on destruction.
	~SlabItem() { }

	// Set a value associated with the item.
	void SetValue(uint32 value) { m_value = value; }
	// Get a value associated with the item.
	uint32 GetValue() const { return m_value; }

	// Get the list node associated with this class.
	b2IntrusiveListNode* GetSlabItemListNode() { return &m_slabItemNode; }

private:
	// Integer associated with this item.
	uint32 m_value;
	// Link within a list.  This is used to infer whether the destructor of
	// SlabItem is called.
	b2IntrusiveListNode m_slabItemNode;
};

// Test b2SlabAllocator.
class SlabAllocatorTests : public ::testing::Test
{
protected:
	// Initialize the number of allocated blocks to 0 between test cases.
	virtual void SetUp()
	{
		b2SetNumAllocs(0);
	}

	// Verify all memory is free at the end of a test case.
	virtual void TearDown()
	{
		EXPECT_EQ(0, b2GetNumAllocs());
	}
};

// Test setting and getting the number of items per slab.
TEST_F(SlabAllocatorTests, SetGetItemsPerSlab)
{
	b2SlabAllocator<SlabItem> allocator(1);
	EXPECT_EQ(1U, allocator.GetItemsPerSlab());
	allocator.SetItemsPerSlab(2);
	EXPECT_EQ(2U, allocator.GetItemsPerSlab());
}

// Try to allocate an item with allocation disabled.
TEST_F(SlabAllocatorTests, AllocateFail)
{
	b2SlabAllocator<SlabItem> allocator(0);
	EXPECT_TRUE(NULL == allocator.Allocate());
}

// Allocate an item and verify that it's deallocated when the allocator
// is destroyed.
TEST_F(SlabAllocatorTests, AllocateFreeOnDestruction)
{
	EXPECT_EQ(0, b2GetNumAllocs());
	{
		b2SlabAllocator<SlabItem> allocator(1);
		SlabItem *item = allocator.Allocate();
		ASSERT_TRUE(item != NULL);
		item->SetValue(0xabadcafe);
		EXPECT_EQ(1, b2GetNumAllocs());
	}
	EXPECT_EQ(0, b2GetNumAllocs());
}

// Allocate an item and verify that it's destroyed when the allocated is
// destroyed.
TEST_F(SlabAllocatorTests, DestroyItemOnAllocatorDestruction)
{
	b2IntrusiveListNode list;
	{
		b2SlabAllocator<SlabItem> allocator(1);
		SlabItem *item = allocator.Allocate();
		ASSERT_TRUE(item != NULL);
		item->SetValue(0xabadcafe);
		list.InsertBefore(item->GetSlabItemListNode());
		EXPECT_EQ(1U, list.GetLength());
	}
	EXPECT_EQ(0U, list.GetLength());
}

// Allocate an item and free it.
TEST_F(SlabAllocatorTests, AllocateFree)
{
	{
		EXPECT_EQ(0, b2GetNumAllocs());
		b2SlabAllocator<SlabItem> allocator(1);
		SlabItem *item = allocator.Allocate();
		ASSERT_TRUE(item != NULL);
		item->SetValue(0xabadcafe);
		EXPECT_EQ(1, b2GetNumAllocs());
		allocator.Free(item);
		// Deallocation can be lazy, so in this case the allocator must be
		// destroyed before verifying all allocated blocks have been
		// deallocated.
	}
	EXPECT_GE(0, b2GetNumAllocs());
}

// Allocate two items, free one and then force the clean up of empty slabs.
TEST_F(SlabAllocatorTests, AllocateFreeEmptySlabs)
{
	EXPECT_EQ(0, b2GetNumAllocs());
	b2SlabAllocator<SlabItem> allocator(1);
	SlabItem *item1 = allocator.Allocate();
	SlabItem *item2 = allocator.Allocate();
	ASSERT_TRUE(item1 != NULL);
	ASSERT_TRUE(item2 != NULL);
	item1->SetValue(0xabadcafe);
	item2->SetValue(0xdeadbeef);
	EXPECT_EQ(2, b2GetNumAllocs());
	allocator.Free(item1);
	allocator.FreeEmptySlabs();
	EXPECT_EQ(1, b2GetNumAllocs());
}

// Test manual slab allocation.
TEST_F(SlabAllocatorTests, ManualAllocateSlab)
{
	b2SlabAllocator<SlabItem> allocator(2);
	// Allocate a slab with two items.
	EXPECT_TRUE(allocator.AllocateSlab());
	allocator.SetItemsPerSlab(0);
	// Allocate all items from the slab.
	SlabItem *item1 = allocator.Allocate();
	SlabItem *item2 = allocator.Allocate();
	// This allocation will fail because the slab only contains two items.
	SlabItem *item3 = allocator.Allocate();
	EXPECT_TRUE(item1 != NULL);
	EXPECT_TRUE(item2 != NULL);
	EXPECT_TRUE(item3 == NULL);
	allocator.Free(item1);
	allocator.Free(item2);
}

// Test manually freeing all allocated slabs.
TEST_F(SlabAllocatorTests, FreeAllSlabs)
{
	b2IntrusiveListNode list;
	b2SlabAllocator<SlabItem> allocator(1);
	SlabItem *item1 = allocator.Allocate();
	SlabItem *item2 = allocator.Allocate();
	list.InsertBefore(item1->GetSlabItemListNode());
	list.InsertBefore(item2->GetSlabItemListNode());
	EXPECT_EQ(2U, list.GetLength());
	allocator.FreeAllSlabs();
	EXPECT_EQ(0U, list.GetLength());
}

// Get the freelist from the slab allocator.
TEST_F(SlabAllocatorTests, GetFreeList)
{
	b2SlabAllocator<SlabItem> allocator(2);
	const b2FreeList* const freeList =
		allocator.GetFreeList().GetFreeList();
	EXPECT_EQ(0U, freeList->GetAllocatedList().GetLength());
	EXPECT_EQ(0U, freeList->GetFreeList().GetLength());
	SlabItem *item = allocator.Allocate();
	ASSERT_TRUE(item != NULL);
	EXPECT_EQ(1U, freeList->GetAllocatedList().GetLength());
	EXPECT_EQ(1U, freeList->GetFreeList().GetLength());
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
