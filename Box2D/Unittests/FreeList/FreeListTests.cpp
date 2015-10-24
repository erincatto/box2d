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
#include <set>
#include <utility>
#include "gtest/gtest.h"
#include "Box2D/Box2D.h"
#include "Box2D/Common/b2FreeList.h"
#include "Box2D/Common/b2IntrusiveList.h"
#include "TestCommon.h"

// Test b2FreeList.
class FreeListTests : public ::testing::Test
{
protected:
	virtual void SetUp()
	{
		// Intentionally blank.
	}

	virtual void TearDown()
	{
		// Intentionally blank.
	}
};

// Test b2TypedFreeList
class TypedFreeListTests : public ::testing::Test
{
protected:
	virtual void SetUp()
	{
		// Intentionally blank.
	}

	virtual void TearDown()
	{
		// Intentionally blank.
	}
};

// Construct an empty free list.
TEST_F(FreeListTests, Empty)
{
	b2FreeList freeList;
	EXPECT_TRUE(freeList.GetAllocatedList().IsEmpty());
	EXPECT_TRUE(freeList.GetFreeList().IsEmpty());
}

// Add free items to the free list.
TEST_F(FreeListTests, AddToFreeList)
{
	b2FreeList freeList;
	b2IntrusiveListNode node;
	freeList.AddToFreeList(&node);
	EXPECT_TRUE(freeList.GetAllocatedList().IsEmpty());
	EXPECT_FALSE(freeList.GetFreeList().IsEmpty());
}

// Add an item to a free list and allocate it.
TEST_F(FreeListTests, Allocate)
{
	b2FreeList freeList;
	b2IntrusiveListNode node;
	freeList.AddToFreeList(&node);
	EXPECT_TRUE(freeList.GetAllocatedList().IsEmpty());
	EXPECT_FALSE(freeList.GetFreeList().IsEmpty());

	EXPECT_EQ(&node, freeList.Allocate());
	EXPECT_FALSE(freeList.GetAllocatedList().IsEmpty());
	EXPECT_TRUE(freeList.GetFreeList().IsEmpty());
}

// Add 1 item to a free list and try to allocate 2 items.
TEST_F(FreeListTests, AllocateFail)
{
	b2FreeList freeList;
	b2IntrusiveListNode node;
	freeList.AddToFreeList(&node);
	EXPECT_EQ(&node, freeList.Allocate());
	EXPECT_TRUE(NULL == freeList.Allocate());
}

// Add 1 item to a free list, try to allocate 2 items, free an item then retry
// allocation.
TEST_F(FreeListTests, AllocateFailFree)
{
	b2FreeList freeList;
	b2IntrusiveListNode node;
	freeList.AddToFreeList(&node);
	EXPECT_EQ(&node, freeList.Allocate());
	EXPECT_TRUE(NULL == freeList.Allocate());
	freeList.Free(&node);
	EXPECT_EQ(&node, freeList.Allocate());
}

// Add a set of items to a free list, allocate a couple and free them.
TEST_F(FreeListTests, AllocateAndFree)
{
	b2FreeList freeList;
	b2IntrusiveListNode nodes[10];
	b2IntrusiveListNode *allocated[B2_ARRAY_SIZE(nodes) / 2];
	// Add nodes to the free list.
	for (uint32 i = 0; i < B2_ARRAY_SIZE(nodes); ++i)
	{
		freeList.AddToFreeList(&nodes[i]);
	}

	// Allocate nodes.
	std::set<const b2IntrusiveListNode*> allocatedSet;
	for (uint32 i = 0; i < B2_ARRAY_SIZE(allocated); ++i)
	{
		allocated[i] = freeList.Allocate();
		EXPECT_TRUE(NULL != allocated[i]);
		// Make sure each allocated node is unique.
		std::pair<std::set<const b2IntrusiveListNode*>::iterator, bool> it =
			allocatedSet.insert(allocated[i]);
		EXPECT_TRUE(it.second);
	}

	// Verify the allocated nodes are tracked by the free list.
	const b2IntrusiveListNode& allocatedList = freeList.GetAllocatedList();
	const b2IntrusiveListNode* allocatedTerminator =
		allocatedList.GetTerminator();
	for (const b2IntrusiveListNode *allocatedNode = allocatedList.GetNext();
		 allocatedNode != allocatedTerminator;
		 allocatedNode = allocatedNode->GetNext())
	{
		EXPECT_EQ(1U, allocatedSet.count(allocatedNode));
	}

	EXPECT_EQ(B2_ARRAY_SIZE(nodes) - B2_ARRAY_SIZE(allocated),
				freeList.GetFreeList().GetLength());

	// Free nodes.
	for (uint32 i = 0; i < B2_ARRAY_SIZE(allocated); ++i)
	{
		freeList.Free(allocated[i]);
	}

	EXPECT_TRUE(freeList.GetAllocatedList().IsEmpty());
	EXPECT_FALSE(freeList.GetFreeList().IsEmpty());
}

// Get the freelist from a b2TypedFreeList.
TEST_F(TypedFreeListTests, GetFreeList)
{
	b2TypedFreeList<ListItem> typedFreeList;
	b2FreeList* freeList = typedFreeList.GetFreeList();
	EXPECT_TRUE(freeList != NULL);
	EXPECT_TRUE(freeList->GetFreeList().IsEmpty());
	EXPECT_TRUE(freeList->GetAllocatedList().IsEmpty());
}

// Allocate an item from an empty typed free list.
TEST_F(TypedFreeListTests, AllocateEmpty)
{
	b2TypedFreeList<ListItem> typedFreeList;
	EXPECT_TRUE(NULL == typedFreeList.Allocate());
}

// Add an item to a free list.
TEST_F(TypedFreeListTests, AddToFreeList)
{
	b2TypedFreeList<ListItem> typedFreeList;
	ListItem item("test");
	EXPECT_TRUE(typedFreeList.GetFreeList()->GetFreeList().IsEmpty());
	typedFreeList.AddToFreeList(&item);
	EXPECT_FALSE(typedFreeList.GetFreeList()->GetFreeList().IsEmpty());
}

// Allocate an item from a typed free list.
TEST_F(TypedFreeListTests, Allocate)
{
	b2TypedFreeList<ListItem> typedFreeList;
	ListItem item("test");
	typedFreeList.AddToFreeList(&item);
	EXPECT_EQ(&item, typedFreeList.Allocate());
}

// Allocate and free an item from a typed free list.
TEST_F(TypedFreeListTests, AllocateFree)
{
	b2TypedFreeList<ListItem> typedFreeList;
	ListItem item("test");
	typedFreeList.AddToFreeList(&item);
	EXPECT_EQ(&item, typedFreeList.Allocate());
	EXPECT_TRUE(NULL == typedFreeList.Allocate());
	typedFreeList.Free(&item);
	EXPECT_EQ(&item, typedFreeList.Allocate());
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
