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
#include "Box2D/Common/b2IntrusiveList.h"
#include "TestCommon.h"

// Test b2IntrusiveListNode.
class IntrusiveListTests : public ::testing::Test
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

/// Object which stores an integer and can be placed in a list.
class IntegerItem : public b2TypedIntrusiveListNode<IntegerItem>
{
public:
	// Initialize the object with a value.
	IntegerItem(int32 value) : m_value(value) { }
	// Remove the object from the list.
	~IntegerItem() { }

	// Get the value associated with this object.
	int32 GetValue() const { return m_value; }

private:
	int32 m_value;
};

// Test b2TypedIntrusiveListNode.
class TypedIntrusiveListTests : public ::testing::Test
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

// Create an empty list and verify the next and previous nodes point to the
// terminator.
TEST_F(IntrusiveListTests, EmptyNextPrevAreTerminator)
{
	b2IntrusiveListNode list;
	const b2IntrusiveListNode* const terminator = list.GetTerminator();
	EXPECT_EQ(terminator, list.GetNext());
	EXPECT_EQ(terminator, list.GetPrevious());
}

// Create an empty list and test IsEmpty().
TEST_F(IntrusiveListTests, EmptyIsEmpty)
{
	b2IntrusiveListNode list;
	EXPECT_TRUE(list.IsEmpty());
}

// Create an empty list and test InList().
TEST_F(IntrusiveListTests, EmptyNotInList)
{
	b2IntrusiveListNode list;
	EXPECT_FALSE(list.InList());
}

// Create a list with one node and test IsEmpty().
TEST_F(IntrusiveListTests, OneElementIsEmpty)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode node;
	list.InsertAfter(&node);
	EXPECT_FALSE(list.IsEmpty());
}

// Create a list with one node and verify InList() returns that the node is
// present in a list.
TEST_F(IntrusiveListTests, OneElementInList)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode node;
	list.InsertAfter(&node);
	EXPECT_TRUE(node.InList());
}

// Add a node to the start of a list and verify that it's possible to
// retrieve it.
TEST_F(IntrusiveListTests, InsertOneNodeAfter)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode node;
	list.InsertAfter(&node);
	EXPECT_EQ(list.GetNext(), &node);
	EXPECT_EQ(list.GetPrevious(), &node);
}

// Add a node to the end of a list and verify that it's possible to
// retrieve it.
TEST_F(IntrusiveListTests, InsertOneNodeBefore)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode node;
	list.InsertBefore(&node);
	EXPECT_EQ(list.GetNext(), &node);
	EXPECT_EQ(list.GetPrevious(), &node);
}

// Add two nodes to the start of a list and verify it's possible to retrieve
// them in the correct order.
TEST_F(IntrusiveListTests, InsertAfter)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode a, b;
	list.InsertAfter(&b);
	list.InsertAfter(&a);
	EXPECT_EQ(list.GetNext(), &a);
	EXPECT_EQ(list.GetNext()->GetNext(), &b);
	EXPECT_EQ(list.GetPrevious(), &b);
	EXPECT_EQ(list.GetPrevious()->GetPrevious(), &a);
}

// Add two nodes to the end of a list and verify it's possible to retrieve
// them in the correct order.
TEST_F(IntrusiveListTests, InsertBefore)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode a, b;
	list.InsertBefore(&a);
	list.InsertBefore(&b);
	EXPECT_EQ(list.GetNext(), &a);
	EXPECT_EQ(list.GetNext()->GetNext(), &b);
	EXPECT_EQ(list.GetPrevious(), &b);
	EXPECT_EQ(list.GetPrevious()->GetPrevious(), &a);
}

// Insert a node and then remove it from a list.
TEST_F(IntrusiveListTests, InsertRemove)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode node;
	list.InsertAfter(&node);
	EXPECT_EQ(list.GetNext(), &node);
	node.Remove();
	EXPECT_EQ(list.GetNext(), list.GetTerminator());
}

// Make sure it's possible to find a node in a list.
TEST_F(IntrusiveListTests, FindInList)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode nodeInList;
	b2IntrusiveListNode nodeNotInList;
	list.InsertAfter(&nodeInList);
	EXPECT_TRUE(list.FindNodeInList(&nodeInList));
	EXPECT_FALSE(list.FindNodeInList(&nodeNotInList));
}

// Ensure a list node is removed from a list when the node's destructor is
// called.
TEST_F(IntrusiveListTests, RemoveOnDestruction)
{
	b2IntrusiveListNode list;
	{
		b2IntrusiveListNode node;
		list.InsertAfter(&node);
		EXPECT_EQ(list.GetNext(), &node);
	}
	EXPECT_EQ(list.GetNext(), list.GetTerminator());
}

// Verify GetLength() returns 0 when the list is empty.
TEST_F(IntrusiveListTests, EmptyGetLength)
{
	b2IntrusiveListNode list;
	EXPECT_EQ(0U, list.GetLength());
}

// Verify GetLength() returns 10 when 10 nodes are present.
TEST_F(IntrusiveListTests, NotEmptyGetLength)
{
	b2IntrusiveListNode list;
	b2IntrusiveListNode nodes[10];
	for (uint32 i = 0; i < B2_ARRAY_SIZE(nodes); ++i)
	{
		list.InsertAfter(&nodes[i]);
	}
	EXPECT_EQ(B2_ARRAY_SIZE(nodes), list.GetLength());
}


// Insert ListItem instances into a list and iterate through them.
TEST_F(IntrusiveListTests, InsertListItemIterate)
{
	static const char *k_expected[] =
	{
		"this", "is", "a", "test",
	};
	b2IntrusiveListNode list;
	ListItem a(k_expected[0]);
	ListItem b(k_expected[1]);
	ListItem c(k_expected[2]);
	ListItem d(k_expected[3]);
	list.InsertBefore(a.GetListNode());
	list.InsertBefore(b.GetListNode());
	list.InsertBefore(c.GetListNode());
	list.InsertBefore(d.GetListNode());
	uint32 checked = 0;
	for (const b2IntrusiveListNode *node = list.GetNext();
		 node != list.GetTerminator(); node = node->GetNext(), ++checked)
	{
		ASSERT_LT(checked, B2_ARRAY_SIZE(k_expected));
		EXPECT_EQ(k_expected[checked],
					ListItem::GetInstanceFromListNode(node)->GetValue());
	}
	EXPECT_EQ(checked, B2_ARRAY_SIZE(k_expected));
}

// Verify first and last nodes of an empty list.
TEST_F(TypedIntrusiveListTests, Empty)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	EXPECT_TRUE(list.GetTerminator() == list.GetNext());
	EXPECT_TRUE(list.GetTerminator() == list.GetPrevious());
}

// Verify an empty list reports itself as empty.
TEST_F(TypedIntrusiveListTests, IsEmpty)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	EXPECT_TRUE(list.IsEmpty());
}

// Insert items into the start of a typed list and verify that it's possible
// to retrieve them
TEST_F(TypedIntrusiveListTests, InsertAfter)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	IntegerItem a(1);
	IntegerItem b(2);
	list.InsertAfter(&a);
	list.InsertAfter(&b);
	EXPECT_EQ(&b, list.GetNext());
	EXPECT_EQ(&a, list.GetPrevious());
}

// Insert items into the end of a typed list and verify that it's possible
// to retrieve them
TEST_F(TypedIntrusiveListTests, InsertBefore)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	IntegerItem a(1);
	IntegerItem b(2);
	list.InsertBefore(&a);
	list.InsertBefore(&b);
	EXPECT_EQ(&a, list.GetNext());
	EXPECT_EQ(&b, list.GetPrevious());
}

// Verify a list with an item isn't reported as empty.
TEST_F(TypedIntrusiveListTests, NotIsEmpty)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	IntegerItem item(1);
	list.InsertAfter(&item);
	EXPECT_FALSE(list.IsEmpty());
}

// Verify that it's possible to add and remove an item to a typed list.
TEST_F(TypedIntrusiveListTests, AddRemove)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	IntegerItem item(1);
	EXPECT_FALSE(item.InList());
	list.InsertAfter(&item);
	EXPECT_TRUE(item.InList());
	EXPECT_EQ(&item, list.GetNext());
	item.Remove();
	EXPECT_FALSE(item.InList());
	EXPECT_TRUE(list.IsEmpty());
}

// Get the length of a list.
TEST_F(TypedIntrusiveListTests, GetLength)
{
	b2TypedIntrusiveListNode<IntegerItem> list;
	IntegerItem a(1);
	IntegerItem b(2);
	list.InsertAfter(&a);
	list.InsertAfter(&b);
	EXPECT_EQ(2U, list.GetLength());
}

// Insert multiple items into a typed list and iterate through them.
TEST_F(TypedIntrusiveListTests, InsertListItemsIterate)
{
	static const int32 k_expected[] = {1, 2, 3, 4};
	IntegerItem a(1);
	IntegerItem b(2);
	IntegerItem c(3);
	IntegerItem d(4);
	b2TypedIntrusiveListNode<IntegerItem> list;
	list.InsertBefore(&a);
	list.InsertBefore(&b);
	list.InsertBefore(&c);
	list.InsertBefore(&d);
	uint32 checked = 0;
	for (IntegerItem* item = list.GetNext(); item != list.GetTerminator();
		 item = item->GetNext(), ++checked)
	{
		ASSERT_LT(checked, B2_ARRAY_SIZE(k_expected));
		EXPECT_EQ(k_expected[checked], item->GetValue());
	}
	EXPECT_EQ(checked, B2_ARRAY_SIZE(k_expected));
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
