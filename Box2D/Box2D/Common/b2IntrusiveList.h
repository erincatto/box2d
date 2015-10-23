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
#ifndef B2_INTRUSIVE_LIST
#define B2_INTRUSIVE_LIST

#include <Box2D/Common/b2Settings.h>

// Whether to enable b2IntrusiveList::ValidateList().
// Be careful when enabling this since this changes the size of
// b2IntrusiveListNode so make sure *all* projects that include Box2D.h
// also define this value in the same way to avoid data corruption.
#ifndef B2_INTRUSIVE_LIST_VALIDATE
#define B2_INTRUSIVE_LIST_VALIDATE 0
#endif  // B2_INTRUSIVE_LIST_VALIDATE

/// b2IntrusiveListNode is used to implement an intrusive doubly-linked
/// list.
///
/// For example:
///
/// class MyClass {
/// public:
/// 	MyClass(const char *msg) : m_msg(msg) {}
/// 	const char* GetMessage() const { return m_msg; }
/// 	B2_INTRUSIVE_LIST_GET_NODE(m_node);
/// 	B2_INTRUSIVE_LIST_NODE_GET_CLASS(MyClass, m_node);
/// private:
/// 	b2IntrusiveListNode m_node;
/// 	const char *m_msg;
/// };
///
/// int main(int argc, char *argv[]) {
/// 	b2IntrusiveListNode list; // NOTE: type is NOT MyClass
/// 	MyClass a("this");
/// 	MyClass b("is");
/// 	MyClass c("a");
/// 	MyClass d("test");
/// 	list.InsertBefore(a.GetListNode());
/// 	list.InsertBefore(b.GetListNode());
/// 	list.InsertBefore(c.GetListNode());
/// 	list.InsertBefore(d.GetListNode());
/// 	for (b2IntrusiveListNode* node = list.GetNext();
/// 		 node != list.GetTerminator(); node = node->GetNext()) {
/// 		MyClass *cls = MyClass::GetInstanceFromListNode(node);
/// 		printf("%s\n", cls->GetMessage());
/// 	}
/// 	return 0;
/// }
class b2IntrusiveListNode
{
public:
	/// Initialize the node.
	b2IntrusiveListNode()
	{
		Initialize();
#if B2_INTRUSIVE_LIST_VALIDATE
		m_magic = k_magic;
#endif // B2_INTRUSIVE_LIST_VALIDATE
	}

	/// If the node is in a list, remove it from the list.
	~b2IntrusiveListNode()
	{
		Remove();
#if B2_INTRUSIVE_LIST_VALIDATE
		m_magic = 0;
#endif // B2_INTRUSIVE_LIST_VALIDATE
	}

	/// Insert this node after the specified node.
	void InsertAfter(b2IntrusiveListNode* const node)
	{
		b2Assert(!node->InList());
		node->m_next = m_next;
		node->m_prev = this;
		m_next->m_prev = node;
		m_next = node;
	}

	/// Insert this node before the specified node.
	void InsertBefore(b2IntrusiveListNode* const node)
	{
		b2Assert(!node->InList());
		node->m_next = this;
		node->m_prev = m_prev;
		m_prev->m_next = node;
		m_prev = node;
	}

	/// Get the terminator of the list.
	const b2IntrusiveListNode* GetTerminator() const
	{
		return this;
	}

	/// Remove this node from the list it's currently in.
	b2IntrusiveListNode* Remove()
	{
		m_prev->m_next = m_next;
		m_next->m_prev = m_prev;
		Initialize();
		return this;
	}

	/// Determine whether this list is empty or the node isn't in a list.
	bool IsEmpty() const
	{
	  return GetNext() == this;
	}

	/// Determine whether this node is in a list or the list contains nodes.
	bool InList() const
	{
	  return !IsEmpty();
	}

	/// Calculate the length of the list.
	uint32 GetLength() const
	{
		uint32 length = 0;
		const b2IntrusiveListNode * const terminator = GetTerminator();
		for (const b2IntrusiveListNode* node = GetNext();
			 node != terminator; node = node->GetNext())
		{
			length++;
		}
		return length;
	}

	/// Get the next node in the list.
	b2IntrusiveListNode* GetNext() const
	{
		return m_next;
	}

	/// Get the previous node in the list.
	b2IntrusiveListNode* GetPrevious() const
	{
		return m_prev;
	}

	/// If B2_INTRUSIVE_LIST_VALIDATE is 1 perform a very rough validation
	/// of all nodes in the list.
	bool ValidateList() const
	{
#if B2_INTRUSIVE_LIST_VALIDATE
	  if (m_magic != k_magic) return false;
	  const b2IntrusiveListNode * const terminator = GetTerminator();
	  for (b2IntrusiveListNode *node = GetNext(); node != terminator;
		   node = node->GetNext()) {
		if (node->m_magic != k_magic) return false;
	  }
#endif  // B2_INTRUSIVE_LIST_VALIDATE
	  return true;
	}

	/// Determine whether the specified node is present in this list.
	bool FindNodeInList(b2IntrusiveListNode* const nodeToFind) const
	{
		const b2IntrusiveListNode * const terminator = GetTerminator();
		for (b2IntrusiveListNode *node = GetNext(); node != terminator;
			 node = node->GetNext())
		{
			if (nodeToFind == node) return true;
		}
		return false;
	}

private:
	/// Initialize the list node.
	void Initialize()
	{
		m_next = this;
		m_prev = this;
	}

private:
#if B2_INTRUSIVE_LIST_VALIDATE
	uint32 m_magic;
#endif  // B2_INTRUSIVE_LIST_VALIDATE
	/// The next node in the list.
	b2IntrusiveListNode *m_prev;
	/// The previous node in the list.
	b2IntrusiveListNode *m_next;

private:
#if B2_INTRUSIVE_LIST_VALIDATE
	static const uint32 k_magic = 0x7157ac01;
#endif  // B2_INTRUSIVE_LIST_VALIDATE
};

/// Declares the member function GetListNode() of Class to retrieve a pointer
/// to NodeMemberName.
/// See #B2_INTRUSIVE_LIST_NODE_GET_CLASS_ACCESSOR()
#define B2_INTRUSIVE_LIST_GET_NODE(NodeMemberName) \
	b2IntrusiveListNode* GetListNode() { return &NodeMemberName; } \
	const b2IntrusiveListNode* GetListNode() const { return &NodeMemberName; }

/// Declares the member function FunctionName of Class to retrieve a pointer
/// to a Class instance from a list node pointer.   NodeMemberName references
/// the name of the b2IntrusiveListNode member of Class.
#define B2_INTRUSIVE_LIST_NODE_GET_CLASS_ACCESSOR( \
	Class, NodeMemberName, FunctionName) \
	static Class* FunctionName(b2IntrusiveListNode *node) \
	{ \
		Class *cls = NULL; \
		/* This effectively performs offsetof(Class, NodeMemberName) */ \
		/* which ends up in the undefined behavior realm of C++ but in */ \
		/* practice this works with most compilers. */ \
		return reinterpret_cast<Class*>((uint8*)(node) - \
										(uint8*)(&cls->NodeMemberName)); \
	} \
	\
	static const Class* FunctionName(const b2IntrusiveListNode *node) \
	{ \
		return FunctionName(const_cast<b2IntrusiveListNode*>(node)); \
	}

/// Declares the member function GetInstanceFromListNode() of Class to retrieve
/// a pointer to a Class instance from a list node pointer.  NodeMemberName
/// reference the name of the b2IntrusiveListNode member of Class.
#define B2_INTRUSIVE_LIST_NODE_GET_CLASS(Class, NodeMemberName) \
	B2_INTRUSIVE_LIST_NODE_GET_CLASS_ACCESSOR(Class, NodeMemberName, \
											  GetInstanceFromListNode)

/// b2TypedIntrusiveListNode which supports inserting an object into a single
/// doubly linked list.  For objects that need to be inserted in multiple
/// doubly linked lists, use b2IntrusiveListNode.
///
/// For example:
///
/// class IntegerItem : public b2TypedIntrusiveListNode<IntegerItem>
/// {
/// public:
/// 	IntegerItem(int32 value) : m_value(value) { }
/// 	~IntegerItem() { }
/// 	int32 GetValue() const { return m_value; }
/// private:
/// 	int32 m_value;
/// };
///
/// int main(int argc, const char *arvg[]) {
/// 	b2TypedIntrusiveListNode<IntegerItem> list;
/// 	IntegerItem a(1);
/// 	IntegerItem b(2);
/// 	IntegerItem c(3);
/// 	list.InsertBefore(&a);
/// 	list.InsertBefore(&b);
/// 	list.InsertBefore(&c);
/// 	for (IntegerItem* item = list.GetNext();
/// 		 item != list.GetTerminator(); item = item->GetNext())
/// 	{
/// 		printf("%d\n", item->GetValue());
/// 	}
/// }
template<typename T>
class b2TypedIntrusiveListNode
{
public:
	b2TypedIntrusiveListNode() { }
	~b2TypedIntrusiveListNode() { }

	/// Insert this object after the specified object.
	void InsertAfter(T* const obj)
	{
		b2Assert(obj);
		GetListNode()->InsertAfter(obj->GetListNode());
	}

	/// Insert this object before the specified object.
	void InsertBefore(T* const obj)
	{
		b2Assert(obj);
		GetListNode()->InsertBefore(obj->GetListNode());
	}

	/// Get the next object in the list.
	/// Check against GetTerminator() before deferencing the object.
	T* GetNext() const
	{
		return GetInstanceFromListNode(GetListNode()->GetNext());
	}

	/// Get the previous object in the list.
	/// Check against GetTerminator() before deferencing the object.
	T* GetPrevious() const
	{
		return GetInstanceFromListNode(GetListNode()->GetPrevious());
	}

	/// Get the terminator of the list.
	/// This should not be dereferenced as it is a pointer to
	/// b2TypedIntrusiveListNode<T> *not* T.
	T* GetTerminator() const
	{
		return (T*)GetListNode();
	}

	/// Remove this object from the list it's currently in.
	T* Remove()
	{
		GetListNode()->Remove();
		return GetInstanceFromListNode(GetListNode());
	}

	/// Determine whether this object is in a list.
	bool InList() const
	{
		return GetListNode()->InList();
	}

	// Determine whether this list is empty.
	bool IsEmpty() const
	{
		return GetListNode()->IsEmpty();
	}

	/// Calculate the length of the list.
	uint32 GetLength() const
	{
		return GetListNode()->GetLength();
	}

	B2_INTRUSIVE_LIST_GET_NODE(m_node);

private:
	// Node within an intrusive list.
	b2IntrusiveListNode m_node;

public:
	/// Get a pointer to the instance of T that contains "node".
	static T* GetInstanceFromListNode(b2IntrusiveListNode* const node)
	{
		b2Assert(node);
		// Calculate the pointer to T from the offset.
		return (T*)((uint8*)node - GetNodeOffset(node));
	}

private:
	// Get the offset of m_node within this class.
	static int32 GetNodeOffset(b2IntrusiveListNode* const node)
	{
		b2Assert(node);
		// Perform some type punning to calculate the offset of m_node in T.
		// WARNING: This could result in undefined behavior with some C++
		// compilers.
		T* obj = (T*)node;
		int32 nodeOffset = (int32)((uint8*)&obj->m_node - (uint8*)obj);
		return nodeOffset;
	}
};

#endif // B2_INTRUSIVE_LIST

