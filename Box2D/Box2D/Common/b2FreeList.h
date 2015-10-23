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
#ifndef B2_FREE_LIST_H
#define B2_FREE_LIST_H

#include <Box2D/Common/b2IntrusiveList.h>
#include <Box2D/Common/b2Settings.h>

/// When B2_FREE_LIST_CHECK_ALLOCATED_ON_FREE is 1, b2FreeList::Free() will
/// check that the deallocated node was allocated from the freelist.
#ifndef B2_FREE_LIST_CHECK_ALLOCATED_ON_FREE
#define B2_FREE_LIST_CHECK_ALLOCATED_ON_FREE 0
#endif // B2_FREE_LIST_CHECK_ALLOCATED_ON_FREE


/// Fast - O(1) - list based allocator for items that can be inserted into
/// b2IntrusiveListNode lists.
class b2FreeList
{
public:
	/// Construct the free list.
	b2FreeList() { }

	/// Destroy the free list.
	~b2FreeList() { }

	/// Allocate an item from the freelist.
	b2IntrusiveListNode* Allocate();

	/// Free an item from the freelist.
	void Free(b2IntrusiveListNode* node);

	/// Add an item to the freelist so that it can be allocated using
	/// b2FreeList::Allocate().
	void AddToFreeList(b2IntrusiveListNode* node);

	/// Remove all items (allocated and free) from the freelist.
	void RemoveAll();

	/// Get the list which tracks allocated items.
	const b2IntrusiveListNode& GetAllocatedList() const {
		return m_allocated;
	}

	/// Get the list which tracks free items.
	const b2IntrusiveListNode& GetFreeList() const {
		return m_free;
	}

protected:
	/// List of allocated items.
	b2IntrusiveListNode m_allocated;
	/// List of free items.
	b2IntrusiveListNode m_free;
};


/// Typed b2FreeList which manages items of type T assuming T implements
/// the GetInstanceFromListNode() and GetListNode() methods.
template<typename T>
class b2TypedFreeList {
public:
	/// Construct the free list.
	b2TypedFreeList() { }

	/// Destroy the free list.
	~b2TypedFreeList() { }

	/// Allocate an item from the free list.
	T* Allocate() {
		b2IntrusiveListNode* const node = m_freeList.Allocate();
		if (!node) return NULL;
		return T::GetInstanceFromListNode(node);
	}

	/// Free an item.
	void Free(T* instance) {
		b2Assert(instance);
		m_freeList.Free(instance->GetListNode());
	}

	/// Add an item to the freelist so that it can be allocated with
	/// b2TypedFreeList::Allocate().
	void AddToFreeList(T* instance)
	{
		b2Assert(instance);
		m_freeList.AddToFreeList(instance->GetListNode());
	}

	// Get the underlying b2FreeList.
	b2FreeList* GetFreeList() { return &m_freeList; }
	const b2FreeList* GetFreeList() const { return &m_freeList; }

protected:
	b2FreeList m_freeList;
};

#endif  // B2_FREE_LIST_H
