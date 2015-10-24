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
#include <Box2D/Common/b2FreeList.h>
#include <Box2D/Common/b2IntrusiveList.h>
#include <Box2D/Common/b2Settings.h>

/// Allocate an item from the freelist.
b2IntrusiveListNode* b2FreeList::Allocate()
{
	if (m_free.IsEmpty()) return NULL;
	b2IntrusiveListNode * const node = m_free.GetNext();
	node->Remove();
	m_allocated.InsertBefore(node);
	return node;
}

void b2FreeList::Free(b2IntrusiveListNode* node)
{
	b2Assert(node);
#if B2_FREE_LIST_CHECK_ALLOCATED_ON_FREE
	b2Assert(m_allocated.FindNodeInList(node));
#endif // B2_FREE_LIST_CHECK_ALLOCATED_ON_FREE
	node->Remove();
	m_free.InsertAfter(node);
}

void b2FreeList::AddToFreeList(b2IntrusiveListNode* node)
{
	b2Assert(node);
	b2Assert(!node->InList());
	m_free.InsertBefore(node);
}

void b2FreeList::RemoveAll()
{
	while (!m_allocated.IsEmpty()) {
		m_allocated.GetNext()->Remove();
	}
	while (!m_free.IsEmpty()) {
		m_free.GetNext()->Remove();
	}
}
