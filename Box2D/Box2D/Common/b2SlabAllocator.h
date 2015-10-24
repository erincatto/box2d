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
#ifndef B2_SLAB_ALLOCATOR_H
#define B2_SLAB_ALLOCATOR_H

#include <stddef.h>
#include <stdint.h>
#include <new>
#include <Box2D/Common/b2IntrusiveList.h>
#include <Box2D/Common/b2FreeList.h>
#include <Box2D/Common/b2Settings.h>
#include <Box2D/Common/b2TrackedBlock.h>

/// Freelist based allocator for fixed sized items from slabs (memory
/// preallocated from the heap).
/// T should be a class which has a default constructor and implements the
/// member function "b2IntrusiveList* GetListNode()".
/// All objects in a slab are constructed when a slab is created and destructed
/// when a slab is freed.
template<typename T>
class b2SlabAllocator
{
private:
	// Information about a slab.
	class Slab
	{
	public:
		/// Initialize a slab with the number of items it contains.
		Slab(uint32 numberOfItems) :
			m_numberOfItems(numberOfItems)
		{
			B2_NOT_USED(m_padding);
			// This assumes that this class is packed on at least a 4-byte
			// boundary with no padding.  Verify the assumption.
			b2Assert(sizeof(*this) == b2_mallocAlignment);
		}

		/// Empty destructor.
		~Slab() { }

		/// Get the number of items in this slab.
		uint32 GetNumberOfItems() const { return m_numberOfItems; }

		/// Get a pointer to the first item in the slab.
		T* GetFirstItem() const
		{
			return (T*)((uint8*)(this + 1));
		}

		/// Get a pointer to the end of the slab.
		/// NOTE: This is a pointer after the last byte of the slab not the
		/// last item in the slab.
		T* GetItemEnd() const { return GetFirstItem() + GetNumberOfItems(); }

	private:
		/// Number of items in the slab.
		uint32 m_numberOfItems;
		/// Padding to align the first item in the slab to b2_mallocAlignment.
		uint8 m_padding[b2_mallocAlignment - sizeof(uint32)];
	};

public:
	/// Initialize the allocator to allocate itemsPerSlab of type T for each
	/// slab that is allocated.
	b2SlabAllocator(const uint32 itemsPerSlab) :
		m_itemsPerSlab(itemsPerSlab)
	{
	}

	/// Free all allocated slabs.
	~b2SlabAllocator()
	{
		FreeAllSlabs();
	}

	/// Set size of the next allocated slab using the number of items per
	/// slab.  Setting this value to zero disables further slab allocation.
	void SetItemsPerSlab(uint32 itemsPerSlab)
	{
		m_itemsPerSlab = itemsPerSlab;
	}

	// Get the size of the next allocated slab.
	uint32 GetItemsPerSlab() const
	{
		return m_itemsPerSlab;
	}

	/// Allocate a item from the slab.
	T* Allocate()
	{
		// Allocate a slab if needed here.
		if (m_freeList.GetFreeList()->GetFreeList().IsEmpty() &&
			!AllocateSlab())
			return NULL;
		return m_freeList.Allocate();
	}

	/// Free an item from the slab.
	void Free(T *object)
	{
		m_freeList.Free(object);
	}

	/// Allocate a slab, construct instances of T and add them to the free
	/// pool.
	bool AllocateSlab()
	{
		if (!m_itemsPerSlab) return false;
		const uint32 slabSize = sizeof(Slab) + (sizeof(T) * m_itemsPerSlab);
		void* const memory = m_slabs.Allocate(slabSize);
		if (!memory) return false;

		Slab* const slab = new (BlockGetSlab(memory)) Slab(m_itemsPerSlab);
		T* item = slab->GetFirstItem();
		for (uint32 i = 0; i < m_itemsPerSlab; ++i, ++item)
		{
			m_freeList.AddToFreeList(new (item) T);
		}
		return true;
	}

	/// Free all slabs.
	void FreeAllSlabs()
	{
		const b2TypedIntrusiveListNode<b2TrackedBlock>& slabList =
			m_slabs.GetList();
		while (!slabList.IsEmpty())
		{
			FreeSlab(BlockGetSlab(slabList.GetNext()->GetMemory()));
		}
	}

	/// Free all empty slabs.
	/// This method is slow - O(M^N) - since this class doesn't track
	/// the association between each item and slab.
	void FreeEmptySlabs()
	{
		const b2IntrusiveListNode& freeItemList =
			m_freeList.GetFreeList()->GetFreeList();
		const b2IntrusiveListNode* freeItemListTerminator =
			freeItemList.GetTerminator();
		const b2TypedIntrusiveListNode<b2TrackedBlock>& slabList =
			m_slabs.GetList();
		const b2TypedIntrusiveListNode<b2TrackedBlock>* slabListTerminator =
			slabList.GetTerminator();
		b2TrackedBlock* block = slabList.GetNext();
		while (block != slabListTerminator)
		{
			// Get the Slab from the memory associated with the block.
			Slab* const slab = BlockGetSlab(block->GetMemory());
			block = block->GetNext();

			// Determine the range of memory the Slab owns.
			const uint8* const slabItemStart = (uint8*)slab->GetFirstItem();
			const uint8* const slabItemEnd = (uint8*)slab->GetItemEnd();

			// Count all free items that are owned by the current slab.
			uint8 freeItems = 0;
			bool empty = false;
			for (b2IntrusiveListNode* itemNode = freeItemList.GetNext();
				 itemNode != freeItemListTerminator;
				 itemNode = itemNode->GetNext())
			{
				const uint8* itemNodeAddress = (uint8*)itemNode;
				if (itemNodeAddress >= slabItemStart &&
					itemNodeAddress <= slabItemEnd)
				{
					++freeItems;
					if (slab->GetNumberOfItems() == freeItems)
					{
						empty = true;
						break;
					}
				}
			}
			// If a slab is empty, free it.
			if (empty)
			{
				FreeSlab(slab);
			}
		}
	}

	/// Get the item allocator freelist.
	const b2TypedFreeList<T>& GetFreeList() const
	{
		return m_freeList;
	}

private:
	/// Destroy all objects in a slab and free the slab.
	void FreeSlab(Slab * const slab)
	{
		b2Assert(slab);
		const uint32 numberOfItems = slab->GetNumberOfItems();
		T* item = slab->GetFirstItem();
		for (uint32 i = 0; i < numberOfItems; ++i, ++item)
		{
			item->~T();
		}
		slab->~Slab();
		m_slabs.Free(slab);
	}

	/// Get a pointer to a Slab from a block of memory in m_slabs.
	Slab* BlockGetSlab(void *memory)
	{
		return (Slab*)memory;
	}

	/// Get a pointer to the first item in the array of items referenced by a
	/// Slab.
	T* SlabGetFirstItem(Slab* slab)
	{
		return (T*)(slab + 1);
	}

private:
	/// Contains a list of b2TrackedBlock instances where each b2TrackedBlock's
	/// associated user memory contains a Slab followed by instances of T.
	b2TrackedBlockAllocator m_slabs;
	/// Number of items to allocate in the next allocated slab.
	uint32 m_itemsPerSlab;
	/// Freelist which contains instances of T.
	b2TypedFreeList<T> m_freeList;
};

#endif  // B2_SLAB_ALLOCATOR_H
