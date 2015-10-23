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
#ifndef B2_GROWABLE_BUFFER_H
#define B2_GROWABLE_BUFFER_H

#include <Box2D/Common/b2BlockAllocator.h>
#include <string.h>
#include <memory.h>
#include <algorithm>


/// A simple array-like container, similar to std::vector.
/// If we ever start using stl, we should replace this with std::vector.
template <typename T>
class b2GrowableBuffer
{
public:
	b2GrowableBuffer(b2BlockAllocator& allocator) :
		data(NULL),
		count(0),
		capacity(0),
		allocator(&allocator)
	{
	#if defined(LIQUIDFUN_SIMD_NEON)
		// b2ParticleAssembly.neon.s assumes these values are at fixed offsets.
        // If this assert fails, be sure to update the assembly offsets!
		// ldr r3, [r9, #0] @ r3 = out = contacts.data
        // ldr r6, [r9, #8] @ r6 = contacts.capacity
		b2Assert((intptr_t)&data - (intptr_t)(this) == 0
			  && (intptr_t)&capacity - (intptr_t)(this) == 8);
	#endif // defined(LIQUIDFUN_SIMD_NEON)
	}

	b2GrowableBuffer(const b2GrowableBuffer<T>& rhs) :
		data(NULL),
		count(rhs.count),
		capacity(rhs.capacity),
		allocator(rhs.allocator)
	{
		if (rhs.data != NULL)
		{
			data = (T*) allocator->Allocate(sizeof(T) * capacity);
			memcpy(data, rhs.data, sizeof(T) * count);
		}
	}

	~b2GrowableBuffer()
	{
		Free();
	}

	T& Append()
	{
		if (count >= capacity)
		{
			Grow();
		}
		return data[count++];
	}

	void Reserve(int32 newCapacity)
	{
		if (capacity >= newCapacity)
			return;

		// Reallocate and copy.
		T* newData = (T*) allocator->Allocate(sizeof(T) * newCapacity);
		if (data)
		{
			memcpy(newData, data, sizeof(T) * count);
			allocator->Free(data, sizeof(T) * capacity);
		}

		// Update pointer and capacity.
		capacity = newCapacity;
		data = newData;
	}

	void Grow()
	{
		// Double the capacity.
		int32 newCapacity = capacity ? 2 * capacity
						  : b2_minParticleSystemBufferCapacity;
		b2Assert(newCapacity > capacity);
		Reserve(newCapacity);
	}

	void Free()
	{
		if (data == NULL)
			return;

		allocator->Free(data, sizeof(data[0]) * capacity);
		data = NULL;
		capacity = 0;
		count = 0;
	}

	void Shorten(const T* newEnd)
	{
		b2Assert(newEnd >= data);
		count = (int32) (newEnd - data);
	}

	T& operator[](int i)
	{
		return data[i];
	}

	const T& operator[](int i) const
	{
		return data[i];
	}

	T* Data()
	{
		return data;
	}

	const T* Data() const
	{
		return data;
	}

	T* Begin()
	{
		return data;
	}

	const T* Begin() const
	{
		return data;
	}

	T* End()
	{
		return &data[count];
	}

	const T* End() const
	{
		return &data[count];
	}

	int32 GetCount() const
	{
		return count;
	}

	void SetCount(int32 newCount)
	{
		b2Assert(0 <= newCount && newCount <= capacity);
		count = newCount;
	}

	int32 GetCapacity() const
	{
		return capacity;
	}

	template<class UnaryPredicate>
	T* RemoveIf(UnaryPredicate pred)
	{
		T* newEnd = std::remove_if(data, data + count, pred);
		Shorten(newEnd);
		return newEnd;
	}

	template<class BinaryPredicate>
	T* Unique(BinaryPredicate pred)
	{
		T* newEnd = std::unique(data, data + count, pred);
		Shorten(newEnd);
		return newEnd;
	}

private:
	T* data;
	int32 count;
	int32 capacity;
	b2BlockAllocator* allocator;
};

#endif // B2_GROWABLE_BUFFER_H

