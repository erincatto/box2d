/*
* Copyright (c) 2013 Google, Inc.
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
#ifndef B2_STACK_QUEUE
#define B2_STACK_QUEUE

#include <Box2D/Common/b2StackAllocator.h>

template <typename T>
class b2StackQueue
{

public:

	b2StackQueue(b2StackAllocator *allocator, int32 capacity)
	{
		m_allocator = allocator;
		m_buffer = (T*) m_allocator->Allocate(sizeof(T) * capacity);
		m_front = 0;
		m_back = 0;
		m_capacity = capacity;
	}

	~b2StackQueue()
	{
		m_allocator->Free(m_buffer);
	}

	void Push(const T &item)
	{
		if (m_back >= m_capacity)
		{
			for (int32 i = m_front; i < m_back; i++)
			{
				m_buffer[i - m_front] = m_buffer[i];
			}
			m_back -= m_front;
			m_front = 0;
			if (m_back >= m_capacity)
			{
				if (m_capacity > 0)
				{
					m_capacity *= 2;
				}
				else
				{
					m_capacity = 1;
				}
				m_buffer = (T*) m_allocator->Reallocate(m_buffer,
														sizeof(T) * m_capacity);
			}
		}
		m_buffer[m_back] = item;
		m_back++;
	}

	void Pop()
	{
		b2Assert(m_front < m_back);
		m_front++;
	}

	bool Empty() const
	{
		b2Assert(m_front <= m_back);
		return m_front == m_back;
	}

	const T &Front() const
	{
		return m_buffer[m_front];
	}

private:

	b2StackAllocator *m_allocator;
	T* m_buffer;
	int32 m_front;
	int32 m_back;
	int32 m_capacity;

};

#endif
