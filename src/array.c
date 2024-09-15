// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "array.h"

#include "allocate.h"
#include "core.h"

#include <string.h>

void* b2CreateArray( int elementSize, int capacity )
{
	void* result = (b2ArrayHeader*)b2Alloc( sizeof( b2ArrayHeader ) + elementSize * capacity ) + 1;
	b2Array( result ).count = 0;
	b2Array( result ).capacity = capacity;
	return result;
}

void b2DestroyArray( void* a, int elementSize )
{
	int capacity = b2Array( a ).capacity;
	int size = sizeof( b2ArrayHeader ) + elementSize * capacity;
	b2Free( ( (b2ArrayHeader*)a ) - 1, size );
}

void b2Array_Grow( void** a, int elementSize )
{
	int capacity = b2Array( *a ).capacity;
	B2_ASSERT( capacity == b2Array( *a ).count );

	// grow by 50%
	int newCapacity = capacity + ( capacity >> 1 );
	newCapacity = newCapacity >= 2 ? newCapacity : 2;
	void* tmp = *a;
	*a = (b2ArrayHeader*)b2Alloc( sizeof( b2ArrayHeader ) + elementSize * newCapacity ) + 1;
	b2Array( *a ).capacity = newCapacity;
	b2Array( *a ).count = capacity;
	memcpy( *a, tmp, capacity * elementSize );
	b2DestroyArray( tmp, elementSize );
}

void b2Array_Resize( void** a, int elementSize, int count )
{
	int capacity = b2Array( *a ).capacity;
	if ( capacity >= count )
	{
		b2Array( *a ).count = count;
		return;
	}

	int originalCount = b2Array( *a ).count;

	// grow by 50%
	int newCapacity = count + ( count >> 1 );
	newCapacity = newCapacity >= 2 ? newCapacity : 2;
	void* tmp = *a;
	*a = (b2ArrayHeader*)b2Alloc( sizeof( b2ArrayHeader ) + elementSize * newCapacity ) + 1;
	b2Array( *a ).capacity = newCapacity;
	b2Array( *a ).count = count;

	// copy existing elements
	memcpy( *a, tmp, originalCount * elementSize );
	b2DestroyArray( tmp, elementSize );
}

#define B2_IMPLEMENT_ARRAY( T )                                                                                                  \
	T##Array T##Array_Create( int capacity )                                                                                     \
	{                                                                                                                            \
		T##Array a;                                                                                                              \
		a.data = b2Alloc( capacity * sizeof( T ) );                                                                              \
		a.count = 0;                                                                                                             \
		a.capacity = capacity;                                                                                                   \
		return a;                                                                                                                \
	}                                                                                                                            \
                                                                                                                                 \
	void T##Array_Reserve( T##Array* a, int newCapacity )                                                                        \
	{                                                                                                                            \
B2_ASSERT(newCapacity > a->capacity); \
		if ( newCapacity <= a->capacity )                                                                                        \
		{                                                                                                                        \
			return;                                                                                                              \
		}                                                                                                                        \
		a->data = b2GrowAlloc( a->data, a->capacity * sizeof( T ), newCapacity * sizeof( T ) );                                  \
		a->capacity = newCapacity;                                                                                               \
	}                                                                                                                            \
                                                                                                                                 \
	void T##Array_Destroy( T##Array* a )                                                                                         \
	{                                                                                                                            \
		b2Free( a->data, a->capacity * sizeof( T ) );                                                                            \
		a->data = NULL;                                                                                                          \
		a->count = 0;                                                                                                            \
		a->capacity = 0;                                                                                                         \
	}

#include "body.h"
B2_IMPLEMENT_ARRAY( b2Body );
