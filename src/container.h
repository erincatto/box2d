// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stddef.h>
#include <string.h>

#define b2DeclareArray( T )                                                                                                      \
	typedef struct b2DynamicArray_##T                                                                                            \
	{                                                                                                                            \
		T* data;                                                                                                                 \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} b2DynamicArray_##T

// Define an array.
// It may be zero initialized:
// b2ArrayC(int) myArray = { 0 };
#define b2ArrayC( T ) b2DynamicArray_##T

// Alternative to zero initialization
#define b2Array_Create( a )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define b2Array_CreateN( a, n )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = ( n ) > 0 ? b2GrowAlloc( NULL, 0, ( n ) * sizeof( *( a ).data ) ) : NULL;                                   \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = ( n );                                                                                                  \
	}                                                                                                                            \
	while ( 0 )

#define b2Array_Destroy( a )                                                                                                     \
	do                                                                                                                           \
	{                                                                                                                            \
		b2Free( ( a ).data, ( a ).capacity * sizeof( *( a ).data ) );                                                            \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define b2Array_Reserve( a, n )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( a ).capacity < n )                                                                                                \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * sizeof( *( a ).data );                                                                \
			int newSize = ( n ) * sizeof( *( a ).data );                                                                         \
			( a ).data = b2GrowAlloc( ( a ).data, oldSize, newSize );                                                            \
			( a ).capacity = ( n );                                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

#define b2Array_Resize( a, n )                                                                                                   \
	do                                                                                                                           \
	{                                                                                                                            \
		b2Array_Reserve( a, n );                                                                                                 \
		( a ).count = ( n );                                                                                                     \
	}                                                                                                                            \
	while ( 0 )

// Push a new element by value
#define b2Array_Push( a, value )                                                                                                 \
	do                                                                                                                           \
	{                                                                                                                            \
		int elementSize = sizeof( *( a ).data );                                                                                 \
		if ( ( a ).count >= ( a ).capacity )                                                                                     \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * elementSize;                                                                          \
			int newCapacity = ( a ).capacity == 0 ? 8 : 2 * ( a ).capacity;                                                      \
			int newSize = newCapacity * elementSize;                                                                             \
			( a ).data = b2GrowAlloc( ( a ).data, oldSize, newSize );                                                            \
			( a ).capacity = newCapacity;                                                                                        \
		}                                                                                                                        \
		( a ).data[( a ).count] = ( value );                                                                                     \
		( a ).count += 1;                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Get a pointer to an element
#define b2Array_Get( a, index ) ( B2_ASSERT( 0 <= index && index < ( a ).count ), ( a ).data + index )

// Create a new uninitialized element and return a pointer to it
#define b2Array_Emplace( a )                                                                                                     \
	( b2EmplaceHelper( (void**)&( a ).data, &( a ).count, &( a ).capacity, sizeof( *( a ).data ) ) )

// Remove the last element and return it by value.
#define b2Array_Pop( a ) ( B2_ASSERT( 0 < ( a ).count ), ( a ).data[-1 + ( a ).count--] )

// Remove an element by swapping with the last element. If the index is the last element it returns
// B2_NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
#define b2Array_RemoveSwap( a, index ) b2RemoveHelper( ( a ).data, &( a ).count, ( index ), sizeof( *( a ).data ) )

// Typedef for a stack array. A stack array uses a fixed size buffer but can grow on the heap if necessary.
// Unfortunately the data pointer can go out of sync with stackData if the stack array is a member of a growable array.
#define b2DeclareStackArray( T, N )                                                                                              \
	typedef struct b2StackArray_##T##N                                                                                           \
	{                                                                                                                            \
		T stackData[N];                                                                                                          \
		T* data;                                                                                                                 \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} b2StackArray_##T##N

// Used to define a stack array instance
#define b2StackArray( T, N ) b2StackArray_##T##N

#define b2StackArray_Create( a )                                                                                                 \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = ( a ).stackData;                                                                                            \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = B2_ARRAY_COUNT( ( a ).stackData );                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define b2StackArray_Destroy( a )                                                                                                \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( a ).data != ( a ).stackData )                                                                                     \
		{                                                                                                                        \
			b2Free( ( a ).data, ( a ).capacity * sizeof( *( a ).data ) );                                                        \
		}                                                                                                                        \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define b2StackArray_Reserve( a, n )                                                                                             \
	do                                                                                                                           \
	{                                                                                                                            \
		B2_ASSERT( ( a ).data != NULL && ( a ).capacity > 0 );                                                                   \
		if ( ( a ).capacity < n )                                                                                                \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * sizeof( *( a ).data );                                                                \
			int newSize = ( n ) * sizeof( *( a ).data );                                                                         \
			if ( ( a ).data == ( a ).stackData )                                                                                 \
			{                                                                                                                    \
				( a ).data = b2GrowAlloc( NULL, 0, newSize );                                                                    \
				memcpy( ( a ).data, ( a ).stackData, oldSize );                                                                  \
			}                                                                                                                    \
			else                                                                                                                 \
			{                                                                                                                    \
				( a ).data = b2GrowAlloc( ( a ).data, oldSize, newSize );                                                        \
			}                                                                                                                    \
			( a ).capacity = ( n );                                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Push a new element by value
#define b2StackArray_Push( a, value )                                                                                            \
	do                                                                                                                           \
	{                                                                                                                            \
		B2_ASSERT( ( a ).data != NULL && ( a ).capacity > 0 );                                                                   \
		if ( ( a ).count >= ( a ).capacity )                                                                                     \
		{                                                                                                                        \
			b2StackArray_Reserve( ( a ), 2 * ( a ).capacity );                                                                   \
		}                                                                                                                        \
		( a ).data[( a ).count] = ( value );                                                                                     \
		( a ).count += 1;                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Get a pointer to an element
#define b2StackArray_Get( a, index ) ( B2_ASSERT( 0 <= index && index < ( a ).count ), ( a ).data + index )

// Remove an element from an int arrayA by swapping with the last element. This updates the index contained
// in the moved element in arrayB. Assumes the integers in arrayA index into arrayB. Assumes
// the elements of arrayB have an indexName member that is the index in arrayA.
#define b2RemoveUpdate( arrayA, arrayB, indexB, indexName )                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		int lastIndex = ( arrayA ).count - 1;                                                                                    \
		B2_ASSERT( 0 <= ( indexB ) && ( indexB ) < ( arrayB ).count );                                                           \
		int indexA = ( arrayB ).data[indexB].indexName;                                                                          \
		B2_ASSERT( 0 <= indexA && indexA < ( arrayA ).count );                                                                   \
		if ( indexA != lastIndex )                                                                                               \
		{                                                                                                                        \
			int movedIndex = ( arrayA ).data[lastIndex];                                                                         \
			( arrayA ).data[indexA] = movedIndex;                                                                                \
			( arrayB ).data[movedIndex].indexName = indexA;                                                                      \
		}                                                                                                                        \
		( arrayA ).count -= 1;                                                                                                   \
	}                                                                                                                            \
	while ( 0 )

B2_INLINE void* b2EmplaceHelper( void** data, int* count, int* capacity, int elementSize )
{
	if ( *count >= *capacity )
	{
		int oldCapacity = *capacity;
		int oldSize = oldCapacity * elementSize;
		int newCapacity = ( oldCapacity == 0 ? 8 : 2 * oldCapacity );
		int newSize = newCapacity * elementSize;
		*data = b2GrowAlloc( *data, oldSize, newSize );
		*capacity = newCapacity;
	}
	return (char*)*data + ( *count )++ * elementSize;
}

B2_INLINE int b2RemoveHelper( void* data, int* count, int index, int elementSize )
{
	B2_ASSERT( 0 <= index && index < *count && "Array index out of bounds" );

	( *count )--;
	if ( index != *count )
	{
		memcpy( (char*)data + index * elementSize, (char*)data + ( *count ) * elementSize, elementSize );
		return *count;
	}

	return B2_NULL_INDEX;
}
