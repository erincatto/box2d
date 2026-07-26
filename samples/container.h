// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define NULL_INDEX -1

#define DeclareArray( T )                                                                                                        \
	typedef struct DynamicArray_##T                                                                                              \
	{                                                                                                                            \
		struct T* data;                                                                                                          \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} DynamicArray_##T

#define DeclareArrayNative( T )                                                                                                  \
	typedef struct DynamicArray_##T                                                                                              \
	{                                                                                                                            \
		T* data;                                                                                                                 \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} DynamicArray_##T

// Define an array.
// It may be zero initialized:
// Array(int) myArray = { 0 };
#define Array( T ) DynamicArray_##T

// Alternative to zero initialization
#define Array_Create( a )                                                                                                        \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define Array_CreateN( a, n )                                                                                                    \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).data = ( n ) > 0 ? GrowAlloc( NULL, 0, ( n ) * sizeof( *( a ).data ) ) : NULL;                                     \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = ( n );                                                                                                  \
	}                                                                                                                            \
	while ( 0 )

#define Array_Destroy( a )                                                                                                       \
	do                                                                                                                           \
	{                                                                                                                            \
		free( ( a ).data );                                                              \
		( a ).data = NULL;                                                                                                       \
		( a ).count = 0;                                                                                                         \
		( a ).capacity = 0;                                                                                                      \
	}                                                                                                                            \
	while ( 0 )

#define Array_Reserve( a, n )                                                                                                    \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( a ).capacity < n )                                                                                                \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * sizeof( *( a ).data );                                                                \
			int newSize = ( n ) * sizeof( *( a ).data );                                                                         \
			( a ).data = GrowAlloc( ( a ).data, oldSize, newSize );                                                              \
			( a ).capacity = ( n );                                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

#define Array_Resize( a, n )                                                                                                     \
	do                                                                                                                           \
	{                                                                                                                            \
		Array_Reserve( a, n );                                                                                                   \
		( a ).count = ( n );                                                                                                     \
	}                                                                                                                            \
	while ( 0 )

#define Array_ResizeAndSetZero( a, n )                                                                                           \
	do                                                                                                                           \
	{                                                                                                                            \
		Array_Reserve( a, n );                                                                                                   \
		memset( ( a ).data, 0, ( n ) * sizeof( *( a ).data ) );                                                                  \
		( a ).count = ( n );                                                                                                     \
	}                                                                                                                            \
	while ( 0 )

// Push a new element by value
#define Array_Push( a, value )                                                                                                   \
	do                                                                                                                           \
	{                                                                                                                            \
		int elementSize = sizeof( *( a ).data );                                                                                 \
		if ( ( a ).count >= ( a ).capacity )                                                                                     \
		{                                                                                                                        \
			int oldSize = ( a ).capacity * elementSize;                                                                          \
			int newCapacity = ( a ).capacity == 0 ? 8 : 2 * ( a ).capacity;                                                      \
			int newSize = newCapacity * elementSize;                                                                             \
			( a ).data = GrowAlloc( ( a ).data, oldSize, newSize );                                                              \
			( a ).capacity = newCapacity;                                                                                        \
		}                                                                                                                        \
		( a ).data[( a ).count] = ( value );                                                                                     \
		( a ).count += 1;                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Get a pointer to an element
#define Array_Get( a, index ) ( assert( 0 <= ( index ) && ( index ) < ( a ).count ), ( a ).data + ( index ) )

// Create a new uninitialized element and return a pointer to it
#define Array_Emplace( a ) ( EmplaceHelper( (void**)&( a ).data, &( a ).count, &( a ).capacity, sizeof( *( a ).data ) ) )

// Remove the last element and return it by value.
#define Array_Pop( a ) ( assert( 0 < ( a ).count ), ( a ).data[-1 + ( a ).count--] )

// Remove an element by swapping with the last element. If the index is the last element it returns
// NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
#define Array_RemoveSwap( a, index ) RemoveHelper( ( a ).data, &( a ).count, ( index ), sizeof( *( a ).data ) )

void* GrowAlloc( void* oldMem, int oldSize, int newSize );

static inline void* EmplaceHelper( void** data, int* count, int* capacity, int elementSize )
{
	if ( *count >= *capacity )
	{
		int oldCapacity = *capacity;
		int oldSize = oldCapacity * elementSize;
		int newCapacity = ( oldCapacity == 0 ? 8 : 2 * oldCapacity );
		int newSize = newCapacity * elementSize;
		*data = GrowAlloc( *data, oldSize, newSize );
		*capacity = newCapacity;
	}
	return (char*)*data + ( *count )++ * elementSize;
}

static inline int RemoveHelper( void* data, int* count, int index, int elementSize )
{
	assert( 0 <= index && index < *count && "Array index out of bounds" );

	( *count )--;
	if ( index != *count )
	{
		memcpy( (char*)data + index * elementSize, (char*)data + ( *count ) * elementSize, elementSize );
		return *count;
	}

	return NULL_INDEX;
}

#define Array_Clear( a )                                                                                                         \
	do                                                                                                                           \
	{                                                                                                                            \
		( a ).count = 0;                                                                                                         \
	}                                                                                                                            \
	while ( 0 )

#define Array_ByteCount( a ) ( ( a ).capacity * (int)sizeof( *( a ).data ) )

DeclareArrayNative( int );
