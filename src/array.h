// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "assert.h"
#include "core.h"

// todo compare with https://github.com/skeeto/growable-buf

typedef struct b2ArrayHeader
{
	int count;
	int capacity;
} b2ArrayHeader;

#define b2Array( a ) ( (b2ArrayHeader*)( a ) )[-1]

void* b2CreateArray( int elementSize, int capacity );
void b2DestroyArray( void* a, int elementSize );
void b2Array_Grow( void** a, int elementSize );
void b2Array_Resize( void** a, int elementSize, int count );

#define b2CheckIndex( a, index ) B2_ASSERT( 0 <= index && index < b2Array( a ).count )
#define b2CheckId( ARRAY, ID ) B2_ASSERT( 0 <= ID && ID < b2Array( ARRAY ).count && ARRAY[ID].id == ID )
#define b2CheckIdAndRevision( ARRAY, ID, REV )                                                                                   \
	B2_ASSERT( 0 <= ID && ID < b2Array( ARRAY ).count && ARRAY[ID].id == ID && ARRAY[ID].revision == REV )

#define b2Array_Clear( a ) b2Array( a ).count = 0

#define b2Array_Push( a, element )                                                                                               \
	if ( b2Array( a ).count == b2Array( a ).capacity )                                                                           \
		b2Array_Grow( (void**)&a, sizeof( element ) );                                                                           \
	B2_ASSERT( b2Array( a ).count < b2Array( a ).capacity );                                                                     \
	a[b2Array( a ).count++] = element

#define b2Array_RemoveSwap( a, index )                                                                                           \
	B2_ASSERT( 0 <= index && index < b2Array( a ).count );                                                                       \
	a[index] = a[b2Array( a ).count - 1];                                                                                        \
	b2Array( a ).count--

#define b2Array_Last( a ) ( a )[b2Array( a ).count - 1];

#define b2Array_Pop( a )                                                                                                         \
	B2_ASSERT( 0 < b2Array( a ).count );                                                                                         \
	b2Array( a ).count--

#define b2GetArrayBytes( a, elementSize ) ( (int)elementSize * b2Array( a ).capacity )

// Macro based dynamic arrays
// Pros
// - type safe
// - debuggable (visible count and capacity)
// - bounds checking
// - forward declaration
// - simple
// Cons
// - macros suck, however all array functions are real, type-safe functions

// Array definition that doesn't need the type T to be defined
#define B2_ARRAY( T, PREFIX )                                                                                                    \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		T* data;                                                                                                                 \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} PREFIX##Array;                                                                                                             \
	PREFIX##Array PREFIX##Array_Create( int capacity );                                                                          \
	void PREFIX##Array_Reserve( PREFIX##Array* a, int newCapacity );                                                             \
	void PREFIX##Array_Destroy( PREFIX##Array* a );

// Inline array functions that need the type T to be defined
#define B2_ARRAY_INLINE( T, PREFIX )                                                                                             \
	static inline T* PREFIX##Array_Get( PREFIX##Array* a, int index )                                                            \
	{                                                                                                                            \
		B2_ASSERT( 0 <= index && index < a->count );                                                                             \
		return a->data + index;                                                                                                  \
	}                                                                                                                            \
                                                                                                                                 \
	static inline void PREFIX##Array_Push( PREFIX##Array* a, T value )                                                           \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			PREFIX##Array_Reserve( a, newCapacity );                                                                                  \
		}                                                                                                                        \
		a->data[a->count] = value;                                                                                               \
		a->count += 1;                                                                                                           \
	}                                                                                                                            \
                                                                                                                                 \
	static inline void PREFIX##Array_Set( PREFIX##Array* a, int index, T value )                                                 \
	{                                                                                                                            \
		B2_ASSERT( 0 <= index && index < a->count );                                                                             \
		a->data[index] = value;                                                                                                  \
	}                                                                                                                            \
                                                                                                                                 \
	static inline void PREFIX##Array_RemoveSwap( PREFIX##Array* a, int index )                                                   \
	{                                                                                                                            \
		B2_ASSERT( 0 <= index && index < a->count );                                                                             \
		if ( index != a->count - 1 )                                                                                             \
		{                                                                                                                        \
			a->data[index] = a->data[a->count - 1];                                                                              \
		}                                                                                                                        \
		a->count -= 1;                                                                                                           \
	}                                                                                                                            \
                                                                                                                                 \
	static inline int PREFIX##Array_ByteCount( PREFIX##Array* a )                                                                \
	{                                                                                                                            \
		return (int)( a->capacity * sizeof( T ) );                                                                               \
	}

// Array implementations to be instantiated in a source file where the type T is known
#define B2_ARRAY_SOURCE( T, PREFIX )                                                                                          \
	PREFIX##Array PREFIX##Array_Create( int capacity )                                                                           \
	{                                                                                                                            \
		PREFIX##Array a;                                                                                                         \
		a.data = b2Alloc( capacity * sizeof( T ) );                                                                              \
		a.count = 0;                                                                                                             \
		a.capacity = capacity;                                                                                                   \
		return a;                                                                                                                \
	}                                                                                                                            \
                                                                                                                                 \
	void PREFIX##Array_Reserve( PREFIX##Array* a, int newCapacity )                                                              \
	{                                                                                                                            \
		if ( newCapacity <= a->capacity )                                                                                        \
		{                                                                                                                        \
			return;                                                                                                              \
		}                                                                                                                        \
		a->data = b2GrowAlloc( a->data, a->capacity * sizeof( T ), newCapacity * sizeof( T ) );                                  \
		a->capacity = newCapacity;                                                                                               \
	}                                                                                                                            \
                                                                                                                                 \
	void PREFIX##Array_Destroy( PREFIX##Array* a )                                                                               \
	{                                                                                                                            \
		b2Free( a->data, a->capacity * sizeof( T ) );                                                                            \
		a->data = NULL;                                                                                                          \
		a->count = 0;                                                                                                            \
		a->capacity = 0;                                                                                                         \
	}

B2_ARRAY( int, b2Int );
B2_ARRAY_INLINE( int, b2Int );

typedef struct b2Body b2Body;
B2_ARRAY( b2Body, b2Body );
