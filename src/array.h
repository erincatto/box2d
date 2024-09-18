// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

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

// Array declaration that doesn't need the type T to be defined
#define B2_DECLARE_ARRAY( T, PREFIX )                                                                                            \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		struct T* data;                                                                                                          \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} PREFIX##Array;                                                                                                             \
	PREFIX##Array PREFIX##Array_Create( int capacity );                                                                          \
	void PREFIX##Array_Reserve( PREFIX##Array* a, int newCapacity );                                                             \
	void PREFIX##Array_Destroy( PREFIX##Array* a );

#define B2_DECLARE_ARRAY_NATIVE( T, PREFIX )                                                                                     \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		T* data;                                                                                                                 \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} PREFIX##Array;                                                                                                             \
	/* Create array with initial capacity. Zero initialization is also supported */                                              \
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
	static inline T* PREFIX##Array_Add( PREFIX##Array* a )                                                                       \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			PREFIX##Array_Reserve( a, newCapacity );                                                                             \
		}                                                                                                                        \
		a->count += 1;                                                                                                           \
		return a->data + ( a->count - 1 );                                                                                       \
	}                                                                                                                            \
                                                                                                                                 \
	static inline void PREFIX##Array_Push( PREFIX##Array* a, T value )                                                           \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			PREFIX##Array_Reserve( a, newCapacity );                                                                             \
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
	static inline int PREFIX##Array_RemoveSwap( PREFIX##Array* a, int index )                                                    \
	{                                                                                                                            \
		B2_ASSERT( 0 <= index && index < a->count );                                                                             \
		int movedIndex = B2_NULL_INDEX;                                                                                          \
		if ( index != a->count - 1 )                                                                                             \
		{                                                                                                                        \
			movedIndex = a->count - 1;                                                                                           \
			a->data[index] = a->data[movedIndex];                                                                                \
		}                                                                                                                        \
		a->count -= 1;                                                                                                           \
		return movedIndex;                                                                                                       \
	}                                                                                                                            \
                                                                                                                                 \
	static inline T PREFIX##Array_Pop( PREFIX##Array* a )                                                                        \
	{                                                                                                                            \
		B2_ASSERT( a->count > 0 );                                                                                               \
		T value = a->data[a->count - 1];                                                                                         \
		a->count -= 1;                                                                                                           \
		return value;                                                                                                            \
	}                                                                                                                            \
                                                                                                                                 \
	static inline int PREFIX##Array_ByteCount( PREFIX##Array* a )                                                                \
	{                                                                                                                            \
		return (int)( a->capacity * sizeof( T ) );                                                                               \
	}

// Array implementations to be instantiated in a source file where the type T is known
#define B2_ARRAY_SOURCE( T, PREFIX )                                                                                             \
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

B2_DECLARE_ARRAY_NATIVE( int, b2Int );
B2_ARRAY_INLINE( int, b2Int );

B2_DECLARE_ARRAY( b2Body, b2Body );
B2_DECLARE_ARRAY( b2BodySim, b2BodySim );
B2_DECLARE_ARRAY( b2BodyState, b2BodyState );
B2_DECLARE_ARRAY( b2Contact, b2Contact );
B2_DECLARE_ARRAY( b2ContactSim, b2ContactSim );
B2_DECLARE_ARRAY( b2Joint, b2Joint );
B2_DECLARE_ARRAY( b2JointSim, b2JointSim );
B2_DECLARE_ARRAY( b2Island, b2Island );
B2_DECLARE_ARRAY( b2IslandSim, b2IslandSim );
