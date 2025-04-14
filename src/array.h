// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

// Macro generated functions for dynamic arrays
// Pros
// - type safe
// - array data debuggable (visible count and capacity)
// - bounds checking
// - forward declaration
// - simple implementation
// - generates functions (like C++ templates)
// - functions have https://en.wikipedia.org/wiki/Sequence_point
// - avoids stretchy buffer dropped pointer update bugs
// Cons
// - cannot debug
// - breaks code navigation

// todo_erin consider code-gen: https://github.com/IbrahimHindawi/haikal

// Array declaration that doesn't need the type T to be defined
#define B2_ARRAY_DECLARE( T, PREFIX )                                                                                            \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		struct T* data;                                                                                                          \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} PREFIX##Array;                                                                                                             \
	PREFIX##Array PREFIX##Array_Create( int capacity );                                                                          \
	void PREFIX##Array_Reserve( PREFIX##Array* a, int newCapacity );                                                             \
	void PREFIX##Array_Destroy( PREFIX##Array* a )

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
	void PREFIX##Array_Destroy( PREFIX##Array* a )

// Inline array functions that need the type T to be defined
#define B2_ARRAY_INLINE( T, PREFIX )                                                                                             \
	/* Resize */                                                                                                                 \
	static inline void PREFIX##Array_Resize( PREFIX##Array* a, int count )                                                       \
	{                                                                                                                            \
		PREFIX##Array_Reserve( a, count );                                                                                       \
		a->count = count;                                                                                                        \
	}                                                                                                                            \
	/* Get */                                                                                                                    \
	static inline T* PREFIX##Array_Get( PREFIX##Array* a, int index )                                                            \
	{                                                                                                                            \
		B2_ASSERT( 0 <= index && index < a->count );                                                                             \
		return a->data + index;                                                                                                  \
	}                                                                                                                            \
	/* Add */                                                                                                                    \
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
	/* Push */                                                                                                                   \
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
	/* Set */                                                                                                                    \
	static inline void PREFIX##Array_Set( PREFIX##Array* a, int index, T value )                                                 \
	{                                                                                                                            \
		B2_ASSERT( 0 <= index && index < a->count );                                                                             \
		a->data[index] = value;                                                                                                  \
	}                                                                                                                            \
	/* RemoveSwap */                                                                                                             \
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
	/* Pop */                                                                                                                    \
	static inline T PREFIX##Array_Pop( PREFIX##Array* a )                                                                        \
	{                                                                                                                            \
		B2_ASSERT( a->count > 0 );                                                                                               \
		T value = a->data[a->count - 1];                                                                                         \
		a->count -= 1;                                                                                                           \
		return value;                                                                                                            \
	}                                                                                                                            \
	/* Clear */                                                                                                                  \
	static inline void PREFIX##Array_Clear( PREFIX##Array* a )                                                                   \
	{                                                                                                                            \
		a->count = 0;                                                                                                            \
	}                                                                                                                            \
	/* ByteCount */                                                                                                              \
	static inline int PREFIX##Array_ByteCount( PREFIX##Array* a )                                                                \
	{                                                                                                                            \
		return (int)( a->capacity * sizeof( T ) );                                                                               \
	}

// Array implementations to be instantiated in a source file where the type T is known
#define B2_ARRAY_SOURCE( T, PREFIX )                                                                                             \
	/* Create */                                                                                                                 \
	PREFIX##Array PREFIX##Array_Create( int capacity )                                                                           \
	{                                                                                                                            \
		PREFIX##Array a = { 0 };                                                                                                 \
		if ( capacity > 0 )                                                                                                      \
		{                                                                                                                        \
			a.data = b2Alloc( capacity * sizeof( T ) );                                                                          \
			a.capacity = capacity;                                                                                               \
		}                                                                                                                        \
		return a;                                                                                                                \
	}                                                                                                                            \
	/* Reserve */                                                                                                                \
	void PREFIX##Array_Reserve( PREFIX##Array* a, int newCapacity )                                                              \
	{                                                                                                                            \
		if ( newCapacity <= a->capacity )                                                                                        \
		{                                                                                                                        \
			return;                                                                                                              \
		}                                                                                                                        \
		a->data = b2GrowAlloc( a->data, a->capacity * sizeof( T ), newCapacity * sizeof( T ) );                                  \
		a->capacity = newCapacity;                                                                                               \
	}                                                                                                                            \
	/* Destroy */                                                                                                                \
	void PREFIX##Array_Destroy( PREFIX##Array* a )                                                                               \
	{                                                                                                                            \
		b2Free( a->data, a->capacity * sizeof( T ) );                                                                            \
		a->data = NULL;                                                                                                          \
		a->count = 0;                                                                                                            \
		a->capacity = 0;                                                                                                         \
	}

B2_DECLARE_ARRAY_NATIVE( int, b2Int );
B2_ARRAY_INLINE( int, b2Int )

// Declare all the arrays
B2_ARRAY_DECLARE( b2Body, b2Body );
B2_ARRAY_DECLARE( b2BodyMoveEvent, b2BodyMoveEvent );
B2_ARRAY_DECLARE( b2BodySim, b2BodySim );
B2_ARRAY_DECLARE( b2BodyState, b2BodyState );
B2_ARRAY_DECLARE( b2ChainShape, b2ChainShape );
B2_ARRAY_DECLARE( b2Contact, b2Contact );
B2_ARRAY_DECLARE( b2ContactBeginTouchEvent, b2ContactBeginTouchEvent );
B2_ARRAY_DECLARE( b2ContactEndTouchEvent, b2ContactEndTouchEvent );
B2_ARRAY_DECLARE( b2ContactHitEvent, b2ContactHitEvent );
B2_ARRAY_DECLARE( b2ContactSim, b2ContactSim );
B2_ARRAY_DECLARE( b2Island, b2Island );
B2_ARRAY_DECLARE( b2IslandSim, b2IslandSim );
B2_ARRAY_DECLARE( b2Joint, b2Joint );
B2_ARRAY_DECLARE( b2JointSim, b2JointSim );
B2_ARRAY_DECLARE( b2Sensor, b2Sensor );
B2_ARRAY_DECLARE( b2SensorBeginTouchEvent, b2SensorBeginTouchEvent );
B2_ARRAY_DECLARE( b2SensorEndTouchEvent, b2SensorEndTouchEvent );
B2_ARRAY_DECLARE( b2SensorTaskContext, b2SensorTaskContext );
B2_ARRAY_DECLARE( b2Shape, b2Shape );
B2_ARRAY_DECLARE( b2ShapeRef, b2ShapeRef );
B2_ARRAY_DECLARE( b2SolverSet, b2SolverSet );
B2_ARRAY_DECLARE( b2TaskContext, b2TaskContext );
