// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#define NULL_INDEX -1

// Array declaration. Works with forward declaration of TYPE.
#define ARRAY_DECLARE( TYPE )                                                                                                    \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		TYPE* data;                                                                                                              \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} TYPE##Array;                                                                                                              \
	TYPE##Array TYPE##Array_Create( int capacity );                                                                            \
	void TYPE##Array_Reserve( TYPE##Array* a, int newCapacity );                                                               \
	void TYPE##Array_Destroy( TYPE##Array* a )

// Inline array functions that need the TYPE to be defined.
#define ARRAY_INLINE( TYPE )                                                                                                     \
	static inline void TYPE##Array_Resize( TYPE##Array* a, int count )                                                         \
	{                                                                                                                            \
		TYPE##Array_Reserve( a, count );                                                                                        \
		a->count = count;                                                                                                        \
	}                                                                                                                            \
	static inline TYPE* TYPE##Array_Get( TYPE##Array* a, int index )                                                           \
	{                                                                                                                            \
		assert( 0 <= index && index < a->count );                                                                                \
		return a->data + index;                                                                                                  \
	}                                                                                                                            \
	static inline TYPE* TYPE##Array_Add( TYPE##Array* a )                                                                      \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			TYPE##Array_Reserve( a, newCapacity );                                                                              \
		}                                                                                                                        \
		a->count += 1;                                                                                                           \
		return a->data + ( a->count - 1 );                                                                                       \
	}                                                                                                                            \
	static inline void TYPE##Array_Push( TYPE##Array* a, TYPE value )                                                          \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			TYPE##Array_Reserve( a, newCapacity );                                                                              \
		}                                                                                                                        \
		a->data[a->count] = value;                                                                                               \
		a->count += 1;                                                                                                           \
	}                                                                                                                            \
	static inline void TYPE##Array_Set( TYPE##Array* a, int index, TYPE value )                                                \
	{                                                                                                                            \
		assert( 0 <= index && index < a->count );                                                                                \
		a->data[index] = value;                                                                                                  \
	}                                                                                                                            \
	static inline int TYPE##Array_RemoveSwap( TYPE##Array* a, int index )                                                      \
	{                                                                                                                            \
		assert( 0 <= index && index < a->count );                                                                                \
		int movedIndex = NULL_INDEX;                                                                                             \
		if ( index != a->count - 1 )                                                                                             \
		{                                                                                                                        \
			movedIndex = a->count - 1;                                                                                           \
			a->data[index] = a->data[movedIndex];                                                                                \
		}                                                                                                                        \
		a->count -= 1;                                                                                                           \
		return movedIndex;                                                                                                       \
	}                                                                                                                            \
	static inline TYPE TYPE##Array_Pop( TYPE##Array* a )                                                                       \
	{                                                                                                                            \
		assert( a->count > 0 );                                                                                                  \
		TYPE value = a->data[a->count - 1];                                                                                      \
		a->count -= 1;                                                                                                           \
		return value;                                                                                                            \
	}                                                                                                                            \

// These functions go in the source file
#define ARRAY_SOURCE( TYPE )                                                                                                     \
	TYPE##Array TYPE##Array_Create( int capacity )                                                                             \
	{                                                                                                                            \
		TYPE##Array a = { 0 };                                                                                                  \
		if ( capacity > 0 )                                                                                                      \
		{                                                                                                                        \
			a.data = malloc( capacity * sizeof( TYPE ) );                                                                        \
			a.capacity = capacity;                                                                                               \
		}                                                                                                                        \
		return a;                                                                                                                \
	}                                                                                                                            \
	void TYPE##Array_Reserve( TYPE##Array* a, int newCapacity )                                                                \
	{                                                                                                                            \
		if ( newCapacity <= a->capacity )                                                                                        \
		{                                                                                                                        \
			return;                                                                                                              \
		}                                                                                                                        \
		a->data = GrowAlloc( a->data, a->capacity * sizeof( TYPE ), newCapacity * sizeof( TYPE ) );                              \
		a->capacity = newCapacity;                                                                                               \
	}                                                                                                                            \
	void TYPE##Array_Destroy( TYPE##Array* a )                                                                                 \
	{                                                                                                                            \
		free( a->data );                                                                           \
		a->data = NULL;                                                                                                          \
		a->count = 0;                                                                                                            \
		a->capacity = 0;                                                                                                         \
	}

void* GrowAlloc( void* oldMem, int oldSize, int newSize );
