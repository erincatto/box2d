// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#define NULL_INDEX -1

// Array declaration that doesn't need the TYPE to be defined
#define ARRAY_DECLARE( TYPE )                                                                                                    \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		TYPE* data;                                                                                                              \
		int count;                                                                                                               \
		int capacity;                                                                                                            \
	} TYPE##_Array;                                                                                                              \
	TYPE##_Array TYPE##_Array_Create( int capacity );                                                                            \
	void TYPE##_Array_Reserve( TYPE##_Array* a, int newCapacity );                                                               \
	void TYPE##_Array_Destroy( TYPE##_Array* a )

// Inline array functions that need the TYPE to be defined
#define ARRAY_INLINE( TYPE )                                                                                                     \
	/* Resize */                                                                                                                 \
	static inline void TYPE##_Array_Resize( TYPE##_Array* a, int count )                                                         \
	{                                                                                                                            \
		TYPE##_Array_Reserve( a, count );                                                                                        \
		a->count = count;                                                                                                        \
	}                                                                                                                            \
	/* Get */                                                                                                                    \
	static inline TYPE* TYPE##_Array_Get( TYPE##_Array* a, int index )                                                           \
	{                                                                                                                            \
		assert( 0 <= index && index < a->count );                                                                                \
		return a->data + index;                                                                                                  \
	}                                                                                                                            \
	/* Add */                                                                                                                    \
	static inline TYPE* TYPE##_Array_Add( TYPE##_Array* a )                                                                      \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			TYPE##_Array_Reserve( a, newCapacity );                                                                              \
		}                                                                                                                        \
		a->count += 1;                                                                                                           \
		return a->data + ( a->count - 1 );                                                                                       \
	}                                                                                                                            \
	/* Push */                                                                                                                   \
	static inline void TYPE##_Array_Push( TYPE##_Array* a, TYPE value )                                                          \
	{                                                                                                                            \
		if ( a->count == a->capacity )                                                                                           \
		{                                                                                                                        \
			int newCapacity = a->capacity < 2 ? 2 : a->capacity + ( a->capacity >> 1 );                                          \
			TYPE##_Array_Reserve( a, newCapacity );                                                                              \
		}                                                                                                                        \
		a->data[a->count] = value;                                                                                               \
		a->count += 1;                                                                                                           \
	}                                                                                                                            \
	/* Set */                                                                                                                    \
	static inline void TYPE##_Array_Set( TYPE##_Array* a, int index, TYPE value )                                                \
	{                                                                                                                            \
		assert( 0 <= index && index < a->count );                                                                                \
		a->data[index] = value;                                                                                                  \
	}                                                                                                                            \
	/* RemoveSwap */                                                                                                             \
	static inline int TYPE##_Array_RemoveSwap( TYPE##_Array* a, int index )                                                      \
	{                                                                                                                            \
		assert( 0 <= index && index < a->count );                                                                                \
		int movedIndex = NULL_INDEX;                                                                                          \
		if ( index != a->count - 1 )                                                                                             \
		{                                                                                                                        \
			movedIndex = a->count - 1;                                                                                           \
			a->data[index] = a->data[movedIndex];                                                                                \
		}                                                                                                                        \
		a->count -= 1;                                                                                                           \
		return movedIndex;                                                                                                       \
	}                                                                                                                            \
	/* Pop */                                                                                                                    \
	static inline TYPE TYPE##_Array_Pop( TYPE##_Array* a )                                                                          \
	{                                                                                                                            \
		assert( a->count > 0 );                                                                                                  \
		TYPE value = a->data[a->count - 1];                                                                                         \
		a->count -= 1;                                                                                                           \
		return value;                                                                                                            \
	}                                                                                                                            \
	/* Clear */                                                                                                                  \
	static inline void TYPE##_Array_Clear( TYPE##_Array* a )                                                                     \
	{                                                                                                                            \
		a->count = 0;                                                                                                            \
	}                                                                                                                            \
	/* ByteCount */                                                                                                              \
	static inline int TYPE##_Array_ByteCount( TYPE##_Array* a )                                                                  \
	{                                                                                                                            \
		return (int)( a->capacity * sizeof( TYPE ) );                                                                               \
	}

#define ARRAY_SOURCE( TYPE )                                                                                                     \
	/* Create */                                                                                                                 \
	TYPE##_Array TYPE##_Array_Create( int capacity )                                                                             \
	{                                                                                                                            \
		TYPE##_Array a = { 0 };                                                                                                  \
		if ( capacity > 0 )                                                                                                      \
		{                                                                                                                        \
			a.data = malloc( capacity * sizeof( TYPE ) );                                                                        \
			a.capacity = capacity;                                                                                               \
		}                                                                                                                        \
		return a;                                                                                                                \
	}                                                                                                                            \
	/* Reserve */                                                                                                                \
	void TYPE##_Array_Reserve( TYPE##_Array* a, int newCapacity )                                                                \
	{                                                                                                                            \
		if ( newCapacity <= a->capacity )                                                                                        \
		{                                                                                                                        \
			return;                                                                                                              \
		}                                                                                                                        \
		a->data = GrowAlloc( a->data, a->capacity * sizeof( TYPE ), newCapacity * sizeof( TYPE ) );                              \
		a->capacity = newCapacity;                                                                                               \
	}                                                                                                                            \
	/* Destroy */                                                                                                                \
	void TYPE##_Array_Destroy( TYPE##_Array* a )                                                                                 \
	{                                                                                                                            \
		free( a->data, a->capacity * sizeof( TYPE ) );                                                                           \
		a->data = NULL;                                                                                                          \
		a->count = 0;                                                                                                            \
		a->capacity = 0;                                                                                                         \
	}

void* GrowAlloc( void* oldMem, int oldSize, int newSize );
