// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

// Compare to SDL_AtomicInt

struct b3AtomicInt
{
	int value;
};

struct b3AtomicU32
{
	uint32_t value;
};

#if defined( _MSC_VER )
	#include <intrin.h>

static inline void b3AtomicStoreInt( b2AtomicInt* obj, int desired )
{
	_InterlockedExchange( (volatile long*)obj, (long)desired );
}

static inline int b3AtomicLoadInt( volatile int* obj )
{
	return _InterlockedOr( (volatile long*)obj, 0 );
}

static inline void b3AtomicStoreUInt( volatile uint32_t* obj, uint32_t desired )
{
	_InterlockedExchange( (volatile long*)obj, (long)desired );
}

static inline int b3AtomicFetchAddInt( volatile int* obj, int increment )
{
	return _InterlockedExchangeAdd( (volatile long*)obj, (long)increment );
}

static inline bool b3AtomicCompareExchangeInt( volatile int* obj, int* expected, int desired )
{
	int original = _InterlockedCompareExchange( (volatile long*)obj, (long)desired, (long)*expected );
	if ( original == *expected )
	{
		return true;
	}

	*expected = original;
	return false;
}

#elif defined( __GNUC__ ) || defined( __clang__ )

static inline void b3AtomicStoreInt( volatile int* obj, int desired )
{
	__atomic_store_n( obj, desired, __ATOMIC_SEQ_CST );
}

static inline int b3AtomicLoadInt( volatile int* obj )
{
	return __atomic_load_n( obj, __ATOMIC_SEQ_CST );
}

static inline void b3AtomicStoreUInt( volatile uint32_t* obj, uint32_t desired )
{
	__atomic_store_n( obj, desired, __ATOMIC_SEQ_CST );
}

static inline int b3AtomicFetchAddInt( volatile int* obj, int arg )
{
	return __atomic_fetch_add( obj, arg, __ATOMIC_SEQ_CST );
}

static inline bool b3AtomicCompareExchangeInt( volatile int* obj, int* expected, int desired )
{
	return __atomic_compare_exchange_n( obj, expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST );
}

#else
	#error "Unsupported platform"
#endif
