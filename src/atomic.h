// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdbool.h>
#include <stdint.h>

#if defined( _MSC_VER )
#include <intrin.h>
#endif

#if defined( _M_X64 ) || defined( __x86_64__ ) || defined( _M_IX86 ) || defined( __i386__ )
#include <immintrin.h>
#endif

static inline void b2AtomicStoreInt( b2AtomicInt* a, int value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int b2AtomicLoadInt( b2AtomicInt* a )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	int value = __iso_volatile_load32( (volatile __int32*)&a->value );
#if defined( _M_ARM ) || defined( _M_ARM64 ) || defined( _M_ARM64EC )
	__dmb( 0xB );
#else
	_ReadWriteBarrier();
#endif
	return value;
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int b2AtomicFetchAddInt( b2AtomicInt* a, int increment )
{
#if defined( _MSC_VER )
	return _InterlockedExchangeAdd( (long*)&a->value, (long)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline bool b2AtomicCompareExchangeInt( b2AtomicInt* a, int expected, int desired )
{
#if defined( _MSC_VER )
	return _InterlockedCompareExchange( (long*)&a->value, (long)desired, (long)expected ) == expected;
#elif defined( __GNUC__ ) || defined( __clang__ )
	// The value written to expected is ignored
	return __atomic_compare_exchange_n( &a->value, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline void b2AtomicStoreU32( b2AtomicU32* a, uint32_t value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline uint32_t b2AtomicLoadU32( b2AtomicU32* a )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	uint32_t value = (uint32_t)__iso_volatile_load32( (volatile __int32*)&a->value );
#if defined( _M_ARM ) || defined( _M_ARM64 ) || defined( _M_ARM64EC )
	__dmb( 0xB );
#else
	_ReadWriteBarrier();
#endif
	return value;
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int64_t b2AtomicFetchAddI64( b2AtomicI64* a, int64_t increment )
{
#if defined( _MSC_VER )
	return (int64_t)_InterlockedExchangeAdd64( (__int64*)&a->value, (__int64)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int64_t b2AtomicLoadI64( b2AtomicI64* a )
{
#if defined( _MSC_VER ) && !defined( __clang__ ) && !defined( _M_ARM )
	int64_t value = __iso_volatile_load64( (volatile __int64*)&a->value );
#if defined( _M_ARM64 ) || defined( _M_ARM64EC )
	__dmb( 0xB );
#else
	_ReadWriteBarrier();
#endif
	return value;
#elif defined( _MSC_VER ) && !defined( __clang__ )
	// 32-bit ARM has no plain atomic 64-bit load
	return _InterlockedOr64( (__int64*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

// Denormal flushing is a per thread mode the host can turn on, usually by linking a module
// built with fast math. This breaks determinism.
static inline bool b2IsDenormalFlushEnabled( void )
{
#if defined( _M_X64 ) || defined( __x86_64__ ) || defined( _M_IX86 ) || defined( __i386__ )
	// FTZ is bit 15, DAZ is bit 6
	uint32_t csr = _mm_getcsr();
	return ( csr & 0x8040 ) != 0;
#elif defined( _M_ARM64 ) || defined( __aarch64__ )
	// FZ is bit 24 and flushes denormal inputs and outputs together
	uint64_t fpcr;
#if defined( _MSC_VER )
	fpcr = _ReadStatusReg( ARM64_FPCR );
#else
	__asm__ __volatile__( "mrs %0, fpcr" : "=r"( fpcr ) );
#endif
	return ( fpcr & ( 1ull << 24 ) ) != 0;
#else
	// wasm has no flush mode
	return false;
#endif
}
