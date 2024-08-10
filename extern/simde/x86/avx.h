/* SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Copyright:
 *   2018-2020 Evan Nemerson <evan@nemerson.com>
 *        2020 Michael R. Crusoe <crusoe@debian.org>
 */

#include "sse.h"
#if !defined(SIMDE_X86_AVX_H)
#define SIMDE_X86_AVX_H

#include "sse4.2.h"
#include "../simde-f16.h"

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DISABLE_UNWANTED_DIAGNOSTICS
SIMDE_BEGIN_DECLS_

typedef union {
  #if defined(SIMDE_VECTOR_SUBSCRIPT)
    SIMDE_ALIGN_TO_32 int8_t          i8 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int16_t        i16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int32_t        i32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int64_t        i64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint8_t         u8 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint16_t       u16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint32_t       u32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint64_t       u64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #if defined(SIMDE_HAVE_INT128_)
    SIMDE_ALIGN_TO_32 simde_int128  i128 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 simde_uint128 u128 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #endif
    SIMDE_ALIGN_TO_32 simde_float32  f32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 simde_float64  f64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int_fast32_t  i32f SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint_fast32_t u32f SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
  #else
    SIMDE_ALIGN_TO_32 int8_t          i8[32];
    SIMDE_ALIGN_TO_32 int16_t        i16[16];
    SIMDE_ALIGN_TO_32 int32_t        i32[8];
    SIMDE_ALIGN_TO_32 int64_t        i64[4];
    SIMDE_ALIGN_TO_32 uint8_t         u8[32];
    SIMDE_ALIGN_TO_32 uint16_t       u16[16];
    SIMDE_ALIGN_TO_32 uint32_t       u32[8];
    SIMDE_ALIGN_TO_32 uint64_t       u64[4];
    SIMDE_ALIGN_TO_32 int_fast32_t  i32f[32 / sizeof(int_fast32_t)];
    SIMDE_ALIGN_TO_32 uint_fast32_t u32f[32 / sizeof(uint_fast32_t)];
    #if defined(SIMDE_HAVE_INT128_)
    SIMDE_ALIGN_TO_32 simde_int128  i128[2];
    SIMDE_ALIGN_TO_32 simde_uint128 u128[2];
    #endif
    SIMDE_ALIGN_TO_32 simde_float32  f32[8];
    SIMDE_ALIGN_TO_32 simde_float64  f64[4];
  #endif

    SIMDE_ALIGN_TO_32 simde__m128_private m128_private[2];
    SIMDE_ALIGN_TO_32 simde__m128         m128[2];

  #if defined(SIMDE_X86_AVX_NATIVE)
    SIMDE_ALIGN_TO_32 __m256         n;
  #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned char)      altivec_u8[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned short)     altivec_u16[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned int)       altivec_u32[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed char)        altivec_i8[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed short)       altivec_i16[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(int)                altivec_i32[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(float)              altivec_f32[2];
    #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE)
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned long long) altivec_u64[2];
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(long long)          altivec_i64[2];
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(double)             altivec_f64[2];
    #endif
  #endif
} simde__m256_private;

typedef union {
  #if defined(SIMDE_VECTOR_SUBSCRIPT)
    SIMDE_ALIGN_TO_32 int8_t          i8 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int16_t        i16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int32_t        i32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int64_t        i64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint8_t         u8 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint16_t       u16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint32_t       u32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint64_t       u64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #if defined(SIMDE_HAVE_INT128_)
    SIMDE_ALIGN_TO_32 simde_int128  i128 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 simde_uint128 u128 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #endif
    SIMDE_ALIGN_TO_32 simde_float32  f32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 simde_float64  f64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int_fast32_t  i32f SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint_fast32_t u32f SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
  #else
    SIMDE_ALIGN_TO_32 int8_t          i8[32];
    SIMDE_ALIGN_TO_32 int16_t        i16[16];
    SIMDE_ALIGN_TO_32 int32_t        i32[8];
    SIMDE_ALIGN_TO_32 int64_t        i64[4];
    SIMDE_ALIGN_TO_32 uint8_t         u8[32];
    SIMDE_ALIGN_TO_32 uint16_t       u16[16];
    SIMDE_ALIGN_TO_32 uint32_t       u32[8];
    SIMDE_ALIGN_TO_32 uint64_t       u64[4];
    #if defined(SIMDE_HAVE_INT128_)
    SIMDE_ALIGN_TO_32 simde_int128  i128[2];
    SIMDE_ALIGN_TO_32 simde_uint128 u128[2];
    #endif
    SIMDE_ALIGN_TO_32 simde_float32  f32[8];
    SIMDE_ALIGN_TO_32 simde_float64  f64[4];
    SIMDE_ALIGN_TO_32 int_fast32_t  i32f[32 / sizeof(int_fast32_t)];
    SIMDE_ALIGN_TO_32 uint_fast32_t u32f[32 / sizeof(uint_fast32_t)];
  #endif

    SIMDE_ALIGN_TO_32 simde__m128d_private m128d_private[2];
    SIMDE_ALIGN_TO_32 simde__m128d         m128d[2];

  #if defined(SIMDE_X86_AVX_NATIVE)
    SIMDE_ALIGN_TO_32 __m256d        n;
  #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned char)      altivec_u8[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned short)     altivec_u16[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned int)       altivec_u32[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed char)        altivec_i8[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed short)       altivec_i16[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed int)         altivec_i32[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(float)              altivec_f32[2];
    #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE)
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned long long) altivec_u64[2];
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed long long)   altivec_i64[2];
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(double)             altivec_f64[2];
    #endif
  #endif
} simde__m256d_private;

typedef union {
  #if defined(SIMDE_VECTOR_SUBSCRIPT)
    SIMDE_ALIGN_TO_32 int8_t          i8 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int16_t        i16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int32_t        i32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int64_t        i64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint8_t         u8 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint16_t       u16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint32_t       u32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint64_t       u64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #if defined(SIMDE_HAVE_INT128_)
    SIMDE_ALIGN_TO_32 simde_int128  i128 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 simde_uint128 u128 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #endif
    #if defined(SIMDE_FLOAT16_VECTOR)
    SIMDE_ALIGN_TO_32 simde_float16  f16 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    #else
    SIMDE_ALIGN_TO_32 simde_float16  f16[16];
    #endif
    SIMDE_ALIGN_TO_32 simde_float32  f32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 simde_float64  f64 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 int_fast32_t  i32f SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
    SIMDE_ALIGN_TO_32 uint_fast32_t u32f SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
  #else
    SIMDE_ALIGN_TO_32 int8_t          i8[32];
    SIMDE_ALIGN_TO_32 int16_t        i16[16];
    SIMDE_ALIGN_TO_32 int32_t        i32[8];
    SIMDE_ALIGN_TO_32 int64_t        i64[4];
    SIMDE_ALIGN_TO_32 uint8_t         u8[32];
    SIMDE_ALIGN_TO_32 uint16_t       u16[16];
    SIMDE_ALIGN_TO_32 uint32_t       u32[8];
    SIMDE_ALIGN_TO_32 uint64_t       u64[4];
    SIMDE_ALIGN_TO_32 int_fast32_t  i32f[32 / sizeof(int_fast32_t)];
    SIMDE_ALIGN_TO_32 uint_fast32_t u32f[32 / sizeof(uint_fast32_t)];
    #if defined(SIMDE_HAVE_INT128_)
    SIMDE_ALIGN_TO_32 simde_int128  i128[2];
    SIMDE_ALIGN_TO_32 simde_uint128 u128[2];
    #endif
    SIMDE_ALIGN_TO_32 simde_float16  f16[16];
    SIMDE_ALIGN_TO_32 simde_float32  f32[8];
    SIMDE_ALIGN_TO_32 simde_float64  f64[4];
  #endif

    SIMDE_ALIGN_TO_32 simde__m128i_private m128i_private[2];
    SIMDE_ALIGN_TO_32 simde__m128i         m128i[2];

  #if defined(SIMDE_X86_AVX_NATIVE)
    SIMDE_ALIGN_TO_32 __m256i        n;
  #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned char)      altivec_u8[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned short)     altivec_u16[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned int)       altivec_u32[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed char)        altivec_i8[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed short)       altivec_i16[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed int)         altivec_i32[2];
    SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(float)              altivec_f32[2];
    #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE)
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(unsigned long long) altivec_u64[2];
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(signed long long)   altivec_i64[2];
      SIMDE_ALIGN_TO_16 SIMDE_POWER_ALTIVEC_VECTOR(double)             altivec_f64[2];
    #endif
  #endif
} simde__m256i_private;

#if defined(SIMDE_X86_AVX_NATIVE)
  typedef __m256 simde__m256;
  typedef __m256i simde__m256i;
  typedef __m256d simde__m256d;
#elif defined(SIMDE_VECTOR_SUBSCRIPT)
  typedef simde_float32 simde__m256  SIMDE_ALIGN_TO_32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
  typedef int_fast32_t  simde__m256i SIMDE_ALIGN_TO_32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
  typedef simde_float64 simde__m256d SIMDE_ALIGN_TO_32 SIMDE_VECTOR(32) SIMDE_MAY_ALIAS;
#else
  typedef simde__m256_private  simde__m256;
  typedef simde__m256i_private simde__m256i;
  typedef simde__m256d_private simde__m256d;
#endif

#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #if !defined(HEDLEY_INTEL_VERSION) && !defined(_AVXINTRIN_H_INCLUDED) && !defined(__AVXINTRIN_H) && !defined(_CMP_EQ_OQ)
    typedef simde__m256 __m256;
    typedef simde__m256i __m256i;
    typedef simde__m256d __m256d;
  #else
    #undef __m256
    #define __m256 simde__m256
    #undef __m256i
    #define __m256i simde__m256i
    #undef __m256d
    #define __m256d simde__m256d
  #endif
#endif

HEDLEY_STATIC_ASSERT(32 == sizeof(simde__m256), "simde__m256 size incorrect");
HEDLEY_STATIC_ASSERT(32 == sizeof(simde__m256_private), "simde__m256_private size incorrect");
HEDLEY_STATIC_ASSERT(32 == sizeof(simde__m256i), "simde__m256i size incorrect");
HEDLEY_STATIC_ASSERT(32 == sizeof(simde__m256i_private), "simde__m256i_private size incorrect");
HEDLEY_STATIC_ASSERT(32 == sizeof(simde__m256d), "simde__m256d size incorrect");
HEDLEY_STATIC_ASSERT(32 == sizeof(simde__m256d_private), "simde__m256d_private size incorrect");
#if defined(SIMDE_CHECK_ALIGNMENT) && defined(SIMDE_ALIGN_OF)
HEDLEY_STATIC_ASSERT(SIMDE_ALIGN_OF(simde__m256) == 32, "simde__m256 is not 32-byte aligned");
HEDLEY_STATIC_ASSERT(SIMDE_ALIGN_OF(simde__m256_private) == 32, "simde__m256_private is not 32-byte aligned");
HEDLEY_STATIC_ASSERT(SIMDE_ALIGN_OF(simde__m256i) == 32, "simde__m256i is not 32-byte aligned");
HEDLEY_STATIC_ASSERT(SIMDE_ALIGN_OF(simde__m256i_private) == 32, "simde__m256i_private is not 32-byte aligned");
HEDLEY_STATIC_ASSERT(SIMDE_ALIGN_OF(simde__m256d) == 32, "simde__m256d is not 32-byte aligned");
HEDLEY_STATIC_ASSERT(SIMDE_ALIGN_OF(simde__m256d_private) == 32, "simde__m256d_private is not 32-byte aligned");
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde__m256_from_private(simde__m256_private v) {
  simde__m256 r;
  simde_memcpy(&r, &v, sizeof(r));
  return r;
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256_private
simde__m256_to_private(simde__m256 v) {
  simde__m256_private r;
  simde_memcpy(&r, &v, sizeof(r));
  return r;
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde__m256i_from_private(simde__m256i_private v) {
  simde__m256i r;
  simde_memcpy(&r, &v, sizeof(r));
  return r;
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i_private
simde__m256i_to_private(simde__m256i v) {
  simde__m256i_private r;
  simde_memcpy(&r, &v, sizeof(r));
  return r;
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde__m256d_from_private(simde__m256d_private v) {
  simde__m256d r;
  simde_memcpy(&r, &v, sizeof(r));
  return r;
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d_private
simde__m256d_to_private(simde__m256d v) {
  simde__m256d_private r;
  simde_memcpy(&r, &v, sizeof(r));
  return r;
}

#define SIMDE_CMP_EQ_OQ     0
#define SIMDE_CMP_LT_OS     1
#define SIMDE_CMP_LE_OS     2
#define SIMDE_CMP_UNORD_Q   3
#define SIMDE_CMP_NEQ_UQ    4
#define SIMDE_CMP_NLT_US    5
#define SIMDE_CMP_NLE_US    6
#define SIMDE_CMP_ORD_Q     7
#define SIMDE_CMP_EQ_UQ     8
#define SIMDE_CMP_NGE_US    9
#define SIMDE_CMP_NGT_US   10
#define SIMDE_CMP_FALSE_OQ 11
#define SIMDE_CMP_NEQ_OQ   12
#define SIMDE_CMP_GE_OS    13
#define SIMDE_CMP_GT_OS    14
#define SIMDE_CMP_TRUE_UQ  15
#define SIMDE_CMP_EQ_OS    16
#define SIMDE_CMP_LT_OQ    17
#define SIMDE_CMP_LE_OQ    18
#define SIMDE_CMP_UNORD_S  19
#define SIMDE_CMP_NEQ_US   20
#define SIMDE_CMP_NLT_UQ   21
#define SIMDE_CMP_NLE_UQ   22
#define SIMDE_CMP_ORD_S    23
#define SIMDE_CMP_EQ_US    24
#define SIMDE_CMP_NGE_UQ   25
#define SIMDE_CMP_NGT_UQ   26
#define SIMDE_CMP_FALSE_OS 27
#define SIMDE_CMP_NEQ_OS   28
#define SIMDE_CMP_GE_OQ    29
#define SIMDE_CMP_GT_OQ    30
#define SIMDE_CMP_TRUE_US  31

#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES) && !defined(_CMP_EQ_OQ)
#define _CMP_EQ_OQ SIMDE_CMP_EQ_OQ
#define _CMP_LT_OS SIMDE_CMP_LT_OS
#define _CMP_LE_OS SIMDE_CMP_LE_OS
#define _CMP_UNORD_Q SIMDE_CMP_UNORD_Q
#define _CMP_NEQ_UQ SIMDE_CMP_NEQ_UQ
#define _CMP_NLT_US SIMDE_CMP_NLT_US
#define _CMP_NLE_US SIMDE_CMP_NLE_US
#define _CMP_ORD_Q SIMDE_CMP_ORD_Q
#define _CMP_EQ_UQ SIMDE_CMP_EQ_UQ
#define _CMP_NGE_US SIMDE_CMP_NGE_US
#define _CMP_NGT_US SIMDE_CMP_NGT_US
#define _CMP_FALSE_OQ SIMDE_CMP_FALSE_OQ
#define _CMP_NEQ_OQ SIMDE_CMP_NEQ_OQ
#define _CMP_GE_OS SIMDE_CMP_GE_OS
#define _CMP_GT_OS SIMDE_CMP_GT_OS
#define _CMP_TRUE_UQ SIMDE_CMP_TRUE_UQ
#define _CMP_EQ_OS SIMDE_CMP_EQ_OS
#define _CMP_LT_OQ SIMDE_CMP_LT_OQ
#define _CMP_LE_OQ SIMDE_CMP_LE_OQ
#define _CMP_UNORD_S SIMDE_CMP_UNORD_S
#define _CMP_NEQ_US SIMDE_CMP_NEQ_US
#define _CMP_NLT_UQ SIMDE_CMP_NLT_UQ
#define _CMP_NLE_UQ SIMDE_CMP_NLE_UQ
#define _CMP_ORD_S SIMDE_CMP_ORD_S
#define _CMP_EQ_US SIMDE_CMP_EQ_US
#define _CMP_NGE_UQ SIMDE_CMP_NGE_UQ
#define _CMP_NGT_UQ SIMDE_CMP_NGT_UQ
#define _CMP_FALSE_OS SIMDE_CMP_FALSE_OS
#define _CMP_NEQ_OS SIMDE_CMP_NEQ_OS
#define _CMP_GE_OQ SIMDE_CMP_GE_OQ
#define _CMP_GT_OQ SIMDE_CMP_GT_OQ
#define _CMP_TRUE_US SIMDE_CMP_TRUE_US
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_castps_pd (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castps_pd(a);
  #else
    return *HEDLEY_REINTERPRET_CAST(simde__m256d*, &a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castps_pd
  #define _mm256_castps_pd(a) simde_mm256_castps_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_castps_si256 (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castps_si256(a);
  #else
    return *HEDLEY_REINTERPRET_CAST(simde__m256i*, &a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castps_si256
  #define _mm256_castps_si256(a) simde_mm256_castps_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_castsi256_pd (simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castsi256_pd(a);
  #else
    return *HEDLEY_REINTERPRET_CAST(simde__m256d*, &a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castsi256_pd
  #define _mm256_castsi256_pd(a) simde_mm256_castsi256_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_castsi256_ps (simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castsi256_ps(a);
  #else
    return *HEDLEY_REINTERPRET_CAST(simde__m256*, &a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castsi256_ps
  #define _mm256_castsi256_ps(a) simde_mm256_castsi256_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_castpd_ps (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castpd_ps(a);
  #else
    return *HEDLEY_REINTERPRET_CAST(simde__m256*, &a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castpd_ps
  #define _mm256_castpd_ps(a) simde_mm256_castpd_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_castpd_si256 (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castpd_si256(a);
  #else
    return *HEDLEY_REINTERPRET_CAST(simde__m256i*, &a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castpd_si256
  #define _mm256_castpd_si256(a) simde_mm256_castpd_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_setzero_si256 (void) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setzero_si256();
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_setzero_si128();
      r_.m128i[1] = simde_mm_setzero_si128();
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = 0;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setzero_si256
  #define _mm256_setzero_si256() simde_mm256_setzero_si256()
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_setzero_ps (void) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setzero_ps();
  #else
    return simde_mm256_castsi256_ps(simde_mm256_setzero_si256());
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setzero_ps
  #define _mm256_setzero_ps() simde_mm256_setzero_ps()
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_setzero_pd (void) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setzero_pd();
  #else
    return simde_mm256_castsi256_pd(simde_mm256_setzero_si256());
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setzero_pd
  #define _mm256_setzero_pd() simde_mm256_setzero_pd()
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_not_ps(simde__m256 a) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a);

  #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
    r_.i32 = ~a_.i32;
  #elif SIMDE_NATURAL_VECTOR_SIZE_GE(128)
    r_.m128[0] = simde_x_mm_not_ps(a_.m128[0]);
    r_.m128[1] = simde_x_mm_not_ps(a_.m128[1]);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = ~(a_.i32[i]);
    }
  #endif

  return simde__m256_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_select_ps(simde__m256 a, simde__m256 b, simde__m256 mask) {
  /* This function is for when you want to blend two elements together
   * according to a mask.  It is similar to _mm256_blendv_ps, except that
   * it is undefined whether the blend is based on the highest bit in
   * each lane (like blendv) or just bitwise operations.  This allows
   * us to implement the function efficiently everywhere.
   *
   * Basically, you promise that all the lanes in mask are either 0 or
   * ~0. */
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_blendv_ps(a, b, mask);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b),
      mask_ = simde__m256_to_private(mask);

    #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32 = a_.i32 ^ ((a_.i32 ^ b_.i32) & mask_.i32);
    #elif SIMDE_NATURAL_VECTOR_SIZE_GE(128)
      r_.m128[0] = simde_x_mm_select_ps(a_.m128[0], b_.m128[0], mask_.m128[0]);
      r_.m128[1] = simde_x_mm_select_ps(a_.m128[1], b_.m128[1], mask_.m128[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] ^ ((a_.i32[i] ^ b_.i32[i]) & mask_.i32[i]);
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_not_pd(simde__m256d a) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a);

  #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
    r_.i64 = ~a_.i64;
  #elif SIMDE_NATURAL_VECTOR_SIZE_GE(128)
    r_.m128d[0] = simde_x_mm_not_pd(a_.m128d[0]);
    r_.m128d[1] = simde_x_mm_not_pd(a_.m128d[1]);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = ~(a_.i64[i]);
    }
  #endif

  return simde__m256d_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_select_pd(simde__m256d a, simde__m256d b, simde__m256d mask) {
  /* This function is for when you want to blend two elements together
   * according to a mask.  It is similar to _mm256_blendv_pd, except that
   * it is undefined whether the blend is based on the highest bit in
   * each lane (like blendv) or just bitwise operations.  This allows
   * us to implement the function efficiently everywhere.
   *
   * Basically, you promise that all the lanes in mask are either 0 or
   * ~0. */
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_blendv_pd(a, b, mask);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b),
      mask_ = simde__m256d_to_private(mask);

    #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i64 = a_.i64 ^ ((a_.i64 ^ b_.i64) & mask_.i64);
    #elif SIMDE_NATURAL_VECTOR_SIZE_GE(128)
      r_.m128d[0] = simde_x_mm_select_pd(a_.m128d[0], b_.m128d[0], mask_.m128d[0]);
      r_.m128d[1] = simde_x_mm_select_pd(a_.m128d[1], b_.m128d[1], mask_.m128d[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i64[i] ^ ((a_.i64[i] ^ b_.i64[i]) & mask_.i64[i]);
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_setone_si256 (void) {
  simde__m256i_private r_;

#if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
  __typeof__(r_.i32f) rv = { 0, };
  r_.i32f = ~rv;
#elif defined(SIMDE_X86_AVX2_NATIVE)
  __m256i t = _mm256_setzero_si256();
  r_.n = _mm256_cmpeq_epi32(t, t);
#else
  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
    r_.i32f[i] = ~HEDLEY_STATIC_CAST(int_fast32_t, 0);
  }
#endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_setone_ps (void) {
  return simde_mm256_castsi256_ps(simde_x_mm256_setone_si256());
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_setone_pd (void) {
  return simde_mm256_castsi256_pd(simde_x_mm256_setone_si256());
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set_epi8 (int8_t e31, int8_t e30, int8_t e29, int8_t e28,
                      int8_t e27, int8_t e26, int8_t e25, int8_t e24,
                      int8_t e23, int8_t e22, int8_t e21, int8_t e20,
                      int8_t e19, int8_t e18, int8_t e17, int8_t e16,
                      int8_t e15, int8_t e14, int8_t e13, int8_t e12,
                      int8_t e11, int8_t e10, int8_t  e9, int8_t  e8,
                      int8_t  e7, int8_t  e6, int8_t  e5, int8_t  e4,
                      int8_t  e3, int8_t  e2, int8_t  e1, int8_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_epi8(e31, e30, e29, e28, e27, e26, e25, e24,
                           e23, e22, e21, e20, e19, e18, e17, e16,
                           e15, e14, e13, e12, e11, e10,  e9,  e8,
                            e7,  e6,  e5,  e4,  e3,  e2,  e1,  e0);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set_epi8(
        e15, e14, e13, e12, e11, e10,  e9,  e8,
        e7,  e6,  e5,  e4,  e3,  e2,  e1,  e0);
      r_.m128i[1] = simde_mm_set_epi8(
        e31, e30, e29, e28, e27, e26, e25, e24,
        e23, e22, e21, e20, e19, e18, e17, e16);
    #else
      r_.i8[ 0] =  e0;
      r_.i8[ 1] =  e1;
      r_.i8[ 2] =  e2;
      r_.i8[ 3] =  e3;
      r_.i8[ 4] =  e4;
      r_.i8[ 5] =  e5;
      r_.i8[ 6] =  e6;
      r_.i8[ 7] =  e7;
      r_.i8[ 8] =  e8;
      r_.i8[ 9] =  e9;
      r_.i8[10] = e10;
      r_.i8[11] = e11;
      r_.i8[12] = e12;
      r_.i8[13] = e13;
      r_.i8[14] = e14;
      r_.i8[15] = e15;
      r_.i8[16] = e16;
      r_.i8[17] = e17;
      r_.i8[18] = e18;
      r_.i8[19] = e19;
      r_.i8[20] = e20;
      r_.i8[21] = e21;
      r_.i8[22] = e22;
      r_.i8[23] = e23;
      r_.i8[24] = e24;
      r_.i8[25] = e25;
      r_.i8[26] = e26;
      r_.i8[27] = e27;
      r_.i8[28] = e28;
      r_.i8[29] = e29;
      r_.i8[30] = e30;
      r_.i8[31] = e31;
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_epi8
  #define _mm256_set_epi8(e31, e30, e29, e28, e27, e26, e25, e24, e23, e22, e21, e20, e19, e18, e17, e16, e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0) \
  simde_mm256_set_epi8(e31, e30, e29, e28, e27, e26, e25, e24, e23, e22, e21, e20, e19, e18, e17, e16, e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set_epi16 (int16_t e15, int16_t e14, int16_t e13, int16_t e12,
                       int16_t e11, int16_t e10, int16_t  e9, int16_t  e8,
                       int16_t  e7, int16_t  e6, int16_t  e5, int16_t  e4,
                       int16_t  e3, int16_t  e2, int16_t  e1, int16_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_epi16(e15, e14, e13, e12, e11, e10,  e9,  e8,
                            e7,  e6,  e5,  e4,  e3,  e2,  e1,  e0);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set_epi16( e7,  e6,  e5,  e4,  e3,  e2,  e1,  e0);
      r_.m128i[1] = simde_mm_set_epi16(e15, e14, e13, e12, e11, e10,  e9,  e8);
    #else
      r_.i16[ 0] =  e0;
      r_.i16[ 1] =  e1;
      r_.i16[ 2] =  e2;
      r_.i16[ 3] =  e3;
      r_.i16[ 4] =  e4;
      r_.i16[ 5] =  e5;
      r_.i16[ 6] =  e6;
      r_.i16[ 7] =  e7;
      r_.i16[ 8] =  e8;
      r_.i16[ 9] =  e9;
      r_.i16[10] = e10;
      r_.i16[11] = e11;
      r_.i16[12] = e12;
      r_.i16[13] = e13;
      r_.i16[14] = e14;
      r_.i16[15] = e15;
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_epi16
  #define _mm256_set_epi16(e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0) \
  simde_mm256_set_epi16(e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set_epi32 (int32_t e7, int32_t e6, int32_t e5, int32_t e4,
                       int32_t e3, int32_t e2, int32_t e1, int32_t e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_epi32(e7, e6, e5, e4, e3, e2, e1, e0);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set_epi32(e3, e2, e1, e0);
      r_.m128i[1] = simde_mm_set_epi32(e7, e6, e5, e4);
    #else
      r_.i32[ 0] =  e0;
      r_.i32[ 1] =  e1;
      r_.i32[ 2] =  e2;
      r_.i32[ 3] =  e3;
      r_.i32[ 4] =  e4;
      r_.i32[ 5] =  e5;
      r_.i32[ 6] =  e6;
      r_.i32[ 7] =  e7;
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_epi32
  #define _mm256_set_epi32(e7, e6, e5, e4, e3, e2, e1, e0) \
  simde_mm256_set_epi32(e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set_epi64x (int64_t  e3, int64_t  e2, int64_t  e1, int64_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_epi64x(e3, e2, e1, e0);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set_epi64x(e1, e0);
      r_.m128i[1] = simde_mm_set_epi64x(e3, e2);
    #else
      r_.i64[0] = e0;
      r_.i64[1] = e1;
      r_.i64[2] = e2;
      r_.i64[3] = e3;
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_epi64x
  #define _mm256_set_epi64x(e3, e2, e1, e0) simde_mm256_set_epi64x(e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_set_epu8 (uint8_t e31, uint8_t e30, uint8_t e29, uint8_t e28,
                        uint8_t e27, uint8_t e26, uint8_t e25, uint8_t e24,
                        uint8_t e23, uint8_t e22, uint8_t e21, uint8_t e20,
                        uint8_t e19, uint8_t e18, uint8_t e17, uint8_t e16,
                        uint8_t e15, uint8_t e14, uint8_t e13, uint8_t e12,
                        uint8_t e11, uint8_t e10, uint8_t  e9, uint8_t  e8,
                        uint8_t  e7, uint8_t  e6, uint8_t  e5, uint8_t  e4,
                        uint8_t  e3, uint8_t  e2, uint8_t  e1, uint8_t  e0) {
  simde__m256i_private r_;

  r_.u8[ 0] =  e0;
  r_.u8[ 1] =  e1;
  r_.u8[ 2] =  e2;
  r_.u8[ 3] =  e3;
  r_.u8[ 4] =  e4;
  r_.u8[ 5] =  e5;
  r_.u8[ 6] =  e6;
  r_.u8[ 7] =  e7;
  r_.u8[ 8] =  e8;
  r_.u8[ 9] =  e9;
  r_.u8[10] = e10;
  r_.u8[11] = e11;
  r_.u8[12] = e12;
  r_.u8[13] = e13;
  r_.u8[14] = e14;
  r_.u8[15] = e15;
  r_.u8[16] = e16;
  r_.u8[17] = e17;
  r_.u8[18] = e18;
  r_.u8[19] = e19;
  r_.u8[20] = e20;
  r_.u8[20] = e20;
  r_.u8[21] = e21;
  r_.u8[22] = e22;
  r_.u8[23] = e23;
  r_.u8[24] = e24;
  r_.u8[25] = e25;
  r_.u8[26] = e26;
  r_.u8[27] = e27;
  r_.u8[28] = e28;
  r_.u8[29] = e29;
  r_.u8[30] = e30;
  r_.u8[31] = e31;

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_set_epu16 (uint16_t e15, uint16_t e14, uint16_t e13, uint16_t e12,
                       uint16_t e11, uint16_t e10, uint16_t  e9, uint16_t  e8,
                       uint16_t  e7, uint16_t  e6, uint16_t  e5, uint16_t  e4,
                       uint16_t  e3, uint16_t  e2, uint16_t  e1, uint16_t  e0) {
  simde__m256i_private r_;

  r_.u16[ 0] =  e0;
  r_.u16[ 1] =  e1;
  r_.u16[ 2] =  e2;
  r_.u16[ 3] =  e3;
  r_.u16[ 4] =  e4;
  r_.u16[ 5] =  e5;
  r_.u16[ 6] =  e6;
  r_.u16[ 7] =  e7;
  r_.u16[ 8] =  e8;
  r_.u16[ 9] =  e9;
  r_.u16[10] = e10;
  r_.u16[11] = e11;
  r_.u16[12] = e12;
  r_.u16[13] = e13;
  r_.u16[14] = e14;
  r_.u16[15] = e15;

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_set_epu32 (uint32_t e7, uint32_t e6, uint32_t e5, uint32_t e4,
                         uint32_t e3, uint32_t e2, uint32_t e1, uint32_t e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_epi32(HEDLEY_STATIC_CAST(int32_t, e7), HEDLEY_STATIC_CAST(int32_t, e6), HEDLEY_STATIC_CAST(int32_t, e5), HEDLEY_STATIC_CAST(int32_t, e4),
                            HEDLEY_STATIC_CAST(int32_t, e3), HEDLEY_STATIC_CAST(int32_t, e2), HEDLEY_STATIC_CAST(int32_t, e1), HEDLEY_STATIC_CAST(int32_t, e0));
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set_epi32(HEDLEY_STATIC_CAST(int32_t, e3), HEDLEY_STATIC_CAST(int32_t, e2), HEDLEY_STATIC_CAST(int32_t, e1), HEDLEY_STATIC_CAST(int32_t, e0));
      r_.m128i[1] = simde_mm_set_epi32(HEDLEY_STATIC_CAST(int32_t, e7), HEDLEY_STATIC_CAST(int32_t, e6), HEDLEY_STATIC_CAST(int32_t, e5), HEDLEY_STATIC_CAST(int32_t, e4));
    #else
      r_.u32[ 0] =  e0;
      r_.u32[ 1] =  e1;
      r_.u32[ 2] =  e2;
      r_.u32[ 3] =  e3;
      r_.u32[ 4] =  e4;
      r_.u32[ 5] =  e5;
      r_.u32[ 6] =  e6;
      r_.u32[ 7] =  e7;
    #endif

    return simde__m256i_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_set_epu64x (uint64_t  e3, uint64_t  e2, uint64_t  e1, uint64_t  e0) {
  simde__m256i_private r_;

  r_.u64[0] = e0;
  r_.u64[1] = e1;
  r_.u64[2] = e2;
  r_.u64[3] = e3;

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_set_ps (simde_float32 e7, simde_float32 e6, simde_float32 e5, simde_float32 e4,
                    simde_float32 e3, simde_float32 e2, simde_float32 e1, simde_float32 e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_ps(e7, e6, e5, e4, e3, e2, e1, e0);
  #else
    simde__m256_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_set_ps(e3, e2, e1, e0);
      r_.m128[1] = simde_mm_set_ps(e7, e6, e5, e4);
    #else
      r_.f32[0] = e0;
      r_.f32[1] = e1;
      r_.f32[2] = e2;
      r_.f32[3] = e3;
      r_.f32[4] = e4;
      r_.f32[5] = e5;
      r_.f32[6] = e6;
      r_.f32[7] = e7;
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_ps
  #define _mm256_set_ps(e7, e6, e5, e4, e3, e2, e1, e0) \
  simde_mm256_set_ps(e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_set_pd (simde_float64 e3, simde_float64 e2, simde_float64 e1, simde_float64 e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set_pd(e3, e2, e1, e0);
  #else
    simde__m256d_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_set_pd(e1, e0);
      r_.m128d[1] = simde_mm_set_pd(e3, e2);
    #else
      r_.f64[0] = e0;
      r_.f64[1] = e1;
      r_.f64[2] = e2;
      r_.f64[3] = e3;
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_pd
  #define _mm256_set_pd(e3, e2, e1, e0) \
  simde_mm256_set_pd(e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_set_m128 (simde__m128 e1, simde__m128 e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_insertf128_ps(_mm256_castps128_ps256(e0), e1, 1);
  #else
    simde__m256_private r_;
    simde__m128_private
      e1_ = simde__m128_to_private(e1),
      e0_ = simde__m128_to_private(e0);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128_private[0] = e0_;
      r_.m128_private[1] = e1_;
    #elif defined(SIMDE_HAVE_INT128_)
      r_.i128[0] = e0_.i128[0];
      r_.i128[1] = e1_.i128[0];
    #else
      r_.i64[0] = e0_.i64[0];
      r_.i64[1] = e0_.i64[1];
      r_.i64[2] = e1_.i64[0];
      r_.i64[3] = e1_.i64[1];
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_m128
  #define _mm256_set_m128(e1, e0) simde_mm256_set_m128(e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_set_m128d (simde__m128d e1, simde__m128d e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_insertf128_pd(_mm256_castpd128_pd256(e0), e1, 1);
  #else
    simde__m256d_private r_;
    simde__m128d_private
      e1_ = simde__m128d_to_private(e1),
      e0_ = simde__m128d_to_private(e0);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d_private[0] = e0_;
      r_.m128d_private[1] = e1_;
    #else
      r_.i64[0] = e0_.i64[0];
      r_.i64[1] = e0_.i64[1];
      r_.i64[2] = e1_.i64[0];
      r_.i64[3] = e1_.i64[1];
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_m128d
  #define _mm256_set_m128d(e1, e0) simde_mm256_set_m128d(e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set_m128i (simde__m128i e1, simde__m128i e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_insertf128_si256(_mm256_castsi128_si256(e0), e1, 1);
  #else
    simde__m256i_private r_;
    simde__m128i_private
      e1_ = simde__m128i_to_private(e1),
      e0_ = simde__m128i_to_private(e0);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i_private[0] = e0_;
      r_.m128i_private[1] = e1_;
    #else
      r_.i64[0] = e0_.i64[0];
      r_.i64[1] = e0_.i64[1];
      r_.i64[2] = e1_.i64[0];
      r_.i64[3] = e1_.i64[1];
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set_m128i
  #define _mm256_set_m128i(e1, e0) simde_mm256_set_m128i(e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set1_epi8 (int8_t a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set1_epi8(a);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set1_epi8(a);
      r_.m128i[1] = simde_mm_set1_epi8(a);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set1_epi8
  #define _mm256_set1_epi8(a) simde_mm256_set1_epi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set1_epi16 (int16_t a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set1_epi16(a);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set1_epi16(a);
      r_.m128i[1] = simde_mm_set1_epi16(a);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set1_epi16
  #define _mm256_set1_epi16(a) simde_mm256_set1_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set1_epi32 (int32_t a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set1_epi32(a);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set1_epi32(a);
      r_.m128i[1] = simde_mm_set1_epi32(a);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set1_epi32
  #define _mm256_set1_epi32(a) simde_mm256_set1_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_set1_epi64x (int64_t a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set1_epi64x(a);
  #else
    simde__m256i_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_set1_epi64x(a);
      r_.m128i[1] = simde_mm_set1_epi64x(a);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set1_epi64x
  #define _mm256_set1_epi64x(a) simde_mm256_set1_epi64x(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_set1_ps (simde_float32 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set1_ps(a);
  #else
    simde__m256_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_set1_ps(a);
      r_.m128[1] = simde_mm_set1_ps(a);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a;
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set1_ps
  #define _mm256_set1_ps(a) simde_mm256_set1_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_set1_pd (simde_float64 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_set1_pd(a);
  #else
    simde__m256d_private r_;

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_set1_pd(a);
      r_.m128d[1] = simde_mm_set1_pd(a);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = a;
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_set1_pd
  #define _mm256_set1_pd(a) simde_mm256_set1_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_deinterleaveeven_epi16 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_x_mm_deinterleaveeven_epi16(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_x_mm_deinterleaveeven_epi16(a_.m128i[1], b_.m128i[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i16 = SIMDE_SHUFFLE_VECTOR_(16, 32, a_.i16, b_.i16, 0, 2, 4, 6, 16, 18, 20, 22, 8, 10, 12, 14, 24, 26, 28, 30);
  #else
    const size_t halfway_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 2;
    const size_t quarter_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.i16[i] = a_.i16[2 * i];
      r_.i16[i + quarter_point] = b_.i16[2 * i];
      r_.i16[halfway_point + i] = a_.i16[halfway_point + 2 * i];
      r_.i16[halfway_point + i + quarter_point] = b_.i16[halfway_point + 2 * i];
    }
  #endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_deinterleaveodd_epi16 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_x_mm_deinterleaveodd_epi16(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_x_mm_deinterleaveodd_epi16(a_.m128i[1], b_.m128i[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i16 = SIMDE_SHUFFLE_VECTOR_(16, 32, a_.i16, b_.i16, 1, 3, 5, 7, 17, 19, 21, 23, 9, 11, 13, 15, 25, 27, 29, 31);
  #else
    const size_t halfway_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 2;
    const size_t quarter_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.i16[i] = a_.i16[2 * i + 1];
      r_.i16[i + quarter_point] = b_.i16[2 * i + 1];
      r_.i16[halfway_point + i] = a_.i16[halfway_point + 2 * i + 1];
      r_.i16[halfway_point + i + quarter_point] = b_.i16[halfway_point + 2 * i + 1];
    }
  #endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_deinterleaveeven_epi32 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_x_mm_deinterleaveeven_epi32(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_x_mm_deinterleaveeven_epi32(a_.m128i[1], b_.m128i[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.i32, b_.i32, 0, 2, 8, 10, 4, 6, 12, 14);
  #else
    const size_t halfway_point = (sizeof(r_.i32) / sizeof(r_.i32[0])) / 2;
    const size_t quarter_point = (sizeof(r_.i32) / sizeof(r_.i32[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.i32[i] = a_.i32[2 * i];
      r_.i32[i + quarter_point] = b_.i32[2 * i];
      r_.i32[halfway_point + i] = a_.i32[halfway_point + 2 * i];
      r_.i32[halfway_point + i + quarter_point] = b_.i32[halfway_point + 2 * i];
    }
  #endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_deinterleaveodd_epi32 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_x_mm_deinterleaveodd_epi32(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_x_mm_deinterleaveodd_epi32(a_.m128i[1], b_.m128i[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.i32, b_.i32, 1, 3, 9, 11, 5, 7, 13, 15);
  #else
    const size_t halfway_point = (sizeof(r_.i32) / sizeof(r_.i32[0])) / 2;
    const size_t quarter_point = (sizeof(r_.i32) / sizeof(r_.i32[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.i32[i] = a_.i32[2 * i + 1];
      r_.i32[i + quarter_point] = b_.i32[2 * i + 1];
      r_.i32[halfway_point + i] = a_.i32[halfway_point + 2 * i + 1];
      r_.i32[halfway_point + i + quarter_point] = b_.i32[halfway_point + 2 * i + 1];
    }
  #endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_deinterleaveeven_ps (simde__m256 a, simde__m256 b) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a),
    b_ = simde__m256_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128[0] = simde_x_mm_deinterleaveeven_ps(a_.m128[0], b_.m128[0]);
    r_.m128[1] = simde_x_mm_deinterleaveeven_ps(a_.m128[1], b_.m128[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.f32, b_.f32, 0, 2, 8, 10, 4, 6, 12, 14);
  #else
    const size_t halfway_point = (sizeof(r_.f32) / sizeof(r_.f32[0])) / 2;
    const size_t quarter_point = (sizeof(r_.f32) / sizeof(r_.f32[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.f32[i] = a_.f32[2 * i];
      r_.f32[i + quarter_point] = b_.f32[2 * i];
      r_.f32[halfway_point + i] = a_.f32[halfway_point + 2 * i];
      r_.f32[halfway_point + i + quarter_point] = b_.f32[halfway_point + 2 * i];
    }
  #endif

  return simde__m256_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_deinterleaveodd_ps (simde__m256 a, simde__m256 b) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a),
    b_ = simde__m256_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128[0] = simde_x_mm_deinterleaveodd_ps(a_.m128[0], b_.m128[0]);
    r_.m128[1] = simde_x_mm_deinterleaveodd_ps(a_.m128[1], b_.m128[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.f32, b_.f32, 1, 3, 9, 11, 5, 7, 13, 15);
  #else
    const size_t halfway_point = (sizeof(r_.f32) / sizeof(r_.f32[0])) / 2;
    const size_t quarter_point = (sizeof(r_.f32) / sizeof(r_.f32[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.f32[i] = a_.f32[2 * i + 1];
      r_.f32[i + quarter_point] = b_.f32[2 * i + 1];
      r_.f32[halfway_point + i] = a_.f32[halfway_point + 2 * i + 1];
      r_.f32[halfway_point + i + quarter_point] = b_.f32[halfway_point + 2 * i + 1];
    }
  #endif

  return simde__m256_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_deinterleaveeven_pd (simde__m256d a, simde__m256d b) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a),
    b_ = simde__m256d_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128d[0] = simde_x_mm_deinterleaveeven_pd(a_.m128d[0], b_.m128d[0]);
    r_.m128d[1] = simde_x_mm_deinterleaveeven_pd(a_.m128d[1], b_.m128d[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.f64, b_.f64, 0, 4, 2, 6);
  #else
    const size_t halfway_point = (sizeof(r_.f64) / sizeof(r_.f64[0])) / 2;
    const size_t quarter_point = (sizeof(r_.f64) / sizeof(r_.f64[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.f64[i] = a_.f64[2 * i];
      r_.f64[i + quarter_point] = b_.f64[2 * i];
      r_.f64[halfway_point + i] = a_.f64[halfway_point + 2 * i];
      r_.f64[halfway_point + i + quarter_point] = b_.f64[halfway_point + 2 * i];
    }
  #endif

  return simde__m256d_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_deinterleaveodd_pd (simde__m256d a, simde__m256d b) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a),
    b_ = simde__m256d_to_private(b);

  #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    r_.m128d[0] = simde_x_mm_deinterleaveodd_pd(a_.m128d[0], b_.m128d[0]);
    r_.m128d[1] = simde_x_mm_deinterleaveodd_pd(a_.m128d[1], b_.m128d[1]);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.f64, b_.f64, 1, 5, 3, 7);
  #else
    const size_t halfway_point = (sizeof(r_.f64) / sizeof(r_.f64[0])) / 2;
    const size_t quarter_point = (sizeof(r_.f64) / sizeof(r_.f64[0])) / 4;
    for (size_t i = 0 ; i < quarter_point ; i++) {
      r_.f64[i] = a_.f64[2 * i + 1];
      r_.f64[i + quarter_point] = b_.f64[2 * i + 1];
      r_.f64[halfway_point + i] = a_.f64[halfway_point + 2 * i + 1];
      r_.f64[halfway_point + i + quarter_point] = b_.f64[halfway_point + 2 * i + 1];
    }
  #endif

  return simde__m256d_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_abs_ps(simde__m256 a) {
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = simde_math_fabsf(a_.f32[i]);
      }
    return simde__m256_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_abs_pd(simde__m256d a) {
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a);

      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = simde_math_fabs(a_.f64[i]);
      }
    return simde__m256d_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_add_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_add_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_add_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_add_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f32 = a_.f32 + b_.f32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[i] + b_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_add_ps
  #define _mm256_add_ps(a, b) simde_mm256_add_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_hadd_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_hadd_ps(a, b);
  #else
    return simde_mm256_add_ps(simde_x_mm256_deinterleaveeven_ps(a, b), simde_x_mm256_deinterleaveodd_ps(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hadd_ps
  #define _mm256_hadd_ps(a, b) simde_mm256_hadd_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_add_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_add_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_add_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_add_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f64 = a_.f64 + b_.f64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = a_.f64[i] + b_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_add_pd
  #define _mm256_add_pd(a, b) simde_mm256_add_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_hadd_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_hadd_pd(a, b);
  #else
      return simde_mm256_add_pd(simde_x_mm256_deinterleaveeven_pd(a, b), simde_x_mm256_deinterleaveodd_pd(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hadd_pd
  #define _mm256_hadd_pd(a, b) simde_mm256_hadd_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_addsub_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_addsub_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_addsub_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_addsub_ps(a_.m128[1], b_.m128[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i += 2) {
        r_.f32[  i  ] = a_.f32[  i  ] - b_.f32[  i  ];
        r_.f32[i + 1] = a_.f32[i + 1] + b_.f32[i + 1];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_addsub_ps
  #define _mm256_addsub_ps(a, b) simde_mm256_addsub_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_addsub_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_addsub_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_addsub_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_addsub_pd(a_.m128d[1], b_.m128d[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i += 2) {
        r_.f64[  i  ] = a_.f64[  i  ] - b_.f64[  i  ];
        r_.f64[i + 1] = a_.f64[i + 1] + b_.f64[i + 1];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_addsub_pd
  #define _mm256_addsub_pd(a, b) simde_mm256_addsub_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_and_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_and_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_and_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_and_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f & b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = a_.i32f[i] & b_.i32f[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_and_ps
  #define _mm256_and_ps(a, b) simde_mm256_and_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_and_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_and_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_and_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_and_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f & b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = a_.i32f[i] & b_.i32f[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_and_pd
  #define _mm256_and_pd(a, b) simde_mm256_and_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_andnot_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_andnot_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_andnot_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_andnot_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = ~a_.i32f & b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = ~a_.i32f[i] & b_.i32f[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_andnot_ps
  #define _mm256_andnot_ps(a, b) simde_mm256_andnot_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_andnot_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_andnot_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_andnot_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_andnot_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = ~a_.i32f & b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = ~a_.i32f[i] & b_.i32f[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_andnot_pd
  #define _mm256_andnot_pd(a, b) simde_mm256_andnot_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_blend_ps (simde__m256 a, simde__m256 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a),
    b_ = simde__m256_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
    r_.f32[i] = ((imm8 >> i) & 1) ? b_.f32[i] : a_.f32[i];
  }

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_blend_ps(a, b, imm8) _mm256_blend_ps(a, b, imm8)
#elif SIMDE_NATURAL_VECTOR_SIZE_LE(128)
#  define simde_mm256_blend_ps(a, b, imm8) \
      simde_mm256_set_m128( \
          simde_mm_blend_ps(simde_mm256_extractf128_ps(a, 1), simde_mm256_extractf128_ps(b, 1), (imm8) >> 4), \
          simde_mm_blend_ps(simde_mm256_extractf128_ps(a, 0), simde_mm256_extractf128_ps(b, 0), (imm8) & 0x0F))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blend_ps
  #define _mm256_blend_ps(a, b, imm8) simde_mm256_blend_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_blend_pd (simde__m256d a, simde__m256d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a),
    b_ = simde__m256d_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
    r_.f64[i] = ((imm8 >> i) & 1) ? b_.f64[i] : a_.f64[i];
  }
  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_blend_pd(a, b, imm8) _mm256_blend_pd(a, b, imm8)
#elif SIMDE_NATURAL_VECTOR_SIZE_LE(128)
#  define simde_mm256_blend_pd(a, b, imm8) \
      simde_mm256_set_m128d( \
          simde_mm_blend_pd(simde_mm256_extractf128_pd(a, 1), simde_mm256_extractf128_pd(b, 1), (imm8) >> 2), \
          simde_mm_blend_pd(simde_mm256_extractf128_pd(a, 0), simde_mm256_extractf128_pd(b, 0), (imm8) & 3))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blend_pd
  #define _mm256_blend_pd(a, b, imm8) simde_mm256_blend_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_blendv_ps (simde__m256 a, simde__m256 b, simde__m256 mask) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_blendv_ps(a, b, mask);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b),
      mask_ = simde__m256_to_private(mask);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_blendv_ps(a_.m128[0], b_.m128[0], mask_.m128[0]);
      r_.m128[1] = simde_mm_blendv_ps(a_.m128[1], b_.m128[1], mask_.m128[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.f32[i] = (mask_.u32[i] & (UINT32_C(1) << 31)) ? b_.f32[i] : a_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blendv_ps
  #define _mm256_blendv_ps(a, b, imm8) simde_mm256_blendv_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_blendv_pd (simde__m256d a, simde__m256d b, simde__m256d mask) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_blendv_pd(a, b, mask);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b),
      mask_ = simde__m256d_to_private(mask);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_blendv_pd(a_.m128d[0], b_.m128d[0], mask_.m128d[0]);
      r_.m128d[1] = simde_mm_blendv_pd(a_.m128d[1], b_.m128d[1], mask_.m128d[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
        r_.f64[i] = (mask_.u64[i] & (UINT64_C(1) << 63)) ? b_.f64[i] : a_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blendv_pd
  #define _mm256_blendv_pd(a, b, imm8) simde_mm256_blendv_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_broadcast_pd (simde__m128d const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_broadcast_pd(mem_addr);
  #else
    simde__m256d_private r_;

    simde__m128d tmp = simde_mm_loadu_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, mem_addr));
    r_.m128d[0] = tmp;
    r_.m128d[1] = tmp;

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcast_pd
  #define _mm256_broadcast_pd(mem_addr) simde_mm256_broadcast_pd(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_broadcast_ps (simde__m128 const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_broadcast_ps(mem_addr);
  #else
    simde__m256_private r_;

    simde__m128 tmp = simde_mm_loadu_ps(HEDLEY_REINTERPRET_CAST(simde_float32 const*, mem_addr));
    r_.m128[0] = tmp;
    r_.m128[1] = tmp;

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcast_ps
  #define _mm256_broadcast_ps(mem_addr) simde_mm256_broadcast_ps(HEDLEY_REINTERPRET_CAST(simde__m128 const*, mem_addr))
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_broadcast_sd (simde_float64 const * a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_broadcast_sd(a);
  #else
    return simde_mm256_set1_pd(*a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcast_sd
  #define _mm256_broadcast_sd(mem_addr) simde_mm256_broadcast_sd(HEDLEY_REINTERPRET_CAST(double const*, mem_addr))
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_broadcast_ss (simde_float32 const * a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_broadcast_ss(a);
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    return simde__m128_from_wasm_v128(wasm_v128_load32_splat(a));
  #else
    return simde_mm_set1_ps(*a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcast_ss
  #define _mm_broadcast_ss(mem_addr) simde_mm_broadcast_ss(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_broadcast_ss (simde_float32 const * a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_broadcast_ss(a);
  #else
    return simde_mm256_set1_ps(*a);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcast_ss
  #define _mm256_broadcast_ss(mem_addr) simde_mm256_broadcast_ss(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_castpd128_pd256 (simde__m128d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castpd128_pd256(a);
  #else
    simde__m256d_private r_;
    simde__m128d_private a_ = simde__m128d_to_private(a);

    r_.m128d_private[0] = a_;

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castpd128_pd256
  #define _mm256_castpd128_pd256(a) simde_mm256_castpd128_pd256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm256_castpd256_pd128 (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castpd256_pd128(a);
  #else
    simde__m256d_private a_ = simde__m256d_to_private(a);
    return a_.m128d[0];
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castpd256_pd128
  #define _mm256_castpd256_pd128(a) simde_mm256_castpd256_pd128(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_castps128_ps256 (simde__m128 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castps128_ps256(a);
  #else
    simde__m256_private r_;
    simde__m128_private a_ = simde__m128_to_private(a);

    r_.m128_private[0] = a_;

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castps128_ps256
  #define _mm256_castps128_ps256(a) simde_mm256_castps128_ps256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm256_castps256_ps128 (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castps256_ps128(a);
  #else
    simde__m256_private a_ = simde__m256_to_private(a);
    return a_.m128[0];
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castps256_ps128
  #define _mm256_castps256_ps128(a) simde_mm256_castps256_ps128(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_castsi128_si256 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castsi128_si256(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    r_.m128i_private[0] = a_;

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castsi128_si256
  #define _mm256_castsi128_si256(a) simde_mm256_castsi128_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_castsi256_si128 (simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_castsi256_si128(a);
  #else
    simde__m256i_private a_ = simde__m256i_to_private(a);
    return a_.m128i[0];
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_castsi256_si128
  #define _mm256_castsi256_si128(a) simde_mm256_castsi256_si128(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_round_ps (simde__m256 a, const int rounding) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a);

  switch (rounding & ~SIMDE_MM_FROUND_NO_EXC) {
    #if defined(simde_math_nearbyintf)
      case SIMDE_MM_FROUND_CUR_DIRECTION:
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.f32[i] = simde_math_nearbyintf(a_.f32[i]);
        }
        break;
    #endif

    #if defined(simde_math_roundf)
      case SIMDE_MM_FROUND_TO_NEAREST_INT:
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.f32[i] = simde_math_roundf(a_.f32[i]);
        }
        break;
    #endif

    #if defined(simde_math_floorf)
      case SIMDE_MM_FROUND_TO_NEG_INF:
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.f32[i] = simde_math_floorf(a_.f32[i]);
        }
        break;
    #endif

    #if defined(simde_math_ceilf)
      case SIMDE_MM_FROUND_TO_POS_INF:
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.f32[i] = simde_math_ceilf(a_.f32[i]);
        }
        break;
    #endif

    #if defined(simde_math_truncf)
      case SIMDE_MM_FROUND_TO_ZERO:
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.f32[i] = simde_math_truncf(a_.f32[i]);
        }
        break;
    #endif

    default:
      HEDLEY_UNREACHABLE_RETURN(simde_mm256_undefined_ps());
  }

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_round_ps(a, rounding) _mm256_round_ps(a, rounding)
#elif SIMDE_NATURAL_VECTOR_SIZE_LE(128) && defined(SIMDE_STATEMENT_EXPR_)
  #define simde_mm256_round_ps(a, rounding) SIMDE_STATEMENT_EXPR_(({ \
    simde__m256_private \
      simde_mm256_round_ps_r_ = simde__m256_to_private(simde_mm256_setzero_ps()), \
      simde_mm256_round_ps_a_ = simde__m256_to_private(a); \
    \
    for (size_t simde_mm256_round_ps_i = 0 ; simde_mm256_round_ps_i < (sizeof(simde_mm256_round_ps_r_.m128) / sizeof(simde_mm256_round_ps_r_.m128[0])) ; simde_mm256_round_ps_i++) { \
      simde_mm256_round_ps_r_.m128[simde_mm256_round_ps_i] = simde_mm_round_ps(simde_mm256_round_ps_a_.m128[simde_mm256_round_ps_i], rounding); \
    } \
    \
    simde__m256_from_private(simde_mm256_round_ps_r_); \
  }))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_round_ps
  #define _mm256_round_ps(a, rounding) simde_mm256_round_ps(a, rounding)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_round_pd (simde__m256d a, const int rounding) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a);

  switch (rounding & ~SIMDE_MM_FROUND_NO_EXC) {
    #if defined(simde_math_nearbyint)
      case SIMDE_MM_FROUND_CUR_DIRECTION:
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_nearbyint(a_.f64[i]);
        }
        break;
    #endif

    #if defined(simde_math_round)
      case SIMDE_MM_FROUND_TO_NEAREST_INT:
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_round(a_.f64[i]);
        }
        break;
    #endif

    #if defined(simde_math_floor)
      case SIMDE_MM_FROUND_TO_NEG_INF:
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_floor(a_.f64[i]);
        }
        break;
    #endif

    #if defined(simde_math_ceil)
      case SIMDE_MM_FROUND_TO_POS_INF:
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_ceil(a_.f64[i]);
        }
        break;
    #endif

    #if defined(simde_math_trunc)
      case SIMDE_MM_FROUND_TO_ZERO:
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_trunc(a_.f64[i]);
        }
        break;
    #endif

    default:
      HEDLEY_UNREACHABLE_RETURN(simde_mm256_undefined_pd());
  }

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_round_pd(a, rounding) _mm256_round_pd(a, rounding)
#elif SIMDE_NATURAL_VECTOR_SIZE_LE(128) && defined(SIMDE_STATEMENT_EXPR_)
  #define simde_mm256_round_pd(a, rounding) SIMDE_STATEMENT_EXPR_(({ \
    simde__m256d_private \
      simde_mm256_round_pd_r_ = simde__m256d_to_private(simde_mm256_setzero_pd()), \
      simde_mm256_round_pd_a_ = simde__m256d_to_private(a); \
    \
    for (size_t simde_mm256_round_pd_i = 0 ; simde_mm256_round_pd_i < (sizeof(simde_mm256_round_pd_r_.m128d) / sizeof(simde_mm256_round_pd_r_.m128d[0])) ; simde_mm256_round_pd_i++) { \
      simde_mm256_round_pd_r_.m128d[simde_mm256_round_pd_i] = simde_mm_round_pd(simde_mm256_round_pd_a_.m128d[simde_mm256_round_pd_i], rounding); \
    } \
    \
    simde__m256d_from_private(simde_mm256_round_pd_r_); \
  }))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_round_pd
  #define _mm256_round_pd(a, rounding) simde_mm256_round_pd(a, rounding)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_ceil_pd (simde__m256d a) {
  return simde_mm256_round_pd(a, SIMDE_MM_FROUND_TO_POS_INF);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_ceil_pd
  #define _mm256_ceil_pd(a) simde_mm256_ceil_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_ceil_ps (simde__m256 a) {
  return simde_mm256_round_ps(a, SIMDE_MM_FROUND_TO_POS_INF);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_ceil_ps
  #define _mm256_ceil_ps(a) simde_mm256_ceil_ps(a)
#endif

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DIAGNOSTIC_DISABLE_FLOAT_EQUAL

/* This implementation does not support signaling NaNs (yet?) */
SIMDE_HUGE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_cmp_pd (simde__m128d a, simde__m128d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 31) {
  switch (imm8) {
    case SIMDE_CMP_EQ_UQ:
    case SIMDE_CMP_EQ_US:
      return simde_mm_or_pd(simde_mm_cmpunord_pd(a, b), simde_mm_cmpeq_pd(a, b));
      break;
    case SIMDE_CMP_EQ_OQ:
    case SIMDE_CMP_EQ_OS:
      return simde_mm_cmpeq_pd(a, b);
      break;
    case SIMDE_CMP_NGE_US:
    case SIMDE_CMP_NGE_UQ:
      return simde_x_mm_not_pd(simde_mm_cmpge_pd(a, b));
      break;
    case SIMDE_CMP_LT_OS:
    case SIMDE_CMP_LT_OQ:
      return simde_mm_cmplt_pd(a, b);
      break;
    case SIMDE_CMP_NGT_US:
    case SIMDE_CMP_NGT_UQ:
      return simde_x_mm_not_pd(simde_mm_cmpgt_pd(a, b));
      break;
    case SIMDE_CMP_LE_OS:
    case SIMDE_CMP_LE_OQ:
      return simde_mm_cmple_pd(a, b);
      break;
    case SIMDE_CMP_NEQ_UQ:
    case SIMDE_CMP_NEQ_US:
      return simde_mm_cmpneq_pd(a, b);
      break;
    case SIMDE_CMP_NEQ_OQ:
    case SIMDE_CMP_NEQ_OS:
      return simde_mm_and_pd(simde_mm_cmpord_pd(a, b), simde_mm_cmpneq_pd(a, b));
      break;
    case SIMDE_CMP_NLT_US:
    case SIMDE_CMP_NLT_UQ:
      return simde_x_mm_not_pd(simde_mm_cmplt_pd(a, b));
      break;
    case SIMDE_CMP_GE_OS:
    case SIMDE_CMP_GE_OQ:
      return simde_mm_cmpge_pd(a, b);
      break;
    case SIMDE_CMP_NLE_US:
    case SIMDE_CMP_NLE_UQ:
      return simde_x_mm_not_pd(simde_mm_cmple_pd(a, b));
      break;
    case SIMDE_CMP_GT_OS:
    case SIMDE_CMP_GT_OQ:
      return simde_mm_cmpgt_pd(a, b);
      break;
    case SIMDE_CMP_FALSE_OQ:
    case SIMDE_CMP_FALSE_OS:
      return simde_mm_setzero_pd();
      break;
    case SIMDE_CMP_TRUE_UQ:
    case SIMDE_CMP_TRUE_US:
      return simde_x_mm_setone_pd();
      break;
    case SIMDE_CMP_UNORD_Q:
    case SIMDE_CMP_UNORD_S:
      return simde_mm_cmpunord_pd(a, b);
      break;
    case SIMDE_CMP_ORD_Q:
    case SIMDE_CMP_ORD_S:
      return simde_mm_cmpord_pd(a, b);
      break;
  }

  HEDLEY_UNREACHABLE_RETURN(simde_mm_setzero_pd());
}
#if defined(__clang__) && defined(__AVX512DQ__)
  #define simde_mm_cmp_pd(a, b, imm8) (__extension__ ({ \
    simde__m128d simde_mm_cmp_pd_r; \
    switch (imm8) { \
      case SIMDE_CMP_FALSE_OQ: \
      case SIMDE_CMP_FALSE_OS: \
        simde_mm_cmp_pd_r = simde_mm_setzero_pd(); \
        break; \
      case SIMDE_CMP_TRUE_UQ: \
      case SIMDE_CMP_TRUE_US: \
        simde_mm_cmp_pd_r = simde_x_mm_setone_pd(); \
        break; \
      default: \
        simde_mm_cmp_pd_r = simde_mm_cmp_pd(a, b, imm8); \
        break; \
    } \
    simde_mm_cmp_pd_r; \
  }))
#elif defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm_cmp_pd(a, b, imm8) _mm_cmp_pd(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_cmp_pd
  #define _mm_cmp_pd(a, b, imm8) simde_mm_cmp_pd(a, b, imm8)
#endif

SIMDE_HUGE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_cmp_ps (simde__m128 a, simde__m128 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 31) {
  switch (imm8) {
    case SIMDE_CMP_EQ_UQ:
    case SIMDE_CMP_EQ_US:
      return simde_mm_or_ps(simde_mm_cmpunord_ps(a, b), simde_mm_cmpeq_ps(a, b));
      break;
    case SIMDE_CMP_EQ_OQ:
    case SIMDE_CMP_EQ_OS:
      return simde_mm_cmpeq_ps(a, b);
      break;
    case SIMDE_CMP_NGE_US:
    case SIMDE_CMP_NGE_UQ:
      return simde_x_mm_not_ps(simde_mm_cmpge_ps(a, b));
      break;
    case SIMDE_CMP_LT_OS:
    case SIMDE_CMP_LT_OQ:
      return simde_mm_cmplt_ps(a, b);
      break;
    case SIMDE_CMP_NGT_US:
    case SIMDE_CMP_NGT_UQ:
      return simde_x_mm_not_ps(simde_mm_cmpgt_ps(a, b));
      break;
    case SIMDE_CMP_LE_OS:
    case SIMDE_CMP_LE_OQ:
      return simde_mm_cmple_ps(a, b);
      break;
    case SIMDE_CMP_NEQ_UQ:
    case SIMDE_CMP_NEQ_US:
      return simde_mm_cmpneq_ps(a, b);
      break;
    case SIMDE_CMP_NEQ_OQ:
    case SIMDE_CMP_NEQ_OS:
      return simde_mm_and_ps(simde_mm_cmpord_ps(a, b), simde_mm_cmpneq_ps(a, b));
      break;
    case SIMDE_CMP_NLT_US:
    case SIMDE_CMP_NLT_UQ:
      return simde_x_mm_not_ps(simde_mm_cmplt_ps(a, b));
      break;
    case SIMDE_CMP_GE_OS:
    case SIMDE_CMP_GE_OQ:
      return simde_mm_cmpge_ps(a, b);
      break;
    case SIMDE_CMP_NLE_US:
    case SIMDE_CMP_NLE_UQ:
      return simde_x_mm_not_ps(simde_mm_cmple_ps(a, b));
      break;
    case SIMDE_CMP_GT_OS:
    case SIMDE_CMP_GT_OQ:
      return simde_mm_cmpgt_ps(a, b);
      break;
    case SIMDE_CMP_FALSE_OQ:
    case SIMDE_CMP_FALSE_OS:
      return simde_mm_setzero_ps();
      break;
    case SIMDE_CMP_TRUE_UQ:
    case SIMDE_CMP_TRUE_US:
      return simde_x_mm_setone_ps();
      break;
    case SIMDE_CMP_UNORD_Q:
    case SIMDE_CMP_UNORD_S:
      return simde_mm_cmpunord_ps(a, b);
      break;
    case SIMDE_CMP_ORD_Q:
    case SIMDE_CMP_ORD_S:
      return simde_mm_cmpord_ps(a, b);
      break;
  }

  HEDLEY_UNREACHABLE_RETURN(simde_mm_setzero_ps());
}
/* Prior to 9.0 clang has problems with _mm{,256}_cmp_{ps,pd} for all four of the true/false
 * comparisons, but only when AVX-512 is enabled. */
#if defined(__clang__) && defined(__AVX512DQ__)
  #define simde_mm_cmp_ps(a, b, imm8) (__extension__ ({ \
    simde__m128 simde_mm_cmp_ps_r; \
    switch (imm8) { \
      case SIMDE_CMP_FALSE_OQ: \
      case SIMDE_CMP_FALSE_OS: \
        simde_mm_cmp_ps_r = simde_mm_setzero_ps(); \
        break; \
      case SIMDE_CMP_TRUE_UQ: \
      case SIMDE_CMP_TRUE_US: \
        simde_mm_cmp_ps_r = simde_x_mm_setone_ps(); \
        break; \
      default: \
        simde_mm_cmp_ps_r = simde_mm_cmp_ps(a, b, imm8); \
        break; \
    } \
    simde_mm_cmp_ps_r; \
  }))
#elif defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm_cmp_ps(a, b, imm8) _mm_cmp_ps(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_cmp_ps
  #define _mm_cmp_ps(a, b, imm8) simde_mm_cmp_ps(a, b, imm8)
#endif

SIMDE_HUGE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_cmp_sd (simde__m128d a, simde__m128d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 31) {
  simde__m128d_private
    a_ = simde__m128d_to_private(a),
    b_ = simde__m128d_to_private(b);

  switch (imm8) {
    case SIMDE_CMP_EQ_OQ:
    case SIMDE_CMP_EQ_OS:
      a_.i64[0] = (a_.f64[0] == b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_LT_OQ:
    case SIMDE_CMP_LT_OS:
      a_.i64[0] = (a_.f64[0] < b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_LE_OQ:
    case SIMDE_CMP_LE_OS:
      a_.i64[0] = (a_.f64[0] <= b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_UNORD_Q:
    case SIMDE_CMP_UNORD_S:
      a_.i64[0] = ((a_.f64[0] != a_.f64[0]) || (b_.f64[0] != b_.f64[0])) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_NEQ_UQ:
    case SIMDE_CMP_NEQ_US:
      a_.i64[0] = ((a_.f64[0] == a_.f64[0]) & (b_.f64[0] == b_.f64[0]) & (a_.f64[0] != b_.f64[0])) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_NEQ_OQ:
    case SIMDE_CMP_NEQ_OS:
      a_.i64[0] = ((a_.f64[0] == a_.f64[0]) & (b_.f64[0] == b_.f64[0]) & (a_.f64[0] != b_.f64[0])) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_NLT_UQ:
    case SIMDE_CMP_NLT_US:
      a_.i64[0] = !(a_.f64[0] < b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_NLE_UQ:
    case SIMDE_CMP_NLE_US:
      a_.i64[0] = !(a_.f64[0] <= b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_ORD_Q:
    case SIMDE_CMP_ORD_S:
      a_.i64[0] = ((a_.f64[0] == a_.f64[0]) & (b_.f64[0] == b_.f64[0])) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_EQ_UQ:
    case SIMDE_CMP_EQ_US:
      a_.i64[0] = ((a_.f64[0] != a_.f64[0]) | (b_.f64[0] != b_.f64[0]) | (a_.f64[0] == b_.f64[0])) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_NGE_UQ:
    case SIMDE_CMP_NGE_US:
      a_.i64[0] = !(a_.f64[0] >= b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_NGT_UQ:
    case SIMDE_CMP_NGT_US:
      a_.i64[0] = !(a_.f64[0] > b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_FALSE_OQ:
    case SIMDE_CMP_FALSE_OS:
      a_.i64[0] = INT64_C(0);
      break;

    case SIMDE_CMP_GE_OQ:
    case SIMDE_CMP_GE_OS:
      a_.i64[0] = (a_.f64[0] >= b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_GT_OQ:
    case SIMDE_CMP_GT_OS:
      a_.i64[0] = (a_.f64[0] > b_.f64[0]) ? ~INT64_C(0) : INT64_C(0);
      break;

    case SIMDE_CMP_TRUE_UQ:
    case SIMDE_CMP_TRUE_US:
      a_.i64[0] = ~INT64_C(0);
      break;

    default:
      HEDLEY_UNREACHABLE();
  }

  return simde__m128d_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm_cmp_sd(a, b, imm8) _mm_cmp_sd(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_cmp_sd
  #define _mm_cmp_sd(a, b, imm8) simde_mm_cmp_sd(a, b, imm8)
#endif

SIMDE_HUGE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_cmp_ss (simde__m128 a, simde__m128 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 31) {
  simde__m128_private
    a_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  switch (imm8) {
    case SIMDE_CMP_EQ_OQ:
    case SIMDE_CMP_EQ_OS:
      a_.i32[0] = (a_.f32[0] == b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_LT_OQ:
    case SIMDE_CMP_LT_OS:
      a_.i32[0] = (a_.f32[0] < b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_LE_OQ:
    case SIMDE_CMP_LE_OS:
      a_.i32[0] = (a_.f32[0] <= b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_UNORD_Q:
    case SIMDE_CMP_UNORD_S:
      a_.i32[0] = ((a_.f32[0] != a_.f32[0]) || (b_.f32[0] != b_.f32[0])) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_NEQ_UQ:
    case SIMDE_CMP_NEQ_US:
      a_.i32[0] = ((a_.f32[0] == a_.f32[0]) & (b_.f32[0] == b_.f32[0]) & (a_.f32[0] != b_.f32[0])) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_NEQ_OQ:
    case SIMDE_CMP_NEQ_OS:
      a_.i32[0] = ((a_.f32[0] == a_.f32[0]) & (b_.f32[0] == b_.f32[0]) & (a_.f32[0] != b_.f32[0])) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_NLT_UQ:
    case SIMDE_CMP_NLT_US:
      a_.i32[0] = !(a_.f32[0] < b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_NLE_UQ:
    case SIMDE_CMP_NLE_US:
      a_.i32[0] = !(a_.f32[0] <= b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_ORD_Q:
    case SIMDE_CMP_ORD_S:
      a_.i32[0] = ((a_.f32[0] == a_.f32[0]) & (b_.f32[0] == b_.f32[0])) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_EQ_UQ:
    case SIMDE_CMP_EQ_US:
      a_.i32[0] = ((a_.f32[0] != a_.f32[0]) | (b_.f32[0] != b_.f32[0]) | (a_.f32[0] == b_.f32[0])) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_NGE_UQ:
    case SIMDE_CMP_NGE_US:
      a_.i32[0] = !(a_.f32[0] >= b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_NGT_UQ:
    case SIMDE_CMP_NGT_US:
      a_.i32[0] = !(a_.f32[0] > b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_FALSE_OQ:
    case SIMDE_CMP_FALSE_OS:
      a_.i32[0] = INT32_C(0);
      break;

    case SIMDE_CMP_GE_OQ:
    case SIMDE_CMP_GE_OS:
      a_.i32[0] = (a_.f32[0] >= b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_GT_OQ:
    case SIMDE_CMP_GT_OS:
      a_.i32[0] = (a_.f32[0] > b_.f32[0]) ? ~INT32_C(0) : INT32_C(0);
      break;

    case SIMDE_CMP_TRUE_UQ:
    case SIMDE_CMP_TRUE_US:
      a_.i32[0] = ~INT32_C(0);
      break;

    default:
      HEDLEY_UNREACHABLE();
  }

  return simde__m128_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm_cmp_ss(a, b, imm8) _mm_cmp_ss(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_cmp_ss
  #define _mm_cmp_ss(a, b, imm8) simde_mm_cmp_ss(a, b, imm8)
#endif

SIMDE_HUGE_FUNCTION_ATTRIBUTES
simde__m256d
#if defined(__clang__) && defined(__AVX512DQ__)
simde_mm256_cmp_pd_internal_
#else
simde_mm256_cmp_pd
#endif
(simde__m256d a, simde__m256d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 31) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a),
    b_ = simde__m256d_to_private(b);

  switch (imm8) {
    case SIMDE_CMP_EQ_OQ:
    case SIMDE_CMP_EQ_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 == b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = (a_.f64[i] == b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_LT_OQ:
    case SIMDE_CMP_LT_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 < b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = (a_.f64[i] < b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_LE_OQ:
    case SIMDE_CMP_LE_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 <= b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = (a_.f64[i] <= b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_UNORD_Q:
    case SIMDE_CMP_UNORD_S:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 != a_.f64) | (b_.f64 != b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = ((a_.f64[i] != a_.f64[i]) || (b_.f64[i] != b_.f64[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NEQ_UQ:
    case SIMDE_CMP_NEQ_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 != b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = (a_.f64[i] != b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NEQ_OQ:
    case SIMDE_CMP_NEQ_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 == a_.f64) & (b_.f64 == b_.f64) & (a_.f64 != b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = ((a_.f64[i] == a_.f64[i]) & (b_.f64[i] == b_.f64[i]) & (a_.f64[i] != b_.f64[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NLT_UQ:
    case SIMDE_CMP_NLT_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), ~(a_.f64 < b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = !(a_.f64[i] < b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NLE_UQ:
    case SIMDE_CMP_NLE_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), ~(a_.f64 <= b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = !(a_.f64[i] <= b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_ORD_Q:
    case SIMDE_CMP_ORD_S:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), ((a_.f64 == a_.f64) & (b_.f64 == b_.f64)));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = ((a_.f64[i] == a_.f64[i]) & (b_.f64[i] == b_.f64[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_EQ_UQ:
    case SIMDE_CMP_EQ_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 != a_.f64) | (b_.f64 != b_.f64) | (a_.f64 == b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = ((a_.f64[i] != a_.f64[i]) | (b_.f64[i] != b_.f64[i]) | (a_.f64[i] == b_.f64[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NGE_UQ:
    case SIMDE_CMP_NGE_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), ~(a_.f64 >= b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = !(a_.f64[i] >= b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NGT_UQ:
    case SIMDE_CMP_NGT_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), ~(a_.f64 > b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = !(a_.f64[i] > b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_FALSE_OQ:
    case SIMDE_CMP_FALSE_OS:
      r_ = simde__m256d_to_private(simde_mm256_setzero_pd());
      break;

    case SIMDE_CMP_GE_OQ:
    case SIMDE_CMP_GE_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 >= b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = (a_.f64[i] >= b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_GT_OQ:
    case SIMDE_CMP_GT_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), (a_.f64 > b_.f64));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.i64[i] = (a_.f64[i] > b_.f64[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_TRUE_UQ:
    case SIMDE_CMP_TRUE_US:
      r_ = simde__m256d_to_private(simde_x_mm256_setone_pd());
      break;

    default:
      HEDLEY_UNREACHABLE();
  }

  return simde__m256d_from_private(r_);
}
#if defined(__clang__) && defined(__AVX512DQ__)
  #define simde_mm256_cmp_pd(a, b, imm8) (__extension__ ({ \
    simde__m256d simde_mm256_cmp_pd_r; \
    switch (imm8) { \
      case SIMDE_CMP_FALSE_OQ: \
      case SIMDE_CMP_FALSE_OS: \
        simde_mm256_cmp_pd_r = simde_mm256_setzero_pd(); \
        break; \
      case SIMDE_CMP_TRUE_UQ: \
      case SIMDE_CMP_TRUE_US: \
        simde_mm256_cmp_pd_r = simde_x_mm256_setone_pd(); \
        break; \
      default: \
        simde_mm256_cmp_pd_r = simde_mm256_cmp_pd_internal_(a, b, imm8); \
        break; \
    } \
    simde_mm256_cmp_pd_r; \
  }))
#elif defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_cmp_pd(a, b, imm8) _mm256_cmp_pd(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmp_pd
  #define _mm256_cmp_pd(a, b, imm8) simde_mm256_cmp_pd(a, b, imm8)
#endif

SIMDE_HUGE_FUNCTION_ATTRIBUTES
simde__m256
#if defined(__clang__) && defined(__AVX512DQ__)
simde_mm256_cmp_ps_internal_
#else
simde_mm256_cmp_ps
#endif
(simde__m256 a, simde__m256 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 31) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a),
    b_ = simde__m256_to_private(b);

  switch (imm8) {
    case SIMDE_CMP_EQ_OQ:
    case SIMDE_CMP_EQ_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 == b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = (a_.f32[i] == b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_LT_OQ:
    case SIMDE_CMP_LT_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 < b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = (a_.f32[i] < b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_LE_OQ:
    case SIMDE_CMP_LE_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 <= b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = (a_.f32[i] <= b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_UNORD_Q:
    case SIMDE_CMP_UNORD_S:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 != a_.f32) | (b_.f32 != b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = ((a_.f32[i] != a_.f32[i]) || (b_.f32[i] != b_.f32[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NEQ_UQ:
    case SIMDE_CMP_NEQ_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 != b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = (a_.f32[i] != b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NEQ_OQ:
    case SIMDE_CMP_NEQ_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 == a_.f32) & (b_.f32 == b_.f32) & (a_.f32 != b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = ((a_.f32[i] == a_.f32[i]) & (b_.f32[i] == b_.f32[i]) & (a_.f32[i] != b_.f32[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NLT_UQ:
    case SIMDE_CMP_NLT_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), ~(a_.f32 < b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = !(a_.f32[i] < b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NLE_UQ:
    case SIMDE_CMP_NLE_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), ~(a_.f32 <= b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = !(a_.f32[i] <= b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_ORD_Q:
    case SIMDE_CMP_ORD_S:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), ((a_.f32 == a_.f32) & (b_.f32 == b_.f32)));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = ((a_.f32[i] == a_.f32[i]) & (b_.f32[i] == b_.f32[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_EQ_UQ:
    case SIMDE_CMP_EQ_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 != a_.f32) | (b_.f32 != b_.f32) | (a_.f32 == b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = ((a_.f32[i] != a_.f32[i]) | (b_.f32[i] != b_.f32[i]) | (a_.f32[i] == b_.f32[i])) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NGE_UQ:
    case SIMDE_CMP_NGE_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), ~(a_.f32 >= b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = !(a_.f32[i] >= b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_NGT_UQ:
    case SIMDE_CMP_NGT_US:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), ~(a_.f32 > b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = !(a_.f32[i] > b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_FALSE_OQ:
    case SIMDE_CMP_FALSE_OS:
      r_ = simde__m256_to_private(simde_mm256_setzero_ps());
      break;

    case SIMDE_CMP_GE_OQ:
    case SIMDE_CMP_GE_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 >= b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = (a_.f32[i] >= b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_GT_OQ:
    case SIMDE_CMP_GT_OS:
      #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
        r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), (a_.f32 > b_.f32));
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
          r_.i32[i] = (a_.f32[i] > b_.f32[i]) ? ~INT32_C(0) : INT32_C(0);
        }
      #endif
      break;

    case SIMDE_CMP_TRUE_UQ:
    case SIMDE_CMP_TRUE_US:
      r_ = simde__m256_to_private(simde_x_mm256_setone_ps());
      break;

    default:
      HEDLEY_UNREACHABLE();
  }

  return simde__m256_from_private(r_);
}
#if defined(__clang__) && defined(__AVX512DQ__)
  #define simde_mm256_cmp_ps(a, b, imm8) (__extension__ ({ \
    simde__m256 simde_mm256_cmp_ps_r; \
    switch (imm8) { \
      case SIMDE_CMP_FALSE_OQ: \
      case SIMDE_CMP_FALSE_OS: \
        simde_mm256_cmp_ps_r = simde_mm256_setzero_ps(); \
        break; \
      case SIMDE_CMP_TRUE_UQ: \
      case SIMDE_CMP_TRUE_US: \
        simde_mm256_cmp_ps_r = simde_x_mm256_setone_ps(); \
        break; \
      default: \
        simde_mm256_cmp_ps_r = simde_mm256_cmp_ps_internal_(a, b, imm8); \
        break; \
    } \
    simde_mm256_cmp_ps_r; \
  }))
#elif defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_cmp_ps(a, b, imm8) _mm256_cmp_ps(a, b, imm8)
#elif defined(SIMDE_STATEMENT_EXPR_) && SIMDE_NATURAL_VECTOR_SIZE_LE(128)
  #define simde_mm256_cmp_ps(a, b, imm8) SIMDE_STATEMENT_EXPR_(({ \
    simde__m256_private \
      simde_mm256_cmp_ps_r_ = simde__m256_to_private(simde_mm256_setzero_ps()), \
      simde_mm256_cmp_ps_a_ = simde__m256_to_private((a)), \
      simde_mm256_cmp_ps_b_ = simde__m256_to_private((b)); \
    \
    for (size_t i = 0 ; i < (sizeof(simde_mm256_cmp_ps_r_.m128) / sizeof(simde_mm256_cmp_ps_r_.m128[0])) ; i++) { \
      simde_mm256_cmp_ps_r_.m128[i] = simde_mm_cmp_ps(simde_mm256_cmp_ps_a_.m128[i], simde_mm256_cmp_ps_b_.m128[i], (imm8)); \
    } \
    \
    simde__m256_from_private(simde_mm256_cmp_ps_r_); \
  }))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmp_ps
  #define _mm256_cmp_ps(a, b, imm8) simde_mm256_cmp_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_copysign_ps(simde__m256 dest, simde__m256 src) {
  simde__m256_private
    r_,
    dest_ = simde__m256_to_private(dest),
    src_ = simde__m256_to_private(src);

  #if defined(simde_math_copysignf)
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = simde_math_copysignf(dest_.f32[i], src_.f32[i]);
    }
  #else
    simde__m256 sgnbit = simde_mm256_xor_ps(simde_mm256_set1_ps(SIMDE_FLOAT32_C(0.0)), simde_mm256_set1_ps(-SIMDE_FLOAT32_C(0.0)));
    return simde_mm256_xor_ps(simde_mm256_and_ps(sgnbit, src), simde_mm256_andnot_ps(sgnbit, dest));
  #endif

  return simde__m256_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_copysign_pd(simde__m256d dest, simde__m256d src) {
  simde__m256d_private
    r_,
    dest_ = simde__m256d_to_private(dest),
    src_ = simde__m256d_to_private(src);

  #if defined(simde_math_copysign)
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = simde_math_copysign(dest_.f64[i], src_.f64[i]);
    }
  #else
    simde__m256d sgnbit = simde_mm256_xor_pd(simde_mm256_set1_pd(SIMDE_FLOAT64_C(0.0)), simde_mm256_set1_pd(-SIMDE_FLOAT64_C(0.0)));
    return simde_mm256_xor_pd(simde_mm256_and_pd(sgnbit, src), simde_mm256_andnot_pd(sgnbit, dest));
  #endif

  return simde__m256d_from_private(r_);
}

HEDLEY_DIAGNOSTIC_POP /* -Wfloat-equal */

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_cvtepi32_pd (simde__m128i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvtepi32_pd(a);
  #else
    simde__m256d_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = HEDLEY_STATIC_CAST(simde_float64, a_.i32[i]);
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi32_pd
  #define _mm256_cvtepi32_pd(a) simde_mm256_cvtepi32_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
  simde_mm256_cvtepi32_ps (simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvtepi32_ps(a);
  #else
    simde__m256_private r_;
    simde__m256i_private a_ = simde__m256i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = HEDLEY_STATIC_CAST(simde_float32, a_.i32[i]);
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi32_ps
  #define _mm256_cvtepi32_ps(a) simde_mm256_cvtepi32_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_cvtpd_epi32 (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvtpd_epi32(a);
  #else
    simde__m128i_private r_;
    simde__m256d_private a_ = simde__m256d_to_private(a);

    #if defined(simde_math_nearbyint)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(a_.f64) / sizeof(a_.f64[0])) ; i++) {
        r_.i32[i] = SIMDE_CONVERT_FTOI(int32_t, simde_math_nearbyint(a_.f64[i]));
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtpd_epi32
  #define _mm256_cvtpd_epi32(a) simde_mm256_cvtpd_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm256_cvtpd_ps (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvtpd_ps(a);
  #else
    simde__m128_private r_;
    simde__m256d_private a_ = simde__m256d_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = HEDLEY_STATIC_CAST(simde_float32, a_.f64[i]);
    }

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtpd_ps
  #define _mm256_cvtpd_ps(a) simde_mm256_cvtpd_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtps_epi32 (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvtps_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m256_private a_ = simde__m256_to_private(a);

    #if defined(simde_math_nearbyintf)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(a_.f32) / sizeof(a_.f32[0])) ; i++) {
        r_.i32[i] = SIMDE_CONVERT_FTOI(int32_t, simde_math_nearbyintf(a_.f32[i]));
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtps_epi32
  #define _mm256_cvtps_epi32(a) simde_mm256_cvtps_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_cvtps_pd (simde__m128 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvtps_pd(a);
  #else
    simde__m256d_private r_;
    simde__m128_private a_ = simde__m128_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.f32) / sizeof(a_.f32[0])) ; i++) {
      r_.f64[i] = HEDLEY_STATIC_CAST(double, a_.f32[i]);
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtps_pd
  #define _mm256_cvtps_pd(a) simde_mm256_cvtps_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde_float64
simde_mm256_cvtsd_f64 (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE) && ( \
      SIMDE_DETECT_CLANG_VERSION_CHECK(3,9,0) || \
      HEDLEY_GCC_VERSION_CHECK(7,0,0) || \
      HEDLEY_INTEL_VERSION_CHECK(13,0,0) || \
      HEDLEY_MSVC_VERSION_CHECK(19,14,0))
    return _mm256_cvtsd_f64(a);
  #else
    simde__m256d_private a_ = simde__m256d_to_private(a);
    return a_.f64[0];
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtsd_f64
  #define _mm256_cvtsd_f64(a) simde_mm256_cvtsd_f64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int32_t
simde_mm256_cvtsi256_si32 (simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE) && ( \
      SIMDE_DETECT_CLANG_VERSION_CHECK(3,9,0) || \
      HEDLEY_INTEL_VERSION_CHECK(13,0,0) || \
      HEDLEY_MSVC_VERSION_CHECK(19,14,0))
    return _mm256_cvtsi256_si32(a);
  #else
    simde__m256i_private a_ = simde__m256i_to_private(a);
    return a_.i32[0];
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtsi256_si32
  #define _mm256_cvtsi256_si32(a) simde_mm256_cvtsi256_si32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde_float32
simde_mm256_cvtss_f32 (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE) && ( \
      SIMDE_DETECT_CLANG_VERSION_CHECK(3,9,0) || \
      HEDLEY_GCC_VERSION_CHECK(7,0,0) || \
      HEDLEY_INTEL_VERSION_CHECK(13,0,0) || \
      HEDLEY_MSVC_VERSION_CHECK(19,14,0))
    return _mm256_cvtss_f32(a);
  #else
    simde__m256_private a_ = simde__m256_to_private(a);
    return a_.f32[0];
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtss_f32
  #define _mm256_cvtss_f32(a) simde_mm256_cvtss_f32(a)
#endif


SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_cvttpd_epi32 (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvttpd_epi32(a);
  #else
    simde__m128i_private r_;
    simde__m256d_private a_ = simde__m256d_to_private(a);

    #if defined(simde_math_trunc)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(a_.f64) / sizeof(a_.f64[0])) ; i++) {
        r_.i32[i] = SIMDE_CONVERT_FTOI(int32_t, simde_math_trunc(a_.f64[i]));
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvttpd_epi32
  #define _mm256_cvttpd_epi32(a) simde_mm256_cvttpd_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvttps_epi32 (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_cvttps_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m256_private a_ = simde__m256_to_private(a);

    #if defined(simde_math_truncf)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(a_.f32) / sizeof(a_.f32[0])) ; i++) {
        r_.i32[i] = SIMDE_CONVERT_FTOI(int32_t, simde_math_truncf(a_.f32[i]));
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvttps_epi32
  #define _mm256_cvttps_epi32(a) simde_mm256_cvttps_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_div_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_div_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_div_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_div_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f32 = a_.f32 / b_.f32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[i] / b_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_div_ps
  #define _mm256_div_ps(a, b) simde_mm256_div_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_div_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_div_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_div_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_div_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f64 = a_.f64 / b_.f64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = a_.f64[i] / b_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_div_pd
  #define _mm256_div_pd(a, b) simde_mm256_div_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm256_extractf128_pd (simde__m256d a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256d_private a_ = simde__m256d_to_private(a);
  return a_.m128d[imm8];
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_extractf128_pd(a, imm8) _mm256_extractf128_pd(a, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extractf128_pd
  #define _mm256_extractf128_pd(a, imm8) simde_mm256_extractf128_pd(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm256_extractf128_ps (simde__m256 a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256_private a_ = simde__m256_to_private(a);
  return a_.m128[imm8];
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_extractf128_ps(a, imm8) _mm256_extractf128_ps(a, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extractf128_ps
  #define _mm256_extractf128_ps(a, imm8) simde_mm256_extractf128_ps(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_extractf128_si256 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  return a_.m128i[imm8];
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_extractf128_si256(a, imm8) _mm256_extractf128_si256(a, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extractf128_si256
  #define _mm256_extractf128_si256(a, imm8) simde_mm256_extractf128_si256(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_floor_pd (simde__m256d a) {
  return simde_mm256_round_pd(a, SIMDE_MM_FROUND_TO_NEG_INF);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_floor_pd
  #define _mm256_floor_pd(a) simde_mm256_floor_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_floor_ps (simde__m256 a) {
  return simde_mm256_round_ps(a, SIMDE_MM_FROUND_TO_NEG_INF);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_floor_ps
  #define _mm256_floor_ps(a) simde_mm256_floor_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_insert_epi8 (simde__m256i a, int8_t i, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 31) {
  simde__m256i_private a_ = simde__m256i_to_private(a);

  a_.i8[index] = i;

  return simde__m256i_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,10,0))
  #define simde_mm256_insert_epi8(a, i, index) _mm256_insert_epi8(a, i, index)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_insert_epi8
  #define _mm256_insert_epi8(a, i, index) simde_mm256_insert_epi8(a, i, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_insert_epi16 (simde__m256i a, int16_t i, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 15)  {
  simde__m256i_private a_ = simde__m256i_to_private(a);

  a_.i16[index] = i;

  return simde__m256i_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,10,0))
  #define simde_mm256_insert_epi16(a, i, index) _mm256_insert_epi16(a, i, index)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_insert_epi16
  #define _mm256_insert_epi16(a, i, imm8) simde_mm256_insert_epi16(a, i, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_insert_epi32 (simde__m256i a, int32_t i, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 7)  {
  simde__m256i_private a_ = simde__m256i_to_private(a);

  a_.i32[index] = i;

  return simde__m256i_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,10,0))
  #define simde_mm256_insert_epi32(a, i, index) _mm256_insert_epi32(a, i, index)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_insert_epi32
  #define _mm256_insert_epi32(a, i, index) simde_mm256_insert_epi32(a, i, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_insert_epi64 (simde__m256i a, int64_t i, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 3)  {
  simde__m256i_private a_ = simde__m256i_to_private(a);

  a_.i64[index] = i;

  return simde__m256i_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE) && defined(SIMDE_ARCH_AMD64) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,20,0)) && \
    SIMDE_DETECT_CLANG_VERSION_CHECK(3,7,0)
  #define simde_mm256_insert_epi64(a, i, index) _mm256_insert_epi64(a, i, index)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && !defined(SIMDE_ARCH_AMD64))
  #undef _mm256_insert_epi64
  #define _mm256_insert_epi64(a, i, index) simde_mm256_insert_epi64(a, i, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d simde_mm256_insertf128_pd(simde__m256d a, simde__m128d b, int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256d_private a_ = simde__m256d_to_private(a);
  simde__m128d_private b_ = simde__m128d_to_private(b);

  a_.m128d_private[imm8] = b_;

  return simde__m256d_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_insertf128_pd(a, b, imm8) _mm256_insertf128_pd(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_insertf128_pd
  #define _mm256_insertf128_pd(a, b, imm8) simde_mm256_insertf128_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256 simde_mm256_insertf128_ps(simde__m256 a, simde__m128 b, int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256_private a_ = simde__m256_to_private(a);
  simde__m128_private b_ = simde__m128_to_private(b);

  a_.m128_private[imm8] = b_;

  return simde__m256_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_insertf128_ps(a, b, imm8) _mm256_insertf128_ps(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_insertf128_ps
  #define _mm256_insertf128_ps(a, b, imm8) simde_mm256_insertf128_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i simde_mm256_insertf128_si256(simde__m256i a, simde__m128i b, int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  simde__m128i_private b_ = simde__m128i_to_private(b);

  a_.m128i_private[imm8] = b_;

  return simde__m256i_from_private(a_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_insertf128_si256(a, b, imm8) _mm256_insertf128_si256(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_insertf128_si256
  #define _mm256_insertf128_si256(a, b, imm8) simde_mm256_insertf128_si256(a, b, imm8)
#endif

#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_dp_ps(a, b, imm8) _mm256_dp_ps(a, b, imm8)
#else
#  define simde_mm256_dp_ps(a, b, imm8) \
    simde_mm256_set_m128( \
      simde_mm_dp_ps(simde_mm256_extractf128_ps(a, 1), simde_mm256_extractf128_ps(b, 1), imm8), \
      simde_mm_dp_ps(simde_mm256_extractf128_ps(a, 0), simde_mm256_extractf128_ps(b, 0), imm8))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_dp_ps
  #define _mm256_dp_ps(a, b, imm8) simde_mm256_dp_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int32_t
simde_mm256_extract_epi32 (simde__m256i a, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 7) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  return a_.i32[index];
}
#if defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,10,0))
  #define simde_mm256_extract_epi32(a, index) _mm256_extract_epi32(a, index)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extract_epi32
  #define _mm256_extract_epi32(a, index) simde_mm256_extract_epi32(a, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int64_t
simde_mm256_extract_epi64 (simde__m256i a, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 3) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  return a_.i64[index];
}
#if defined(SIMDE_X86_AVX_NATIVE) && defined(SIMDE_ARCH_AMD64)
  #if !defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,20,0)
    #define simde_mm256_extract_epi64(a, index) _mm256_extract_epi64(a, index)
  #endif
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && !defined(SIMDE_ARCH_AMD64))
  #undef _mm256_extract_epi64
  #define _mm256_extract_epi64(a, index) simde_mm256_extract_epi64(a, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_lddqu_si256 (simde__m256i const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_si256(mem_addr);
  #else
    simde__m256i r;
    simde_memcpy(&r, SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256i), sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_lddqu_si256
  #define _mm256_lddqu_si256(a) simde_mm256_lddqu_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_load_pd (const double mem_addr[HEDLEY_ARRAY_PARAM(4)]) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_load_pd(mem_addr);
  #else
    simde__m256d r;
    simde_memcpy(&r, SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256d), sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_load_pd
  #define _mm256_load_pd(a) simde_mm256_load_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_load_ps (const float mem_addr[HEDLEY_ARRAY_PARAM(8)]) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_load_ps(mem_addr);
  #else
    simde__m256 r;
    simde_memcpy(&r, SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256), sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_load_ps
  #define _mm256_load_ps(a) simde_mm256_load_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_load_si256 (simde__m256i const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_load_si256(mem_addr);
  #else
    simde__m256i r;
    simde_memcpy(&r, SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256i), sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_load_si256
  #define _mm256_load_si256(a) simde_mm256_load_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_loadu_pd (const double a[HEDLEY_ARRAY_PARAM(4)]) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_pd(a);
  #else
    simde__m256d r;
    simde_memcpy(&r, a, sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_loadu_pd
  #define _mm256_loadu_pd(a) simde_mm256_loadu_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_loadu_ps (const float a[HEDLEY_ARRAY_PARAM(8)]) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_ps(a);
  #else
    simde__m256 r;
    simde_memcpy(&r, a, sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_loadu_ps
  #define _mm256_loadu_ps(a) simde_mm256_loadu_ps(a)
#endif

#if defined(SIMDE_X86_AVX512VL_NATIVE) && defined(SIMDE_X86_AVX512BW_NATIVE) \
    && !defined(SIMDE_BUG_GCC_95483) && !defined(SIMDE_BUG_CLANG_REV_344862) \
    && (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,20,0))
  #define simde_mm256_loadu_epi8(mem_addr) _mm256_loadu_epi8(mem_addr)
#else
SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_loadu_epi8(void const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_si256(SIMDE_ALIGN_CAST(__m256i const *, mem_addr));
  #else
    simde__m256i r;
    simde_memcpy(&r, mem_addr, sizeof(r));
    return r;
  #endif
}
#endif
#define simde_x_mm256_loadu_epi8(mem_addr) simde_mm256_loadu_epi8(mem_addr)
#if defined(SIMDE_X86_AVX512VL_ENABLE_NATIVE_ALIASES) || defined(SIMDE_X86_AVX512BW_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && (defined(SIMDE_BUG_GCC_95483) || defined(SIMDE_BUG_CLANG_REV_344862)))
  #undef _mm256_loadu_epi8
  #define _mm256_loadu_epi8(a) simde_mm256_loadu_epi8(a)
#endif

#if defined(SIMDE_X86_AVX512VL_NATIVE) && defined(SIMDE_X86_AVX512BW_NATIVE) \
    && !defined(SIMDE_BUG_GCC_95483) && !defined(SIMDE_BUG_CLANG_REV_344862) \
    && (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,20,0))
  #define simde_mm256_loadu_epi16(mem_addr) _mm256_loadu_epi16(mem_addr)
#else
SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_loadu_epi16(void const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_si256(SIMDE_ALIGN_CAST(__m256i const *, mem_addr));
  #else
    simde__m256i r;
    simde_memcpy(&r, mem_addr, sizeof(r));
    return r;
  #endif
}
#endif
#define simde_x_mm256_loadu_epi16(mem_addr) simde_mm256_loadu_epi16(mem_addr)
#if defined(SIMDE_X86_AVX512VL_ENABLE_NATIVE_ALIASES) || defined(SIMDE_X86_AVX512BW_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && (defined(SIMDE_BUG_GCC_95483) || defined(SIMDE_BUG_CLANG_REV_344862)))
  #undef _mm256_loadu_epi16
  #define _mm256_loadu_epi16(a) simde_mm256_loadu_epi16(a)
#endif

#if defined(SIMDE_X86_AVX512VL_NATIVE) && !defined(SIMDE_BUG_GCC_95483) \
    && !defined(SIMDE_BUG_CLANG_REV_344862) \
    && (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,20,0))
  #define simde_mm256_loadu_epi32(mem_addr) _mm256_loadu_epi32(mem_addr)
#else
SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_loadu_epi32(void const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_si256(SIMDE_ALIGN_CAST(__m256i const *, mem_addr));
  #else
    simde__m256i r;
    simde_memcpy(&r, mem_addr, sizeof(r));
    return r;
  #endif
}
#endif
#define simde_x_mm256_loadu_epi32(mem_addr) simde_mm256_loadu_epi32(mem_addr)
#if defined(SIMDE_X86_AVX512VL_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && (defined(SIMDE_BUG_GCC_95483) || defined(SIMDE_BUG_CLANG_REV_344862)))
  #undef _mm256_loadu_epi32
  #define _mm256_loadu_epi32(a) simde_mm256_loadu_epi32(a)
#endif

#if defined(SIMDE_X86_AVX512VL_NATIVE) && !defined(SIMDE_BUG_GCC_95483) \
    && !defined(SIMDE_BUG_CLANG_REV_344862) \
    && (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,20,0))
  #define simde_mm256_loadu_epi64(mem_addr) _mm256_loadu_epi64(mem_addr)
#else
SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_loadu_epi64(void const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_si256(SIMDE_ALIGN_CAST(__m256i const *, mem_addr));
  #else
    simde__m256i r;
    simde_memcpy(&r, mem_addr, sizeof(r));
    return r;
  #endif
}
#endif
#define simde_x_mm256_loadu_epi64(mem_addr) simde_mm256_loadu_epi64(mem_addr)
#if defined(SIMDE_X86_AVX512VL_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && (defined(SIMDE_BUG_GCC_95483) || defined(SIMDE_BUG_CLANG_REV_344862)))
  #undef _mm256_loadu_epi64
  #define _mm256_loadu_epi64(a) simde_mm256_loadu_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_loadu_si256 (void const * mem_addr) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_loadu_si256(SIMDE_ALIGN_CAST(const __m256i*, mem_addr));
  #else
    simde__m256i r;
    simde_memcpy(&r, mem_addr, sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_loadu_si256
  #define _mm256_loadu_si256(mem_addr) simde_mm256_loadu_si256(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_loadu2_m128 (const float hiaddr[HEDLEY_ARRAY_PARAM(4)], const float loaddr[HEDLEY_ARRAY_PARAM(4)]) {
  #if defined(SIMDE_X86_AVX_NATIVE) && !defined(SIMDE_BUG_GCC_91341) && !defined(SIMDE_BUG_MCST_LCC_MISSING_AVX_LOAD_STORE_M128_FUNCS)
    return _mm256_loadu2_m128(hiaddr, loaddr);
  #else
    return
      simde_mm256_insertf128_ps(simde_mm256_castps128_ps256(simde_mm_loadu_ps(loaddr)),
              simde_mm_loadu_ps(hiaddr), 1);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_loadu2_m128
  #define _mm256_loadu2_m128(hiaddr, loaddr) simde_mm256_loadu2_m128(hiaddr, loaddr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_loadu2_m128d (const double hiaddr[HEDLEY_ARRAY_PARAM(2)], const double loaddr[HEDLEY_ARRAY_PARAM(2)]) {
  #if defined(SIMDE_X86_AVX_NATIVE) && !defined(SIMDE_BUG_GCC_91341) && !defined(SIMDE_BUG_MCST_LCC_MISSING_AVX_LOAD_STORE_M128_FUNCS)
    return _mm256_loadu2_m128d(hiaddr, loaddr);
  #else
    return
      simde_mm256_insertf128_pd(simde_mm256_castpd128_pd256(simde_mm_loadu_pd(loaddr)),
              simde_mm_loadu_pd(hiaddr), 1);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_loadu2_m128d
  #define _mm256_loadu2_m128d(hiaddr, loaddr) simde_mm256_loadu2_m128d(hiaddr, loaddr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_loadu2_m128i (const simde__m128i* hiaddr, const simde__m128i* loaddr) {
  #if defined(SIMDE_X86_AVX_NATIVE) && !defined(SIMDE_BUG_GCC_91341) && !defined(SIMDE_BUG_MCST_LCC_MISSING_AVX_LOAD_STORE_M128_FUNCS)
    return _mm256_loadu2_m128i(hiaddr, loaddr);
  #else
    return
      simde_mm256_insertf128_si256(simde_mm256_castsi128_si256(simde_mm_loadu_si128(loaddr)),
          simde_mm_loadu_si128(hiaddr), 1);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_loadu2_m128i
  #define _mm256_loadu2_m128i(hiaddr, loaddr) simde_mm256_loadu2_m128i(hiaddr, loaddr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_maskload_pd (const simde_float64 mem_addr[HEDLEY_ARRAY_PARAM(2)], simde__m128i mask) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      return _mm_maskload_pd(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m128d, mask));
    #else
      return _mm_maskload_pd(mem_addr, mask);
    #endif
  #else
    simde__m128d_private r_;
    simde__m128i_private
      mask_ = simde__m128i_to_private(mask),
      mask_shr_;

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      mask_shr_.neon_i64 = vshrq_n_s64(mask_.neon_i64, 63);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      return simde_mm_and_pd(simde_mm_load_pd(mem_addr),
          simde__m128d_from_wasm_v128(wasm_i64x2_shr(mask_.wasm_v128, 63)));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(mask_.i64) / sizeof(mask_.i64[0])) ; i++) {
        mask_shr_.i64[i] = mask_.i64[i] >> 63;
      }
    #endif
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = mask_shr_.i64[i] ? mem_addr[i] : SIMDE_FLOAT64_C(0.0);
      }

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskload_pd
  #define _mm_maskload_pd(mem_addr, mask) simde_mm_maskload_pd(HEDLEY_REINTERPRET_CAST(double const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_maskload_pd (const simde_float64 mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m256i mask) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      return _mm256_maskload_pd(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m256d, mask));
    #else
      return _mm256_maskload_pd(mem_addr, mask);
    #endif
  #else
    simde__m256d_private r_;
    simde__m256i_private mask_ = simde__m256i_to_private(mask);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = (mask_.i64[i] >> 63) ? mem_addr[i] : SIMDE_FLOAT64_C(0.0);
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskload_pd
  #define _mm256_maskload_pd(mem_addr, mask) simde_mm256_maskload_pd(HEDLEY_REINTERPRET_CAST(double const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_maskload_ps (const simde_float32 mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m128i mask) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      return _mm_maskload_ps(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m128, mask));
    #else
      return _mm_maskload_ps(mem_addr, mask);
    #endif
  #else
    simde__m128_private r_;
    simde__m128i_private
      mask_ = simde__m128i_to_private(mask),
      mask_shr_;

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      mask_shr_.neon_i32 = vshrq_n_s32(mask_.neon_i32, 31);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      return simde_mm_and_ps(simde_mm_load_ps(mem_addr),
          simde__m128_from_wasm_v128(wasm_i32x4_shr(mask_.wasm_v128, 31)));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(mask_.i32) / sizeof(mask_.i32[0])) ; i++) {
        mask_shr_.i32[i] = mask_.i32[i] >> 31;
      }
    #endif

      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = mask_shr_.i32[i] ? mem_addr[i] : SIMDE_FLOAT32_C(0.0);
      }

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskload_ps
  #define _mm_maskload_ps(mem_addr, mask) simde_mm_maskload_ps(HEDLEY_REINTERPRET_CAST(float const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_maskload_ps (const simde_float32 mem_addr[HEDLEY_ARRAY_PARAM(8)], simde__m256i mask) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      return _mm256_maskload_ps(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m256, mask));
    #else
      return _mm256_maskload_ps(mem_addr, mask);
    #endif
  #else
    simde__m256_private r_;
    simde__m256i_private mask_ = simde__m256i_to_private(mask);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = (mask_.i32[i] >> 31) ? mem_addr[i] : SIMDE_FLOAT32_C(0.0);
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskload_ps
  #define _mm256_maskload_ps(mem_addr, mask) simde_mm256_maskload_ps(HEDLEY_REINTERPRET_CAST(float const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm_maskstore_pd (simde_float64 mem_addr[HEDLEY_ARRAY_PARAM(2)], simde__m128i mask, simde__m128d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      _mm_maskstore_pd(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m128d, mask), a);
    #else
      _mm_maskstore_pd(mem_addr, mask, a);
    #endif
  #else
    simde__m128i_private mask_ = simde__m128i_to_private(mask);
    simde__m128d_private a_ = simde__m128d_to_private(a);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      if ((HEDLEY_STATIC_CAST(unsigned long long, wasm_i64x2_extract_lane(mask_.wasm_v128, 0)) & 0x8000000000000000ull) != 0)
        mem_addr[0] = wasm_f64x2_extract_lane(a_.wasm_v128, 0);
      if ((HEDLEY_STATIC_CAST(unsigned long long, wasm_i64x2_extract_lane(mask_.wasm_v128, 1)) & 0x8000000000000000ull) != 0)
        mem_addr[1] = wasm_f64x2_extract_lane(a_.wasm_v128, 1);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(a_.f64) / sizeof(a_.f64[0])) ; i++) {
        if (mask_.u64[i] >> 63)
          mem_addr[i] = a_.f64[i];
      }
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskstore_pd
  #define _mm_maskstore_pd(mem_addr, mask, a) simde_mm_maskstore_pd(HEDLEY_REINTERPRET_CAST(double*, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_maskstore_pd (simde_float64 mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m256i mask, simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      _mm256_maskstore_pd(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m256d, mask), a);
    #else
      _mm256_maskstore_pd(mem_addr, mask, a);
    #endif
  #else
    simde__m256i_private mask_ = simde__m256i_to_private(mask);
    simde__m256d_private a_ = simde__m256d_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.f64) / sizeof(a_.f64[0])) ; i++) {
      if (mask_.u64[i] & (UINT64_C(1) << 63))
        mem_addr[i] = a_.f64[i];
    }
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskstore_pd
  #define _mm256_maskstore_pd(mem_addr, mask, a) simde_mm256_maskstore_pd(HEDLEY_REINTERPRET_CAST(double*, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm_maskstore_ps (simde_float32 mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m128i mask, simde__m128 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      _mm_maskstore_ps(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m128, mask), a);
    #else
      _mm_maskstore_ps(mem_addr, mask, a);
    #endif
  #else
    simde__m128i_private mask_ = simde__m128i_to_private(mask);
    simde__m128_private a_ = simde__m128_to_private(a);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      if ((HEDLEY_STATIC_CAST(unsigned long long, wasm_i32x4_extract_lane(mask_.wasm_v128, 0)) & 0x80000000ull) != 0)
        mem_addr[0] = wasm_f32x4_extract_lane(a_.wasm_v128, 0);
      if ((HEDLEY_STATIC_CAST(unsigned long long, wasm_i32x4_extract_lane(mask_.wasm_v128, 1)) & 0x80000000ull) != 0)
        mem_addr[1] = wasm_f32x4_extract_lane(a_.wasm_v128, 1);
      if ((HEDLEY_STATIC_CAST(unsigned long long, wasm_i32x4_extract_lane(mask_.wasm_v128, 2)) & 0x80000000ull) != 0)
        mem_addr[2] = wasm_f32x4_extract_lane(a_.wasm_v128, 2);
      if ((HEDLEY_STATIC_CAST(unsigned long long, wasm_i32x4_extract_lane(mask_.wasm_v128, 3)) & 0x80000000ull) != 0)
        mem_addr[3] = wasm_f32x4_extract_lane(a_.wasm_v128, 3);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(a_.f32) / sizeof(a_.f32[0])) ; i++) {
        if (mask_.u32[i] & (UINT32_C(1) << 31))
          mem_addr[i] = a_.f32[i];
      }
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskstore_ps
  #define _mm_maskstore_ps(mem_addr, mask, a) simde_mm_maskstore_ps(HEDLEY_REINTERPRET_CAST(float*, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_maskstore_ps (simde_float32 mem_addr[HEDLEY_ARRAY_PARAM(8)], simde__m256i mask, simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      _mm256_maskstore_ps(mem_addr, HEDLEY_REINTERPRET_CAST(simde__m256, mask), a);
    #else
      _mm256_maskstore_ps(mem_addr, mask, a);
    #endif
  #else
    simde__m256i_private mask_ = simde__m256i_to_private(mask);
    simde__m256_private a_ = simde__m256_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.f32) / sizeof(a_.f32[0])) ; i++) {
      if (mask_.u32[i] & (UINT32_C(1) << 31))
        mem_addr[i] = a_.f32[i];
    }
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskstore_ps
  #define _mm256_maskstore_ps(mem_addr, mask, a) simde_mm256_maskstore_ps(HEDLEY_REINTERPRET_CAST(float*, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_min_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_min_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_min_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_min_ps(a_.m128[1], b_.m128[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = (a_.f32[i] < b_.f32[i]) ? a_.f32[i] : b_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_ps
  #define _mm256_min_ps(a, b) simde_mm256_min_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_min_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_min_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_min_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_min_pd(a_.m128d[1], b_.m128d[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = (a_.f64[i] < b_.f64[i]) ? a_.f64[i] : b_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_pd
  #define _mm256_min_pd(a, b) simde_mm256_min_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_max_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_max_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_max_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_max_ps(a_.m128[1], b_.m128[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = (a_.f32[i] > b_.f32[i]) ? a_.f32[i] : b_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_ps
  #define _mm256_max_ps(a, b) simde_mm256_max_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_max_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_max_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_max_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_max_pd(a_.m128d[1], b_.m128d[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = (a_.f64[i] > b_.f64[i]) ? a_.f64[i] : b_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_pd
  #define _mm256_max_pd(a, b) simde_mm256_max_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_movedup_pd (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_movedup_pd(a);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.f64, a_.f64, 0, 0, 2, 2);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i += 2) {
        r_.f64[i] = r_.f64[i + 1] = a_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_movedup_pd
  #define _mm256_movedup_pd(a) simde_mm256_movedup_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_movehdup_ps (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_movehdup_ps(a);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.f32, a_.f32, 1, 1, 3, 3, 5, 5, 7, 7);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 1 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i += 2) {
        r_.f32[i - 1] = r_.f32[i] = a_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_movehdup_ps
  #define _mm256_movehdup_ps(a) simde_mm256_movehdup_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_moveldup_ps (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_moveldup_ps(a);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.f32, a_.f32, 0, 0, 2, 2, 4, 4, 6, 6);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i += 2) {
        r_.f32[i] = r_.f32[i + 1] = a_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_moveldup_ps
  #define _mm256_moveldup_ps(a) simde_mm256_moveldup_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_movemask_ps (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_movemask_ps(a);
  #else
    simde__m256_private a_ = simde__m256_to_private(a);
    int r = 0;

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.f32) / sizeof(a_.f32[0])) ; i++) {
      r |= (a_.u32[i] >> 31) << i;
    }

    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_movemask_ps
  #define _mm256_movemask_ps(a) simde_mm256_movemask_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_movemask_pd (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_movemask_pd(a);
  #else
    simde__m256d_private a_ = simde__m256d_to_private(a);
    int r = 0;

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.f64) / sizeof(a_.f64[0])) ; i++) {
      r |= (a_.u64[i] >> 63) << i;
    }

    return r;
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_movemask_pd
  #define _mm256_movemask_pd(a) simde_mm256_movemask_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_mul_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_mul_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_mul_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_mul_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f32 = a_.f32 * b_.f32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[i] * b_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mul_ps
  #define _mm256_mul_ps(a, b) simde_mm256_mul_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_mul_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_mul_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_mul_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_mul_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f64 = a_.f64 * b_.f64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = a_.f64[i] * b_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mul_pd
  #define _mm256_mul_pd(a, b) simde_mm256_mul_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_or_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_or_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_or_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_or_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f | b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = a_.u32[i] | b_.u32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_or_ps
  #define _mm256_or_ps(a, b) simde_mm256_or_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_or_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_or_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_or_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_or_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f | b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
        r_.u64[i] = a_.u64[i] | b_.u64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_or_pd
  #define _mm256_or_pd(a, b) simde_mm256_or_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_permute_ps (simde__m256 a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
    r_.f32[i] = a_.m128_private[i >> 2].f32[(imm8 >> ((i << 1) & 7)) & 3];
  }

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_permute_ps(a, imm8) _mm256_permute_ps(a, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute_ps
  #define _mm256_permute_ps(a, imm8) simde_mm256_permute_ps(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_permute_pd (simde__m256d a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
    r_.f64[i] = a_.f64[((imm8 >> i) & 1) + (i & 2)];
  }

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_permute_pd(a, imm8) _mm256_permute_pd(a, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute_pd
  #define _mm256_permute_pd(a, imm8) simde_mm256_permute_pd(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_permute_ps (simde__m128 a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m128_private
    r_,
    a_ = simde__m128_to_private(a);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
    r_.f32[i] = a_.f32[(imm8 >> ((i << 1) & 7)) & 3];
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm_permute_ps(a, imm8) _mm_permute_ps(a, imm8)
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_permute_ps(a, imm8) simde__m128_from_wasm_v128(wasm_i32x4_shuffle(simde__m128_to_wasm_v128(a), simde__m128_to_wasm_v128(a), ((imm8) & 3), (((imm8) >> 2) & 3 ), (((imm8) >> 4) & 3), (((imm8) >> 6) & 3)))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_permute_ps
  #define _mm_permute_ps(a, imm8) simde_mm_permute_ps(a, imm8)
#endif


SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_permute_pd (simde__m128d a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 3) {
  simde__m128d_private
    r_,
    a_ = simde__m128d_to_private(a);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
    r_.f64[i] = a_.f64[((imm8 >> i) & 1) + (i & 2)];
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm_permute_pd(a, imm8) _mm_permute_pd(a, imm8)
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_permute_pd(a, imm8) simde__m128d_from_wasm_v128(wasm_i64x2_shuffle(simde__m128d_to_wasm_v128(a), simde__m128d_to_wasm_v128(a), ((imm8) & 1), (((imm8) >> 1) & 1 )))
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_permute_pd
  #define _mm_permute_pd(a, imm8) simde_mm_permute_pd(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_permutevar_ps (simde__m128 a, simde__m128i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_permutevar_ps(a, b);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a);
    simde__m128i_private b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_f32x4_make(
        (a_.f32[wasm_i32x4_extract_lane(b_.wasm_v128, 0) & 3]),
        (a_.f32[wasm_i32x4_extract_lane(b_.wasm_v128, 1) & 3]),
        (a_.f32[wasm_i32x4_extract_lane(b_.wasm_v128, 2) & 3]),
        (a_.f32[wasm_i32x4_extract_lane(b_.wasm_v128, 3) & 3]));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[b_.i32[i] & 3];
      }
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_permutevar_ps
  #define _mm_permutevar_ps(a, b) simde_mm_permutevar_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_permutevar_pd (simde__m128d a, simde__m128i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_permutevar_pd(a, b);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a);
    simde__m128i_private b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_f64x2_make(
        (a_.f64[(wasm_i64x2_extract_lane(b_.wasm_v128, 0) >> 1) & 1]),
        (a_.f64[(wasm_i64x2_extract_lane(b_.wasm_v128, 1) >> 1) & 1]));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = a_.f64[(b_.i64[i] & 2) >> 1];
      }
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_permutevar_pd
  #define _mm_permutevar_pd(a, b) simde_mm_permutevar_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_permutevar_ps (simde__m256 a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_permutevar_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);
    simde__m256i_private b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = a_.f32[(b_.i32[i] & 3) + (i & 4)];
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permutevar_ps
  #define _mm256_permutevar_ps(a, b) simde_mm256_permutevar_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_permutevar_pd (simde__m256d a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_permutevar_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a);
    simde__m256i_private b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = a_.f64[((b_.i64[i] & 2) >> 1) + (i & 2)];
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permutevar_pd
  #define _mm256_permutevar_pd(a, b) simde_mm256_permutevar_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_permute2f128_ps (simde__m256 a, simde__m256 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a),
    b_ = simde__m256_to_private(b);

  r_.m128_private[0] = (imm8 & 0x08) ? simde__m128_to_private(simde_mm_setzero_ps()) : ((imm8 & 0x02) ? b_.m128_private[(imm8     ) & 1] : a_.m128_private[(imm8     ) & 1]);
  r_.m128_private[1] = (imm8 & 0x80) ? simde__m128_to_private(simde_mm_setzero_ps()) : ((imm8 & 0x20) ? b_.m128_private[(imm8 >> 4) & 1] : a_.m128_private[(imm8 >> 4) & 1]);

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_permute2f128_ps(a, b, imm8) _mm256_permute2f128_ps(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute2f128_ps
  #define _mm256_permute2f128_ps(a, b, imm8) simde_mm256_permute2f128_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_permute2f128_pd (simde__m256d a, simde__m256d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a),
    b_ = simde__m256d_to_private(b);

  r_.m128d_private[0] = (imm8 & 0x08) ? simde__m128d_to_private(simde_mm_setzero_pd()) : ((imm8 & 0x02) ? b_.m128d_private[(imm8     ) & 1] : a_.m128d_private[(imm8     ) & 1]);
  r_.m128d_private[1] = (imm8 & 0x80) ? simde__m128d_to_private(simde_mm_setzero_pd()) : ((imm8 & 0x20) ? b_.m128d_private[(imm8 >> 4) & 1] : a_.m128d_private[(imm8 >> 4) & 1]);

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_permute2f128_pd(a, b, imm8) _mm256_permute2f128_pd(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute2f128_pd
  #define _mm256_permute2f128_pd(a, b, imm8) simde_mm256_permute2f128_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_permute2f128_si256 (simde__m256i a, simde__m256i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  r_.m128i_private[0] = (imm8 & 0x08) ? simde__m128i_to_private(simde_mm_setzero_si128()) : ((imm8 & 0x02) ? b_.m128i_private[(imm8     ) & 1] : a_.m128i_private[(imm8     ) & 1]);
  r_.m128i_private[1] = (imm8 & 0x80) ? simde__m128i_to_private(simde_mm_setzero_si128()) : ((imm8 & 0x20) ? b_.m128i_private[(imm8 >> 4) & 1] : a_.m128i_private[(imm8 >> 4) & 1]);

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
#  define simde_mm256_permute2f128_si128(a, b, imm8) _mm256_permute2f128_si128(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute2f128_si256
  #define _mm256_permute2f128_si256(a, b, imm8) simde_mm256_permute2f128_si256(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_rcp_ps (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_rcp_ps(a);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_rcp_ps(a_.m128[0]);
      r_.m128[1] = simde_mm_rcp_ps(a_.m128[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = SIMDE_FLOAT32_C(1.0) / a_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_rcp_ps
  #define _mm256_rcp_ps(a) simde_mm256_rcp_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_rsqrt_ps (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_rsqrt_ps(a);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

    #if defined(simde_math_sqrtf)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = 1.0f / simde_math_sqrtf(a_.f32[i]);
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_rsqrt_ps
  #define _mm256_rsqrt_ps(a) simde_mm256_rsqrt_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_setr_epi8 (
    int8_t e31, int8_t e30, int8_t e29, int8_t e28, int8_t e27, int8_t e26, int8_t e25, int8_t e24,
    int8_t e23, int8_t e22, int8_t e21, int8_t e20, int8_t e19, int8_t e18, int8_t e17, int8_t e16,
    int8_t e15, int8_t e14, int8_t e13, int8_t e12, int8_t e11, int8_t e10, int8_t  e9, int8_t  e8,
    int8_t  e7, int8_t  e6, int8_t  e5, int8_t  e4, int8_t  e3, int8_t  e2, int8_t  e1, int8_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setr_epi8(
        e31, e30, e29, e28, e27, e26, e25, e24,
        e23, e22, e21, e20, e19, e18, e17, e16,
        e15, e14, e13, e12, e11, e10,  e9,  e8,
        e7,  e6,  e5,  e4,  e3,  e2,  e1,  e0);
  #else
    return simde_mm256_set_epi8(
        e0,  e1,  e2,  e3,  e4,  e5,  e6,  e7,
        e8,  e9, e10, e11, e12, e13, e14, e15,
        e16, e17, e18, e19, e20, e21, e22, e23,
        e24, e25, e26, e27, e28, e29, e30, e31);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_epi8
  #define _mm256_setr_epi8(e31, e30, e29, e28, e27, e26, e25, e24, e23, e22, e21, e20, e19, e18, e17, e16, e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0) \
    simde_mm256_setr_epi8(e31, e30, e29, e28, e27, e26, e25, e24, e23, e22, e21, e20, e19, e18, e17, e16, e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_setr_epi16 (
    int16_t e15, int16_t e14, int16_t e13, int16_t e12, int16_t e11, int16_t e10, int16_t  e9, int16_t  e8,
    int16_t  e7, int16_t  e6, int16_t  e5, int16_t  e4, int16_t  e3, int16_t  e2, int16_t  e1, int16_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setr_epi16(
        e15, e14, e13, e12, e11, e10,  e9,  e8,
        e7,  e6,  e5,  e4,  e3,  e2,  e1,  e0);
  #else
    return simde_mm256_set_epi16(
        e0,  e1,  e2,  e3,  e4,  e5,  e6,  e7,
        e8,  e9, e10, e11, e12, e13, e14, e15);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_epi16
  #define _mm256_setr_epi16(e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0) \
    simde_mm256_setr_epi16(e15, e14, e13, e12, e11, e10, e9, e8, e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_setr_epi32 (
    int32_t  e7, int32_t  e6, int32_t  e5, int32_t  e4, int32_t  e3, int32_t  e2, int32_t  e1, int32_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setr_epi32(e7, e6, e5, e4, e3, e2, e1, e0);
  #else
    return simde_mm256_set_epi32(e0, e1, e2, e3, e4, e5, e6, e7);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_epi32
  #define _mm256_setr_epi32(e7, e6, e5, e4, e3, e2, e1, e0) \
    simde_mm256_setr_epi32(e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_setr_epi64x (int64_t  e3, int64_t  e2, int64_t  e1, int64_t  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setr_epi64x(e3, e2, e1, e0);
  #else
    return simde_mm256_set_epi64x(e0, e1, e2, e3);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_epi64x
  #define _mm256_setr_epi64x(e3, e2, e1, e0) \
    simde_mm256_setr_epi64x(e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_setr_ps (
    simde_float32  e7, simde_float32  e6, simde_float32  e5, simde_float32  e4,
    simde_float32  e3, simde_float32  e2, simde_float32  e1, simde_float32  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setr_ps(e7, e6, e5, e4, e3, e2, e1, e0);
  #else
    return simde_mm256_set_ps(e0, e1, e2, e3, e4, e5, e6, e7);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_ps
  #define _mm256_setr_ps(e7, e6, e5, e4, e3, e2, e1, e0) \
    simde_mm256_setr_ps(e7, e6, e5, e4, e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_setr_pd (simde_float64  e3, simde_float64  e2, simde_float64  e1, simde_float64  e0) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_setr_pd(e3, e2, e1, e0);
  #else
    return simde_mm256_set_pd(e0, e1, e2, e3);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_pd
  #define _mm256_setr_pd(e3, e2, e1, e0) \
    simde_mm256_setr_pd(e3, e2, e1, e0)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_setr_m128 (simde__m128 lo, simde__m128 hi) {
  #if defined(SIMDE_X86_AVX_NATIVE) && \
      !defined(SIMDE_BUG_GCC_REV_247851) && \
      SIMDE_DETECT_CLANG_VERSION_CHECK(3,6,0)
    return _mm256_setr_m128(lo, hi);
  #else
    return simde_mm256_set_m128(hi, lo);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_m128
  #define _mm256_setr_m128(lo, hi) \
    simde_mm256_setr_m128(lo, hi)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_setr_m128d (simde__m128d lo, simde__m128d hi) {
  #if defined(SIMDE_X86_AVX_NATIVE) && \
      !defined(SIMDE_BUG_GCC_REV_247851) && \
      SIMDE_DETECT_CLANG_VERSION_CHECK(3,6,0)
    return _mm256_setr_m128d(lo, hi);
  #else
    return simde_mm256_set_m128d(hi, lo);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_m128d
  #define _mm256_setr_m128d(lo, hi) \
    simde_mm256_setr_m128d(lo, hi)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_setr_m128i (simde__m128i lo, simde__m128i hi) {
  #if defined(SIMDE_X86_AVX_NATIVE) && \
      !defined(SIMDE_BUG_GCC_REV_247851) && \
      SIMDE_DETECT_CLANG_VERSION_CHECK(3,6,0)
    return _mm256_setr_m128i(lo, hi);
  #else
    return simde_mm256_set_m128i(hi, lo);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_setr_m128i
  #define _mm256_setr_m128i(lo, hi) \
    simde_mm256_setr_m128i(lo, hi)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_shuffle_ps (simde__m256 a, simde__m256 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256_private
    r_,
    a_ = simde__m256_to_private(a),
    b_ = simde__m256_to_private(b);

  r_.f32[0] = a_.m128_private[0].f32[(imm8 >> 0) & 3];
  r_.f32[1] = a_.m128_private[0].f32[(imm8 >> 2) & 3];
  r_.f32[2] = b_.m128_private[0].f32[(imm8 >> 4) & 3];
  r_.f32[3] = b_.m128_private[0].f32[(imm8 >> 6) & 3];
  r_.f32[4] = a_.m128_private[1].f32[(imm8 >> 0) & 3];
  r_.f32[5] = a_.m128_private[1].f32[(imm8 >> 2) & 3];
  r_.f32[6] = b_.m128_private[1].f32[(imm8 >> 4) & 3];
  r_.f32[7] = b_.m128_private[1].f32[(imm8 >> 6) & 3];

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_shuffle_ps(a, b, imm8) _mm256_shuffle_ps(a, b, imm8)
#elif SIMDE_NATURAL_VECTOR_SIZE_LE(128)
  #define simde_mm256_shuffle_ps(a, b, imm8) \
      simde_mm256_set_m128( \
          simde_mm_shuffle_ps(simde_mm256_extractf128_ps(a, 1), simde_mm256_extractf128_ps(b, 1), (imm8)), \
          simde_mm_shuffle_ps(simde_mm256_extractf128_ps(a, 0), simde_mm256_extractf128_ps(b, 0), (imm8)))
#elif defined(SIMDE_SHUFFLE_VECTOR_)
  #define simde_mm256_shuffle_ps(a, b, imm8) \
    SIMDE_SHUFFLE_VECTOR_(32, 32, a, b, \
      (((imm8) >> 0) & 3) + 0, \
      (((imm8) >> 2) & 3) + 0, \
      (((imm8) >> 4) & 3) + 8, \
      (((imm8) >> 6) & 3) + 8, \
      (((imm8) >> 0) & 3) + 4, \
      (((imm8) >> 2) & 3) + 4, \
      (((imm8) >> 4) & 3) + 12, \
      (((imm8) >> 6) & 3) + 12)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_shuffle_ps
  #define _mm256_shuffle_ps(a, b, imm8) simde_mm256_shuffle_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_shuffle_pd (simde__m256d a, simde__m256d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a),
    b_ = simde__m256d_to_private(b);

  r_.f64[0] = a_.f64[((imm8     ) & 1)    ];
  r_.f64[1] = b_.f64[((imm8 >> 1) & 1)    ];
  r_.f64[2] = a_.f64[((imm8 >> 2) & 1) | 2];
  r_.f64[3] = b_.f64[((imm8 >> 3) & 1) | 2];

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_NATIVE)
  #define simde_mm256_shuffle_pd(a, b, imm8) _mm256_shuffle_pd(a, b, imm8)
#elif SIMDE_NATURAL_VECTOR_SIZE_LE(128)
  #define simde_mm256_shuffle_pd(a, b, imm8) \
      simde_mm256_set_m128d( \
          simde_mm_shuffle_pd(simde_mm256_extractf128_pd(a, 1), simde_mm256_extractf128_pd(b, 1), (imm8 >> 2) & 3), \
          simde_mm_shuffle_pd(simde_mm256_extractf128_pd(a, 0), simde_mm256_extractf128_pd(b, 0), (imm8 >> 0) & 3))
#elif defined(SIMDE_SHUFFLE_VECTOR_)
  #define simde_mm256_shuffle_pd(a, b, imm8) \
    SIMDE_SHUFFLE_VECTOR_(64, 32, a, b, \
      (((imm8) >> 0) & 1) + 0, \
      (((imm8) >> 1) & 1) + 4, \
      (((imm8) >> 2) & 1) + 2, \
      (((imm8) >> 3) & 1) + 6)
#endif
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_shuffle_pd
  #define _mm256_shuffle_pd(a, b, imm8) simde_mm256_shuffle_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_sqrt_ps (simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_sqrt_ps(a);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_sqrt_ps(a_.m128[0]);
      r_.m128[1] = simde_mm_sqrt_ps(a_.m128[1]);
    #elif defined(simde_math_sqrtf)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = simde_math_sqrtf(a_.f32[i]);
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sqrt_ps
  #define _mm256_sqrt_ps(a) simde_mm256_sqrt_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_sqrt_pd (simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_sqrt_pd(a);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_sqrt_pd(a_.m128d[0]);
      r_.m128d[1] = simde_mm_sqrt_pd(a_.m128d[1]);
    #elif defined(simde_math_sqrt)
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = simde_math_sqrt(a_.f64[i]);
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sqrt_pd
  #define _mm256_sqrt_pd(a) simde_mm256_sqrt_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_store_ps (simde_float32 mem_addr[8], simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_store_ps(mem_addr, a);
  #else
    simde_memcpy(SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256), &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_store_ps
  #define _mm256_store_ps(mem_addr, a) simde_mm256_store_ps(HEDLEY_REINTERPRET_CAST(float*, mem_addr), a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_store_pd (simde_float64 mem_addr[4], simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_store_pd(mem_addr, a);
  #else
    simde_memcpy(SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256d), &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_store_pd
  #define _mm256_store_pd(mem_addr, a) simde_mm256_store_pd(HEDLEY_REINTERPRET_CAST(double*, mem_addr), a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_store_si256 (simde__m256i* mem_addr, simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_store_si256(mem_addr, a);
  #else
  simde_memcpy(SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256i), &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_store_si256
  #define _mm256_store_si256(mem_addr, a) simde_mm256_store_si256(mem_addr, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_storeu_ps (simde_float32 mem_addr[8], simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_storeu_ps(mem_addr, a);
  #else
    simde_memcpy(mem_addr, &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_storeu_ps
  #define _mm256_storeu_ps(mem_addr, a) simde_mm256_storeu_ps(HEDLEY_REINTERPRET_CAST(float*, mem_addr), a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_storeu_pd (simde_float64 mem_addr[4], simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_storeu_pd(mem_addr, a);
  #else
    simde_memcpy(mem_addr, &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_storeu_pd
  #define _mm256_storeu_pd(mem_addr, a) simde_mm256_storeu_pd(HEDLEY_REINTERPRET_CAST(double*, mem_addr), a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_storeu_si256 (void* mem_addr, simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_storeu_si256(SIMDE_ALIGN_CAST(__m256i*, mem_addr), a);
  #else
    simde_memcpy(mem_addr, &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_storeu_si256
  #define _mm256_storeu_si256(mem_addr, a) simde_mm256_storeu_si256(mem_addr, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_storeu2_m128 (simde_float32 hi_addr[4], simde_float32 lo_addr[4], simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE) && !defined(SIMDE_BUG_GCC_91341) && !defined(SIMDE_BUG_MCST_LCC_MISSING_AVX_LOAD_STORE_M128_FUNCS)
    _mm256_storeu2_m128(hi_addr, lo_addr, a);
  #else
    simde_mm_storeu_ps(lo_addr, simde_mm256_castps256_ps128(a));
    simde_mm_storeu_ps(hi_addr, simde_mm256_extractf128_ps(a, 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_storeu2_m128
  #define _mm256_storeu2_m128(hi_addr, lo_addr, a) simde_mm256_storeu2_m128(hi_addr, lo_addr, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_storeu2_m128d (simde_float64 hi_addr[2], simde_float64 lo_addr[2], simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE) && !defined(SIMDE_BUG_GCC_91341) && !defined(SIMDE_BUG_MCST_LCC_MISSING_AVX_LOAD_STORE_M128_FUNCS)
    _mm256_storeu2_m128d(hi_addr, lo_addr, a);
  #else
    simde_mm_storeu_pd(lo_addr, simde_mm256_castpd256_pd128(a));
    simde_mm_storeu_pd(hi_addr, simde_mm256_extractf128_pd(a, 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_storeu2_m128d
  #define _mm256_storeu2_m128d(hi_addr, lo_addr, a) simde_mm256_storeu2_m128d(hi_addr, lo_addr, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_storeu2_m128i (simde__m128i* hi_addr, simde__m128i* lo_addr, simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE) && !defined(SIMDE_BUG_GCC_91341) && !defined(SIMDE_BUG_MCST_LCC_MISSING_AVX_LOAD_STORE_M128_FUNCS)
    _mm256_storeu2_m128i(hi_addr, lo_addr, a);
  #else
    simde_mm_storeu_si128(lo_addr, simde_mm256_castsi256_si128(a));
    simde_mm_storeu_si128(hi_addr, simde_mm256_extractf128_si256(a, 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_storeu2_m128i
  #define _mm256_storeu2_m128i(hi_addr, lo_addr, a) simde_mm256_storeu2_m128i(hi_addr, lo_addr, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_stream_ps (simde_float32 mem_addr[8], simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_stream_ps(mem_addr, a);
  #elif HEDLEY_HAS_BUILTIN(__builtin_nontemporal_store) && defined(SIMDE_VECTOR_SUBSCRIPT)
    __builtin_nontemporal_store(a, SIMDE_ALIGN_CAST(__typeof__(a)*, mem_addr));
  #else
    simde_memcpy(SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256), &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_stream_ps
  #define _mm256_stream_ps(mem_addr, a) simde_mm256_stream_ps(HEDLEY_REINTERPRET_CAST(float*, mem_addr), a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_stream_pd (simde_float64 mem_addr[4], simde__m256d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_stream_pd(mem_addr, a);
  #elif HEDLEY_HAS_BUILTIN(__builtin_nontemporal_store) && defined(SIMDE_VECTOR_SUBSCRIPT)
    __builtin_nontemporal_store(a, SIMDE_ALIGN_CAST(__typeof__(a)*, mem_addr));
  #else
    simde_memcpy(SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256d), &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_stream_pd
  #define _mm256_stream_pd(mem_addr, a) simde_mm256_stream_pd(HEDLEY_REINTERPRET_CAST(double*, mem_addr), a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_stream_si256 (simde__m256i* mem_addr, simde__m256i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    _mm256_stream_si256(mem_addr, a);
  #elif HEDLEY_HAS_BUILTIN(__builtin_nontemporal_store) && defined(SIMDE_VECTOR_SUBSCRIPT)
    __builtin_nontemporal_store(a, SIMDE_ALIGN_CAST(__typeof__(a)*, mem_addr));
  #else
    simde_memcpy(SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256i), &a, sizeof(a));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_stream_si256
  #define _mm256_stream_si256(mem_addr, a) simde_mm256_stream_si256(mem_addr, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_sub_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_sub_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_sub_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_sub_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f32 = a_.f32 - b_.f32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[i] - b_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sub_ps
  #define _mm256_sub_ps(a, b) simde_mm256_sub_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_hsub_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_hsub_ps(a, b);
  #else
      return simde_mm256_sub_ps(simde_x_mm256_deinterleaveeven_ps(a, b), simde_x_mm256_deinterleaveodd_ps(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hsub_ps
  #define _mm256_hsub_ps(a, b) simde_mm256_hsub_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_sub_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_sub_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_sub_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_sub_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.f64 = a_.f64 - b_.f64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = a_.f64[i] - b_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sub_pd
  #define _mm256_sub_pd(a, b) simde_mm256_sub_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_hsub_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_hsub_pd(a, b);
  #else
      return simde_mm256_sub_pd(simde_x_mm256_deinterleaveeven_pd(a, b), simde_x_mm256_deinterleaveodd_pd(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hsub_pd
  #define _mm256_hsub_pd(a, b) simde_mm256_hsub_pd(a, b)
#endif

#if defined(SIMDE_DIAGNOSTIC_DISABLE_UNINITIALIZED_)
  HEDLEY_DIAGNOSTIC_PUSH
  SIMDE_DIAGNOSTIC_DISABLE_UNINITIALIZED_
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_undefined_ps (void) {
  simde__m256_private r_;

#if \
    defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_GCC_VERSION) || HEDLEY_GCC_VERSION_CHECK(5,0,0)) && \
    (!defined(__has_builtin) || HEDLEY_HAS_BUILTIN(__builtin_ia32_undef256))
  r_.n = _mm256_undefined_ps();
#elif !defined(SIMDE_DIAGNOSTIC_DISABLE_UNINITIALIZED_)
  r_ = simde__m256_to_private(simde_mm256_setzero_ps());
#endif

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_undefined_ps
  #define _mm256_undefined_ps() simde_mm256_undefined_ps()
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_undefined_pd (void) {
  simde__m256d_private r_;

#if \
    defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_GCC_VERSION) || HEDLEY_GCC_VERSION_CHECK(5,0,0)) && \
    (!defined(__has_builtin) || HEDLEY_HAS_BUILTIN(__builtin_ia32_undef256))
  r_.n = _mm256_undefined_pd();
#elif !defined(SIMDE_DIAGNOSTIC_DISABLE_UNINITIALIZED_)
  r_ = simde__m256d_to_private(simde_mm256_setzero_pd());
#endif

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_undefined_pd
  #define _mm256_undefined_pd() simde_mm256_undefined_pd()
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_undefined_si256 (void) {
  simde__m256i_private r_;
#if \
    defined(SIMDE_X86_AVX_NATIVE) && \
    (!defined(HEDLEY_GCC_VERSION) || HEDLEY_GCC_VERSION_CHECK(5,0,0)) && \
    (!defined(__has_builtin) || HEDLEY_HAS_BUILTIN(__builtin_ia32_undef256))
  r_.n = _mm256_undefined_si256();
#elif !defined(SIMDE_DIAGNOSTIC_DISABLE_UNINITIALIZED_)
  r_ = simde__m256i_to_private(simde_mm256_setzero_si256());
#endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_undefined_si256
  #define _mm256_undefined_si256() simde_mm256_undefined_si256()
#endif

#if defined(SIMDE_DIAGNOSTIC_DISABLE_UNINITIALIZED_)
  HEDLEY_DIAGNOSTIC_POP
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_xor_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_xor_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128[0] = simde_mm_xor_ps(a_.m128[0], b_.m128[0]);
      r_.m128[1] = simde_mm_xor_ps(a_.m128[1], b_.m128[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f ^ b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = a_.u32[i] ^ b_.u32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_xor_ps
  #define _mm256_xor_ps(a, b) simde_mm256_xor_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_xor_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_xor_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r_.m128d[0] = simde_mm_xor_pd(a_.m128d[0], b_.m128d[0]);
      r_.m128d[1] = simde_mm_xor_pd(a_.m128d[1], b_.m128d[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f ^ b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
        r_.u64[i] = a_.u64[i] ^ b_.u64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_xor_pd
  #define _mm256_xor_pd(a, b) simde_mm256_xor_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_xorsign_ps(simde__m256 dest, simde__m256 src) {
  return simde_mm256_xor_ps(simde_mm256_and_ps(simde_mm256_set1_ps(-0.0f), src), dest);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_xorsign_pd(simde__m256d dest, simde__m256d src) {
  return simde_mm256_xor_pd(simde_mm256_and_pd(simde_mm256_set1_pd(-0.0), src), dest);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_x_mm256_negate_ps(simde__m256 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return simde_mm256_xor_ps(a,_mm256_set1_ps(SIMDE_FLOAT32_C(-0.0)));
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);

    #if defined(SIMDE_VECTOR_NEGATE)
      r_.f32 = -a_.f32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = -a_.f32[i];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_x_mm256_negate_pd(simde__m256d a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return simde_mm256_xor_pd(a, _mm256_set1_pd(SIMDE_FLOAT64_C(-0.0)));
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a);

    #if defined(SIMDE_VECTOR_NEGATE)
      r_.f64 = -a_.f64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = -a_.f64[i];
      }
    #endif

    return simde__m256d_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_unpackhi_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_unpackhi_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.f32, b_.f32, 2, 10, 3, 11, 6, 14, 7, 15);
    #else
      r_.f32[0] = a_.f32[2];
      r_.f32[1] = b_.f32[2];
      r_.f32[2] = a_.f32[3];
      r_.f32[3] = b_.f32[3];
      r_.f32[4] = a_.f32[6];
      r_.f32[5] = b_.f32[6];
      r_.f32[6] = a_.f32[7];
      r_.f32[7] = b_.f32[7];
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpackhi_ps
  #define _mm256_unpackhi_ps(a, b) simde_mm256_unpackhi_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_unpackhi_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_unpackhi_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.f64, b_.f64, 1, 5, 3, 7);
    #else
      r_.f64[0] = a_.f64[1];
      r_.f64[1] = b_.f64[1];
      r_.f64[2] = a_.f64[3];
      r_.f64[3] = b_.f64[3];
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpackhi_pd
  #define _mm256_unpackhi_pd(a, b) simde_mm256_unpackhi_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_unpacklo_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_unpacklo_ps(a, b);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.f32, b_.f32, 0, 8, 1, 9, 4, 12, 5, 13);
    #else
      r_.f32[0] = a_.f32[0];
      r_.f32[1] = b_.f32[0];
      r_.f32[2] = a_.f32[1];
      r_.f32[3] = b_.f32[1];
      r_.f32[4] = a_.f32[4];
      r_.f32[5] = b_.f32[4];
      r_.f32[6] = a_.f32[5];
      r_.f32[7] = b_.f32[5];
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpacklo_ps
  #define _mm256_unpacklo_ps(a, b) simde_mm256_unpacklo_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_unpacklo_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_unpacklo_pd(a, b);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.f64, b_.f64, 0, 4, 2, 6);
    #else
      r_.f64[0] = a_.f64[0];
      r_.f64[1] = b_.f64[0];
      r_.f64[2] = a_.f64[2];
      r_.f64[3] = b_.f64[2];
    #endif

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpacklo_pd
  #define _mm256_unpacklo_pd(a, b) simde_mm256_unpacklo_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_zextps128_ps256 (simde__m128 a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_insertf128_ps(_mm256_setzero_ps(), a, 0);
  #else
    simde__m256_private r_;

    r_.m128_private[0] = simde__m128_to_private(a);
    r_.m128_private[1] = simde__m128_to_private(simde_mm_setzero_ps());

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_zextps128_ps256
  #define _mm256_zextps128_ps256(a) simde_mm256_zextps128_ps256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_zextpd128_pd256 (simde__m128d a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_insertf128_pd(_mm256_setzero_pd(), a, 0);
  #else
    simde__m256d_private r_;

    r_.m128d_private[0] = simde__m128d_to_private(a);
    r_.m128d_private[1] = simde__m128d_to_private(simde_mm_setzero_pd());

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_zextpd128_pd256
  #define _mm256_zextpd128_pd256(a) simde_mm256_zextpd128_pd256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_zextsi128_si256 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_insertf128_si256(_mm256_setzero_si256(), a, 0);
  #else
    simde__m256i_private r_;

    r_.m128i_private[0] = simde__m128i_to_private(a);
    r_.m128i_private[1] = simde__m128i_to_private(simde_mm_setzero_si128());

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_zextsi128_si256
  #define _mm256_zextsi128_si256(a) simde_mm256_zextsi128_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testc_ps (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_testc_ps(a, b);
  #else
    simde__m128_private
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_u32x4_shr(wasm_v128_or(wasm_v128_not(b_.wasm_v128), a_.wasm_v128), 31);
      m = wasm_v128_and(m, simde_mm_movehl_ps(m, m));
      m = wasm_v128_and(m, simde_mm_shuffle_epi32(m, SIMDE_MM_SHUFFLE(3, 2, 0, 1)));
      return wasm_i32x4_extract_lane(m, 0);
    #else
      uint_fast32_t r = 0;
      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.u32) / sizeof(a_.u32[0])) ; i++) {
        r |= ~a_.u32[i] & b_.u32[i];
      }

      return HEDLEY_STATIC_CAST(int, ((~r >> 31) & 1));
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_testc_ps
  #define _mm_testc_ps(a, b) simde_mm_testc_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testc_pd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_testc_pd(a, b);
  #else
    simde__m128d_private
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_u64x2_shr(wasm_v128_or(wasm_v128_not(b_.wasm_v128), a_.wasm_v128), 63);
      return HEDLEY_STATIC_CAST(int, wasm_i64x2_extract_lane(m, 0) & wasm_i64x2_extract_lane(m, 1));
    #else
      uint_fast64_t r = 0;
      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
        r |= ~a_.u64[i] & b_.u64[i];
      }

      return HEDLEY_STATIC_CAST(int, ((~r >> 63) & 1));
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_testc_pd
  #define _mm_testc_pd(a, b) simde_mm_testc_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testc_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testc_ps(a, b);
  #else
    uint_fast32_t r = 0;
    simde__m256_private
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.u32) / sizeof(a_.u32[0])) ; i++) {
      r |= ~a_.u32[i] & b_.u32[i];
    }

    return HEDLEY_STATIC_CAST(int, ((~r >> 31) & 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testc_ps
  #define _mm256_testc_ps(a, b) simde_mm256_testc_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testc_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testc_pd(a, b);
  #else
    uint_fast64_t r = 0;
    simde__m256d_private
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
      r |= ~a_.u64[i] & b_.u64[i];
    }

    return HEDLEY_STATIC_CAST(int, ((~r >> 63) & 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testc_pd
  #define _mm256_testc_pd(a, b) simde_mm256_testc_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testc_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testc_si256(a, b);
  #else
    int_fast32_t r = 0;
    simde__m256i_private
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.i32f) / sizeof(a_.i32f[0])) ; i++) {
      r |= ~a_.i32f[i] & b_.i32f[i];
    }

    return HEDLEY_STATIC_CAST(int, !r);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testc_si256
  #define _mm256_testc_si256(a, b) simde_mm256_testc_si256(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testz_ps (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_testz_ps(a, b);
  #else
    simde__m128_private
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_u32x4_shr(wasm_v128_not(wasm_v128_and(a_.wasm_v128, b_.wasm_v128)), 31);
      m = wasm_v128_and(m, simde_mm_movehl_ps(m, m));
      m = wasm_v128_and(m, simde_mm_shuffle_epi32(m, SIMDE_MM_SHUFFLE(3, 2, 0, 1)));
      return wasm_i32x4_extract_lane(m, 0);
    #else
      uint_fast32_t r = 0;
      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.u32) / sizeof(a_.u32[0])) ; i++) {
        r |= a_.u32[i] & b_.u32[i];
      }

      return HEDLEY_STATIC_CAST(int, ((~r >> 31) & 1));
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_testz_ps
  #define _mm_testz_ps(a, b) simde_mm_testz_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testz_pd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_testz_pd(a, b);
  #else
    simde__m128d_private
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_u64x2_shr(wasm_v128_not(wasm_v128_and(a_.wasm_v128, b_.wasm_v128)), 63);
      return HEDLEY_STATIC_CAST(int, wasm_i64x2_extract_lane(m, 0) & wasm_i64x2_extract_lane(m, 1));
    #else
      uint_fast64_t r = 0;
      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
        r |= a_.u64[i] & b_.u64[i];
      }

      return HEDLEY_STATIC_CAST(int, ((~r >> 63) & 1));
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_testz_pd
  #define _mm_testz_pd(a, b) simde_mm_testz_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testz_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testz_ps(a, b);
  #else
    uint_fast32_t r = 0;
    simde__m256_private
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.u32) / sizeof(a_.u32[0])) ; i++) {
      r |= a_.u32[i] & b_.u32[i];
    }

    return HEDLEY_STATIC_CAST(int, ((~r >> 31) & 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testz_ps
  #define _mm256_testz_ps(a, b) simde_mm256_testz_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testz_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testz_pd(a, b);
  #else
    uint_fast64_t r = 0;
    simde__m256d_private
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    SIMDE_VECTORIZE_REDUCTION(|:r)
    for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
      r |= a_.u64[i] & b_.u64[i];
    }

    return HEDLEY_STATIC_CAST(int, ((~r >> 63) & 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testz_pd
  #define _mm256_testz_pd(a, b) simde_mm256_testz_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testz_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testz_si256(a, b);
  #else
    int_fast32_t r = 0;
    simde__m256i_private
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_VECTOR_SIZE_LE(128)
      r = simde_mm_testz_si128(a_.m128i[0], b_.m128i[0]) && simde_mm_testz_si128(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.i32f) / sizeof(a_.i32f[0])) ; i++) {
        r |= a_.i32f[i] & b_.i32f[i];
      }

      r = !r;
    #endif

    return HEDLEY_STATIC_CAST(int, r);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testz_si256
  #define _mm256_testz_si256(a, b) simde_mm256_testz_si256(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testnzc_ps (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_testnzc_ps(a, b);
  #else
    simde__m128_private
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b);

    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_u32x4_shr(wasm_v128_and(a_.wasm_v128, b_.wasm_v128), 31);
      v128_t m2 = wasm_u32x4_shr(wasm_v128_andnot(b_.wasm_v128, a_.wasm_v128), 31);
      m  = wasm_v128_or(m,  simde_mm_movehl_ps(m, m));
      m2 = wasm_v128_or(m2, simde_mm_movehl_ps(m2, m2));
      m  = wasm_v128_or(m,  simde_mm_shuffle_epi32(m, SIMDE_MM_SHUFFLE(3, 2, 0, 1)));
      m2 = wasm_v128_or(m2, simde_mm_shuffle_epi32(m2, SIMDE_MM_SHUFFLE(3, 2, 0, 1)));
      return wasm_i32x4_extract_lane(m, 0) & wasm_i32x4_extract_lane(m2, 0);
    #else
      uint32_t rz = 0, rc = 0;
      for (size_t i = 0 ; i < (sizeof(a_.u32) / sizeof(a_.u32[0])) ; i++) {
        rc |= ~a_.u32[i] & b_.u32[i];
        rz |=  a_.u32[i] & b_.u32[i];
      }

      return
        (rc >> ((sizeof(rc) * CHAR_BIT) - 1)) &
        (rz >> ((sizeof(rz) * CHAR_BIT) - 1));
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_testnzc_ps
  #define _mm_testnzc_ps(a, b) simde_mm_testnzc_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testnzc_pd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm_testnzc_pd(a, b);
  #else
    simde__m128d_private
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b);
    #if defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_u64x2_shr(wasm_v128_and(a_.wasm_v128, b_.wasm_v128), 63);
      v128_t m2 = wasm_u64x2_shr(wasm_v128_andnot(b_.wasm_v128, a_.wasm_v128), 63);
      return HEDLEY_STATIC_CAST(int, (wasm_i64x2_extract_lane(m, 0)  | wasm_i64x2_extract_lane(m, 1))
                                   & (wasm_i64x2_extract_lane(m2, 0) | wasm_i64x2_extract_lane(m2, 1)));
    #else
      uint64_t rc = 0, rz = 0;
      for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
        rc |= ~a_.u64[i] & b_.u64[i];
        rz |=  a_.u64[i] & b_.u64[i];
      }

      return
        (rc >> ((sizeof(rc) * CHAR_BIT) - 1)) &
        (rz >> ((sizeof(rz) * CHAR_BIT) - 1));
    #endif
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm_testnzc_pd
  #define _mm_testnzc_pd(a, b) simde_mm_testnzc_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testnzc_ps (simde__m256 a, simde__m256 b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testnzc_ps(a, b);
  #else
    uint32_t rc = 0, rz = 0;
    simde__m256_private
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b);

    for (size_t i = 0 ; i < (sizeof(a_.u32) / sizeof(a_.u32[0])) ; i++) {
      rc |= ~a_.u32[i] & b_.u32[i];
      rz |=  a_.u32[i] & b_.u32[i];
    }

    return
      (rc >> ((sizeof(rc) * CHAR_BIT) - 1)) &
      (rz >> ((sizeof(rz) * CHAR_BIT) - 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testnzc_ps
  #define _mm256_testnzc_ps(a, b) simde_mm256_testnzc_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testnzc_pd (simde__m256d a, simde__m256d b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testnzc_pd(a, b);
  #else
    uint64_t rc = 0, rz = 0;
    simde__m256d_private
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b);

    for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
      rc |= ~a_.u64[i] & b_.u64[i];
      rz |=  a_.u64[i] & b_.u64[i];
    }

    return
      (rc >> ((sizeof(rc) * CHAR_BIT) - 1)) &
      (rz >> ((sizeof(rz) * CHAR_BIT) - 1));
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testnzc_pd
  #define _mm256_testnzc_pd(a, b) simde_mm256_testnzc_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_testnzc_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX_NATIVE)
    return _mm256_testnzc_si256(a, b);
  #else
    int32_t rc = 0, rz = 0;
    simde__m256i_private
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    for (size_t i = 0 ; i < (sizeof(a_.i32f) / sizeof(a_.i32f[0])) ; i++) {
      rc |= ~a_.i32f[i] & b_.i32f[i];
      rz |=  a_.i32f[i] & b_.i32f[i];
    }

    return !!(rc & rz);
  #endif
}
#if defined(SIMDE_X86_AVX_ENABLE_NATIVE_ALIASES)
  #undef _mm256_testnzc_si256
  #define _mm256_testnzc_si256(a, b) simde_mm256_testnzc_si256(a, b)
#endif

SIMDE_END_DECLS_

HEDLEY_DIAGNOSTIC_POP

#endif /* !defined(SIMDE_X86_AVX_H) */
