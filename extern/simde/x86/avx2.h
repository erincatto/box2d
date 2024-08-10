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
 *   2019-2020 Michael R. Crusoe <crusoe@debian.org>
 *   2020      Himanshi Mathur <himanshi18037@iiitd.ac.in>
 *   2020      Hidayat Khan <huk2209@gmail.com>
 */

#if !defined(SIMDE_X86_AVX2_H)
#define SIMDE_X86_AVX2_H

#include "avx.h"

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DISABLE_UNWANTED_DIAGNOSTICS
SIMDE_BEGIN_DECLS_

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_abs_epi8 (simde__m256i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_abs_epi8(a);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_abs_epi8(a_.m128i[0]);
      r_.m128i[1] = simde_mm_abs_epi8(a_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = (a_.i8[i] < INT32_C(0)) ? -a_.i8[i] : a_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_abs_epi8
  #define _mm256_abs_epi8(a) simde_mm256_abs_epi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_abs_epi16 (simde__m256i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_abs_epi16(a);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_abs_epi16(a_.m128i[0]);
      r_.m128i[1] = simde_mm_abs_epi16(a_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (a_.i16[i] < INT32_C(0)) ? -a_.i16[i] : a_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_abs_epi16
  #define _mm256_abs_epi16(a) simde_mm256_abs_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_abs_epi32(simde__m256i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_abs_epi32(a);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_abs_epi32(a_.m128i[0]);
      r_.m128i[1] = simde_mm_abs_epi32(a_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0; i < (sizeof(r_.i32) / sizeof(r_.i32[0])); i++) {
        r_.i32[i] = (a_.i32[i] < INT32_C(0)) ? -a_.i32[i] : a_.i32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_abs_epi32
  #define _mm256_abs_epi32(a) simde_mm256_abs_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_add_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_add_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_add_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_add_epi8(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i8 = a_.i8 + b_.i8;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[i] + b_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_add_epi8
  #define _mm256_add_epi8(a, b) simde_mm256_add_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_add_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_add_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_add_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_add_epi16(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i16 = a_.i16 + b_.i16;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a_.i16[i] + b_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_add_epi16
  #define _mm256_add_epi16(a, b) simde_mm256_add_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_hadd_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_hadd_epi16(a, b);
  #else
    return simde_mm256_add_epi16(simde_x_mm256_deinterleaveeven_epi16(a, b), simde_x_mm256_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hadd_epi16
  #define _mm256_hadd_epi16(a, b) simde_mm256_hadd_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_add_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_add_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_add_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_add_epi32(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32 = a_.i32 + b_.i32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] + b_.i32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_add_epi32
  #define _mm256_add_epi32(a, b) simde_mm256_add_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_hadd_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_hadd_epi32(a, b);
  #else
    return simde_mm256_add_epi32(simde_x_mm256_deinterleaveeven_epi32(a, b), simde_x_mm256_deinterleaveodd_epi32(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hadd_epi32
  #define _mm256_hadd_epi32(a, b) simde_mm256_hadd_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_add_epi64 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_add_epi64(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_add_epi64(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_add_epi64(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS) && !defined(SIMDE_BUG_CLANG_BAD_VI64_OPS)
      r_.i64 = a_.i64 + b_.i64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i64[i] + b_.i64[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_add_epi64
  #define _mm256_add_epi64(a, b) simde_mm256_add_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_alignr_epi8 (simde__m256i a, simde__m256i b, int count)
    SIMDE_REQUIRE_CONSTANT_RANGE(count, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  if (HEDLEY_UNLIKELY(count > 31))
    return simde_mm256_setzero_si256();

  for (size_t h = 0 ; h < (sizeof(r_.m128i) / sizeof(r_.m128i[0])) ; h++) {
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.m128i_private[h].i8) / sizeof(r_.m128i_private[h].i8[0])) ; i++) {
      const int srcpos = count + HEDLEY_STATIC_CAST(int, i);
      if (srcpos > 31) {
        r_.m128i_private[h].i8[i] = 0;
      } else if (srcpos > 15) {
        r_.m128i_private[h].i8[i] = a_.m128i_private[h].i8[(srcpos) & 15];
      } else {
        r_.m128i_private[h].i8[i] = b_.m128i_private[h].i8[srcpos];
      }
    }
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE) && !defined(SIMDE_BUG_PGI_30106)
#  define simde_mm256_alignr_epi8(a, b, count) _mm256_alignr_epi8(a, b, count)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_alignr_epi8(a, b, count) \
      simde_mm256_set_m128i( \
          simde_mm_alignr_epi8(simde_mm256_extracti128_si256(a, 1), simde_mm256_extracti128_si256(b, 1), (count)), \
          simde_mm_alignr_epi8(simde_mm256_extracti128_si256(a, 0), simde_mm256_extracti128_si256(b, 0), (count)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_alignr_epi8
  #define _mm256_alignr_epi8(a, b, count) simde_mm256_alignr_epi8(a, b, (count))
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_and_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_and_si256(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_and_si128(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_and_si128(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f & b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i64[i] & b_.i64[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_and_si256
  #define _mm256_and_si256(a, b) simde_mm256_and_si256(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_andnot_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_andnot_si256(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_andnot_si128(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_andnot_si128(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = ~(a_.i32f[i]) & b_.i32f[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_andnot_si256
  #define _mm256_andnot_si256(a, b) simde_mm256_andnot_si256(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_adds_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_adds_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_adds_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_adds_epi8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = simde_math_adds_i8(a_.i8[i], b_.i8[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_adds_epi8
  #define _mm256_adds_epi8(a, b) simde_mm256_adds_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_adds_epi16(simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_adds_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_adds_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_adds_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = simde_math_adds_i16(a_.i16[i], b_.i16[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_adds_epi16
  #define _mm256_adds_epi16(a, b) simde_mm256_adds_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_hadds_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_hadds_epi16(a, b);
  #else
    return simde_mm256_adds_epi16(simde_x_mm256_deinterleaveeven_epi16(a, b), simde_x_mm256_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hadds_epi16
  #define _mm256_hadds_epi16(a, b) simde_mm256_hadds_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_adds_epu8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_adds_epu8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_adds_epu8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_adds_epu8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
        r_.u8[i] = simde_math_adds_u8(a_.u8[i], b_.u8[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_adds_epu8
  #define _mm256_adds_epu8(a, b) simde_mm256_adds_epu8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_adds_epu16(simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_adds_epu16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_adds_epu16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_adds_epu16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
        r_.u16[i] = simde_math_adds_u16(a_.u16[i], b_.u16[i]);
    }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_adds_epu16
  #define _mm256_adds_epu16(a, b) simde_mm256_adds_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_avg_epu8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_avg_epu8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
      r_.u8[i] = (a_.u8[i] + b_.u8[i] + 1) >> 1;
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_avg_epu8
  #define _mm256_avg_epu8(a, b) simde_mm256_avg_epu8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_avg_epu16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_avg_epu16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
      r_.u16[i] = (a_.u16[i] + b_.u16[i] + 1) >> 1;
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_avg_epu16
  #define _mm256_avg_epu16(a, b) simde_mm256_avg_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_blend_epi32(simde__m128i a, simde__m128i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
    r_.i32[i] = ((imm8 >> i) & 1) ? b_.i32[i] : a_.i32[i];
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm_blend_epi32(a, b, imm8) _mm_blend_epi32(a, b, imm8)
#elif SIMDE_NATURAL_FLOAT_VECTOR_SIZE_LE(128)
#  define simde_mm_blend_epi32(a, b, imm8) \
  simde_mm_castps_si128(simde_mm_blend_ps(simde_mm_castsi128_ps(a), simde_mm_castsi128_ps(b), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_blend_epi32
  #define _mm_blend_epi32(a, b, imm8) simde_mm_blend_epi32(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_blend_epi16(simde__m256i a, simde__m256i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
    r_.i16[i] = ((imm8 >> i%8) & 1) ? b_.i16[i] : a_.i16[i];
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE) && defined(SIMDE_BUG_CLANG_REV_234560)
#  define simde_mm256_blend_epi16(a, b, imm8) _mm256_castpd_si256(_mm256_blend_epi16(a, b, imm8))
#elif defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_blend_epi16(a, b, imm8) _mm256_blend_epi16(a, b, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_blend_epi16(a, b, imm8) \
      simde_mm256_set_m128i( \
          simde_mm_blend_epi16(simde_mm256_extracti128_si256(a, 1), simde_mm256_extracti128_si256(b, 1), (imm8)), \
          simde_mm_blend_epi16(simde_mm256_extracti128_si256(a, 0), simde_mm256_extracti128_si256(b, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blend_epi16
  #define _mm256_blend_epi16(a, b, imm8) simde_mm256_blend_epi16(a, b, imm8)
#endif


SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_blend_epi32(simde__m256i a, simde__m256i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
    r_.i32[i] = ((imm8 >> i) & 1) ? b_.i32[i] : a_.i32[i];
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_blend_epi32(a, b, imm8) _mm256_blend_epi32(a, b, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_blend_epi32(a, b, imm8) \
      simde_mm256_set_m128i( \
          simde_mm_blend_epi32(simde_mm256_extracti128_si256(a, 1), simde_mm256_extracti128_si256(b, 1), (imm8) >> 4), \
          simde_mm_blend_epi32(simde_mm256_extracti128_si256(a, 0), simde_mm256_extracti128_si256(b, 0), (imm8) & 0x0F))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blend_epi32
  #define _mm256_blend_epi32(a, b, imm8) simde_mm256_blend_epi32(a, b, imm8)
#endif


SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_blendv_epi8(simde__m256i a, simde__m256i b, simde__m256i mask) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_blendv_epi8(a, b, mask);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b),
      mask_ = simde__m256i_to_private(mask);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_blendv_epi8(a_.m128i[0], b_.m128i[0], mask_.m128i[0]);
      r_.m128i[1] = simde_mm_blendv_epi8(a_.m128i[1], b_.m128i[1], mask_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
      __typeof__(mask_.i8) tmp = mask_.i8 >> 7;
      r_.i8 = (tmp & b_.i8) | (~tmp & a_.i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
        int8_t tmp = mask_.i8[i] >> 7;
        r_.i8[i] = (tmp & b_.i8[i]) | (~tmp & a_.i8[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_blendv_epi8(a, b, imm8)  _mm256_blendv_epi8(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_blendv_epi8
  #define _mm256_blendv_epi8(a, b, mask) simde_mm256_blendv_epi8(a, b, mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_broadcastb_epi8 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_broadcastb_epi8(a);
  #else
    simde__m128i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
      r_.i8[i] = a_.i8[0];
    }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcastb_epi8
  #define _mm_broadcastb_epi8(a) simde_mm_broadcastb_epi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_broadcastb_epi8 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_broadcastb_epi8(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
      r_.i8[i] = a_.i8[0];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastb_epi8
  #define _mm256_broadcastb_epi8(a) simde_mm256_broadcastb_epi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_broadcastw_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_broadcastw_epi16(a);
  #else
    simde__m128i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = a_.i16[0];
    }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcastw_epi16
  #define _mm_broadcastw_epi16(a) simde_mm_broadcastw_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_broadcastw_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_broadcastw_epi16(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = a_.i16[0];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastw_epi16
  #define _mm256_broadcastw_epi16(a) simde_mm256_broadcastw_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_broadcastd_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_broadcastd_epi32(a);
  #else
    simde__m128i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = a_.i32[0];
    }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcastd_epi32
  #define _mm_broadcastd_epi32(a) simde_mm_broadcastd_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_broadcastd_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_broadcastd_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = a_.i32[0];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastd_epi32
  #define _mm256_broadcastd_epi32(a) simde_mm256_broadcastd_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_broadcastq_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_broadcastq_epi64(a);
  #else
    simde__m128i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = a_.i64[0];
    }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcastq_epi64
  #define _mm_broadcastq_epi64(a) simde_mm_broadcastq_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_broadcastq_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_broadcastq_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_= simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = a_.i64[0];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastq_epi64
  #define _mm256_broadcastq_epi64(a) simde_mm256_broadcastq_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_broadcastss_ps (simde__m128 a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_broadcastss_ps(a);
  #elif defined(SIMDE_X86_SSE_NATIVE)
    return simde_mm_shuffle_ps(a, a, 0);
  #else
    simde__m128_private r_;
    simde__m128_private a_= simde__m128_to_private(a);

    #if defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.f32, a_.f32, 0, 0, 0, 0);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[0];
      }
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcastss_ps
  #define _mm_broadcastss_ps(a) simde_mm_broadcastss_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_broadcastss_ps (simde__m128 a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_broadcastss_ps(a);
  #else
    simde__m256_private r_;
    simde__m128_private a_= simde__m128_to_private(a);

    #if defined(SIMDE_X86_AVX_NATIVE)
      __m128 tmp = _mm_permute_ps(a_.n, 0);
      r_.n = _mm256_insertf128_ps(_mm256_castps128_ps256(tmp), tmp, 1);
    #elif HEDLEY_HAS_BUILTIN(__builtin_shufflevector)
      r_.f32 = __builtin_shufflevector(a_.f32, a_.f32, 0, 0, 0, 0, 0, 0, 0, 0);
    #elif SIMDE_NATURAL_FLOAT_VECTOR_SIZE_LE(128)
      r_.m128[0] = r_.m128[1] = simde_mm_broadcastss_ps(simde__m128_from_private(a_));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[0];
      }
    #endif

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastss_ps
  #define _mm256_broadcastss_ps(a) simde_mm256_broadcastss_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_broadcastsd_pd (simde__m128d a) {
  return simde_mm_movedup_pd(a);
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_broadcastsd_pd
  #define _mm_broadcastsd_pd(a) simde_mm_broadcastsd_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_broadcastsd_pd (simde__m128d a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_broadcastsd_pd(a);
  #else
    simde__m256d_private r_;
    simde__m128d_private a_= simde__m128d_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = a_.f64[0];
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastsd_pd
  #define _mm256_broadcastsd_pd(a) simde_mm256_broadcastsd_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_broadcastsi128_si256 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE) && \
      (!defined(HEDLEY_GCC_VERSION) || HEDLEY_GCC_VERSION_CHECK(4,8,0))
    return _mm256_broadcastsi128_si256(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i_private[0] = a_;
      r_.m128i_private[1] = a_;
    #else
      r_.i64[0] = a_.i64[0];
      r_.i64[1] = a_.i64[1];
      r_.i64[2] = a_.i64[0];
      r_.i64[3] = a_.i64[1];
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#define simde_mm_broadcastsi128_si256(a) simde_mm256_broadcastsi128_si256(a)
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_broadcastsi128_si256
  #define _mm256_broadcastsi128_si256(a) simde_mm256_broadcastsi128_si256(a)
  #undef _mm_broadcastsi128_si256
  #define _mm_broadcastsi128_si256(a) simde_mm256_broadcastsi128_si256(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_bslli_epi128 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);
    const int ssize = HEDLEY_STATIC_CAST(int, (sizeof(r_.i8) / sizeof(r_.i8[0])));

    SIMDE_VECTORIZE
    for (int i = 0 ; i < ssize ; i++) {
      const int e = i - imm8;
      if(i >= (ssize/2)) {
        if(e >= (ssize/2) && e < ssize)
          r_.i8[i] = a_.i8[e];
        else
          r_.i8[i] = 0;
      }
      else{
        if(e >= 0 && e < (ssize/2))
          r_.i8[i] = a_.i8[e];
        else
          r_.i8[i] = 0;
      }
    }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE) && \
    (!defined(HEDLEY_GCC_VERSION) || HEDLEY_GCC_VERSION_CHECK(4,8,0)) && \
    SIMDE_DETECT_CLANG_VERSION_CHECK(3,7,0)
  #define simde_mm256_bslli_epi128(a, imm8) _mm256_bslli_epi128(a, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_bslli_epi128
  #define _mm256_bslli_epi128(a, imm8) simde_mm256_bslli_epi128(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_bsrli_epi128 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);
    const int ssize = HEDLEY_STATIC_CAST(int, (sizeof(r_.i8) / sizeof(r_.i8[0])));

    SIMDE_VECTORIZE
    for (int i = 0 ; i < ssize ; i++) {
      const int e = i + imm8;
      if(i < (ssize/2)) {
        if(e >= 0 && e < (ssize/2))
          r_.i8[i] = a_.i8[e];
        else
          r_.i8[i] = 0;
      }
      else{
        if(e >= (ssize/2) && e < ssize)
          r_.i8[i] = a_.i8[e];
        else
          r_.i8[i] = 0;
      }
    }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE) && \
    (!defined(HEDLEY_GCC_VERSION) || HEDLEY_GCC_VERSION_CHECK(4,8,0)) && \
    SIMDE_DETECT_CLANG_VERSION_CHECK(3,7,0)
  #define simde_mm256_bsrli_epi128(a, imm8) _mm256_bsrli_epi128(a, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_bsrli_epi128
  #define _mm256_bsrli_epi128(a, imm8) simde_mm256_bsrli_epi128(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpeq_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpeq_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpeq_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpeq_epi8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = (a_.i8[i] == b_.i8[i]) ? ~INT8_C(0) : INT8_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpeq_epi8
  #define _mm256_cmpeq_epi8(a, b) simde_mm256_cmpeq_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpeq_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpeq_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpeq_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpeq_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (a_.i16[i] == b_.i16[i]) ? ~INT16_C(0) : INT16_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpeq_epi16
  #define _mm256_cmpeq_epi16(a, b) simde_mm256_cmpeq_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpeq_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpeq_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpeq_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpeq_epi32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = (a_.i32[i] == b_.i32[i]) ? ~INT32_C(0) : INT32_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpeq_epi32
  #define _mm256_cmpeq_epi32(a, b) simde_mm256_cmpeq_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpeq_epi64 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpeq_epi64(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpeq_epi64(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpeq_epi64(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = (a_.i64[i] == b_.i64[i]) ? ~INT64_C(0) : INT64_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpeq_epi64
  #define _mm256_cmpeq_epi64(a, b) simde_mm256_cmpeq_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpgt_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpgt_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpgt_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpgt_epi8(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i8 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i8), a_.i8 > b_.i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = (a_.i8[i] > b_.i8[i]) ? ~INT8_C(0) : INT8_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpgt_epi8
  #define _mm256_cmpgt_epi8(a, b) simde_mm256_cmpgt_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpgt_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpgt_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpgt_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpgt_epi16(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i16 = a_.i16 > b_.i16;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (a_.i16[i] > b_.i16[i]) ? ~INT16_C(0) : INT16_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpgt_epi16
  #define _mm256_cmpgt_epi16(a, b) simde_mm256_cmpgt_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpgt_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpgt_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpgt_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpgt_epi32(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), a_.i32 > b_.i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = (a_.i32[i] > b_.i32[i]) ? ~INT32_C(0) : INT32_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpgt_epi32
  #define _mm256_cmpgt_epi32(a, b) simde_mm256_cmpgt_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cmpgt_epi64 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cmpgt_epi64(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_cmpgt_epi64(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_cmpgt_epi64(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), a_.i64 > b_.i64);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = (a_.i64[i] > b_.i64[i]) ? ~INT64_C(0) : INT64_C(0);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cmpgt_epi64
  #define _mm256_cmpgt_epi64(a, b) simde_mm256_cmpgt_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepi8_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepi8_epi16(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i16, a_.i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi8_epi16
  #define _mm256_cvtepi8_epi16(a) simde_mm256_cvtepi8_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepi8_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepi8_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i32, a_.m64_private[0].i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi8_epi32
  #define _mm256_cvtepi8_epi32(a) simde_mm256_cvtepi8_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepi8_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepi8_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = a_.i8[i];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi8_epi64
  #define _mm256_cvtepi8_epi64(a) simde_mm256_cvtepi8_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepi16_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepi16_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i32, a_.i16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi16_epi32
  #define _mm256_cvtepi16_epi32(a) simde_mm256_cvtepi16_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepi16_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepi16_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i64, a_.m64_private[0].i16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi16_epi64
  #define _mm256_cvtepi16_epi64(a) simde_mm256_cvtepi16_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepi32_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepi32_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i64, a_.i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepi32_epi64
  #define _mm256_cvtepi32_epi64(a) simde_mm256_cvtepi32_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepu8_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepu8_epi16(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i16, a_.u8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a_.u8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepu8_epi16
  #define _mm256_cvtepu8_epi16(a) simde_mm256_cvtepu8_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepu8_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepu8_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i32, a_.m64_private[0].u8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.u8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepu8_epi32
  #define _mm256_cvtepu8_epi32(a) simde_mm256_cvtepu8_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepu8_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepu8_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = a_.u8[i];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepu8_epi64
  #define _mm256_cvtepu8_epi64(a) simde_mm256_cvtepu8_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepu16_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepu16_epi32(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i32, a_.u16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.u16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepu16_epi32
  #define _mm256_cvtepu16_epi32(a) simde_mm256_cvtepu16_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepu16_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepu16_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i64, a_.m64_private[0].u16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.u16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepu16_epi64
  #define _mm256_cvtepu16_epi64(a) simde_mm256_cvtepu16_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_cvtepu32_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_cvtepu32_epi64(a);
  #else
    simde__m256i_private r_;
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i64, a_.u32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.u32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_cvtepu32_epi64
  #define _mm256_cvtepu32_epi64(a) simde_mm256_cvtepu32_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_extract_epi8 (simde__m256i a, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 31){
  simde__m256i_private a_ = simde__m256i_to_private(a);
  return a_.i8[index];
}
#if defined(SIMDE_X86_AVX2_NATIVE) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,10,0))
  #define simde_mm256_extract_epi8(a, index) _mm256_extract_epi8(a, index)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extract_epi8
  #define _mm256_extract_epi8(a, index) simde_mm256_extract_epi8(a, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm256_extract_epi16 (simde__m256i a, const int index)
    SIMDE_REQUIRE_RANGE(index, 0, 15)  {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  return a_.i16[index];
}
#if defined(SIMDE_X86_AVX2_NATIVE) && \
    (!defined(HEDLEY_MSVC_VERSION) || HEDLEY_MSVC_VERSION_CHECK(19,10,0))
  #define simde_mm256_extract_epi16(a, index) _mm256_extract_epi16(a, index)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extract_epi16
  #define _mm256_extract_epi16(a, index) simde_mm256_extract_epi16(a, index)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_extracti128_si256 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  return a_.m128i[imm8];
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_extracti128_si256(a, imm8) _mm256_extracti128_si256(a, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_extracti128_si256
  #define _mm256_extracti128_si256(a, imm8) simde_mm256_extracti128_si256(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_i32gather_epi32(const int32_t* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int32_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i32[i] = dst;
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_i32gather_epi32(base_addr, vindex, scale) _mm_i32gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i32gather_epi32
  #define _mm_i32gather_epi32(base_addr, vindex, scale) simde_mm_i32gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mask_i32gather_epi32(simde__m128i src, const int32_t* base_addr, simde__m128i vindex, simde__m128i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    src_ = simde__m128i_to_private(src),
    mask_ = simde__m128i_to_private(mask),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int32_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i32[i] = dst;
    }
    else {
      r_.i32[i] = src_.i32[i];
    }
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_mask_i32gather_epi32(src, base_addr, vindex, mask, scale) _mm_mask_i32gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i32gather_epi32
  #define _mm_mask_i32gather_epi32(src, base_addr, vindex, mask, scale) simde_mm_mask_i32gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_i32gather_epi32(const int32_t* base_addr, simde__m256i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int32_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i32[i] = dst;
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_i32gather_epi32(base_addr, vindex, scale) _mm256_i32gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i32gather_epi32
  #define _mm256_i32gather_epi32(base_addr, vindex, scale) simde_mm256_i32gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mask_i32gather_epi32(simde__m256i src, const int32_t* base_addr, simde__m256i vindex, simde__m256i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex),
    src_ = simde__m256i_to_private(src),
    mask_ = simde__m256i_to_private(mask),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int32_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i32[i] = dst;
    }
    else {
      r_.i32[i] = src_.i32[i];
    }
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_mask_i32gather_epi32(src, base_addr, vindex, mask, scale) _mm256_mask_i32gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i32gather_epi32
  #define _mm256_mask_i32gather_epi32(src, base_addr, vindex, mask, scale) simde_mm256_mask_i32gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_i64gather_epi32(const int32_t* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    r_ = simde__m128i_to_private(simde_mm_setzero_si128());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int32_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i32[i] = dst;
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_i64gather_epi32(base_addr, vindex, scale) _mm_i64gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i64gather_epi32
  #define _mm_i64gather_epi32(base_addr, vindex, scale) simde_mm_i64gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mask_i64gather_epi32(simde__m128i src, const int32_t* base_addr, simde__m128i vindex, simde__m128i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    src_ = simde__m128i_to_private(src),
    mask_ = simde__m128i_to_private(mask),
    r_ = simde__m128i_to_private(simde_mm_setzero_si128());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int32_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i32[i] = dst;
    }
    else {
      r_.i32[i] = src_.i32[i];
    }
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_mask_i64gather_epi32(src, base_addr, vindex, mask, scale) _mm_mask_i64gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i64gather_epi32
  #define _mm_mask_i64gather_epi32(src, base_addr, vindex, mask, scale) simde_mm_mask_i64gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_i64gather_epi32(const int32_t* base_addr, simde__m256i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m128i_private
    r_ = simde__m128i_to_private(simde_mm_setzero_si128());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int32_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i32[i] = dst;
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_i64gather_epi32(base_addr, vindex, scale) _mm256_i64gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i64gather_epi32
  #define _mm256_i64gather_epi32(base_addr, vindex, scale) simde_mm256_i64gather_epi32(SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm256_mask_i64gather_epi32(simde__m128i src, const int32_t* base_addr, simde__m256i vindex, simde__m128i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m128i_private
    src_ = simde__m128i_to_private(src),
    mask_ = simde__m128i_to_private(mask),
    r_ = simde__m128i_to_private(simde_mm_setzero_si128());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int32_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i32[i] = dst;
    }
    else {
      r_.i32[i] = src_.i32[i];
    }
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_mask_i64gather_epi32(src, base_addr, vindex, mask, scale) _mm256_mask_i64gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int const*, int32_t const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i64gather_epi32
  #define _mm256_mask_i64gather_epi32(src, base_addr, vindex, mask, scale) simde_mm256_mask_i64gather_epi32(src, SIMDE_CHECKED_REINTERPRET_CAST(int32_t const*, int const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_i32gather_epi64(const int64_t* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int64_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i64[i] = dst;
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm_i32gather_epi64(base_addr, vindex, scale) _mm_i32gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
  #else
    #define simde_mm_i32gather_epi64(base_addr, vindex, scale) _mm_i32gather_epi64(HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i32gather_epi64
  #define _mm_i32gather_epi64(base_addr, vindex, scale) simde_mm_i32gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mask_i32gather_epi64(simde__m128i src, const int64_t* base_addr, simde__m128i vindex, simde__m128i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    src_ = simde__m128i_to_private(src),
    mask_ = simde__m128i_to_private(mask),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int64_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i64[i] = dst;
    }
    else {
      r_.i64[i] = src_.i64[i];
    }
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm_mask_i32gather_epi64(src, base_addr, vindex, mask, scale) _mm_mask_i32gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
  #else
    #define simde_mm_mask_i32gather_epi64(src, base_addr, vindex, mask, scale) _mm_mask_i32gather_epi64(src, HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, mask, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i32gather_epi64
  #define _mm_mask_i32gather_epi64(src, base_addr, vindex, mask, scale) simde_mm_mask_i32gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_i32gather_epi64(const int64_t* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m256i_private
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int64_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i64[i] = dst;
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm256_i32gather_epi64(base_addr, vindex, scale) _mm256_i32gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
  #else
    #define simde_mm256_i32gather_epi64(base_addr, vindex, scale) _mm256_i32gather_epi64(HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i32gather_epi64
  #define _mm256_i32gather_epi64(base_addr, vindex, scale) simde_mm256_i32gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mask_i32gather_epi64(simde__m256i src, const int64_t* base_addr, simde__m128i vindex, simde__m256i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    src_ = simde__m256i_to_private(src),
    mask_ = simde__m256i_to_private(mask),
    r_;
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int64_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i64[i] = dst;
    }
    else {
      r_.i64[i] = src_.i64[i];
    }
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm256_mask_i32gather_epi64(src, base_addr, vindex, mask, scale) _mm256_mask_i32gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
  #else
    #define simde_mm256_mask_i32gather_epi64(src, base_addr, vindex, mask, scale) _mm256_mask_i32gather_epi64(src, HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, mask, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i32gather_epi64
  #define _mm256_mask_i32gather_epi64(src, base_addr, vindex, mask, scale) simde_mm256_mask_i32gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_i64gather_epi64(const int64_t* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    r_ = simde__m128i_to_private(simde_mm_setzero_si128());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int64_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i64[i] = dst;
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm_i64gather_epi64(base_addr, vindex, scale) _mm_i64gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
  #else
    #define simde_mm_i64gather_epi64(base_addr, vindex, scale) _mm_i64gather_epi64(HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i64gather_epi64
  #define _mm_i64gather_epi64(base_addr, vindex, scale) simde_mm_i64gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mask_i64gather_epi64(simde__m128i src, const int64_t* base_addr, simde__m128i vindex, simde__m128i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex),
    src_ = simde__m128i_to_private(src),
    mask_ = simde__m128i_to_private(mask),
    r_ = simde__m128i_to_private(simde_mm_setzero_si128());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int64_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i64[i] = dst;
    }
    else {
      r_.i64[i] = src_.i64[i];
    }
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm_mask_i64gather_epi64(src, base_addr, vindex, mask, scale) _mm_mask_i64gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
  #else
    #define simde_mm_mask_i64gather_epi64(src, base_addr, vindex, mask, scale) _mm_mask_i64gather_epi64(src, HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, mask, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i64gather_epi64
  #define _mm_mask_i64gather_epi64(src, base_addr, vindex, mask, scale) simde_mm_mask_i64gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_i64gather_epi64(const int64_t* base_addr, simde__m256i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex),
    r_ = simde__m256i_to_private(simde_mm256_setzero_si256());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    int64_t dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.i64[i] = dst;
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm256_i64gather_epi64(base_addr, vindex, scale) _mm256_i64gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
  #else
    #define simde_mm256_i64gather_epi64(base_addr, vindex, scale) _mm256_i64gather_epi64(HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i64gather_epi64
  #define _mm256_i64gather_epi64(base_addr, vindex, scale) simde_mm256_i64gather_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mask_i64gather_epi64(simde__m256i src, const int64_t* base_addr, simde__m256i vindex, simde__m256i mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex),
    src_ = simde__m256i_to_private(src),
    mask_ = simde__m256i_to_private(mask),
    r_ = simde__m256i_to_private(simde_mm256_setzero_si256());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      int64_t dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.i64[i] = dst;
    }
    else {
      r_.i64[i] = src_.i64[i];
    }
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #if SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
    #define simde_mm256_mask_i64gather_epi64(src, base_addr, vindex, mask, scale) _mm256_mask_i64gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
  #else
    #define simde_mm256_mask_i64gather_epi64(src, base_addr, vindex, mask, scale) _mm256_mask_i64gather_epi64(src, HEDLEY_REINTERPRET_CAST(long long const*, base_addr), vindex, mask, scale)
  #endif
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i64gather_epi64
  #define _mm256_mask_i64gather_epi64(src, base_addr, vindex, mask, scale) simde_mm256_mask_i64gather_epi64(src, HEDLEY_REINTERPRET_CAST(int64_t const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_i32gather_ps(const simde_float32* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128_private
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float32 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f32[i] = dst;
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_i32gather_ps(base_addr, vindex, scale) _mm_i32gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i32gather_ps
  #define _mm_i32gather_ps(base_addr, vindex, scale) simde_mm_i32gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_mask_i32gather_ps(simde__m128 src, const simde_float32* base_addr, simde__m128i vindex, simde__m128 mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128_private
    src_ = simde__m128_to_private(src),
    mask_ = simde__m128_to_private(mask),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float32 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f32[i] = dst;
    }
    else {
      r_.f32[i] = src_.f32[i];
    }
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_mask_i32gather_ps(src, base_addr, vindex, mask, scale) _mm_mask_i32gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i32gather_ps
  #define _mm_mask_i32gather_ps(src, base_addr, vindex, mask, scale) simde_mm_mask_i32gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_i32gather_ps(const simde_float32* base_addr, simde__m256i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m256_private
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float32 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f32[i] = dst;
  }

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_i32gather_ps(base_addr, vindex, scale) _mm256_i32gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, (base_addr)), (vindex), (scale))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i32gather_ps
  #define _mm256_i32gather_ps(base_addr, vindex, scale) simde_mm256_i32gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, (base_addr)), (vindex), (scale))
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_mask_i32gather_ps(simde__m256 src, const simde_float32* base_addr, simde__m256i vindex, simde__m256 mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m256_private
    src_ = simde__m256_to_private(src),
    mask_ = simde__m256_to_private(mask),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float32 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f32[i] = dst;
    }
    else {
      r_.f32[i] = src_.f32[i];
    }
  }

  return simde__m256_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_mask_i32gather_ps(src, base_addr, vindex, mask, scale) _mm256_mask_i32gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i32gather_ps
  #define _mm256_mask_i32gather_ps(src, base_addr, vindex, mask, scale) simde_mm256_mask_i32gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_i64gather_ps(const simde_float32* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128_private
    r_ = simde__m128_to_private(simde_mm_setzero_ps());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float32 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f32[i] = dst;
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_i64gather_ps(base_addr, vindex, scale) _mm_i64gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i64gather_ps
  #define _mm_i64gather_ps(base_addr, vindex, scale) simde_mm_i64gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_mask_i64gather_ps(simde__m128 src, const simde_float32* base_addr, simde__m128i vindex, simde__m128 mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128_private
    src_ = simde__m128_to_private(src),
    mask_ = simde__m128_to_private(mask),
    r_ = simde__m128_to_private(simde_mm_setzero_ps());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
     simde_float32 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f32[i] = dst;
    }
    else {
      r_.f32[i] = src_.f32[i];
    }
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_mask_i64gather_ps(src, base_addr, vindex, mask, scale) _mm_mask_i64gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(float const*, float32_t const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i64gather_ps
  #define _mm_mask_i64gather_ps(src, base_addr, vindex, mask, scale) simde_mm_mask_i64gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm256_i64gather_ps(const simde_float32* base_addr, simde__m256i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m128_private
    r_ = simde__m128_to_private(simde_mm_setzero_ps());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float32 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f32[i] = dst;
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_i64gather_ps(base_addr, vindex, scale) _mm256_i64gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i64gather_ps
  #define _mm256_i64gather_ps(base_addr, vindex, scale) simde_mm256_i64gather_ps(SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm256_mask_i64gather_ps(simde__m128 src, const simde_float32* base_addr, simde__m256i vindex, simde__m128 mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m128_private
    src_ = simde__m128_to_private(src),
    mask_ = simde__m128_to_private(mask),
    r_ = simde__m128_to_private(simde_mm_setzero_ps());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i32[i] >> 31) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float32 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f32[i] = dst;
    }
    else {
      r_.f32[i] = src_.f32[i];
    }
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_mask_i64gather_ps(src, base_addr, vindex, mask, scale) _mm256_mask_i64gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(float const*, simde_float32 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i64gather_ps
  #define _mm256_mask_i64gather_ps(src, base_addr, vindex, mask, scale) simde_mm256_mask_i64gather_ps(src, SIMDE_CHECKED_REINTERPRET_CAST(simde_float32 const*, float const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_i32gather_pd(const simde_float64* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128d_private
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float64 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f64[i] = dst;
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_i32gather_pd(base_addr, vindex, scale) _mm_i32gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i32gather_pd
  #define _mm_i32gather_pd(base_addr, vindex, scale) simde_mm_i32gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_mask_i32gather_pd(simde__m128d src, const simde_float64* base_addr, simde__m128i vindex, simde__m128d mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128d_private
    src_ = simde__m128d_to_private(src),
    mask_ = simde__m128d_to_private(mask),
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float64 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f64[i] = dst;
    }
    else {
      r_.f64[i] = src_.f64[i];
    }
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_mask_i32gather_pd(src, base_addr, vindex, mask, scale) _mm_mask_i32gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i32gather_pd
  #define _mm_mask_i32gather_pd(src, base_addr, vindex, mask, scale) simde_mm_mask_i32gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_i32gather_pd(const simde_float64* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m256d_private
    r_;
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float64 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f64[i] = dst;
  }

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_i32gather_pd(base_addr, vindex, scale) _mm256_i32gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i32gather_pd
  #define _mm256_i32gather_pd(base_addr, vindex, scale) simde_mm256_i32gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_mask_i32gather_pd(simde__m256d src, const simde_float64* base_addr, simde__m128i vindex, simde__m256d mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256d_private
    src_ = simde__m256d_to_private(src),
    mask_ = simde__m256d_to_private(mask),
    r_;
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i32) / sizeof(vindex_.i32[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i32[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float64 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f64[i] = dst;
    }
    else {
      r_.f64[i] = src_.f64[i];
    }
  }

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_mask_i32gather_pd(src, base_addr, vindex, mask, scale) _mm256_mask_i32gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i32gather_pd
  #define _mm256_mask_i32gather_pd(src, base_addr, vindex, mask, scale) simde_mm256_mask_i32gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_i64gather_pd(const simde_float64* base_addr, simde__m128i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128d_private
    r_ = simde__m128d_to_private(simde_mm_setzero_pd());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float64 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f64[i] = dst;
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_i64gather_pd(base_addr, vindex, scale) _mm_i64gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_i64gather_pd
  #define _mm_i64gather_pd(base_addr, vindex, scale) simde_mm_i64gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_mask_i64gather_pd(simde__m128d src, const simde_float64* base_addr, simde__m128i vindex, simde__m128d mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m128i_private
    vindex_ = simde__m128i_to_private(vindex);
  simde__m128d_private
    src_ = simde__m128d_to_private(src),
    mask_ = simde__m128d_to_private(mask),
    r_ = simde__m128d_to_private(simde_mm_setzero_pd());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float64 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f64[i] = dst;
    }
    else {
      r_.f64[i] = src_.f64[i];
    }
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_mask_i64gather_pd(src, base_addr, vindex, mask, scale) _mm_mask_i64gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_mask_i64gather_pd
  #define _mm_mask_i64gather_pd(src, base_addr, vindex, mask, scale) simde_mm_mask_i64gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_i64gather_pd(const simde_float64* base_addr, simde__m256i vindex, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m256d_private
    r_ = simde__m256d_to_private(simde_mm256_setzero_pd());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    const uint8_t* src = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
    simde_float64 dst;
    simde_memcpy(&dst, src, sizeof(dst));
    r_.f64[i] = dst;
  }

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_i64gather_pd(base_addr, vindex, scale) _mm256_i64gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_i64gather_pd
  #define _mm256_i64gather_pd(base_addr, vindex, scale) simde_mm256_i64gather_pd(HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_mask_i64gather_pd(simde__m256d src, const simde_float64* base_addr, simde__m256i vindex, simde__m256d mask, const int32_t scale)
    SIMDE_REQUIRE_CONSTANT(scale)
    HEDLEY_REQUIRE_MSG((scale && scale <= 8 && !(scale & (scale - 1))), "`scale' must be a power of two less than or equal to 8") {
  simde__m256i_private
    vindex_ = simde__m256i_to_private(vindex);
  simde__m256d_private
    src_ = simde__m256d_to_private(src),
    mask_ = simde__m256d_to_private(mask),
    r_ = simde__m256d_to_private(simde_mm256_setzero_pd());
  const uint8_t* addr = HEDLEY_REINTERPRET_CAST(const uint8_t*, base_addr);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(vindex_.i64) / sizeof(vindex_.i64[0])) ; i++) {
    if ((mask_.i64[i] >> 63) & 1) {
      const uint8_t* src1 = addr + (HEDLEY_STATIC_CAST(size_t , vindex_.i64[i]) * HEDLEY_STATIC_CAST(size_t , scale));
      simde_float64 dst;
      simde_memcpy(&dst, src1, sizeof(dst));
      r_.f64[i] = dst;
    }
    else {
      r_.f64[i] = src_.f64[i];
    }
  }

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_mask_i64gather_pd(src, base_addr, vindex, mask, scale) _mm256_mask_i64gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mask_i64gather_pd
  #define _mm256_mask_i64gather_pd(src, base_addr, vindex, mask, scale) simde_mm256_mask_i64gather_pd(src, HEDLEY_REINTERPRET_CAST(simde_float64 const*, base_addr), vindex, mask, scale)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_inserti128_si256(simde__m256i a, simde__m128i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  simde__m128i_private b_ = simde__m128i_to_private(b);

  a_.m128i_private[ imm8 & 1 ] = b_;

  return simde__m256i_from_private(a_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_inserti128_si256(a, b, imm8) _mm256_inserti128_si256(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_inserti128_si256
  #define _mm256_inserti128_si256(a, b, imm8) simde_mm256_inserti128_si256(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_madd_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_madd_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_madd_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_madd_epi16(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS) && defined(SIMDE_CONVERT_VECTOR_) && HEDLEY_HAS_BUILTIN(__builtin_shufflevector)
      SIMDE_ALIGN_TO_32 int32_t product SIMDE_VECTOR(64);
      SIMDE_ALIGN_TO_32 int32_t a32x16 SIMDE_VECTOR(64);
      SIMDE_ALIGN_TO_32 int32_t b32x16 SIMDE_VECTOR(64);
      SIMDE_ALIGN_TO_32 int32_t even SIMDE_VECTOR(32);
      SIMDE_ALIGN_TO_32 int32_t odd SIMDE_VECTOR(32);

      SIMDE_CONVERT_VECTOR_(a32x16, a_.i16);
      SIMDE_CONVERT_VECTOR_(b32x16, b_.i16);
      product = a32x16 * b32x16;

      even = __builtin_shufflevector(product, product, 0, 2, 4, 6, 8, 10, 12, 14);
      odd  = __builtin_shufflevector(product, product, 1, 3, 5, 7, 9, 11, 13, 15);

      r_.i32 = even + odd;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_) / sizeof(r_.i16[0])) ; i += 2) {
        r_.i32[i / 2] = (a_.i16[i] * b_.i16[i]) + (a_.i16[i + 1] * b_.i16[i + 1]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_madd_epi16
  #define _mm256_madd_epi16(a, b) simde_mm256_madd_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_maddubs_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_maddubs_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_maddubs_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_maddubs_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        const int idx = HEDLEY_STATIC_CAST(int, i) << 1;
        int32_t ts =
          (HEDLEY_STATIC_CAST(int16_t, a_.u8[  idx  ]) * HEDLEY_STATIC_CAST(int16_t, b_.i8[  idx  ])) +
          (HEDLEY_STATIC_CAST(int16_t, a_.u8[idx + 1]) * HEDLEY_STATIC_CAST(int16_t, b_.i8[idx + 1]));
        r_.i16[i] = (ts > INT16_MIN) ? ((ts < INT16_MAX) ? HEDLEY_STATIC_CAST(int16_t, ts) : INT16_MAX) : INT16_MIN;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maddubs_epi16
  #define _mm256_maddubs_epi16(a, b) simde_mm256_maddubs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_maskload_epi32 (const int32_t mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m128i mask) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_maskload_epi32(mem_addr, mask);
  #else
    simde__m128i_private
      r_,
      mask_ = simde__m128i_to_private(mask),
      mask_shr_;

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      mask_shr_.neon_i32 = vshrq_n_s32(mask_.neon_i32, 31);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        mask_shr_.i32[i] = mask_.i32[i] >> 31;
      }
    #endif

      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = mask_shr_.i32[i] ? mem_addr[i] : INT32_C(0);
      }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskload_epi32
  #define _mm_maskload_epi32(mem_addr, mask) simde_mm_maskload_epi32(HEDLEY_REINTERPRET_CAST(int32_t const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_maskload_epi32 (const int32_t mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m256i mask) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_maskload_epi32(mem_addr, mask);
  #else
    simde__m256i_private
      mask_ = simde__m256i_to_private(mask),
      r_;

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = (mask_.i32[i] >> 31) ? mem_addr[i] : INT32_C(0);
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskload_epi32
  #define _mm256_maskload_epi32(mem_addr, mask) simde_mm256_maskload_epi32(HEDLEY_REINTERPRET_CAST(int32_t const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_maskload_epi64 (const int64_t mem_addr[HEDLEY_ARRAY_PARAM(2)], simde__m128i mask) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_maskload_epi64(HEDLEY_REINTERPRET_CAST(const long long *, mem_addr), mask);
  #else
    simde__m128i_private
      r_,
      mask_ = simde__m128i_to_private(mask),
      mask_shr_;

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      mask_shr_.neon_i64 = vshrq_n_s64(mask_.neon_i64, 63);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(mask_.i64) / sizeof(mask_.i64[0])) ; i++) {
        mask_shr_.i64[i] = mask_.i64[i] >> 63;
      }
    #endif

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = mask_shr_.i64[i] ? mem_addr[i] : INT64_C(0);
    }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskload_epi64
  #define _mm_maskload_epi64(mem_addr, mask) simde_mm_maskload_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_maskload_epi64 (const int64_t mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m256i mask) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_maskload_epi64(HEDLEY_REINTERPRET_CAST(const long long *, mem_addr), mask);
  #else
    simde__m256i_private
      mask_ = simde__m256i_to_private(mask),
      r_;

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      r_.i64[i] = (mask_.i64[i] >> 63) ? mem_addr[i] : INT64_C(0);
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskload_epi64
  #define _mm256_maskload_epi64(mem_addr, mask) simde_mm256_maskload_epi64(HEDLEY_REINTERPRET_CAST(int64_t const*, mem_addr), mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm_maskstore_epi32 (int32_t mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m128i mask, simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    _mm_maskstore_epi32(mem_addr, mask, a);
  #else
    simde__m128i_private mask_ = simde__m128i_to_private(mask);
    simde__m128i_private a_ = simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.i32) / sizeof(a_.i32[0])) ; i++) {
      if (mask_.u32[i] & (UINT32_C(1) << 31))
        mem_addr[i] = a_.i32[i];
    }
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskstore_epi32
  #define _mm_maskstore_epi32(mem_addr, mask, a) simde_mm_maskstore_epi32(HEDLEY_REINTERPRET_CAST(int32_t *, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_maskstore_epi32 (int32_t mem_addr[HEDLEY_ARRAY_PARAM(8)], simde__m256i mask, simde__m256i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    _mm256_maskstore_epi32(mem_addr, mask, a);
  #else
    simde__m256i_private mask_ = simde__m256i_to_private(mask);
    simde__m256i_private a_ = simde__m256i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.i32) / sizeof(a_.i32[0])) ; i++) {
      if (mask_.u32[i] & (UINT32_C(1) << 31))
        mem_addr[i] = a_.i32[i];
    }
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskstore_epi32
  #define _mm256_maskstore_epi32(mem_addr, mask, a) simde_mm256_maskstore_epi32(HEDLEY_REINTERPRET_CAST(int32_t *, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm_maskstore_epi64 (int64_t mem_addr[HEDLEY_ARRAY_PARAM(2)], simde__m128i mask, simde__m128i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    _mm_maskstore_epi64(HEDLEY_REINTERPRET_CAST(long long *, mem_addr), mask, a);
  #else
    simde__m128i_private mask_ = simde__m128i_to_private(mask);
    simde__m128i_private a_ = simde__m128i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.i64) / sizeof(a_.i64[0])) ; i++) {
      if (mask_.u64[i] >> 63)
        mem_addr[i] = a_.i64[i];
    }
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_maskstore_epi64
  #define _mm_maskstore_epi64(mem_addr, mask, a) simde_mm_maskstore_epi64(HEDLEY_REINTERPRET_CAST(int64_t *, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
void
simde_mm256_maskstore_epi64 (int64_t mem_addr[HEDLEY_ARRAY_PARAM(4)], simde__m256i mask, simde__m256i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    _mm256_maskstore_epi64(HEDLEY_REINTERPRET_CAST(long long *, mem_addr), mask, a);
  #else
    simde__m256i_private mask_ = simde__m256i_to_private(mask);
    simde__m256i_private a_ = simde__m256i_to_private(a);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(a_.i64) / sizeof(a_.i64[0])) ; i++) {
      if (mask_.u64[i] & (UINT64_C(1) << 63))
        mem_addr[i] = a_.i64[i];
    }
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_maskstore_epi64
  #define _mm256_maskstore_epi64(mem_addr, mask, a) simde_mm256_maskstore_epi64(HEDLEY_REINTERPRET_CAST(int64_t *, mem_addr), mask, a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_max_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE) && !defined(__PGI)
    return _mm256_max_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_max_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_max_epi8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[i] > b_.i8[i] ? a_.i8[i] : b_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_epi8
  #define _mm256_max_epi8(a, b) simde_mm256_max_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_max_epu8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_max_epu8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_max_epu8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_max_epu8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
        r_.u8[i] = (a_.u8[i] > b_.u8[i]) ? a_.u8[i] : b_.u8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_epu8
  #define _mm256_max_epu8(a, b) simde_mm256_max_epu8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_max_epu16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_max_epu16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_max_epu16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_max_epu16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
        r_.u16[i] = (a_.u16[i] > b_.u16[i]) ? a_.u16[i] : b_.u16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_epu16
  #define _mm256_max_epu16(a, b) simde_mm256_max_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_max_epu32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_max_epu32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_max_epu32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_max_epu32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = (a_.u32[i] > b_.u32[i]) ? a_.u32[i] : b_.u32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_epu32
  #define _mm256_max_epu32(a, b) simde_mm256_max_epu32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_max_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_max_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_max_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_max_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (a_.i16[i] > b_.i16[i]) ? a_.i16[i] : b_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_epi16
  #define _mm256_max_epi16(a, b) simde_mm256_max_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_max_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_max_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_max_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_max_epi32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] > b_.i32[i] ? a_.i32[i] : b_.i32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_max_epi32
  #define _mm256_max_epi32(a, b) simde_mm256_max_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_min_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE) && !defined(__PGI)
    return _mm256_min_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_min_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_min_epi8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[i] < b_.i8[i] ? a_.i8[i] : b_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_epi8
  #define _mm256_min_epi8(a, b) simde_mm256_min_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_min_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_min_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_min_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_min_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (a_.i16[i] < b_.i16[i]) ? a_.i16[i] : b_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_epi16
  #define _mm256_min_epi16(a, b) simde_mm256_min_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_min_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_min_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_min_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_min_epi32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] < b_.i32[i] ? a_.i32[i] : b_.i32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_epi32
  #define _mm256_min_epi32(a, b) simde_mm256_min_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_min_epu8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_min_epu8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_min_epu8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_min_epu8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
        r_.u8[i] = (a_.u8[i] < b_.u8[i]) ? a_.u8[i] : b_.u8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_epu8
  #define _mm256_min_epu8(a, b) simde_mm256_min_epu8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_min_epu16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_min_epu16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_min_epu16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_min_epu16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
        r_.u16[i] = (a_.u16[i] < b_.u16[i]) ? a_.u16[i] : b_.u16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_epu16
  #define _mm256_min_epu16(a, b) simde_mm256_min_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_min_epu32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_min_epu32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_min_epu32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_min_epu32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = (a_.u32[i] < b_.u32[i]) ? a_.u32[i] : b_.u32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_min_epu32
  #define _mm256_min_epu32(a, b) simde_mm256_min_epu32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int32_t
simde_mm256_movemask_epi8 (simde__m256i a) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_movemask_epi8(a);
  #else
    simde__m256i_private a_ = simde__m256i_to_private(a);
    uint32_t r = 0;

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      for (size_t i = 0 ; i < (sizeof(a_.m128i) / sizeof(a_.m128i[0])) ; i++) {
        r |= HEDLEY_STATIC_CAST(uint32_t,simde_mm_movemask_epi8(a_.m128i[i])) << (16 * i);
      }
    #else
      r = 0;
      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.u8) / sizeof(a_.u8[0])) ; i++) {
        r |= HEDLEY_STATIC_CAST(uint32_t, (a_.u8[31 - i] >> 7)) << (31 - i);
      }
    #endif

    return HEDLEY_STATIC_CAST(int32_t, r);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_movemask_epi8
  #define _mm256_movemask_epi8(a) simde_mm256_movemask_epi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mpsadbw_epu8 (simde__m256i a, simde__m256i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  const int a_offset1 = imm8 & 4;
  const int b_offset1 = (imm8 & 3) << 2;
  const int a_offset2 = (imm8 >> 3) & 4;
  const int b_offset2 = ((imm8 >> 3) & 3) << 2;

  #if defined(simde_math_abs)
    const int halfway_point = HEDLEY_STATIC_CAST(int, (sizeof(r_.u16) / sizeof(r_.u16[0])) ) / 2;
    for (int i = 0 ; i < halfway_point ; i++) {
      r_.u16[i] =
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset1 + i + 0] - b_.u8[b_offset1 + 0]))) +
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset1 + i + 1] - b_.u8[b_offset1 + 1]))) +
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset1 + i + 2] - b_.u8[b_offset1 + 2]))) +
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset1 + i + 3] - b_.u8[b_offset1 + 3])));
      r_.u16[halfway_point + i] =
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[2 * halfway_point + a_offset2 + i + 0] - b_.u8[2 * halfway_point + b_offset2 + 0]))) +
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[2 * halfway_point + a_offset2 + i + 1] - b_.u8[2 * halfway_point + b_offset2 + 1]))) +
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[2 * halfway_point + a_offset2 + i + 2] - b_.u8[2 * halfway_point + b_offset2 + 2]))) +
        HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[2 * halfway_point + a_offset2 + i + 3] - b_.u8[2 * halfway_point + b_offset2 + 3])));
    }
  #else
    HEDLEY_UNREACHABLE();
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE) && SIMDE_DETECT_CLANG_VERSION_CHECK(3,9,0)
  #define simde_mm256_mpsadbw_epu8(a, b, imm8) _mm256_mpsadbw_epu8(a, b, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
  #define simde_mm256_mpsadbw_epu8(a, b, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_mpsadbw_epu8(simde_mm256_extracti128_si256(a, 1), simde_mm256_extracti128_si256(b, 1), (imm8 >> 3)), \
       simde_mm_mpsadbw_epu8(simde_mm256_extracti128_si256(a, 0), simde_mm256_extracti128_si256(b, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mpsadbw_epu8
  #define _mm256_mpsadbw_epu8(a, b, imm8) simde_mm256_mpsadbw_epu8(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mul_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mul_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_mul_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_mul_epi32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] =
          HEDLEY_STATIC_CAST(int64_t, a_.i32[i * 2]) *
          HEDLEY_STATIC_CAST(int64_t, b_.i32[i * 2]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
#  define _mm256_mul_epi32(a, b) simde_mm256_mul_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mul_epu32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mul_epu32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_mul_epu32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_mul_epu32(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
        r_.u64[i] = HEDLEY_STATIC_CAST(uint64_t, a_.u32[i * 2]) * HEDLEY_STATIC_CAST(uint64_t, b_.u32[i * 2]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
#  define _mm256_mul_epu32(a, b) simde_mm256_mul_epu32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mulhi_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mulhi_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.u16[i] = HEDLEY_STATIC_CAST(uint16_t, (HEDLEY_STATIC_CAST(uint32_t, HEDLEY_STATIC_CAST(int32_t, a_.i16[i]) * HEDLEY_STATIC_CAST(int32_t, b_.i16[i])) >> 16));
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
#  define _mm256_mulhi_epi16(a, b) simde_mm256_mulhi_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mulhi_epu16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mulhi_epu16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
      r_.u16[i] = HEDLEY_STATIC_CAST(uint16_t, HEDLEY_STATIC_CAST(uint32_t, a_.u16[i]) * HEDLEY_STATIC_CAST(uint32_t, b_.u16[i]) >> 16);
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
#  define _mm256_mulhi_epu16(a, b) simde_mm256_mulhi_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mulhrs_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mulhrs_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, (((HEDLEY_STATIC_CAST(int32_t, a_.i16[i]) * HEDLEY_STATIC_CAST(int32_t, b_.i16[i])) + 0x4000) >> 15));
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
#  define _mm256_mulhrs_epi16(a, b) simde_mm256_mulhrs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mullo_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mullo_epi16(a, b);
  #else
    simde__m256i_private
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b),
    r_;

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, a_.i16[i] * b_.i16[i]);
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mullo_epi16
  #define _mm256_mullo_epi16(a, b) simde_mm256_mullo_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_mullo_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_mullo_epi32(a, b);
  #else
    simde__m256i_private
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b),
    r_;

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = HEDLEY_STATIC_CAST(int32_t, a_.i32[i] * b_.i32[i]);
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_mullo_epi32
  #define _mm256_mullo_epi32(a, b) simde_mm256_mullo_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_mullo_epu32 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

    #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.u32 = a_.u32 * b_.u32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = a_.u32[i] * b_.u32[i];
      }
    #endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_or_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_or_si256(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_or_si128(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_or_si128(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f | b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32f) / sizeof(r_.i32f[0])) ; i++) {
        r_.i32f[i] = a_.i32f[i] | b_.i32f[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_or_si256
  #define _mm256_or_si256(a, b) simde_mm256_or_si256(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_packs_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_packs_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_packs_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_packs_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      const size_t halfway_point = (sizeof(r_.i8) / sizeof(r_.i8[0]))/2;
      const size_t quarter_point = (sizeof(r_.i8) / sizeof(r_.i8[0]))/4;
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < quarter_point ; i++) {
        r_.i8[i]     = (a_.i16[i] > INT8_MAX) ? INT8_MAX : ((a_.i16[i] < INT8_MIN) ? INT8_MIN : HEDLEY_STATIC_CAST(int8_t, a_.i16[i]));
        r_.i8[i + quarter_point] = (b_.i16[i] > INT8_MAX) ? INT8_MAX : ((b_.i16[i] < INT8_MIN) ? INT8_MIN : HEDLEY_STATIC_CAST(int8_t, b_.i16[i]));
        r_.i8[halfway_point + i]     = (a_.i16[quarter_point + i] > INT8_MAX) ? INT8_MAX : ((a_.i16[quarter_point + i] < INT8_MIN) ? INT8_MIN : HEDLEY_STATIC_CAST(int8_t, a_.i16[quarter_point + i]));
        r_.i8[halfway_point + i + quarter_point] = (b_.i16[quarter_point + i] > INT8_MAX) ? INT8_MAX : ((b_.i16[quarter_point + i] < INT8_MIN) ? INT8_MIN : HEDLEY_STATIC_CAST(int8_t, b_.i16[quarter_point + i]));
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_packs_epi16
  #define _mm256_packs_epi16(a, b) simde_mm256_packs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_packs_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_packs_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      v_[] = {
        simde__m256i_to_private(a),
        simde__m256i_to_private(b)
      };

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_packs_epi32(v_[0].m128i[0], v_[1].m128i[0]);
      r_.m128i[1] = simde_mm_packs_epi32(v_[0].m128i[1], v_[1].m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        const int32_t v = v_[(i >> 2) & 1].i32[(i & 11) - ((i & 8) >> 1)];
        r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, (v > INT16_MAX) ? INT16_MAX : ((v < INT16_MIN) ? INT16_MIN : v));
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_packs_epi32
  #define _mm256_packs_epi32(a, b) simde_mm256_packs_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_packus_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_packus_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_packus_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_packus_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      const size_t halfway_point = (sizeof(r_.i8) / sizeof(r_.i8[0])) / 2;
      const size_t quarter_point = (sizeof(r_.i8) / sizeof(r_.i8[0])) / 4;
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < quarter_point ; i++) {
        r_.u8[i] = (a_.i16[i] > UINT8_MAX) ? UINT8_MAX : ((a_.i16[i] < 0) ? UINT8_C(0) : HEDLEY_STATIC_CAST(uint8_t, a_.i16[i]));
        r_.u8[i + quarter_point] = (b_.i16[i] > UINT8_MAX) ? UINT8_MAX : ((b_.i16[i] < 0) ? UINT8_C(0) : HEDLEY_STATIC_CAST(uint8_t, b_.i16[i]));
        r_.u8[halfway_point + i] = (a_.i16[quarter_point + i] > UINT8_MAX) ? UINT8_MAX : ((a_.i16[quarter_point + i] < 0) ? UINT8_C(0) : HEDLEY_STATIC_CAST(uint8_t, a_.i16[quarter_point + i]));
        r_.u8[halfway_point + i + quarter_point] = (b_.i16[quarter_point + i] > UINT8_MAX) ? UINT8_MAX : ((b_.i16[quarter_point + i] < 0) ? UINT8_C(0) : HEDLEY_STATIC_CAST(uint8_t, b_.i16[quarter_point + i]));
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_packus_epi16
  #define _mm256_packus_epi16(a, b) simde_mm256_packus_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_packus_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_packus_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_packus_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_packus_epi32(a_.m128i[1], b_.m128i[1]);
    #else
      const size_t halfway_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 2;
      const size_t quarter_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 4;
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < quarter_point ; i++) {
        r_.u16[i] = (a_.i32[i] > UINT16_MAX) ? UINT16_MAX : ((a_.i32[i] < 0) ? UINT16_C(0) : HEDLEY_STATIC_CAST(uint16_t, a_.i32[i]));
        r_.u16[i + quarter_point] = (b_.i32[i] > UINT16_MAX) ? UINT16_MAX : ((b_.i32[i] < 0) ? UINT16_C(0) : HEDLEY_STATIC_CAST(uint16_t, b_.i32[i]));
        r_.u16[halfway_point + i]     = (a_.i32[quarter_point + i] > UINT16_MAX) ? UINT16_MAX : ((a_.i32[quarter_point + i] < 0) ? UINT16_C(0) : HEDLEY_STATIC_CAST(uint16_t, a_.i32[quarter_point + i]));
        r_.u16[halfway_point + i + quarter_point] = (b_.i32[quarter_point + i] > UINT16_MAX) ? UINT16_MAX : ((b_.i32[quarter_point + i] < 0) ? UINT16_C(0) : HEDLEY_STATIC_CAST(uint16_t, b_.i32[quarter_point + i]));
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_packus_epi32
  #define _mm256_packus_epi32(a, b) simde_mm256_packus_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_permute2x128_si256 (simde__m256i a, simde__m256i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  r_.m128i_private[0] = (imm8 & 0x08) ? simde__m128i_to_private(simde_mm_setzero_si128()) : ((imm8 & 0x02) ? b_.m128i_private[(imm8     ) & 1] : a_.m128i_private[(imm8     ) & 1]);
  r_.m128i_private[1] = (imm8 & 0x80) ? simde__m128i_to_private(simde_mm_setzero_si128()) : ((imm8 & 0x20) ? b_.m128i_private[(imm8 >> 4) & 1] : a_.m128i_private[(imm8 >> 4) & 1]);

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_permute2x128_si256(a, b, imm8) _mm256_permute2x128_si256(a, b, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute2x128_si256
  #define _mm256_permute2x128_si256(a, b, imm8) simde_mm256_permute2x128_si256(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_permute4x64_epi64 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  r_.i64[0] = (imm8 & 0x02) ? a_.i64[((imm8       ) & 1)+2] : a_.i64[(imm8       ) & 1];
  r_.i64[1] = (imm8 & 0x08) ? a_.i64[((imm8 >> 2  ) & 1)+2] : a_.i64[(imm8 >> 2  ) & 1];
  r_.i64[2] = (imm8 & 0x20) ? a_.i64[((imm8 >> 4  ) & 1)+2] : a_.i64[(imm8 >> 4  ) & 1];
  r_.i64[3] = (imm8 & 0x80) ? a_.i64[((imm8 >> 6  ) & 1)+2] : a_.i64[(imm8 >> 6  ) & 1];

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_permute4x64_epi64(a, imm8) _mm256_permute4x64_epi64(a, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute4x64_epi64
  #define _mm256_permute4x64_epi64(a, imm8) simde_mm256_permute4x64_epi64(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_permute4x64_pd (simde__m256d a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256d_private
    r_,
    a_ = simde__m256d_to_private(a);

  r_.f64[0] = (imm8 & 0x02) ? a_.f64[((imm8       ) & 1)+2] : a_.f64[(imm8       ) & 1];
  r_.f64[1] = (imm8 & 0x08) ? a_.f64[((imm8 >> 2  ) & 1)+2] : a_.f64[(imm8 >> 2  ) & 1];
  r_.f64[2] = (imm8 & 0x20) ? a_.f64[((imm8 >> 4  ) & 1)+2] : a_.f64[(imm8 >> 4  ) & 1];
  r_.f64[3] = (imm8 & 0x80) ? a_.f64[((imm8 >> 6  ) & 1)+2] : a_.f64[(imm8 >> 6  ) & 1];

  return simde__m256d_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_permute4x64_pd(a, imm8) _mm256_permute4x64_pd(a, imm8)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permute4x64_pd
  #define _mm256_permute4x64_pd(a, imm8) simde_mm256_permute4x64_pd(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_permutevar8x32_epi32 (simde__m256i a, simde__m256i idx) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_permutevar8x32_epi32(a, idx);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      idx_ = simde__m256i_to_private(idx);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = a_.i32[idx_.i32[i] & 7];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permutevar8x32_epi32
  #define _mm256_permutevar8x32_epi32(a, idx) simde_mm256_permutevar8x32_epi32(a, idx)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_permutevar8x32_ps (simde__m256 a, simde__m256i idx) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    #if defined(__clang__) && !SIMDE_DETECT_CLANG_VERSION_CHECK(3,8,0)
      return _mm256_permutevar8x32_ps(a, HEDLEY_REINTERPRET_CAST(simde__m256, idx));
    #else
      return _mm256_permutevar8x32_ps(a, idx);
    #endif
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a);
    simde__m256i_private
      idx_ = simde__m256i_to_private(idx);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = a_.f32[idx_.i32[i] & 7];
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_permutevar8x32_ps
  #define _mm256_permutevar8x32_ps(a, idx) simde_mm256_permutevar8x32_ps(a, idx)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sad_epu8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sad_epu8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sad_epu8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_sad_epu8(a_.m128i[1], b_.m128i[1]);
    #else
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        uint16_t tmp = 0;
        SIMDE_VECTORIZE_REDUCTION(+:tmp)
        for (size_t j = 0 ; j < ((sizeof(r_.u8) / sizeof(r_.u8[0])) / 4) ; j++) {
          const size_t e = j + (i * 8);
          tmp += (a_.u8[e] > b_.u8[e]) ? (a_.u8[e] - b_.u8[e]) : (b_.u8[e] - a_.u8[e]);
        }
        r_.i64[i] = tmp;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sad_epu8
  #define _mm256_sad_epu8(a, b) simde_mm256_sad_epu8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_shuffle_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_shuffle_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_shuffle_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_shuffle_epi8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < ((sizeof(r_.u8) / sizeof(r_.u8[0])) / 2) ; i++) {
        r_.u8[  i   ] = (b_.u8[  i   ] & 0x80) ? 0 : a_.u8[(b_.u8[  i   ] & 0x0f)     ];
        r_.u8[i + 16] = (b_.u8[i + 16] & 0x80) ? 0 : a_.u8[(b_.u8[i + 16] & 0x0f) + 16];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_shuffle_epi8
  #define _mm256_shuffle_epi8(a, b) simde_mm256_shuffle_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_shuffle_epi32 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  for (size_t i = 0 ; i < ((sizeof(r_.i32) / sizeof(r_.i32[0])) / 2) ; i++) {
    r_.i32[i] = a_.i32[(imm8 >> (i * 2)) & 3];
  }
  for (size_t i = 0 ; i < ((sizeof(r_.i32) / sizeof(r_.i32[0])) / 2) ; i++) {
    r_.i32[i + 4] = a_.i32[((imm8 >> (i * 2)) & 3) + 4];
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_shuffle_epi32(a, imm8) _mm256_shuffle_epi32(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128) && !defined(__PGI)
#  define simde_mm256_shuffle_epi32(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_shuffle_epi32(simde_mm256_extracti128_si256(a, 1), (imm8)), \
       simde_mm_shuffle_epi32(simde_mm256_extracti128_si256(a, 0), (imm8)))
#elif defined(SIMDE_SHUFFLE_VECTOR_)
#  define simde_mm256_shuffle_epi32(a, imm8) (__extension__ ({ \
      const simde__m256i_private simde_tmp_a_ = simde__m256i_to_private(a); \
      simde__m256i_from_private((simde__m256i_private) { .i32 = \
          SIMDE_SHUFFLE_VECTOR_(32, 32, \
                                (simde_tmp_a_).i32, \
                                (simde_tmp_a_).i32, \
                                ((imm8)     ) & 3, \
                                ((imm8) >> 2) & 3, \
                                ((imm8) >> 4) & 3, \
                                ((imm8) >> 6) & 3, \
                                (((imm8)     ) & 3) + 4, \
                                (((imm8) >> 2) & 3) + 4, \
                                (((imm8) >> 4) & 3) + 4, \
                                (((imm8) >> 6) & 3) + 4) }); }))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_shuffle_epi32
  #define _mm256_shuffle_epi32(a, imm8) simde_mm256_shuffle_epi32(a, imm8)
#endif

#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_shufflehi_epi16(a, imm8) _mm256_shufflehi_epi16(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_shufflehi_epi16(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_shufflehi_epi16(simde_mm256_extracti128_si256(a, 1), (imm8)), \
       simde_mm_shufflehi_epi16(simde_mm256_extracti128_si256(a, 0), (imm8)))
#elif defined(SIMDE_SHUFFLE_VECTOR_)
#  define simde_mm256_shufflehi_epi16(a, imm8) (__extension__ ({ \
      const simde__m256i_private simde_tmp_a_ = simde__m256i_to_private(a); \
      simde__m256i_from_private((simde__m256i_private) { .i16 = \
        SIMDE_SHUFFLE_VECTOR_(16, 32, \
          (simde_tmp_a_).i16, \
          (simde_tmp_a_).i16, \
          0, 1, 2, 3, \
          (((imm8)     ) & 3) + 4, \
          (((imm8) >> 2) & 3) + 4, \
          (((imm8) >> 4) & 3) + 4, \
          (((imm8) >> 6) & 3) + 4, \
          8, 9, 10, 11, \
          ((((imm8)     ) & 3) + 8 + 4), \
          ((((imm8) >> 2) & 3) + 8 + 4), \
          ((((imm8) >> 4) & 3) + 8 + 4), \
          ((((imm8) >> 6) & 3) + 8 + 4) \
          ) }); }))
#else
#  define simde_mm256_shufflehi_epi16(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_shufflehi_epi16(simde_mm256_extracti128_si256(a, 1), imm8), \
       simde_mm_shufflehi_epi16(simde_mm256_extracti128_si256(a, 0), imm8))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_shufflehi_epi16
  #define _mm256_shufflehi_epi16(a, imm8) simde_mm256_shufflehi_epi16(a, imm8)
#endif

#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_shufflelo_epi16(a, imm8) _mm256_shufflelo_epi16(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_shufflelo_epi16(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_shufflelo_epi16(simde_mm256_extracti128_si256(a, 1), (imm8)), \
       simde_mm_shufflelo_epi16(simde_mm256_extracti128_si256(a, 0), (imm8)))
#elif defined(SIMDE_SHUFFLE_VECTOR_)
#  define simde_mm256_shufflelo_epi16(a, imm8) (__extension__ ({ \
      const simde__m256i_private simde_tmp_a_ = simde__m256i_to_private(a); \
      simde__m256i_from_private((simde__m256i_private) { .i16 = \
        SIMDE_SHUFFLE_VECTOR_(16, 32, \
          (simde_tmp_a_).i16, \
          (simde_tmp_a_).i16, \
          (((imm8)     ) & 3), \
          (((imm8) >> 2) & 3), \
          (((imm8) >> 4) & 3), \
          (((imm8) >> 6) & 3), \
          4, 5, 6, 7, \
          ((((imm8)     ) & 3) + 8), \
          ((((imm8) >> 2) & 3) + 8), \
          ((((imm8) >> 4) & 3) + 8), \
          ((((imm8) >> 6) & 3) + 8), \
          12, 13, 14, 15) }); }))
#else
#  define simde_mm256_shufflelo_epi16(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_shufflelo_epi16(simde_mm256_extracti128_si256(a, 1), imm8), \
       simde_mm_shufflelo_epi16(simde_mm256_extracti128_si256(a, 0), imm8))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_shufflelo_epi16
  #define _mm256_shufflelo_epi16(a, imm8) simde_mm256_shufflelo_epi16(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sign_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sign_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
      r_.i8[i] = (b_.i8[i] < INT32_C(0)) ? -a_.i8[i] : a_.i8[i];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sign_epi8
  #define _mm256_sign_epi8(a, b) simde_mm256_sign_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sign_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sign_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = (b_.i16[i] < INT32_C(0)) ? -a_.i16[i] : a_.i16[i];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sign_epi16
  #define _mm256_sign_epi16(a, b) simde_mm256_sign_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sign_epi32(simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sign_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    SIMDE_VECTORIZE
    for (size_t i = 0; i < (sizeof(r_.i32) / sizeof(r_.i32[0])); i++) {
      r_.i32[i] = (b_.i32[i] < INT32_C(0)) ? -a_.i32[i] : a_.i32[i];
    }

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sign_epi32
  #define _mm256_sign_epi32(a, b) simde_mm256_sign_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sll_epi16 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sll_epi16(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sll_epi16(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_sll_epi16(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t, count_.i64[0]);
      if (shift > 15)
        return simde_mm256_setzero_si256();

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.i16 = a_.i16 << HEDLEY_STATIC_CAST(int16_t, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
          r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, a_.i16[i] << (shift));
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sll_epi16
  #define _mm256_sll_epi16(a, count) simde_mm256_sll_epi16(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sll_epi32 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sll_epi32(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sll_epi32(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_sll_epi32(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t, count_.i64[0]);
      if (shift > 31)
        return simde_mm256_setzero_si256();

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.i32 = a_.i32 << HEDLEY_STATIC_CAST(int32_t, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
          r_.i32[i] = HEDLEY_STATIC_CAST(int32_t, a_.i32[i] << (shift));
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sll_epi32
  #define _mm256_sll_epi32(a, count) simde_mm256_sll_epi32(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sll_epi64 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sll_epi64(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sll_epi64(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_sll_epi64(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t, count_.i64[0]);
      if (shift > 63)
        return simde_mm256_setzero_si256();

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.i64 = a_.i64 << HEDLEY_STATIC_CAST(int64_t, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
          r_.i64[i] = HEDLEY_STATIC_CAST(int64_t, a_.i64[i] << (shift));
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sll_epi64
  #define _mm256_sll_epi64(a, count) simde_mm256_sll_epi64(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_slli_epi16 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  /* Note: There is no consistency in how compilers handle values outside of
     the expected range, hence the discrepancy between what we allow and what
     Intel specifies.  Some compilers will return 0, others seem to just mask
     off everything outside of the range. */
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_POWER_ALTIVEC_VECTOR(unsigned short) sv = vec_splats(HEDLEY_STATIC_CAST(unsigned short, imm8));
    for (size_t i = 0 ; i < (sizeof(a_.altivec_i16) / sizeof(a_.altivec_i16[0])) ; i++) {
      r_.altivec_i16[i] = vec_sl(a_.altivec_i16[i], sv);
    }
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.i16 = a_.i16 << HEDLEY_STATIC_CAST(int16_t, imm8);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, a_.i16[i] << (imm8 & 0xff));
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_slli_epi16(a, imm8) _mm256_slli_epi16(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_slli_epi16(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_slli_epi16(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_slli_epi16(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_slli_epi16
  #define _mm256_slli_epi16(a, imm8) simde_mm256_slli_epi16(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_slli_epi32 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_POWER_ALTIVEC_VECTOR(unsigned int) sv = vec_splats(HEDLEY_STATIC_CAST(unsigned int, imm8));
    for (size_t i = 0 ; i < (sizeof(a_.altivec_i32) / sizeof(a_.altivec_i32[0])) ; i++) {
      r_.altivec_i32[i] = vec_sl(a_.altivec_i32[i], sv);
    }
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.i32 = a_.i32 << HEDLEY_STATIC_CAST(int32_t, imm8);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = a_.i32[i] << (imm8 & 0xff);
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_slli_epi32(a, imm8) _mm256_slli_epi32(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_slli_epi32(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_slli_epi32(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_slli_epi32(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_slli_epi32
  #define _mm256_slli_epi32(a, imm8) simde_mm256_slli_epi32(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_slli_epi64 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

#if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
  r_.i64 = a_.i64 << HEDLEY_STATIC_CAST(int64_t, imm8);
#else
  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
    r_.i64[i] = a_.i64[i] << (imm8 & 0xff);
  }
#endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_slli_epi64(a, imm8) _mm256_slli_epi64(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_slli_epi64(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_slli_epi64(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_slli_epi64(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_slli_epi64
  #define _mm256_slli_epi64(a, imm8) simde_mm256_slli_epi64(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_slli_si256 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  for (size_t h = 0 ; h < (sizeof(r_.m128i_private) / sizeof(r_.m128i_private[0])) ; h++) {
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.m128i_private[h].i8) / sizeof(r_.m128i_private[h].i8[0])) ; i++) {
      const int e = HEDLEY_STATIC_CAST(int, i) - imm8;
      r_.m128i_private[h].i8[i] = (e >= 0) ? a_.m128i_private[h].i8[e] : 0;
    }
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_slli_si256(a, imm8) _mm256_slli_si256(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128) && !defined(__PGI)
#  define simde_mm256_slli_si256(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_slli_si128(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_slli_si128(simde_mm256_extracti128_si256(a, 0), (imm8)))
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm256_slli_si256(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_bslli_si128(simde_mm256_extracti128_si256(a, 1), (imm8)), \
       simde_mm_bslli_si128(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_slli_si256
  #define _mm256_slli_si256(a, imm8) simde_mm256_slli_si256(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_sllv_epi32 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b),
    r_;

  #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    r_.neon_u32 = vshlq_u32(a_.neon_u32, vreinterpretq_s32_u32(b_.neon_u32));
    r_.neon_u32 = vandq_u32(r_.neon_u32, vcltq_u32(b_.neon_u32, vdupq_n_u32(32)));
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u32), (b_.u32 < UINT32_C(32))) & (a_.u32 << b_.u32);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
      r_.u32[i] = (b_.u32[i] < 32) ? (a_.u32[i] << b_.u32[i]) : 0;
    }
  #endif

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_sllv_epi32(a, b) _mm_sllv_epi32(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_sllv_epi32
  #define _mm_sllv_epi32(a, b) simde_mm_sllv_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sllv_epi32 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b),
    r_;

  #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_mm_sllv_epi32(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_mm_sllv_epi32(a_.m128i[1], b_.m128i[1]);
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u32), (b_.u32 < 32)) & (a_.u32 << b_.u32);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
      r_.u32[i] = (b_.u32[i] < 32) ? (a_.u32[i] << b_.u32[i]) : 0;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_sllv_epi32(a, b) _mm256_sllv_epi32(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sllv_epi32
  #define _mm256_sllv_epi32(a, b) simde_mm256_sllv_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_sllv_epi64 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b),
    r_;

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_u64 = vshlq_u64(a_.neon_u64, vreinterpretq_s64_u64(b_.neon_u64));
    r_.neon_u64 = vandq_u64(r_.neon_u64, vcltq_u64(b_.neon_u64, vdupq_n_u64(64)));
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u64), (b_.u64 < 64)) & (a_.u64 << b_.u64);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
      r_.u64[i] = (b_.u64[i] < 64) ? (a_.u64[i] << b_.u64[i]) : 0;
    }
  #endif

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_sllv_epi64(a, b) _mm_sllv_epi64(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_sllv_epi64
  #define _mm_sllv_epi64(a, b) simde_mm_sllv_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sllv_epi64 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b),
    r_;

  #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_mm_sllv_epi64(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_mm_sllv_epi64(a_.m128i[1], b_.m128i[1]);
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u64), (b_.u64 < 64)) & (a_.u64 << b_.u64);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
      r_.u64[i] = (b_.u64[i] < 64) ? (a_.u64[i] << b_.u64[i]) : 0;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_sllv_epi64(a, b) _mm256_sllv_epi64(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sllv_epi64
  #define _mm256_sllv_epi64(a, b) simde_mm256_sllv_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sra_epi16 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sra_epi16(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sra_epi16(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_sra_epi16(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t, count_.i64[0]);

      if (shift > 15) shift = 15;

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.i16 = a_.i16 >> HEDLEY_STATIC_CAST(int16_t, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
          r_.i16[i] = a_.i16[i] >> shift;
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sra_epi16
  #define _mm256_sra_epi16(a, count) simde_mm256_sra_epi16(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sra_epi32 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sra_epi32(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sra_epi32(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_sra_epi32(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);
      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t, count_.i64[0]);

      if (shift > 31) shift = 31;

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.i32 = a_.i32 >> HEDLEY_STATIC_CAST(int16_t, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
          r_.i32[i] = a_.i32[i] >> shift;
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sra_epi32
  #define _mm256_sra_epi32(a, count) simde_mm256_sra_epi32(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srai_epi16 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);
  unsigned int shift = HEDLEY_STATIC_CAST(unsigned int, imm8);

  if (shift > 15) shift = 15;

  #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.i16 = a_.i16 >> HEDLEY_STATIC_CAST(int16_t, shift);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
      r_.i16[i] = a_.i16[i] >> shift;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_srai_epi16(a, imm8) _mm256_srai_epi16(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_srai_epi16(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_srai_epi16(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_srai_epi16(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srai_epi16
  #define _mm256_srai_epi16(a, imm8) simde_mm256_srai_epi16(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srai_epi32 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);
  unsigned int shift = HEDLEY_STATIC_CAST(unsigned int, imm8);

  if (shift > 31) shift = 31;

  #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.i32 = a_.i32 >> HEDLEY_STATIC_CAST(int16_t, shift);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
      r_.i32[i] = a_.i32[i] >> shift;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_srai_epi32(a, imm8) _mm256_srai_epi32(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_srai_epi32(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_srai_epi32(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_srai_epi32(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srai_epi32
  #define _mm256_srai_epi32(a, imm8) simde_mm256_srai_epi32(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_srav_epi32 (simde__m128i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm_srav_epi32(a, count);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      count_ = simde__m128i_to_private(count);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int32x4_t cnt = vreinterpretq_s32_u32(vminq_u32(count_.neon_u32, vdupq_n_u32(31)));
      r_.neon_i32 = vshlq_s32(a_.neon_i32, vnegq_s32(cnt));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        uint32_t shift = HEDLEY_STATIC_CAST(uint32_t, count_.i32[i]);
        r_.i32[i] = a_.i32[i] >> HEDLEY_STATIC_CAST(int, shift > 31 ? 31 : shift);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_srav_epi32
  #define _mm_srav_epi32(a, count) simde_mm_srav_epi32(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srav_epi32 (simde__m256i a, simde__m256i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_srav_epi32(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      count_ = simde__m256i_to_private(count);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_srav_epi32(a_.m128i[0], count_.m128i[0]);
      r_.m128i[1] = simde_mm_srav_epi32(a_.m128i[1], count_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        uint32_t shift = HEDLEY_STATIC_CAST(uint32_t, count_.i32[i]);
        if (shift > 31) shift = 31;
        r_.i32[i] = a_.i32[i] >> shift;
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srav_epi32
  #define _mm256_srav_epi32(a, count) simde_mm256_srav_epi32(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srl_epi16 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_srl_epi16(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_srl_epi16(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_srl_epi16(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t , (count_.i64[0] > 16 ? 16 : count_.i64[0]));

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.u16 = a_.u16 >> SIMDE_CAST_VECTOR_SHIFT_COUNT(16, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
          r_.u16[i] = a_.u16[i] >> (shift);
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srl_epi16
  #define _mm256_srl_epi16(a, count) simde_mm256_srl_epi16(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srl_epi32 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_srl_epi32(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_srl_epi32(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_srl_epi32(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t , (count_.i64[0] > 32 ? 32 : count_.i64[0]));

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.u32 = a_.u32 >> SIMDE_CAST_VECTOR_SHIFT_COUNT(32, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
          r_.u32[i] = a_.u32[i] >> (shift);
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srl_epi32
  #define _mm256_srl_epi32(a, count) simde_mm256_srl_epi32(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srl_epi64 (simde__m256i a, simde__m128i count) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_srl_epi64(a, count);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_srl_epi64(a_.m128i[0], count);
      r_.m128i[1] = simde_mm_srl_epi64(a_.m128i[1], count);
    #else
      simde__m128i_private
        count_ = simde__m128i_to_private(count);

      uint64_t shift = HEDLEY_STATIC_CAST(uint64_t , (count_.i64[0] > 64 ? 64 : count_.i64[0]));

      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.u64 = a_.u64 >> SIMDE_CAST_VECTOR_SHIFT_COUNT(64, shift);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
          r_.u64[i] = a_.u64[i] >> (shift);
        }
      #endif
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srl_epi64
  #define _mm256_srl_epi64(a, count) simde_mm256_srl_epi64(a, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srli_epi16 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  if (imm8 > 15)
    return simde_mm256_setzero_si256();

  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_POWER_ALTIVEC_VECTOR(unsigned short) sv = vec_splats(HEDLEY_STATIC_CAST(unsigned short, imm8));
    for (size_t i = 0 ; i < (sizeof(a_.altivec_u16) / sizeof(a_.altivec_u16[0])) ; i++) {
      r_.altivec_u16[i] = vec_sr(a_.altivec_u16[i], sv);
    }
  #else
    if (HEDLEY_STATIC_CAST(unsigned int, imm8) > 15) {
      simde_memset(&r_, 0, sizeof(r_));
    } else {
      #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
        r_.u16 = a_.u16 >> SIMDE_CAST_VECTOR_SHIFT_COUNT(16, imm8);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
          r_.u16[i] = a_.u16[i] >> imm8;
        }
      #endif
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_srli_epi16(a, imm8) _mm256_srli_epi16(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_srli_epi16(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_srli_epi16(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_srli_epi16(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srli_epi16
  #define _mm256_srli_epi16(a, imm8) simde_mm256_srli_epi16(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srli_epi32 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    SIMDE_POWER_ALTIVEC_VECTOR(unsigned int) sv = vec_splats(HEDLEY_STATIC_CAST(unsigned int, imm8));
    for (size_t i = 0 ; i < (sizeof(a_.altivec_u32) / sizeof(a_.altivec_u32[0])) ; i++) {
      r_.altivec_u32[i] = vec_sr(a_.altivec_u32[i], sv);
    }
  #elif defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u32 = a_.u32 >> SIMDE_CAST_VECTOR_SHIFT_COUNT(16, imm8);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
      r_.u32[i] = a_.u32[i] >> imm8;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_srli_epi32(a, imm8) _mm256_srli_epi32(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_srli_epi32(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_srli_epi32(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_srli_epi32(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srli_epi32
  #define _mm256_srli_epi32(a, imm8) simde_mm256_srli_epi32(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srli_epi64 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

#if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
  r_.u64 = a_.u64 >> SIMDE_CAST_VECTOR_SHIFT_COUNT(32, imm8);
#else
  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
    r_.u64[i] = a_.u64[i] >> imm8;
  }
#endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_srli_epi64(a, imm8) _mm256_srli_epi64(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
#  define simde_mm256_srli_epi64(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_srli_epi64(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_srli_epi64(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srli_epi64
  #define _mm256_srli_epi64(a, imm8) simde_mm256_srli_epi64(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srli_si256 (simde__m256i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a);

  for (size_t h = 0 ; h < (sizeof(r_.m128i_private) / sizeof(r_.m128i_private[0])) ; h++) {
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.m128i_private[h].i8) / sizeof(r_.m128i_private[h].i8[0])) ; i++) {
      const int e = imm8 + HEDLEY_STATIC_CAST(int, i);
      r_.m128i_private[h].i8[i] = (e < 16) ? a_.m128i_private[h].i8[e] : 0;
    }
  }

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
#  define simde_mm256_srli_si256(a, imm8) _mm256_srli_si256(a, imm8)
#elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128) && !defined(__PGI)
#  define simde_mm256_srli_si256(a, imm8) \
     simde_mm256_set_m128i( \
         simde_mm_srli_si128(simde_mm256_extracti128_si256(a, 1), (imm8)), \
         simde_mm_srli_si128(simde_mm256_extracti128_si256(a, 0), (imm8)))
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm256_srli_si256(a, imm8) \
     simde_mm256_set_m128i( \
       simde_mm_bsrli_si128(simde_mm256_extracti128_si256(a, 1), (imm8)), \
       simde_mm_bsrli_si128(simde_mm256_extracti128_si256(a, 0), (imm8)))
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srli_si256
  #define _mm256_srli_si256(a, imm8) simde_mm256_srli_si256(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_srlv_epi32 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b),
    r_;

  #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u32), (b_.u32 < 32)) & (a_.u32 >> b_.u32);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
      r_.u32[i] = (b_.u32[i] < 32) ? (a_.u32[i] >> b_.u32[i]) : 0;
    }
  #endif

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_srlv_epi32(a, b) _mm_srlv_epi32(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_srlv_epi32
  #define _mm_srlv_epi32(a, b) simde_mm_srlv_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srlv_epi32 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b),
    r_;

  #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u32), (b_.u32 < 32)) & (a_.u32 >> b_.u32);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
      r_.u32[i] = (b_.u32[i] < 32) ? (a_.u32[i] >> b_.u32[i]) : 0;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_srlv_epi32(a, b) _mm256_srlv_epi32(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srlv_epi32
  #define _mm256_srlv_epi32(a, b) simde_mm256_srlv_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_srlv_epi64 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b),
    r_;

  #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u64), (b_.u64 < 64)) & (a_.u64 >> b_.u64);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
      r_.u64[i] = (b_.u64[i] < 64) ? (a_.u64[i] >> b_.u64[i]) : 0;
    }
  #endif

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm_srlv_epi64(a, b) _mm_srlv_epi64(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm_srlv_epi64
  #define _mm_srlv_epi64(a, b) simde_mm_srlv_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_srlv_epi64 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b),
    r_;

  #if defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
    r_.u64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.u64), (b_.u64 < 64)) & (a_.u64 >> b_.u64);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
      r_.u64[i] = (b_.u64[i] < 64) ? (a_.u64[i] >> b_.u64[i]) : 0;
    }
  #endif

  return simde__m256i_from_private(r_);
}
#if defined(SIMDE_X86_AVX2_NATIVE)
  #define simde_mm256_srlv_epi64(a, b) _mm256_srlv_epi64(a, b)
#endif
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_srlv_epi64
  #define _mm256_srlv_epi64(a, b) simde_mm256_srlv_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_stream_load_si256 (const simde__m256i* mem_addr) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_stream_load_si256(HEDLEY_CONST_CAST(simde__m256i*, mem_addr));
  #elif HEDLEY_HAS_BUILTIN(__builtin_nontemporal_store) && defined(SIMDE_VECTOR_SUBSCRIPT)
    return __builtin_nontemporal_load(mem_addr);
  #else
    simde__m256i r;
    simde_memcpy(&r, SIMDE_ALIGN_ASSUME_LIKE(mem_addr, simde__m256i), sizeof(r));
    return r;
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
#  define _mm256_stream_load_si256(mem_addr) simde_mm256_stream_load_si256(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sub_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sub_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sub_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_sub_epi8(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i8 = a_.i8 - b_.i8;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[i] - b_.i8[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sub_epi8
  #define _mm256_sub_epi8(a, b) simde_mm256_sub_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sub_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sub_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sub_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_sub_epi16(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i16 = a_.i16 - b_.i16;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a_.i16[i] - b_.i16[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sub_epi16
  #define _mm256_sub_epi16(a, b) simde_mm256_sub_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_hsub_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_hsub_epi16(a, b);
  #else
    return simde_mm256_sub_epi16(simde_x_mm256_deinterleaveeven_epi16(a, b), simde_x_mm256_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hsub_epi16
  #define _mm256_hsub_epi16(a, b) simde_mm256_hsub_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sub_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sub_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sub_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_sub_epi32(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32 = a_.i32 - b_.i32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] - b_.i32[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sub_epi32
  #define _mm256_sub_epi32(a, b) simde_mm256_sub_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_hsub_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_hsub_epi32(a, b);
  #else
    return simde_mm256_sub_epi32(simde_x_mm256_deinterleaveeven_epi32(a, b), simde_x_mm256_deinterleaveodd_epi32(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hsub_epi32
  #define _mm256_hsub_epi32(a, b) simde_mm256_hsub_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_sub_epi64 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_sub_epi64(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_sub_epi64(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_sub_epi64(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i64 = a_.i64 - b_.i64;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i64[i] - b_.i64[i];
      }
    #endif

  return simde__m256i_from_private(r_);
#endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_sub_epi64
  #define _mm256_sub_epi64(a, b) simde_mm256_sub_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_x_mm256_sub_epu32 (simde__m256i a, simde__m256i b) {
  simde__m256i_private
    r_,
    a_ = simde__m256i_to_private(a),
    b_ = simde__m256i_to_private(b);

  #if defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
    r_.u32 = a_.u32 - b_.u32;
  #elif SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
    r_.m128i[0] = simde_x_mm_sub_epu32(a_.m128i[0], b_.m128i[0]);
    r_.m128i[1] = simde_x_mm_sub_epu32(a_.m128i[1], b_.m128i[1]);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
      r_.u32[i] = a_.u32[i] - b_.u32[i];
    }
  #endif

  return simde__m256i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_subs_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_subs_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_subs_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_subs_epi8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = simde_math_subs_i8(a_.i8[i], b_.i8[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_subs_epi8
  #define _mm256_subs_epi8(a, b) simde_mm256_subs_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_subs_epi16(simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_subs_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_subs_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_subs_epi16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = simde_math_subs_i16(a_.i16[i], b_.i16[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_subs_epi16
  #define _mm256_subs_epi16(a, b) simde_mm256_subs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_hsubs_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_hsubs_epi16(a, b);
  #else
    return simde_mm256_subs_epi16(simde_x_mm256_deinterleaveeven_epi16(a, b), simde_x_mm256_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_hsubs_epi16
  #define _mm256_hsubs_epi16(a, b) simde_mm256_hsubs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_subs_epu8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_subs_epu8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_subs_epu8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_subs_epu8(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
        r_.u8[i] = simde_math_subs_u8(a_.u8[i], b_.u8[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_subs_epu8
  #define _mm256_subs_epu8(a, b) simde_mm256_subs_epu8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_subs_epu16(simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_subs_epu16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_subs_epu16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_subs_epu16(a_.m128i[1], b_.m128i[1]);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
        r_.u16[i] = simde_math_subs_u16(a_.u16[i], b_.u16[i]);
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_subs_epu16
  #define _mm256_subs_epu16(a, b) simde_mm256_subs_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_x_mm256_test_all_ones (simde__m256i a) {
  simde__m256i_private a_ = simde__m256i_to_private(a);
  int r;
  int_fast32_t r_ = ~HEDLEY_STATIC_CAST(int_fast32_t, 0);

  SIMDE_VECTORIZE_REDUCTION(&:r_)
  for (size_t i = 0 ; i < (sizeof(a_.i32f) / sizeof(a_.i32f[0])) ; i++) {
    r_ &= a_.i32f[i];
  }

  r = (r_ == ~HEDLEY_STATIC_CAST(int_fast32_t, 0));

  return r;
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpacklo_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpacklo_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpacklo_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpacklo_epi8(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i8 = SIMDE_SHUFFLE_VECTOR_(8, 32, a_.i8, b_.i8,
           0, 32,  1, 33,  2, 34,  3, 35,
           4, 36,  5, 37,  6, 38,  7, 39,
          16, 48, 17, 49, 18, 50, 19, 51,
          20, 52, 21, 53, 22, 54, 23, 55);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0]) / 2) ; i++) {
        r_.i8[2 * i] = a_.i8[i + ~(~i | 7)];
        r_.i8[2 * i + 1] = b_.i8[i + ~(~i | 7)];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpacklo_epi8
  #define _mm256_unpacklo_epi8(a, b) simde_mm256_unpacklo_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpacklo_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpacklo_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpacklo_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpacklo_epi16(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i16 =SIMDE_SHUFFLE_VECTOR_(16, 32, a_.i16, b_.i16,
        0, 16, 1, 17, 2, 18, 3, 19, 8, 24, 9, 25, 10, 26, 11, 27);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0]) / 2) ; i++) {
        r_.i16[2 * i] = a_.i16[i + ~(~i | 3)];
        r_.i16[2 * i + 1] = b_.i16[i + ~(~i | 3)];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpacklo_epi16
  #define _mm256_unpacklo_epi16(a, b) simde_mm256_unpacklo_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpacklo_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpacklo_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpacklo_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpacklo_epi32(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.i32, b_.i32,
                                    0, 8, 1, 9, 4, 12, 5, 13);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0]) / 2) ; i++) {
        r_.i32[2 * i] = a_.i32[i + ~(~i | 1)];
        r_.i32[2 * i + 1] = b_.i32[i + ~(~i | 1)];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpacklo_epi32
  #define _mm256_unpacklo_epi32(a, b) simde_mm256_unpacklo_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpacklo_epi64 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpacklo_epi64(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpacklo_epi64(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpacklo_epi64(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.i64, b_.i64, 0, 4, 2, 6);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0]) / 2) ; i++) {
        r_.i64[2 * i] = a_.i64[2 * i];
        r_.i64[2 * i + 1] = b_.i64[2 * i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpacklo_epi64
  #define _mm256_unpacklo_epi64(a, b) simde_mm256_unpacklo_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpackhi_epi8 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpackhi_epi8(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpackhi_epi8(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpackhi_epi8(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i8 = SIMDE_SHUFFLE_VECTOR_(8, 32, a_.i8, b_.i8,
           8, 40,  9, 41, 10, 42, 11, 43,
          12, 44, 13, 45, 14, 46, 15, 47,
          24, 56, 25, 57, 26, 58, 27, 59,
          28, 60, 29, 61, 30, 62, 31, 63);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0]) / 2) ; i++) {
        r_.i8[2 * i] = a_.i8[i + 8 + ~(~i | 7)];
        r_.i8[2 * i + 1] = b_.i8[i + 8 + ~(~i | 7)];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpackhi_epi8
  #define _mm256_unpackhi_epi8(a, b) simde_mm256_unpackhi_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpackhi_epi16 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpackhi_epi16(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpackhi_epi16(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpackhi_epi16(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i16 = SIMDE_SHUFFLE_VECTOR_(16, 32, a_.i16, b_.i16,
         4, 20,  5, 21,  6, 22,  7, 23,
        12, 28, 13, 29, 14, 30, 15, 31);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0]) / 2) ; i++) {
        r_.i16[2 * i] = a_.i16[i + 4 + ~(~i | 3)];
        r_.i16[2 * i + 1] = b_.i16[i + 4 + ~(~i | 3)];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpackhi_epi16
  #define _mm256_unpackhi_epi16(a, b) simde_mm256_unpackhi_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpackhi_epi32 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpackhi_epi32(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpackhi_epi32(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpackhi_epi32(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i32 = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.i32, b_.i32,
                                    2, 10, 3, 11, 6, 14, 7, 15);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0]) / 2) ; i++) {
        r_.i32[2 * i] = a_.i32[i + 2 + ~(~i | 1)];
        r_.i32[2 * i + 1] = b_.i32[i + 2 + ~(~i | 1)];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpackhi_epi32
  #define _mm256_unpackhi_epi32(a, b) simde_mm256_unpackhi_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_unpackhi_epi64 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_unpackhi_epi64(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_unpackhi_epi64(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_unpackhi_epi64(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i64 = SIMDE_SHUFFLE_VECTOR_(64, 32, a_.i64, b_.i64, 1, 5, 3, 7);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0]) / 2) ; i++) {
        r_.i64[2 * i] = a_.i64[2 * i + 1];
        r_.i64[2 * i + 1] = b_.i64[2 * i + 1];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_unpackhi_epi64
  #define _mm256_unpackhi_epi64(a, b) simde_mm256_unpackhi_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256i
simde_mm256_xor_si256 (simde__m256i a, simde__m256i b) {
  #if defined(SIMDE_X86_AVX2_NATIVE)
    return _mm256_xor_si256(a, b);
  #else
    simde__m256i_private
      r_,
      a_ = simde__m256i_to_private(a),
      b_ = simde__m256i_to_private(b);

    #if SIMDE_NATURAL_INT_VECTOR_SIZE_LE(128)
      r_.m128i[0] = simde_mm_xor_si128(a_.m128i[0], b_.m128i[0]);
      r_.m128i[1] = simde_mm_xor_si128(a_.m128i[1], b_.m128i[1]);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i32f = a_.i32f ^ b_.i32f;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i64[i] ^ b_.i64[i];
      }
    #endif

    return simde__m256i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_AVX2_ENABLE_NATIVE_ALIASES)
  #undef _mm256_xor_si256
  #define _mm256_xor_si256(a, b) simde_mm256_xor_si256(a, b)
#endif

SIMDE_END_DECLS_

HEDLEY_DIAGNOSTIC_POP

#endif /* !defined(SIMDE_X86_AVX2_H) */
