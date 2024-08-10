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
 *   2017-2020 Evan Nemerson <evan@nemerson.com>
 */

#if !defined(SIMDE_X86_SSSE3_H)
#define SIMDE_X86_SSSE3_H

#include "sse3.h"

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DISABLE_UNWANTED_DIAGNOSTICS
SIMDE_BEGIN_DECLS_

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_abs_epi8 (simde__m128i a) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_abs_epi8(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_min_epu8(a, _mm_sub_epi8(_mm_setzero_si128(), a));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i8 = vabsq_s8(a_.neon_i8);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i8 = vec_abs(a_.altivec_i8);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i8x16_abs(a_.wasm_v128);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.u8[i] = HEDLEY_STATIC_CAST(uint8_t, (a_.i8[i] < 0) ? (- a_.i8[i]) : a_.i8[i]);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_abs_epi8(a) simde_mm_abs_epi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_abs_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_abs_epi16(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_max_epi16(a, _mm_sub_epi16(_mm_setzero_si128(), a));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i16 = vabsq_s16(a_.neon_i16);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i16 = vec_abs(a_.altivec_i16);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i16x8_abs(a_.wasm_v128);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.u16[i] = HEDLEY_STATIC_CAST(uint16_t, (a_.i16[i] < 0) ? (- a_.i16[i]) : a_.i16[i]);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_abs_epi16(a) simde_mm_abs_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_abs_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_abs_epi32(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    const __m128i m = _mm_cmpgt_epi32(_mm_setzero_si128(), a);
    return _mm_sub_epi32(_mm_xor_si128(a, m), m);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vabsq_s32(a_.neon_i32);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i32 = vec_abs(a_.altivec_i32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_abs(a_.wasm_v128);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        #if defined(_MSC_VER)
          HEDLEY_DIAGNOSTIC_PUSH
          #pragma warning(disable:4146)
        #endif
        r_.u32[i] = (a_.i32[i] < 0) ? (- HEDLEY_STATIC_CAST(uint32_t, a_.i32[i])) : HEDLEY_STATIC_CAST(uint32_t, a_.i32[i]);
        #if defined(_MSC_VER)
          HEDLEY_DIAGNOSTIC_POP
        #endif
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_abs_epi32(a) simde_mm_abs_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_abs_pi8 (simde__m64 a) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_abs_pi8(a);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i8 = vabs_s8(a_.neon_i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.u8[i] = HEDLEY_STATIC_CAST(uint8_t, (a_.i8[i] < 0) ? (- a_.i8[i]) : a_.i8[i]);
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_abs_pi8(a) simde_mm_abs_pi8(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_abs_pi16 (simde__m64 a) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_abs_pi16(a);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i16 = vabs_s16(a_.neon_i16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.u16[i] = HEDLEY_STATIC_CAST(uint16_t, (a_.i16[i] < 0) ? (- a_.i16[i]) : a_.i16[i]);
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_abs_pi16(a) simde_mm_abs_pi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_abs_pi32 (simde__m64 a) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_abs_pi32(a);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vabs_s32(a_.neon_i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.u32[i] = HEDLEY_STATIC_CAST(uint32_t, (a_.i32[i] < 0) ? (- a_.i32[i]) : a_.i32[i]);
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_abs_pi32(a) simde_mm_abs_pi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_alignr_epi8 (simde__m128i a, simde__m128i b, int count)
    SIMDE_REQUIRE_CONSTANT_RANGE(count, 0, 255) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  if (HEDLEY_UNLIKELY(count > 31))
    return simde_mm_setzero_si128();

  for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
    const int srcpos = count + HEDLEY_STATIC_CAST(int, i);
    if (srcpos > 31) {
      r_.i8[i] = 0;
    } else if (srcpos > 15) {
      r_.i8[i] = a_.i8[(srcpos) & 15];
    } else {
      r_.i8[i] = b_.i8[srcpos];
    }
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_SSSE3_NATIVE)
  #define simde_mm_alignr_epi8(a, b, count) _mm_alignr_epi8(a, b, count)
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
  #define simde_mm_alignr_epi8(a, b, count) \
    ( \
      ((count) > 31) \
        ? simde__m128i_from_neon_i8(vdupq_n_s8(0)) \
        : ( \
          ((count) > 15) \
            ? (simde__m128i_from_neon_i8(vextq_s8(simde__m128i_to_neon_i8(a), vdupq_n_s8(0), (count) & 15))) \
            : (simde__m128i_from_neon_i8(vextq_s8(simde__m128i_to_neon_i8(b), simde__m128i_to_neon_i8(a), ((count) & 15))))))
#endif
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
  #define _mm_alignr_epi8(a, b, count) simde_mm_alignr_epi8(a, b, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_alignr_pi8 (simde__m64 a, simde__m64 b, const int count)
    SIMDE_REQUIRE_CONSTANT(count) {
  simde__m64_private
    r_,
    a_ = simde__m64_to_private(a),
    b_ = simde__m64_to_private(b);

  if (HEDLEY_UNLIKELY(count > 15))
    return simde_mm_setzero_si64();

  for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
    const int srcpos = count + HEDLEY_STATIC_CAST(int, i);
    if (srcpos > 15) {
      r_.i8[i] = 0;
    } else if (srcpos > 7) {
      r_.i8[i] = a_.i8[(srcpos) & 7];
    } else {
      r_.i8[i] = b_.i8[srcpos];
    }
  }

  return simde__m64_from_private(r_);
}
#if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
#  define simde_mm_alignr_pi8(a, b, count) _mm_alignr_pi8(a, b, count)
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
  #define simde_mm_alignr_pi8(a, b, count) \
    ( \
      ((count) > 15) \
        ? simde__m64_from_neon_i8(vdup_n_s8(0)) \
        : ( \
          ((count) > 7) \
            ? (simde__m64_from_neon_i8(vext_s8(simde__m64_to_neon_i8(a), vdup_n_s8(0), (count) & 7))) \
            : (simde__m64_from_neon_i8(vext_s8(simde__m64_to_neon_i8(b), simde__m64_to_neon_i8(a), ((count) & 7))))))
#endif
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_alignr_pi8(a, b, count) simde_mm_alignr_pi8(a, b, count)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_shuffle_epi8 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_shuffle_epi8(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_i8 = vqtbl1q_s8(a_.neon_i8, vandq_u8(b_.neon_u8, vdupq_n_u8(0x8F)));
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      /* Mask out the bits we're not interested in.  vtbl will result in 0
       * for any values outside of [0, 15], so if the high bit is set it
       * will return 0, just like in SSSE3. */
      b_.neon_i8 = vandq_s8(b_.neon_i8, vdupq_n_s8(HEDLEY_STATIC_CAST(int8_t, (1 << 7) | 15)));

      /* Convert a from an int8x16_t to an int8x8x2_t */
      int8x8x2_t i;
      i.val[0] = vget_low_s8(a_.neon_i8);
      i.val[1] = vget_high_s8(a_.neon_i8);

      /* Table lookups */
      int8x8_t l = vtbl2_s8(i, vget_low_s8(b_.neon_i8));
      int8x8_t h = vtbl2_s8(i, vget_high_s8(b_.neon_i8));

      r_.neon_i8 = vcombine_s8(l, h);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
      /* This is a bit ugly because of the casts and the awful type
       * macros (SIMDE_POWER_ALTIVEC_VECTOR), but it's really just
       * vec_sel(vec_perm(a, a, b), 0, vec_cmplt(b, 0)) */
      SIMDE_POWER_ALTIVEC_VECTOR(signed char) z = { 0, };
      SIMDE_POWER_ALTIVEC_VECTOR(signed char) msb_mask = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(signed char), vec_cmplt(b_.altivec_i8, z));
      SIMDE_POWER_ALTIVEC_VECTOR(signed char) c = vec_perm(a_.altivec_i8, a_.altivec_i8, HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(unsigned char), b_.altivec_i8));
      r_.altivec_i8 = vec_sel(c, z, HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(unsigned char), msb_mask));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i8x16_swizzle(
        a_.wasm_v128, wasm_v128_and(b_.wasm_v128, wasm_i8x16_splat(0x8F)));
    #else
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[b_.i8[i] & 15] & (~(b_.i8[i]) >> 7);
      }
    #endif

    return simde__m128i_from_private(r_);
#endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_shuffle_epi8(a, b) simde_mm_shuffle_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_shuffle_pi8 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_shuffle_pi8(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      b_.neon_i8 = vand_s8(b_.neon_i8, vdup_n_s8(HEDLEY_STATIC_CAST(int8_t, (1 << 7) | 7)));
      r_.neon_i8 = vtbl1_s8(a_.neon_i8, b_.neon_i8);
    #else
      for (size_t i = 0 ; i < (sizeof(r_.u8) / sizeof(r_.u8[0])) ; i++) {
        r_.i8[i] = a_.i8[b_.i8[i] & 7] & (~(b_.i8[i]) >> 7);
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_shuffle_pi8(a, b) simde_mm_shuffle_pi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_hadd_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_hadd_epi16(a, b);
  #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    return simde__m128i_from_neon_i16(vpaddq_s16(simde__m128i_to_neon_i16(a), simde__m128i_to_neon_i16(b)));
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int16x8x2_t t = vuzpq_s16(simde__m128i_to_neon_i16(a), simde__m128i_to_neon_i16(b));
    return simde__m128i_from_neon_i16(vaddq_s16(t.val[0], t.val[1]));
  #else
    return simde_mm_add_epi16(simde_x_mm_deinterleaveeven_epi16(a, b), simde_x_mm_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadd_epi16(a, b) simde_mm_hadd_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_hadd_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_hadd_epi32(a, b);
  #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    return simde__m128i_from_neon_i32(vpaddq_s32(simde__m128i_to_neon_i32(a), simde__m128i_to_neon_i32(b)));
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int32x4x2_t t = vuzpq_s32(simde__m128i_to_neon_i32(a), simde__m128i_to_neon_i32(b));
    return simde__m128i_from_neon_i32(vaddq_s32(t.val[0], t.val[1]));
  #else
    return simde_mm_add_epi32(simde_x_mm_deinterleaveeven_epi32(a, b), simde_x_mm_deinterleaveodd_epi32(a, b));
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadd_epi32(a, b) simde_mm_hadd_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_hadd_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_hadd_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_i16 = vpadd_s16(a_.neon_i16, b_.neon_i16);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int16x4x2_t t = vuzp_s16(a_.neon_i16, b_.neon_i16);
      r_.neon_i16 = vadd_s16(t.val[0], t.val[1]);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i16 =
        SIMDE_SHUFFLE_VECTOR_(16, 8, a_.i16, b_.i16, 0, 2, 4, 6) +
        SIMDE_SHUFFLE_VECTOR_(16, 8, a_.i16, b_.i16, 1, 3, 5, 7);
    #else
      r_.i16[0] = a_.i16[0] + a_.i16[1];
      r_.i16[1] = a_.i16[2] + a_.i16[3];
      r_.i16[2] = b_.i16[0] + b_.i16[1];
      r_.i16[3] = b_.i16[2] + b_.i16[3];
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadd_pi16(a, b) simde_mm_hadd_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_hadd_pi32 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_hadd_pi32(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_i32 = vpadd_s32(a_.neon_i32, b_.neon_i32);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int32x2x2_t t = vuzp_s32(a_.neon_i32, b_.neon_i32);
      r_.neon_i32 = vadd_s32(t.val[0], t.val[1]);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i32 =
        SIMDE_SHUFFLE_VECTOR_(32, 8, a_.i32, b_.i32, 0, 2) +
        SIMDE_SHUFFLE_VECTOR_(32, 8, a_.i32, b_.i32, 1, 3);
    #else
      r_.i32[0] = a_.i32[0] + a_.i32[1];
      r_.i32[1] = b_.i32[0] + b_.i32[1];
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadd_pi32(a, b) simde_mm_hadd_pi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_hadds_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_hadds_epi16(a, b);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int16x8x2_t t = vuzpq_s16(simde__m128i_to_neon_i16(a), simde__m128i_to_neon_i16(b));
    return simde__m128i_from_neon_i16(vqaddq_s16(t.val[0], t.val[1]));
  #else
    return simde_mm_adds_epi16(simde_x_mm_deinterleaveeven_epi16(a, b), simde_x_mm_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadds_epi16(a, b) simde_mm_hadds_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_hadds_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_hadds_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int16x4x2_t t = vuzp_s16(a_.neon_i16, b_.neon_i16);
      r_.neon_i16 = vqadd_s16(t.val[0], t.val[1]);
    #else
      for (size_t i = 0 ; i < ((sizeof(r_.i16) / sizeof(r_.i16[0])) / 2) ; i++) {
        int32_t ta = HEDLEY_STATIC_CAST(int32_t, a_.i16[i * 2]) + HEDLEY_STATIC_CAST(int32_t, a_.i16[(i * 2) + 1]);
        r_.i16[  i  ] = HEDLEY_LIKELY(ta > INT16_MIN) ? (HEDLEY_LIKELY(ta < INT16_MAX) ? HEDLEY_STATIC_CAST(int16_t, ta) : INT16_MAX) : INT16_MIN;
        int32_t tb = HEDLEY_STATIC_CAST(int32_t, b_.i16[i * 2]) + HEDLEY_STATIC_CAST(int32_t, b_.i16[(i * 2) + 1]);
        r_.i16[i + 2] = HEDLEY_LIKELY(tb > INT16_MIN) ? (HEDLEY_LIKELY(tb < INT16_MAX) ? HEDLEY_STATIC_CAST(int16_t, tb) : INT16_MAX) : INT16_MIN;
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadds_pi16(a, b) simde_mm_hadds_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_hsub_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_hsub_epi16(a, b);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int16x8x2_t t = vuzpq_s16(simde__m128i_to_neon_i16(a), simde__m128i_to_neon_i16(b));
    return simde__m128i_from_neon_i16(vsubq_s16(t.val[0], t.val[1]));
  #else
    return simde_mm_sub_epi16(simde_x_mm_deinterleaveeven_epi16(a, b), simde_x_mm_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsub_epi16(a, b) simde_mm_hsub_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_hsub_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_hsub_epi32(a, b);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int32x4x2_t t = vuzpq_s32(simde__m128i_to_neon_i32(a), simde__m128i_to_neon_i32(b));
    return simde__m128i_from_neon_i32(vsubq_s32(t.val[0], t.val[1]));
  #else
    return simde_mm_sub_epi32(simde_x_mm_deinterleaveeven_epi32(a, b), simde_x_mm_deinterleaveodd_epi32(a, b));
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsub_epi32(a, b) simde_mm_hsub_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_hsub_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_hsub_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int16x4x2_t t = vuzp_s16(a_.neon_i16, b_.neon_i16);
      r_.neon_i16 = vsub_s16(t.val[0], t.val[1]);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i16 =
        SIMDE_SHUFFLE_VECTOR_(16, 8, a_.i16, b_.i16, 0, 2, 4, 6) -
        SIMDE_SHUFFLE_VECTOR_(16, 8, a_.i16, b_.i16, 1, 3, 5, 7);
    #else
      r_.i16[0] = a_.i16[0] - a_.i16[1];
      r_.i16[1] = a_.i16[2] - a_.i16[3];
      r_.i16[2] = b_.i16[0] - b_.i16[1];
      r_.i16[3] = b_.i16[2] - b_.i16[3];
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsub_pi16(a, b) simde_mm_hsub_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_hsub_pi32 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_hsub_pi32(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int32x2x2_t t = vuzp_s32(a_.neon_i32, b_.neon_i32);
      r_.neon_i32 = vsub_s32(t.val[0], t.val[1]);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.i32 =
        SIMDE_SHUFFLE_VECTOR_(32, 8, a_.i32, b_.i32, 0, 2) -
        SIMDE_SHUFFLE_VECTOR_(32, 8, a_.i32, b_.i32, 1, 3);
    #else
      r_.i32[0] = a_.i32[0] - a_.i32[1];
      r_.i32[1] = b_.i32[0] - b_.i32[1];
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsub_pi32(a, b) simde_mm_hsub_pi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_hsubs_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_hsubs_epi16(a, b);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int16x8x2_t t = vuzpq_s16(simde__m128i_to_neon_i16(a), simde__m128i_to_neon_i16(b));
    return simde__m128i_from_neon_i16(vqsubq_s16(t.val[0], t.val[1]));
  #else
    return simde_mm_subs_epi16(simde_x_mm_deinterleaveeven_epi16(a, b), simde_x_mm_deinterleaveodd_epi16(a, b));
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsubs_epi16(a, b) simde_mm_hsubs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_hsubs_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_hsubs_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int16x4x2_t t = vuzp_s16(a_.neon_i16, b_.neon_i16);
      r_.neon_i16 = vqsub_s16(t.val[0], t.val[1]);
    #else
      for (size_t i = 0 ; i < ((sizeof(r_.i16) / sizeof(r_.i16[0])) / 2) ; i++) {
        r_.i16[  i  ] = simde_math_subs_i16(a_.i16[i * 2], a_.i16[(i * 2) + 1]);
        r_.i16[i + 2] = simde_math_subs_i16(b_.i16[i * 2], b_.i16[(i * 2) + 1]);
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsubs_pi16(a, b) simde_mm_hsubs_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_maddubs_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_maddubs_epi16(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      /* Zero extend a */
      int16x8_t a_odd = vreinterpretq_s16_u16(vshrq_n_u16(a_.neon_u16, 8));
      int16x8_t a_even = vreinterpretq_s16_u16(vbicq_u16(a_.neon_u16, vdupq_n_u16(0xff00)));

      /* Sign extend by shifting left then shifting right. */
      int16x8_t b_even = vshrq_n_s16(vshlq_n_s16(b_.neon_i16, 8), 8);
      int16x8_t b_odd = vshrq_n_s16(b_.neon_i16, 8);

      /* multiply */
      int16x8_t prod1 = vmulq_s16(a_even, b_even);
      int16x8_t prod2 = vmulq_s16(a_odd, b_odd);

      /* saturated add */
      r_.neon_i16 = vqaddq_s16(prod1, prod2);
    #else
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        const int idx = HEDLEY_STATIC_CAST(int, i) << 1;
        int32_t ts =
          (HEDLEY_STATIC_CAST(int16_t, a_.u8[  idx  ]) * HEDLEY_STATIC_CAST(int16_t, b_.i8[  idx  ])) +
          (HEDLEY_STATIC_CAST(int16_t, a_.u8[idx + 1]) * HEDLEY_STATIC_CAST(int16_t, b_.i8[idx + 1]));
        r_.i16[i] = (ts > INT16_MIN) ? ((ts < INT16_MAX) ? HEDLEY_STATIC_CAST(int16_t, ts) : INT16_MAX) : INT16_MIN;
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_maddubs_epi16(a, b) simde_mm_maddubs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_maddubs_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_maddubs_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      int16x8_t ai = vreinterpretq_s16_u16(vmovl_u8(a_.neon_u8));
      int16x8_t bi = vmovl_s8(b_.neon_i8);
      int16x8_t p = vmulq_s16(ai, bi);
      int16x4_t l = vget_low_s16(p);
      int16x4_t h = vget_high_s16(p);
      r_.neon_i16 = vqadd_s16(vuzp1_s16(l, h), vuzp2_s16(l, h));
    #else
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        const int idx = HEDLEY_STATIC_CAST(int, i) << 1;
        int32_t ts =
          (HEDLEY_STATIC_CAST(int16_t, a_.u8[  idx  ]) * HEDLEY_STATIC_CAST(int16_t, b_.i8[  idx  ])) +
          (HEDLEY_STATIC_CAST(int16_t, a_.u8[idx + 1]) * HEDLEY_STATIC_CAST(int16_t, b_.i8[idx + 1]));
        r_.i16[i] = (ts > INT16_MIN) ? ((ts < INT16_MAX) ? HEDLEY_STATIC_CAST(int16_t, ts) : INT16_MAX) : INT16_MIN;
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_maddubs_pi16(a, b) simde_mm_maddubs_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mulhrs_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_mulhrs_epi16(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      /* Multiply */
      int32x4_t mul_lo = vmull_s16(vget_low_s16(a_.neon_i16),
                                  vget_low_s16(b_.neon_i16));
      int32x4_t mul_hi = vmull_s16(vget_high_s16(a_.neon_i16),
                                  vget_high_s16(b_.neon_i16));

      /* Rounding narrowing shift right
       * narrow = (int16_t)((mul + 16384) >> 15); */
      int16x4_t narrow_lo = vrshrn_n_s32(mul_lo, 15);
      int16x4_t narrow_hi = vrshrn_n_s32(mul_hi, 15);

      /* Join together */
      r_.neon_i16 = vcombine_s16(narrow_lo, narrow_hi);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
        v128_t __lo = wasm_i32x4_mul(wasm_i32x4_extend_low_i16x8(a_.wasm_v128), wasm_i32x4_extend_low_i16x8(b_.wasm_v128));
        v128_t __hi = wasm_i32x4_mul(wasm_i32x4_extend_high_i16x8(a_.wasm_v128), wasm_i32x4_extend_high_i16x8(b_.wasm_v128));
        const v128_t __inc = wasm_i32x4_splat(0x4000);
        __lo = wasm_i32x4_add(__lo, __inc);
        __hi = wasm_i32x4_add(__hi, __inc);
        __lo = wasm_i32x4_add(__lo, __lo);
        __hi = wasm_i32x4_add(__hi, __hi);
        r_.wasm_v128 = wasm_i16x8_shuffle(__lo, __hi, 1, 3, 5, 7, 9, 11, 13, 15);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, (((HEDLEY_STATIC_CAST(int32_t, a_.i16[i]) * HEDLEY_STATIC_CAST(int32_t, b_.i16[i])) + 0x4000) >> 15));
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_mulhrs_epi16(a, b) simde_mm_mulhrs_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_mulhrs_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_mulhrs_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      /* Multiply */
      int32x4_t mul = vmull_s16(a_.neon_i16, b_.neon_i16);

      /* Rounding narrowing shift right
       * narrow = (int16_t)((mul + 16384) >> 15); */
      int16x4_t narrow = vrshrn_n_s32(mul, 15);

      /* Join together */
      r_.neon_i16 = narrow;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = HEDLEY_STATIC_CAST(int16_t, (((HEDLEY_STATIC_CAST(int32_t, a_.i16[i]) * HEDLEY_STATIC_CAST(int32_t, b_.i16[i])) + 0x4000) >> 15));
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_mulhrs_pi16(a, b) simde_mm_mulhrs_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_sign_epi8 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_sign_epi8(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint8x16_t aneg_mask = vreinterpretq_u8_s8(vshrq_n_s8(b_.neon_i8, 7));
      uint8x16_t bnz_mask;
      #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        bnz_mask = vceqzq_s8(b_.neon_i8);
      #else
        bnz_mask = vceqq_s8(b_.neon_i8, vdupq_n_s8(0));
      #endif
      bnz_mask = vmvnq_u8(bnz_mask);

      r_.neon_i8 = vbslq_s8(aneg_mask, vnegq_s8(a_.neon_i8), vandq_s8(a_.neon_i8, vreinterpretq_s8_u8(bnz_mask)));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      simde__m128i mask = wasm_i8x16_shr(b_.wasm_v128, 7);
      simde__m128i zeromask = simde_mm_cmpeq_epi8(b_.wasm_v128, simde_mm_setzero_si128());
      r_.wasm_v128 = simde_mm_andnot_si128(zeromask, simde_mm_xor_si128(simde_mm_add_epi8(a_.wasm_v128, mask), mask));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = (b_.i8[i] < 0) ? (- a_.i8[i]) : ((b_.i8[i] != 0) ? (a_.i8[i]) : INT8_C(0));
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_sign_epi8(a, b) simde_mm_sign_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_sign_epi16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_sign_epi16(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint16x8_t aneg_mask = vreinterpretq_u16_s16(vshrq_n_s16(b_.neon_i16, 15));
      uint16x8_t bnz_mask;
      #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        bnz_mask = vceqzq_s16(b_.neon_i16);
      #else
        bnz_mask = vceqq_s16(b_.neon_i16, vdupq_n_s16(0));
      #endif
      bnz_mask = vmvnq_u16(bnz_mask);

      r_.neon_i16 = vbslq_s16(aneg_mask, vnegq_s16(a_.neon_i16), vandq_s16(a_.neon_i16, vreinterpretq_s16_u16(bnz_mask)));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      simde__m128i mask = simde_mm_srai_epi16(b_.wasm_v128, 15);
      simde__m128i zeromask = simde_mm_cmpeq_epi16(b_.wasm_v128, simde_mm_setzero_si128());
      r_.wasm_v128 = simde_mm_andnot_si128(zeromask, simde_mm_xor_si128(simde_mm_add_epi16(a_.wasm_v128, mask), mask));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (b_.i16[i] < 0) ? (- a_.i16[i]) : ((b_.i16[i] != 0) ? (a_.i16[i]) : INT16_C(0));
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_sign_epi16(a, b) simde_mm_sign_epi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_sign_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE)
    return _mm_sign_epi32(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint32x4_t aneg_mask = vreinterpretq_u32_s32(vshrq_n_s32(b_.neon_i32, 31));
      uint32x4_t bnz_mask;
      #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        bnz_mask = vceqzq_s32(b_.neon_i32);
      #else
        bnz_mask = vceqq_s32(b_.neon_i32, vdupq_n_s32(0));
      #endif
      bnz_mask = vmvnq_u32(bnz_mask);

      r_.neon_i32 = vbslq_s32(aneg_mask, vnegq_s32(a_.neon_i32), vandq_s32(a_.neon_i32, vreinterpretq_s32_u32(bnz_mask)));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      simde__m128i mask = simde_mm_srai_epi32(b_.wasm_v128, 31);
      simde__m128i zeromask = simde_mm_cmpeq_epi32(b_.wasm_v128, simde_mm_setzero_si128());
      r_.wasm_v128 = simde_mm_andnot_si128(zeromask, simde_mm_xor_si128(simde_mm_add_epi32(a_.wasm_v128, mask), mask));
    #else
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = (b_.i32[i] < 0) ? (- a_.i32[i]) : ((b_.i32[i] != 0) ? (a_.i32[i]) : INT32_C(0));
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_sign_epi32(a, b) simde_mm_sign_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_sign_pi8 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_sign_pi8(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint8x8_t aneg_mask = vreinterpret_u8_s8(vshr_n_s8(b_.neon_i8, 7));
      uint8x8_t bnz_mask;
      #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        bnz_mask = vceqz_s8(b_.neon_i8);
      #else
        bnz_mask = vceq_s8(b_.neon_i8, vdup_n_s8(0));
      #endif
      bnz_mask = vmvn_u8(bnz_mask);

      r_.neon_i8 = vbsl_s8(aneg_mask, vneg_s8(a_.neon_i8), vand_s8(a_.neon_i8, vreinterpret_s8_u8(bnz_mask)));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = (b_.i8[i] < 0) ? (- a_.i8[i]) : ((b_.i8[i] != 0) ? (a_.i8[i]) : INT8_C(0));
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_sign_pi8(a, b) simde_mm_sign_pi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_sign_pi16 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_sign_pi16(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint16x4_t aneg_mask = vreinterpret_u16_s16(vshr_n_s16(b_.neon_i16, 15));
      uint16x4_t bnz_mask;
      #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        bnz_mask = vceqz_s16(b_.neon_i16);
      #else
        bnz_mask = vceq_s16(b_.neon_i16, vdup_n_s16(0));
      #endif
      bnz_mask = vmvn_u16(bnz_mask);

      r_.neon_i16 = vbsl_s16(aneg_mask, vneg_s16(a_.neon_i16), vand_s16(a_.neon_i16, vreinterpret_s16_u16(bnz_mask)));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = (b_.i16[i] < 0) ? (- a_.i16[i]) : ((b_.i16[i] > 0) ? (a_.i16[i]) : INT16_C(0));
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_sign_pi16(a, b) simde_mm_sign_pi16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m64
simde_mm_sign_pi32 (simde__m64 a, simde__m64 b) {
  #if defined(SIMDE_X86_SSSE3_NATIVE) && defined(SIMDE_X86_MMX_NATIVE)
    return _mm_sign_pi32(a, b);
  #else
    simde__m64_private
      r_,
      a_ = simde__m64_to_private(a),
      b_ = simde__m64_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint32x2_t aneg_mask = vreinterpret_u32_s32(vshr_n_s32(b_.neon_i32, 31));
      uint32x2_t bnz_mask;
      #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        bnz_mask = vceqz_s32(b_.neon_i32);
      #else
        bnz_mask = vceq_s32(b_.neon_i32, vdup_n_s32(0));
      #endif
      bnz_mask = vmvn_u32(bnz_mask);

      r_.neon_i32 = vbsl_s32(aneg_mask, vneg_s32(a_.neon_i32), vand_s32(a_.neon_i32, vreinterpret_s32_u32(bnz_mask)));
    #else
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = (b_.i32[i] < 0) ? (- a_.i32[i]) : ((b_.i32[i] > 0) ? (a_.i32[i]) : INT32_C(0));
      }
    #endif

    return simde__m64_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_sign_pi32(a, b) simde_mm_sign_pi32(a, b)
#endif

SIMDE_END_DECLS_

HEDLEY_DIAGNOSTIC_POP

#endif /* !defined(SIMDE_X86_SSE2_H) */
