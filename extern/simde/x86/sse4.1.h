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

#include "sse.h"
#if !defined(SIMDE_X86_SSE4_1_H)
#define SIMDE_X86_SSE4_1_H

#include "ssse3.h"

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DISABLE_UNWANTED_DIAGNOSTICS
SIMDE_BEGIN_DECLS_

#if !defined(SIMDE_X86_SSE4_1_NATIVE) && defined(SIMDE_ENABLE_NATIVE_ALIASES)
#  define SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_blend_epi16 (simde__m128i a, simde__m128i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
    r_.u16[i] = ((imm8 >> i) & 1) ? b_.u16[i] : a_.u16[i];
  }

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #define simde_mm_blend_epi16(a, b, imm8) _mm_blend_epi16(a, b, imm8)
#elif defined(SIMDE_SHUFFLE_VECTOR_)
  #define simde_mm_blend_epi16(a, b, imm8) \
    (__extension__ ({ \
      simde__m128i_private \
        simde_mm_blend_epi16_a_ = simde__m128i_to_private(a), \
        simde_mm_blend_epi16_b_ = simde__m128i_to_private(b), \
        simde_mm_blend_epi16_r_; \
      \
      simde_mm_blend_epi16_r_.i16 = \
        SIMDE_SHUFFLE_VECTOR_( \
          16, 16, \
          simde_mm_blend_epi16_a_.i16, \
          simde_mm_blend_epi16_b_.i16, \
          ((imm8) & (1 << 0)) ?  8 : 0, \
          ((imm8) & (1 << 1)) ?  9 : 1, \
          ((imm8) & (1 << 2)) ? 10 : 2, \
          ((imm8) & (1 << 3)) ? 11 : 3, \
          ((imm8) & (1 << 4)) ? 12 : 4, \
          ((imm8) & (1 << 5)) ? 13 : 5, \
          ((imm8) & (1 << 6)) ? 14 : 6, \
          ((imm8) & (1 << 7)) ? 15 : 7  \
        ); \
      \
      simde__m128i_from_private(simde_mm_blend_epi16_r_); \
    }))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_blend_epi16
  #define _mm_blend_epi16(a, b, imm8) simde_mm_blend_epi16(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_blend_pd (simde__m128d a, simde__m128d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 3)  {
  simde__m128d_private
    r_,
    a_ = simde__m128d_to_private(a),
    b_ = simde__m128d_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
    r_.f64[i] = ((imm8 >> i) & 1) ? b_.f64[i] : a_.f64[i];
  }
  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #define simde_mm_blend_pd(a, b, imm8) _mm_blend_pd(a, b, imm8)
#elif defined(SIMDE_SHUFFLE_VECTOR_)
  #define simde_mm_blend_pd(a, b, imm8) \
    (__extension__ ({ \
      simde__m128d_private \
        simde_mm_blend_pd_a_ = simde__m128d_to_private(a), \
        simde_mm_blend_pd_b_ = simde__m128d_to_private(b), \
        simde_mm_blend_pd_r_; \
      \
      simde_mm_blend_pd_r_.f64 = \
        SIMDE_SHUFFLE_VECTOR_( \
          64, 16, \
          simde_mm_blend_pd_a_.f64, \
          simde_mm_blend_pd_b_.f64, \
          ((imm8) & (1 << 0)) ?  2 : 0, \
          ((imm8) & (1 << 1)) ?  3 : 1  \
        ); \
      \
      simde__m128d_from_private(simde_mm_blend_pd_r_); \
    }))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_blend_pd
  #define _mm_blend_pd(a, b, imm8) simde_mm_blend_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_blend_ps (simde__m128 a, simde__m128 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15)  {
  simde__m128_private
    r_,
    a_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
    r_.f32[i] = ((imm8 >> i) & 1) ? b_.f32[i] : a_.f32[i];
  }
  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
#  define simde_mm_blend_ps(a, b, imm8) _mm_blend_ps(a, b, imm8)
#elif defined(SIMDE_SHUFFLE_VECTOR_)
  #define simde_mm_blend_ps(a, b, imm8) \
    (__extension__ ({ \
      simde__m128_private \
        simde_mm_blend_ps_a_ = simde__m128_to_private(a), \
        simde_mm_blend_ps_b_ = simde__m128_to_private(b), \
        simde_mm_blend_ps_r_; \
      \
      simde_mm_blend_ps_r_.f32 = \
        SIMDE_SHUFFLE_VECTOR_( \
          32, 16, \
          simde_mm_blend_ps_a_.f32, \
          simde_mm_blend_ps_b_.f32, \
          ((imm8) & (1 << 0)) ? 4 : 0, \
          ((imm8) & (1 << 1)) ? 5 : 1, \
          ((imm8) & (1 << 2)) ? 6 : 2, \
          ((imm8) & (1 << 3)) ? 7 : 3  \
        ); \
      \
      simde__m128_from_private(simde_mm_blend_ps_r_); \
    }))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_blend_ps
  #define _mm_blend_ps(a, b, imm8) simde_mm_blend_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_blendv_epi8 (simde__m128i a, simde__m128i b, simde__m128i mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_blendv_epi8(a, b, mask);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i m = _mm_cmpgt_epi8(_mm_setzero_si128(), mask);
    return _mm_xor_si128(_mm_subs_epu8(_mm_xor_si128(a, b), m), b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b),
      mask_ = simde__m128i_to_private(mask);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      /* Use a signed shift right to create a mask with the sign bit */
      mask_.neon_i8 = vshrq_n_s8(mask_.neon_i8, 7);
      r_.neon_i8 = vbslq_s8(mask_.neon_u8, b_.neon_i8, a_.neon_i8);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_i8x16_shr(mask_.wasm_v128, 7);
      r_.wasm_v128 = wasm_v128_bitselect(b_.wasm_v128, a_.wasm_v128, m);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i8 = vec_sel(a_.altivec_i8, b_.altivec_i8, vec_cmplt(mask_.altivec_i8, vec_splat_s8(0)));
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      /* https://software.intel.com/en-us/forums/intel-c-compiler/topic/850087 */
      #if defined(HEDLEY_INTEL_VERSION_CHECK)
        __typeof__(mask_.i8) z = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        mask_.i8 = HEDLEY_REINTERPRET_CAST(__typeof__(mask_.i8), mask_.i8 < z);
      #else
        mask_.i8 >>= (CHAR_BIT * sizeof(mask_.i8[0])) - 1;
      #endif

      r_.i8 = (mask_.i8 & b_.i8) | (~mask_.i8 & a_.i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        int8_t m = mask_.i8[i] >> 7;
        r_.i8[i] = (m & b_.i8[i]) | (~m & a_.i8[i]);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_blendv_epi8
  #define _mm_blendv_epi8(a, b, mask) simde_mm_blendv_epi8(a, b, mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_blendv_epi16 (simde__m128i a, simde__m128i b, simde__m128i mask) {
  #if defined(SIMDE_X86_SSE2_NATIVE)
    mask = simde_mm_srai_epi16(mask, 15);
    return simde_mm_or_si128(simde_mm_and_si128(mask, b), simde_mm_andnot_si128(mask, a));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b),
      mask_ = simde__m128i_to_private(mask);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      mask_ = simde__m128i_to_private(simde_mm_cmplt_epi16(mask, simde_mm_setzero_si128()));
      r_.neon_i16 = vbslq_s16(mask_.neon_u16, b_.neon_i16, a_.neon_i16);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i16 = vec_sel(a_.altivec_i16, b_.altivec_i16, vec_cmplt(mask_.altivec_i16, vec_splat_s16(0)));
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      #if defined(HEDLEY_INTEL_VERSION_CHECK)
        __typeof__(mask_.i16) z = { 0, 0, 0, 0, 0, 0, 0, 0 };
        mask_.i16 = mask_.i16 < z;
      #else
        mask_.i16 >>= (CHAR_BIT * sizeof(mask_.i16[0])) - 1;
      #endif

      r_.i16 = (mask_.i16 & b_.i16) | (~mask_.i16 & a_.i16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        int16_t m = mask_.i16[i] >> 15;
        r_.i16[i] = (m & b_.i16[i]) | (~m & a_.i16[i]);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_blendv_epi32 (simde__m128i a, simde__m128i b, simde__m128i mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_castps_si128(_mm_blendv_ps(_mm_castsi128_ps(a), _mm_castsi128_ps(b), _mm_castsi128_ps(mask)));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b),
      mask_ = simde__m128i_to_private(mask);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      mask_ = simde__m128i_to_private(simde_mm_cmplt_epi32(mask, simde_mm_setzero_si128()));
      r_.neon_i32 = vbslq_s32(mask_.neon_u32, b_.neon_i32, a_.neon_i32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_i32x4_shr(mask_.wasm_v128, 31);
      r_.wasm_v128 = wasm_v128_or(wasm_v128_and(b_.wasm_v128, m), wasm_v128_andnot(a_.wasm_v128, m));
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i32 = vec_sel(a_.altivec_i32, b_.altivec_i32, vec_cmplt(mask_.altivec_i32, vec_splat_s32(0)));
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      #if defined(HEDLEY_INTEL_VERSION_CHECK)
        __typeof__(mask_.i32) z = { 0, 0, 0, 0 };
        mask_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(mask_.i32), mask_.i32 < z);
      #else
        mask_.i32 >>= (CHAR_BIT * sizeof(mask_.i32[0])) - 1;
      #endif

      r_.i32 = (mask_.i32 & b_.i32) | (~mask_.i32 & a_.i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        int32_t m = mask_.i32[i] >> 31;
        r_.i32[i] = (m & b_.i32[i]) | (~m & a_.i32[i]);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_blendv_epi64 (simde__m128i a, simde__m128i b, simde__m128i mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_castpd_si128(_mm_blendv_pd(_mm_castsi128_pd(a), _mm_castsi128_pd(b), _mm_castsi128_pd(mask)));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b),
      mask_ = simde__m128i_to_private(mask);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      mask_.neon_u64 = vcltq_s64(mask_.neon_i64, vdupq_n_s64(UINT64_C(0)));
      r_.neon_i64 = vbslq_s64(mask_.neon_u64, b_.neon_i64, a_.neon_i64);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_i64x2_shr(mask_.wasm_v128, 63);
      r_.wasm_v128 = wasm_v128_or(wasm_v128_and(b_.wasm_v128, m), wasm_v128_andnot(a_.wasm_v128, m));
    #elif (defined(SIMDE_POWER_ALTIVEC_P8_NATIVE) && !defined(SIMDE_BUG_CLANG_46770)) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i64 = vec_sel(a_.altivec_i64, b_.altivec_i64, vec_cmplt(mask_.altivec_i64, vec_splats(HEDLEY_STATIC_CAST(signed long long, 0))));
    #elif defined(SIMDE_POWER_ALTIVEC_P8_NATIVE)
      SIMDE_POWER_ALTIVEC_VECTOR(signed long long) selector = vec_sra(mask_.altivec_i64, vec_splats(HEDLEY_STATIC_CAST(unsigned long long, 63)));
      r_.altivec_i32 = vec_sel(a_.altivec_i32, b_.altivec_i32, HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(unsigned int), selector));
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      #if defined(HEDLEY_INTEL_VERSION_CHECK)
        __typeof__(mask_.i64) z = { 0, 0 };
        mask_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(mask_.i64), mask_.i64 < z);
      #else
        mask_.i64 >>= (CHAR_BIT * sizeof(mask_.i64[0])) - 1;
      #endif

    r_.i64 = (mask_.i64 & b_.i64) | (~mask_.i64 & a_.i64);
  #else
    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
      int64_t m = mask_.i64[i] >> 63;
      r_.i64[i] = (m & b_.i64[i]) | (~m & a_.i64[i]);
    }
  #endif

    return simde__m128i_from_private(r_);
  #endif
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_blendv_pd (simde__m128d a, simde__m128d b, simde__m128d mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_blendv_pd(a, b, mask);
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    v128_t m_ = wasm_i64x2_shr(HEDLEY_REINTERPRET_CAST(v128_t, mask), 63);
    return simde__m128d_from_wasm_v128(wasm_v128_bitselect(simde__m128d_to_wasm_v128(b), simde__m128d_to_wasm_v128(a), m_));
  #else
    return simde_mm_castsi128_pd(simde_x_mm_blendv_epi64(simde_mm_castpd_si128(a), simde_mm_castpd_si128(b), simde_mm_castpd_si128(mask)));
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_blendv_pd
  #define _mm_blendv_pd(a, b, mask) simde_mm_blendv_pd(a, b, mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_blendv_ps (simde__m128 a, simde__m128 b, simde__m128 mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_blendv_ps(a, b, mask);
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    v128_t m_ = wasm_i32x4_shr(HEDLEY_REINTERPRET_CAST(v128_t, mask), 31);
    return simde__m128d_from_wasm_v128(wasm_v128_bitselect(simde__m128d_to_wasm_v128(b), simde__m128d_to_wasm_v128(a), m_));
  #else
    return simde_mm_castsi128_ps(simde_x_mm_blendv_epi32(simde_mm_castps_si128(a), simde_mm_castps_si128(b), simde_mm_castps_si128(mask)));
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_blendv_ps
  #define _mm_blendv_ps(a, b, mask) simde_mm_blendv_ps(a, b, mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_round_pd (simde__m128d a, int rounding)
    SIMDE_REQUIRE_CONSTANT_RANGE(rounding, 0, 15) {
  simde__m128d_private
    r_,
    a_ = simde__m128d_to_private(a);

  /* For architectures which lack a current direction SIMD instruction. */
  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    if ((rounding & 7) == SIMDE_MM_FROUND_CUR_DIRECTION)
      rounding = HEDLEY_STATIC_CAST(int, SIMDE_MM_GET_ROUNDING_MODE()) << 13;
  #endif

  switch (rounding & ~SIMDE_MM_FROUND_NO_EXC) {
    case SIMDE_MM_FROUND_CUR_DIRECTION:
      #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
        r_.altivec_f64 = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(double), vec_round(a_.altivec_f64));
      #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        r_.neon_f64 = vrndiq_f64(a_.neon_f64);
      #elif defined(SIMDE_WASM_SIMD128_NATIVE)
        r_.wasm_v128 = wasm_f64x2_nearest(a_.wasm_v128);
      #elif defined(simde_math_nearbyint)
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_nearbyint(a_.f64[i]);
        }
      #else
        HEDLEY_UNREACHABLE_RETURN(simde_mm_undefined_pd());
      #endif
      break;

    case SIMDE_MM_FROUND_TO_NEAREST_INT:
      #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
        r_.altivec_f64 = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(double), vec_round(a_.altivec_f64));
      #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        r_.neon_f64 = vrndaq_f64(a_.neon_f64);
      #elif defined(SIMDE_WASM_SIMD128_NATIVE)
        r_.wasm_v128 = wasm_f64x2_nearest(a_.wasm_v128);
      #elif defined(simde_math_roundeven)
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_roundeven(a_.f64[i]);
        }
      #else
        HEDLEY_UNREACHABLE_RETURN(simde_mm_undefined_pd());
      #endif
      break;

    case SIMDE_MM_FROUND_TO_NEG_INF:
      #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
        r_.altivec_f64 = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(double), vec_floor(a_.altivec_f64));
      #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        r_.neon_f64 = vrndmq_f64(a_.neon_f64);
      #elif defined(SIMDE_WASM_SIMD128_NATIVE)
        r_.wasm_v128 = wasm_f64x2_floor(a_.wasm_v128);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_floor(a_.f64[i]);
        }
      #endif
      break;

    case SIMDE_MM_FROUND_TO_POS_INF:
      #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
        r_.altivec_f64 = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(double), vec_ceil(a_.altivec_f64));
      #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        r_.neon_f64 = vrndpq_f64(a_.neon_f64);
      #elif defined(SIMDE_WASM_SIMD128_NATIVE)
        r_.wasm_v128 = wasm_f64x2_ceil(a_.wasm_v128);
      #elif defined(simde_math_ceil)
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_ceil(a_.f64[i]);
        }
      #else
        HEDLEY_UNREACHABLE_RETURN(simde_mm_undefined_pd());
      #endif
      break;

    case SIMDE_MM_FROUND_TO_ZERO:
      #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
        r_.altivec_f64 = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(double), vec_trunc(a_.altivec_f64));
      #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
        r_.neon_f64 = vrndq_f64(a_.neon_f64);
      #elif defined(SIMDE_WASM_SIMD128_NATIVE)
        r_.wasm_v128 = wasm_f64x2_trunc(a_.wasm_v128);
      #else
        SIMDE_VECTORIZE
        for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
          r_.f64[i] = simde_math_trunc(a_.f64[i]);
        }
      #endif
      break;

    default:
      HEDLEY_UNREACHABLE_RETURN(simde_mm_undefined_pd());
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #define simde_mm_round_pd(a, rounding) _mm_round_pd(a, rounding)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_round_pd
  #define _mm_round_pd(a, rounding) simde_mm_round_pd(a, rounding)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_ceil_pd (simde__m128d a) {
  #if defined(SIMDE_WASM_SIMD128_NATIVE)
    return simde__m128d_from_wasm_v128(wasm_f64x2_ceil(simde__m128d_to_wasm_v128(a)));
  #endif
  return simde_mm_round_pd(a, SIMDE_MM_FROUND_TO_POS_INF);
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_ceil_pd
  #define _mm_ceil_pd(a) simde_mm_ceil_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_ceil_ps (simde__m128 a) {
  #if defined(SIMDE_WASM_SIMD128_NATIVE)
    return simde__m128_from_wasm_v128(wasm_f32x4_ceil(simde__m128_to_wasm_v128(a)));
  #endif
  return simde_mm_round_ps(a, SIMDE_MM_FROUND_TO_POS_INF);
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_ceil_ps
  #define _mm_ceil_ps(a) simde_mm_ceil_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_ceil_sd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_ceil_sd(a, b);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b);

    #if defined(simde_math_ceilf)
      r_ = simde__m128d_to_private(simde_mm_set_pd(a_.f64[1], simde_math_ceil(b_.f64[0])));
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_ceil_sd
  #define _mm_ceil_sd(a, b) simde_mm_ceil_sd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_ceil_ss (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_ceil_ss(a, b);
  #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_FAST_EXCEPTIONS)
    return simde_mm_move_ss(a, simde_mm_ceil_ps(b));
  #elif (SIMDE_NATURAL_VECTOR_SIZE > 0)
    return simde_mm_move_ss(a, simde_mm_ceil_ps(simde_x_mm_broadcastlow_ps(b)));
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b);

    #if defined(simde_math_ceilf)
      r_ = simde__m128_to_private(simde_mm_set_ps(a_.f32[3], a_.f32[2], a_.f32[1], simde_math_ceilf(b_.f32[0])));
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_ceil_ss
  #define _mm_ceil_ss(a, b) simde_mm_ceil_ss(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cmpeq_epi64 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cmpeq_epi64(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_u64 = vceqq_u64(a_.neon_u64, b_.neon_u64);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      /* (a == b) -> (a_lo == b_lo) && (a_hi == b_hi) */
      uint32x4_t cmp = vceqq_u32(a_.neon_u32, b_.neon_u32);
      uint32x4_t swapped = vrev64q_u32(cmp);
      r_.neon_u32 = vandq_u32(cmp, swapped);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), a_.i64 == b_.i64);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
      r_.altivec_i64 = HEDLEY_REINTERPRET_CAST(SIMDE_POWER_ALTIVEC_VECTOR(signed long long), vec_cmpeq(a_.altivec_i64, b_.altivec_i64));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u64) / sizeof(r_.u64[0])) ; i++) {
        r_.u64[i] = (a_.u64[i] == b_.u64[i]) ? ~UINT64_C(0) : UINT64_C(0);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cmpeq_epi64
  #define _mm_cmpeq_epi64(a, b) simde_mm_cmpeq_epi64(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepi8_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepi8_epi16(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_srai_epi16(_mm_unpacklo_epi8(a, a), 8);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int8x16_t s8x16 = a_.neon_i8;                   /* xxxx xxxx xxxx DCBA */
      int16x8_t s16x8 = vmovl_s8(vget_low_s8(s8x16)); /* 0x0x 0x0x 0D0C 0B0A */
      r_.neon_i16 = s16x8;
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i16x8_extend_low_i8x16(a_.wasm_v128);
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && defined(SIMDE_VECTOR_SCALAR) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      r_.i16 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i16), SIMDE_SHUFFLE_VECTOR_(8, 16, a_.i8, a_.i8,
          -1,  0, -1,  1, -1,  2,  -1,  3,
          -1,  4, -1,  5, -1,  6,  -1,  7));
      r_.i16 >>= 8;
    #elif defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i16, a_.m64_private[0].i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a_.i8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepi8_epi16
  #define _mm_cvtepi8_epi16(a) simde_mm_cvtepi8_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepi8_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepi8_epi32(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i tmp = _mm_unpacklo_epi8(a, a);
    tmp = _mm_unpacklo_epi16(tmp, tmp);
    return _mm_srai_epi32(tmp, 24);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int8x16_t s8x16 = a_.neon_i8;                     /* xxxx xxxx xxxx DCBA */
      int16x8_t s16x8 = vmovl_s8(vget_low_s8(s8x16));   /* 0x0x 0x0x 0D0C 0B0A */
      int32x4_t s32x4 = vmovl_s16(vget_low_s16(s16x8)); /* 000D 000C 000B 000A */
      r_.neon_i32 = s32x4;
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_extend_low_i16x8(wasm_i16x8_extend_low_i8x16(a_.wasm_v128));
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && defined(SIMDE_VECTOR_SCALAR) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), SIMDE_SHUFFLE_VECTOR_(8, 16, a_.i8, a_.i8,
          -1, -1, -1,  0, -1, -1,  -1,  1,
          -1, -1, -1,  2, -1, -1,  -1,  3));
      r_.i32 >>= 24;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepi8_epi32
  #define _mm_cvtepi8_epi32(a) simde_mm_cvtepi8_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepi8_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepi8_epi64(a);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int8x16_t s8x16 = a_.neon_i8;                     /* xxxx xxxx xxxx xxBA */
      int16x8_t s16x8 = vmovl_s8(vget_low_s8(s8x16));   /* 0x0x 0x0x 0x0x 0B0A */
      int32x4_t s32x4 = vmovl_s16(vget_low_s16(s16x8)); /* 000x 000x 000B 000A */
      int64x2_t s64x2 = vmovl_s32(vget_low_s32(s32x4)); /* 0000 000B 0000 000A */
      r_.neon_i64 = s64x2;
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t extra = wasm_i32x4_extend_low_i16x8(wasm_i16x8_extend_low_i8x16(a_.wasm_v128));
      v128_t sign = wasm_i32x4_gt(wasm_i64x2_const(0, 0), extra);
      r_.wasm_v128 = wasm_i32x4_shuffle(extra, sign, 0, 4, 1, 5);
    #elif (!defined(SIMDE_ARCH_X86) && !defined(SIMDE_ARCH_AMD64)) && defined(SIMDE_SHUFFLE_VECTOR_) && defined(SIMDE_VECTOR_SCALAR) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      /* Disabled on x86 due to lack of 64-bit arithmetic shift until
       * until AVX-512 (at which point we would be using the native
       * _mm_cvtepi_epi64 anyways). */
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), SIMDE_SHUFFLE_VECTOR_(8, 16, a_.i8, a_.i8,
          -1, -1, -1, -1, -1, -1,  -1,  0,
          -1, -1, -1, -1, -1, -1,  -1,  1));
      r_.i64 >>= 56;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepi8_epi64
  #define _mm_cvtepi8_epi64(a) simde_mm_cvtepi8_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepu8_epi16 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepu8_epi16(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_unpacklo_epi8(a, _mm_setzero_si128());
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint8x16_t u8x16 = a_.neon_u8;                   /* xxxx xxxx xxxx DCBA */
      uint16x8_t u16x8 = vmovl_u8(vget_low_u8(u8x16)); /* 0x0x 0x0x 0D0C 0B0A */
      r_.neon_u16 = u16x8;
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u16x8_extend_low_u8x16(a_.wasm_v128);
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      __typeof__(r_.i8) z = { 0, };
      r_.i16 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i16), SIMDE_SHUFFLE_VECTOR_(8, 16, a_.i8, z,
          0, 16, 1, 17, 2, 18, 3, 19,
          4, 20, 5, 21, 6, 22, 7, 23));
    #elif defined(SIMDE_CONVERT_VECTOR_) && !defined(SIMDE_BUG_CLANG_45541) && (!defined(SIMDE_ARCH_POWER) || !defined(__clang__))
      SIMDE_CONVERT_VECTOR_(r_.i16, a_.m64_private[0].u8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        r_.i16[i] = a_.u8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepu8_epi16
  #define _mm_cvtepu8_epi16(a) simde_mm_cvtepu8_epi16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepu8_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepu8_epi32(a);
  #elif defined(SIMDE_X86_SSSE3_NATIVE)
    __m128i s = _mm_set_epi8(
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x03),
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x02),
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x01),
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x00));
    return _mm_shuffle_epi8(a, s);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i z = _mm_setzero_si128();
    return _mm_unpacklo_epi16(_mm_unpacklo_epi8(a, z), z);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint8x16_t u8x16 = a_.neon_u8;                     /* xxxx xxxx xxxx DCBA */
      uint16x8_t u16x8 = vmovl_u8(vget_low_u8(u8x16));   /* 0x0x 0x0x 0D0C 0B0A */
      uint32x4_t u32x4 = vmovl_u16(vget_low_u16(u16x8)); /* 000D 000C 000B 000A */
      r_.neon_u32 = u32x4;
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u32x4_extend_low_u16x8(wasm_u16x8_extend_low_u8x16(a_.wasm_v128));
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      __typeof__(r_.i8) z = { 0, };
      r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), SIMDE_SHUFFLE_VECTOR_(8, 16, a_.i8, z,
          0, 17, 18, 19, 1, 21, 22, 23,
          2, 25, 26, 27, 3, 29, 30, 31));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.u8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepu8_epi32
  #define _mm_cvtepu8_epi32(a) simde_mm_cvtepu8_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepu8_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepu8_epi64(a);
  #elif defined(SIMDE_X86_SSSE3_NATIVE)
    __m128i s = _mm_set_epi8(
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80),
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x01),
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80),
        HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x80), HEDLEY_STATIC_CAST(char, 0x00));
    return _mm_shuffle_epi8(a, s);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i z = _mm_setzero_si128();
    return _mm_unpacklo_epi32(_mm_unpacklo_epi16(_mm_unpacklo_epi8(a, z), z), z);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint8x16_t u8x16 = a_.neon_u8;                     /* xxxx xxxx xxxx xxBA */
      uint16x8_t u16x8 = vmovl_u8(vget_low_u8(u8x16));   /* 0x0x 0x0x 0x0x 0B0A */
      uint32x4_t u32x4 = vmovl_u16(vget_low_u16(u16x8)); /* 000x 000x 000B 000A */
      uint64x2_t u64x2 = vmovl_u32(vget_low_u32(u32x4)); /* 0000 000B 0000 000A */
      r_.neon_u64 = u64x2;
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      __typeof__(r_.i8) z = { 0, };
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), SIMDE_SHUFFLE_VECTOR_(8, 16, a_.i8, z,
          0, 17, 18, 19, 20, 21, 22, 23,
          1, 25, 26, 27, 28, 29, 30, 31));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.u8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepu8_epi64
  #define _mm_cvtepu8_epi64(a) simde_mm_cvtepu8_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepi16_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepi16_epi32(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_srai_epi32(_mm_unpacklo_epi16(a, a), 16);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vmovl_s16(vget_low_s16(a_.neon_i16));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_extend_low_i16x8(a_.wasm_v128);
    #elif !defined(SIMDE_ARCH_X86) && defined(SIMDE_SHUFFLE_VECTOR_) && defined(SIMDE_VECTOR_SCALAR) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), SIMDE_SHUFFLE_VECTOR_(16, 16, a_.i16, a_.i16, 8, 0, 10, 1, 12, 2, 14, 3));
      r_.i32 >>= 16;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i16[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepi16_epi32
  #define _mm_cvtepi16_epi32(a) simde_mm_cvtepi16_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepu16_epi32 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepu16_epi32(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_unpacklo_epi16(a, _mm_setzero_si128());
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u32 = vmovl_u16(vget_low_u16(a_.neon_u16));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u32x4_extend_low_u16x8(a_.wasm_v128);
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      __typeof__(r_.u16) z = { 0, };
      r_.i32 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i32), SIMDE_SHUFFLE_VECTOR_(16, 16, a_.u16, z,
          0, 9, 1, 11, 2, 13, 3, 15));
    #elif defined(SIMDE_CONVERT_VECTOR_) && !defined(SIMDE_BUG_CLANG_45541) && (!defined(SIMDE_ARCH_POWER) || !defined(__clang__))
      SIMDE_CONVERT_VECTOR_(r_.i32, a_.m64_private[0].u16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.u16[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepu16_epi32
  #define _mm_cvtepu16_epi32(a) simde_mm_cvtepu16_epi32(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepu16_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepu16_epi64(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i z = _mm_setzero_si128();
    return _mm_unpacklo_epi32(_mm_unpacklo_epi16(a, z), z);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      uint16x8_t u16x8 = a_.neon_u16;                    /* xxxx xxxx xxxx 0B0A */
      uint32x4_t u32x4 = vmovl_u16(vget_low_u16(u16x8)); /* 000x 000x 000B 000A */
      uint64x2_t u64x2 = vmovl_u32(vget_low_u32(u32x4)); /* 0000 000B 0000 000A */
      r_.neon_u64 = u64x2;
    #elif defined(SIMDE_SHUFFLE_VECTOR_) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      __typeof__(r_.u16) z = { 0, };
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), SIMDE_SHUFFLE_VECTOR_(16, 16, a_.u16, z,
          0,  9, 10, 11,
          1, 13, 14, 15));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.u16[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepu16_epi64
  #define _mm_cvtepu16_epi64(a) simde_mm_cvtepu16_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepi16_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepi16_epi64(a);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int16x8_t s16x8 = a_.neon_i16;                    /* xxxx xxxx xxxx 0B0A */
      int32x4_t s32x4 = vmovl_s16(vget_low_s16(s16x8)); /* 000x 000x 000B 000A */
      int64x2_t s64x2 = vmovl_s32(vget_low_s32(s32x4)); /* 0000 000B 0000 000A */
      r_.neon_i64 = s64x2;
    #elif (!defined(SIMDE_ARCH_X86) && !defined(SIMDE_ARCH_AMD64)) && defined(SIMDE_SHUFFLE_VECTOR_) && defined(SIMDE_VECTOR_SCALAR) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), SIMDE_SHUFFLE_VECTOR_(16, 16, a_.i16, a_.i16,
           8,  9, 10, 0,
          12, 13, 14, 1));
      r_.i64 >>= 48;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i16[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepi16_epi64
  #define _mm_cvtepi16_epi64(a) simde_mm_cvtepi16_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepi32_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepi32_epi64(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i tmp = _mm_shuffle_epi32(a, 0x50);
    tmp = _mm_srai_epi32(tmp, 31);
    tmp = _mm_shuffle_epi32(tmp, 0xed);
    return _mm_unpacklo_epi32(a, tmp);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i64 = vmovl_s32(vget_low_s32(a_.neon_i32));
    #elif !defined(SIMDE_ARCH_X86) && defined(SIMDE_SHUFFLE_VECTOR_) && defined(SIMDE_VECTOR_SCALAR) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), SIMDE_SHUFFLE_VECTOR_(32, 16, a_.i32, a_.i32, -1, 0, -1, 1));
      r_.i64 >>= 32;
    #elif defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i64, a_.m64_private[0].i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.i32[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepi32_epi64
  #define _mm_cvtepi32_epi64(a) simde_mm_cvtepi32_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_cvtepu32_epi64 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_cvtepu32_epi64(a);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    return _mm_unpacklo_epi32(a, _mm_setzero_si128());
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u64 = vmovl_u32(vget_low_u32(a_.neon_u32));
    #elif defined(SIMDE_VECTOR_SCALAR) && defined(SIMDE_SHUFFLE_VECTOR_) && (SIMDE_ENDIAN_ORDER == SIMDE_ENDIAN_LITTLE)
      __typeof__(r_.u32) z = { 0, };
      r_.i64 = HEDLEY_REINTERPRET_CAST(__typeof__(r_.i64), SIMDE_SHUFFLE_VECTOR_(32, 16, a_.u32, z, 0, 4, 1, 6));
    #elif defined(SIMDE_CONVERT_VECTOR_)
      SIMDE_CONVERT_VECTOR_(r_.i64, a_.m64_private[0].u32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] = a_.u32[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_cvtepu32_epi64
  #define _mm_cvtepu32_epi64(a) simde_mm_cvtepu32_epi64(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_dp_pd (simde__m128d a, simde__m128d b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
  simde__m128d_private
    r_,
    a_ = simde__m128d_to_private(a),
    b_ = simde__m128d_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_f64 = vmulq_f64(a_.neon_f64, b_.neon_f64);

    switch (imm8) {
      case 0xff:
        r_.neon_f64 = vaddq_f64(r_.neon_f64, vextq_f64(r_.neon_f64, r_.neon_f64, 1));
        break;
      case 0x13:
        r_.neon_f64 = vdupq_lane_f64(vget_low_f64(r_.neon_f64), 0);
        break;
      default:
        { /* imm8 is a compile-time constant, so this all becomes just a load */
          uint64_t mask_data[] = {
            (imm8 & (1 << 4)) ? ~UINT64_C(0) : UINT64_C(0),
            (imm8 & (1 << 5)) ? ~UINT64_C(0) : UINT64_C(0),
          };
          r_.neon_f64 = vreinterpretq_f64_u64(vandq_u64(vld1q_u64(mask_data), vreinterpretq_u64_f64(r_.neon_f64)));
        }

        r_.neon_f64 = vdupq_n_f64(vaddvq_f64(r_.neon_f64));

        {
          uint64_t mask_data[] = {
            (imm8 & 1) ? ~UINT64_C(0) : UINT64_C(0),
            (imm8 & 2) ? ~UINT64_C(0) : UINT64_C(0)
          };
          r_.neon_f64 = vreinterpretq_f64_u64(vandq_u64(vld1q_u64(mask_data), vreinterpretq_u64_f64(r_.neon_f64)));
        }
        break;
    }
  #else
    simde_float64 sum = SIMDE_FLOAT64_C(0.0);

    SIMDE_VECTORIZE_REDUCTION(+:sum)
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      sum += ((imm8 >> (i + 4)) & 1) ? (a_.f64[i] * b_.f64[i]) : 0.0;
    }

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = ((imm8 >> i) & 1) ? sum : 0.0;
    }
  #endif

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
#  define simde_mm_dp_pd(a, b, imm8) _mm_dp_pd(a, b, imm8)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_dp_pd
  #define _mm_dp_pd(a, b, imm8) simde_mm_dp_pd(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_dp_ps (simde__m128 a, simde__m128 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
  simde__m128_private
    r_,
    a_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_f32 = vmulq_f32(a_.neon_f32, b_.neon_f32);

    switch (imm8) {
      case 0xff:
        r_.neon_f32 = vdupq_n_f32(vaddvq_f32(r_.neon_f32));
        break;
      case 0x7f:
        r_.neon_f32 = vsetq_lane_f32(0, r_.neon_f32, 3);
        r_.neon_f32 = vdupq_n_f32(vaddvq_f32(r_.neon_f32));
        break;
      default:
        {
          {
            uint32_t mask_data[] = {
              (imm8 & (1 << 4)) ? ~UINT32_C(0) : UINT32_C(0),
              (imm8 & (1 << 5)) ? ~UINT32_C(0) : UINT32_C(0),
              (imm8 & (1 << 6)) ? ~UINT32_C(0) : UINT32_C(0),
              (imm8 & (1 << 7)) ? ~UINT32_C(0) : UINT32_C(0)
            };
            r_.neon_f32 = vreinterpretq_f32_u32(vandq_u32(vld1q_u32(mask_data), vreinterpretq_u32_f32(r_.neon_f32)));
          }

          r_.neon_f32 = vdupq_n_f32(vaddvq_f32(r_.neon_f32));

          {
            uint32_t mask_data[] = {
              (imm8 & 1) ? ~UINT32_C(0) : UINT32_C(0),
              (imm8 & 2) ? ~UINT32_C(0) : UINT32_C(0),
              (imm8 & 4) ? ~UINT32_C(0) : UINT32_C(0),
              (imm8 & 8) ? ~UINT32_C(0) : UINT32_C(0)
            };
            r_.neon_f32 = vreinterpretq_f32_u32(vandq_u32(vld1q_u32(mask_data), vreinterpretq_u32_f32(r_.neon_f32)));
          }
        }
        break;
    }
  #else
    simde_float32 sum = SIMDE_FLOAT32_C(0.0);

    SIMDE_VECTORIZE_REDUCTION(+:sum)
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      sum += ((imm8 >> (i + 4)) & 1) ? (a_.f32[i] * b_.f32[i]) : SIMDE_FLOAT32_C(0.0);
    }

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = ((imm8 >> i) & 1) ? sum : SIMDE_FLOAT32_C(0.0);
    }
  #endif

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #if defined(HEDLEY_MCST_LCC_VERSION)
    #define simde_mm_dp_ps(a, b, imm8) (__extension__ ({ \
      SIMDE_LCC_DISABLE_DEPRECATED_WARNINGS \
      _mm_dp_ps((a), (b), (imm8)); \
      SIMDE_LCC_REVERT_DEPRECATED_WARNINGS \
    }))
  #else
    #define simde_mm_dp_ps(a, b, imm8) _mm_dp_ps(a, b, imm8)
  #endif
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_dp_ps
  #define _mm_dp_ps(a, b, imm8) simde_mm_dp_ps(a, b, imm8)
#endif

#if defined(simde_mm_extract_epi8)
#  undef simde_mm_extract_epi8
#endif
SIMDE_FUNCTION_ATTRIBUTES
int8_t
simde_mm_extract_epi8 (simde__m128i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15)  {
  simde__m128i_private
    a_ = simde__m128i_to_private(a);

  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    #if defined(SIMDE_BUG_GCC_95227)
      (void) a_;
      (void) imm8;
    #endif
    return vec_extract(a_.altivec_i8, imm8);
  #else
    return a_.i8[imm8 & 15];
  #endif
}
#if defined(SIMDE_X86_SSE4_1_NATIVE) && !defined(SIMDE_BUG_GCC_BAD_MM_EXTRACT_EPI8)
#  define simde_mm_extract_epi8(a, imm8) HEDLEY_STATIC_CAST(int8_t, _mm_extract_epi8(a, imm8))
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm_extract_epi8(a, imm8) vgetq_lane_s8(simde__m128i_to_neon_i8(a), imm8)
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_extract_epi8(a, imm8) wasm_u8x16_extract_lane(simde__m128i_to_wasm_v128((a)), (imm8) & 15)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_extract_epi8
  #define _mm_extract_epi8(a, imm8) HEDLEY_STATIC_CAST(int, simde_mm_extract_epi8(a, imm8))
#endif

#if defined(simde_mm_extract_epi32)
#  undef simde_mm_extract_epi32
#endif
SIMDE_FUNCTION_ATTRIBUTES
int32_t
simde_mm_extract_epi32 (simde__m128i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 3)  {
  simde__m128i_private
    a_ = simde__m128i_to_private(a);

  #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
    #if defined(SIMDE_BUG_GCC_95227)
      (void) a_;
      (void) imm8;
    #endif
    return vec_extract(a_.altivec_i32, imm8);
  #else
    return a_.i32[imm8 & 3];
  #endif
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
#  define simde_mm_extract_epi32(a, imm8) _mm_extract_epi32(a, imm8)
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm_extract_epi32(a, imm8) vgetq_lane_s32(simde__m128i_to_neon_i32(a), imm8)
#elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
#  define simde_mm_extract_epi32(a, imm8) HEDLEY_STATIC_CAST(int32_t, vec_extract(simde__m128i_to_altivec_i32(a), imm8))
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_extract_epi32(a, imm8) wasm_i32x4_extract_lane(simde__m128i_to_wasm_v128((a)), (imm8) & 3)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_extract_epi32
  #define _mm_extract_epi32(a, imm8) simde_mm_extract_epi32(a, imm8)
#endif

#if defined(simde_mm_extract_epi64)
#  undef simde_mm_extract_epi64
#endif
SIMDE_FUNCTION_ATTRIBUTES
int64_t
simde_mm_extract_epi64 (simde__m128i a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1)  {
  simde__m128i_private
    a_ = simde__m128i_to_private(a);

  #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE)
    #if defined(SIMDE_BUG_GCC_95227)
      (void) a_;
      (void) imm8;
    #endif
    return vec_extract(a_.altivec_i64, imm8);
  #else
    return a_.i64[imm8 & 1];
  #endif
}
#if defined(SIMDE_X86_SSE4_1_NATIVE) && defined(SIMDE_ARCH_AMD64)
#  define simde_mm_extract_epi64(a, imm8) _mm_extract_epi64(a, imm8)
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm_extract_epi64(a, imm8) vgetq_lane_s64(simde__m128i_to_neon_i64(a), imm8)
#elif defined(SIMDE_POWER_ALTIVEC_P7_NATIVE)
#  define simde_mm_extract_epi64(a, imm8) HEDLEY_STATIC_CAST(int64_t, vec_extract(simde__m128i_to_altivec_i64(a), imm8))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && !defined(SIMDE_ARCH_AMD64))
  #undef _mm_extract_epi64
  #define _mm_extract_epi64(a, imm8) simde_mm_extract_epi64(a, imm8)
#endif

#if defined(simde_mm_extract_ps)
#  undef simde_mm_extract_ps
#endif
SIMDE_FUNCTION_ATTRIBUTES
int32_t
simde_mm_extract_ps (simde__m128 a, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 3)  {
  simde__m128_private
    a_ = simde__m128_to_private(a);

  return a_.i32[imm8 & 3];
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #define simde_mm_extract_ps(a, imm8) _mm_extract_ps(a, imm8)
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
  #define simde_mm_extract_ps(a, imm8) vgetq_lane_s32(simde__m128_to_neon_i32(a), imm8)
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
  #define simde_mm_extract_ps(a, imm8) wasm_i32x4_extract_lane(simde__m128_to_wasm_v128((a)), (imm8) & 3)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_extract_ps
  #define _mm_extract_ps(a, imm8) simde_mm_extract_ps(a, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_floor_pd (simde__m128d a) {
  #if defined(SIMDE_WASM_SIMD128_NATIVE)
    return simde__m128d_from_wasm_v128(wasm_f64x2_floor(simde__m128d_to_wasm_v128(a)));
  #endif
  return simde_mm_round_pd(a, SIMDE_MM_FROUND_TO_NEG_INF);
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_floor_pd
  #define _mm_floor_pd(a) simde_mm_floor_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_floor_ps (simde__m128 a) {
  #if defined(SIMDE_WASM_SIMD128_NATIVE)
    return simde__m128_from_wasm_v128(wasm_f32x4_floor(simde__m128_to_wasm_v128(a)));
  #endif
  return simde_mm_round_ps(a, SIMDE_MM_FROUND_TO_NEG_INF);
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_floor_ps
  #define _mm_floor_ps(a) simde_mm_floor_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_floor_sd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_floor_sd(a, b);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b);

    #if defined(simde_math_floor)
      r_.f64[0] = simde_math_floor(b_.f64[0]);
      r_.f64[1] = a_.f64[1];
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_floor_sd
  #define _mm_floor_sd(a, b) simde_mm_floor_sd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_floor_ss (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_floor_ss(a, b);
  #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_FAST_EXCEPTIONS)
      return simde_mm_move_ss(a, simde_mm_floor_ps(b));
  #elif (SIMDE_NATURAL_VECTOR_SIZE > 0)
    return simde_mm_move_ss(a, simde_mm_floor_ps(simde_x_mm_broadcastlow_ps(b)));
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b);

    #if defined(simde_math_floorf)
      r_.f32[0] = simde_math_floorf(b_.f32[0]);
      for (size_t i = 1 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = a_.f32[i];
      }
    #else
      HEDLEY_UNREACHABLE();
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_floor_ss
  #define _mm_floor_ss(a, b) simde_mm_floor_ss(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_insert_epi8 (simde__m128i a, int i, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 15)  {
  simde__m128i_private
    r_ = simde__m128i_to_private(a);

  r_.i8[imm8] = HEDLEY_STATIC_CAST(int8_t, i);

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  /* clang-3.8 returns an incompatible type, so we need the cast.  MSVC
   * can't handle the cast ("error C2440: 'type cast': cannot convert
   * from '__m128i' to '__m128i'").  */
  #if defined(__clang__)
    #define simde_mm_insert_epi8(a, i, imm8) HEDLEY_REINTERPRET_CAST(__m128i, _mm_insert_epi8(a, i, imm8))
  #else
    #define simde_mm_insert_epi8(a, i, imm8) _mm_insert_epi8(a, i, imm8)
  #endif
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm_insert_epi8(a, i, imm8) simde__m128i_from_neon_i8(vsetq_lane_s8(i, simde__m128i_to_neon_i8(a), imm8))
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_insert_epi8(a, i, imm8) simde__m128i_from_wasm_v128(wasm_i8x16_replace_lane(simde__m128i_to_wasm_v128((a)), (imm8) & 15, HEDLEY_STATIC_CAST(int8_t, (i))))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_insert_epi8
  #define _mm_insert_epi8(a, i, imm8) simde_mm_insert_epi8(a, i, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_insert_epi32 (simde__m128i a, int i, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 3)  {
  simde__m128i_private
    r_ = simde__m128i_to_private(a);

  r_.i32[imm8] = HEDLEY_STATIC_CAST(int32_t, i);

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #if defined(__clang__)
    #define simde_mm_insert_epi32(a, i, imm8) HEDLEY_REINTERPRET_CAST(__m128i, _mm_insert_epi32(a, i, imm8))
  #else
    #define simde_mm_insert_epi32(a, i, imm8) _mm_insert_epi32(a, i, imm8)
  #endif
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm_insert_epi32(a, i, imm8) simde__m128i_from_neon_i32(vsetq_lane_s32(i, simde__m128i_to_neon_i32(a), imm8))
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_insert_epi32(a, i, imm8) simde__m128i_from_wasm_v128(wasm_i32x4_replace_lane(simde__m128i_to_wasm_v128((a)), (imm8) & 3, (i)))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_insert_epi32
  #define _mm_insert_epi32(a, i, imm8) simde_mm_insert_epi32(a, i, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_insert_epi64 (simde__m128i a, int64_t i, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 1)  {
  #if defined(SIMDE_BUG_GCC_94482)
    simde__m128i_private
      a_ = simde__m128i_to_private(a);

    switch(imm8) {
      case 0:
        return simde_mm_set_epi64x(a_.i64[1], i);
        break;
      case 1:
        return simde_mm_set_epi64x(i, a_.i64[0]);
        break;
      default:
        HEDLEY_UNREACHABLE();
        break;
    }
  #else
    simde__m128i_private
      r_ = simde__m128i_to_private(a);

    r_.i64[imm8] = i;
    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_NATIVE) && defined(SIMDE_ARCH_AMD64)
#  define simde_mm_insert_epi64(a, i, imm8) _mm_insert_epi64(a, i, imm8)
#elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
#  define simde_mm_insert_epi64(a, i, imm8) simde__m128i_from_neon_i64(vsetq_lane_s64(i, simde__m128i_to_neon_i64(a), imm8))
#elif defined(SIMDE_WASM_SIMD128_NATIVE)
#  define simde_mm_insert_epi64(a, i, imm8) simde__m128i_from_wasm_v128(wasm_i64x2_replace_lane(simde__m128i_to_wasm_v128((a)), (imm8) & 1, (i)))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES) || (defined(SIMDE_ENABLE_NATIVE_ALIASES) && !defined(SIMDE_ARCH_AMD64))
  #undef _mm_insert_epi64
  #define _mm_insert_epi64(a, i, imm8) simde_mm_insert_epi64(a, i, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_insert_ps (simde__m128 a, simde__m128 b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
  simde__m128_private
    r_,
    a_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  float tmp1_ = b_.f32[(imm8 >> 6) & 3];
  a_.f32[(imm8 >> 4) & 3] = tmp1_;

  SIMDE_VECTORIZE
  for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
    r_.f32[i] = ((imm8 >> i) & 1 ) ? SIMDE_FLOAT32_C(0.0) : a_.f32[i];
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
#  define simde_mm_insert_ps(a, b, imm8) _mm_insert_ps(a, b, imm8)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_insert_ps
  #define _mm_insert_ps(a, b, imm8) simde_mm_insert_ps(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_max_epi8 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE) && !defined(__PGI)
    return _mm_max_epi8(a, b);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i m = _mm_cmpgt_epi8(a, b);
    return _mm_or_si128(_mm_and_si128(m, a), _mm_andnot_si128(m, b));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i8 = vmaxq_s8(a_.neon_i8, b_.neon_i8);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i8x16_max(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i8 = vec_max(a_.altivec_i8, b_.altivec_i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[i] > b_.i8[i] ? a_.i8[i] : b_.i8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_max_epi8
  #define _mm_max_epi8(a, b) simde_mm_max_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_max_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE) && !defined(__PGI)
    return _mm_max_epi32(a, b);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    __m128i m = _mm_cmpgt_epi32(a, b);
    return _mm_or_si128(_mm_and_si128(m, a), _mm_andnot_si128(m, b));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vmaxq_s32(a_.neon_i32, b_.neon_i32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_max(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i32 = vec_max(a_.altivec_i32, b_.altivec_i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] > b_.i32[i] ? a_.i32[i] : b_.i32[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_max_epi32
  #define _mm_max_epi32(a, b) simde_mm_max_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_max_epu16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_max_epu16(a, b);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    /* https://github.com/simd-everywhere/simde/issues/855#issuecomment-881656284 */
    return _mm_add_epi16(b, _mm_subs_epu16(a, b));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u16 = vmaxq_u16(a_.neon_u16, b_.neon_u16);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u16x8_max(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_u16 = vec_max(a_.altivec_u16, b_.altivec_u16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
        r_.u16[i] = a_.u16[i] > b_.u16[i] ? a_.u16[i] : b_.u16[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_max_epu16
  #define _mm_max_epu16(a, b) simde_mm_max_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_max_epu32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_max_epu32(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u32 = vmaxq_u32(a_.neon_u32, b_.neon_u32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u32x4_max(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_u32 = vec_max(a_.altivec_u32, b_.altivec_u32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = a_.u32[i] > b_.u32[i] ? a_.u32[i] : b_.u32[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_max_epu32
  #define _mm_max_epu32(a, b) simde_mm_max_epu32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_min_epi8 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE) && !defined(__PGI)
    return _mm_min_epi8(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i8 = vminq_s8(a_.neon_i8, b_.neon_i8);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i8x16_min(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i8 = vec_min(a_.altivec_i8, b_.altivec_i8);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i8) / sizeof(r_.i8[0])) ; i++) {
        r_.i8[i] = a_.i8[i] < b_.i8[i] ? a_.i8[i] : b_.i8[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_min_epi8
  #define _mm_min_epi8(a, b) simde_mm_min_epi8(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_min_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE) && !defined(__PGI)
    return _mm_min_epi32(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vminq_s32(a_.neon_i32, b_.neon_i32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_min(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_i32 = vec_min(a_.altivec_i32, b_.altivec_i32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.i32[i] = a_.i32[i] < b_.i32[i] ? a_.i32[i] : b_.i32[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_min_epi32
  #define _mm_min_epi32(a, b) simde_mm_min_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_min_epu16 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_min_epu16(a, b);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    /* https://github.com/simd-everywhere/simde/issues/855#issuecomment-881656284 */
    return _mm_sub_epi16(a, _mm_subs_epu16(a, b));
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u16 = vminq_u16(a_.neon_u16, b_.neon_u16);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u16x8_min(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_u16 = vec_min(a_.altivec_u16, b_.altivec_u16);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
        r_.u16[i] = a_.u16[i] < b_.u16[i] ? a_.u16[i] : b_.u16[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_min_epu16
  #define _mm_min_epu16(a, b) simde_mm_min_epu16(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_min_epu32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_min_epu32(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u32 = vminq_u32(a_.neon_u32, b_.neon_u32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u32x4_min(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_u32 = vec_min(a_.altivec_u32, b_.altivec_u32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = a_.u32[i] < b_.u32[i] ? a_.u32[i] : b_.u32[i];
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_min_epu32
  #define _mm_min_epu32(a, b) simde_mm_min_epu32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_minpos_epu16 (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_minpos_epu16(a);
  #else
    simde__m128i_private
      r_ = simde__m128i_to_private(simde_mm_setzero_si128()),
      a_ = simde__m128i_to_private(a);

    r_.u16[0] = UINT16_MAX;
    for (size_t i = 0 ; i < (sizeof(r_.u16) / sizeof(r_.u16[0])) ; i++) {
      if (a_.u16[i] < r_.u16[0]) {
        r_.u16[0] = a_.u16[i];
        r_.u16[1] = HEDLEY_STATIC_CAST(uint16_t, i);
      }
    }

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_minpos_epu16
  #define _mm_minpos_epu16(a) simde_mm_minpos_epu16(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mpsadbw_epu8 (simde__m128i a, simde__m128i b, const int imm8)
    SIMDE_REQUIRE_CONSTANT_RANGE(imm8, 0, 255)  {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  const int a_offset = imm8 & 4;
  const int b_offset = (imm8 & 3) << 2;

#if defined(simde_math_abs)
  for (int i = 0 ; i < HEDLEY_STATIC_CAST(int, (sizeof(r_.u16) / sizeof(r_.u16[0]))) ; i++) {
    r_.u16[i] =
      HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset + i + 0] - b_.u8[b_offset + 0]))) +
      HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset + i + 1] - b_.u8[b_offset + 1]))) +
      HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset + i + 2] - b_.u8[b_offset + 2]))) +
      HEDLEY_STATIC_CAST(uint16_t, simde_math_abs(HEDLEY_STATIC_CAST(int, a_.u8[a_offset + i + 3] - b_.u8[b_offset + 3])));
  }
#else
  HEDLEY_UNREACHABLE();
#endif

  return simde__m128i_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE) && !defined(SIMDE_BUG_PGI_30107)
#  define simde_mm_mpsadbw_epu8(a, b, imm8) _mm_mpsadbw_epu8(a, b, imm8)
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_mpsadbw_epu8
  #define _mm_mpsadbw_epu8(a, b, imm8) simde_mm_mpsadbw_epu8(a, b, imm8)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mul_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_mul_epi32(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      // vmull_s32 upcasts instead of masking, so we downcast.
      int32x2_t a_lo = vmovn_s64(a_.neon_i64);
      int32x2_t b_lo = vmovn_s64(b_.neon_i64);
      r_.neon_i64 = vmull_s32(a_lo, b_lo);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i64x2_make(
        wasm_i32x4_extract_lane(a_.wasm_v128, 0) * HEDLEY_STATIC_CAST(int64_t, wasm_i32x4_extract_lane(b_.wasm_v128, 0)),
        wasm_i32x4_extract_lane(a_.wasm_v128, 2) * HEDLEY_STATIC_CAST(int64_t, wasm_i32x4_extract_lane(b_.wasm_v128, 2)));
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i64) / sizeof(r_.i64[0])) ; i++) {
        r_.i64[i] =
          HEDLEY_STATIC_CAST(int64_t, a_.i32[i * 2]) *
          HEDLEY_STATIC_CAST(int64_t, b_.i32[i * 2]);
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_mul_epi32
  #define _mm_mul_epi32(a, b) simde_mm_mul_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_mullo_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_mullo_epi32(a, b);
  #else
    simde__m128i_private
      r_,
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vmulq_s32(a_.neon_i32, b_.neon_i32);
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
      (void) a_;
      (void) b_;
      r_.altivec_i32 = vec_mul(a_.altivec_i32, b_.altivec_i32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_mul(a_.wasm_v128, b_.wasm_v128);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i32) / sizeof(r_.i32[0])) ; i++) {
        r_.u32[i] = HEDLEY_STATIC_CAST(uint32_t, (HEDLEY_STATIC_CAST(uint64_t, (HEDLEY_STATIC_CAST(int64_t, a_.i32[i]) * HEDLEY_STATIC_CAST(int64_t, b_.i32[i]))) & 0xffffffff));
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_mullo_epi32
  #define _mm_mullo_epi32(a, b) simde_mm_mullo_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_mullo_epu32 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u32 = vmulq_u32(a_.neon_u32, b_.neon_u32);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS)
      r_.u32 = a_.u32 * b_.u32;
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.u32) / sizeof(r_.u32[0])) ; i++) {
        r_.u32[i] = a_.u32[i] * b_.u32[i];
      }
    #endif

  return simde__m128i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_packus_epi32 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_packus_epi32(a, b);
  #elif defined(SIMDE_X86_SSE2_NATIVE)
    const __m128i max = _mm_set1_epi32(UINT16_MAX);
    const __m128i tmpa = _mm_andnot_si128(_mm_srai_epi32(a, 31), a);
    const __m128i tmpb = _mm_andnot_si128(_mm_srai_epi32(b, 31), b);
    return
      _mm_packs_epi32(
        _mm_srai_epi32(_mm_slli_epi32(_mm_or_si128(tmpa, _mm_cmpgt_epi32(tmpa, max)), 16), 16),
        _mm_srai_epi32(_mm_slli_epi32(_mm_or_si128(tmpb, _mm_cmpgt_epi32(tmpb, max)), 16), 16)
      );
  #else
    simde__m128i_private
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b),
      r_;

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      #if defined(SIMDE_BUG_CLANG_46840)
        r_.neon_u16 = vqmovun_high_s32(vreinterpret_s16_u16(vqmovun_s32(a_.neon_i32)), b_.neon_i32);
      #else
        r_.neon_u16 = vqmovun_high_s32(vqmovun_s32(a_.neon_i32), b_.neon_i32);
      #endif
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_u16 =
        vcombine_u16(
          vqmovun_s32(a_.neon_i32),
          vqmovun_s32(b_.neon_i32)
        );
    #elif defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
      r_.altivec_u16 = vec_packsu(a_.altivec_i32, b_.altivec_i32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_u16x8_narrow_i32x4(a_.wasm_v128, b_.wasm_v128);
    #elif defined(SIMDE_CONVERT_VECTOR_) && HEDLEY_HAS_BUILTIN(__builtin_shufflevector) && defined(SIMDE_VECTOR_SUBSCRIPT_SCALAR)
      int32_t v SIMDE_VECTOR(32) = SIMDE_SHUFFLE_VECTOR_(32, 32, a_.i32, b_.i32, 0, 1, 2, 3, 4, 5, 6, 7);

      v &= ~(v >> 31);
      v |= HEDLEY_REINTERPRET_CAST(__typeof__(v), v > UINT16_MAX);

      SIMDE_CONVERT_VECTOR_(r_.i16, v);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.i16) / sizeof(r_.i16[0])) ; i++) {
        int32_t v = (i < (sizeof(a_.i32) / sizeof(a_.i32[0]))) ? a_.i32[i] : b_.i32[i & 3];
        r_.u16[i] = (v < 0) ? UINT16_C(0) : ((v > UINT16_MAX) ? UINT16_MAX : HEDLEY_STATIC_CAST(uint16_t, v));
      }
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_packus_epi32
  #define _mm_packus_epi32(a, b) simde_mm_packus_epi32(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_round_sd (simde__m128d a, simde__m128d b, int rounding)
    SIMDE_REQUIRE_CONSTANT_RANGE(rounding, 0, 15) {
  simde__m128d_private
    r_ = simde__m128d_to_private(a),
    b_ = simde__m128d_to_private(b);

  switch (rounding & ~SIMDE_MM_FROUND_NO_EXC) {
    #if defined(simde_math_nearbyint)
      case SIMDE_MM_FROUND_TO_NEAREST_INT:
      case SIMDE_MM_FROUND_CUR_DIRECTION:
        r_.f64[0] = simde_math_nearbyint(b_.f64[0]);
        break;
    #endif

    #if defined(simde_math_floor)
      case SIMDE_MM_FROUND_TO_NEG_INF:
        r_.f64[0] = simde_math_floor(b_.f64[0]);
        break;
    #endif

    #if defined(simde_math_ceil)
      case SIMDE_MM_FROUND_TO_POS_INF:
        r_.f64[0] = simde_math_ceil(b_.f64[0]);
        break;
    #endif

    #if defined(simde_math_trunc)
      case SIMDE_MM_FROUND_TO_ZERO:
        r_.f64[0] = simde_math_trunc(b_.f64[0]);
        break;
    #endif

    default:
      HEDLEY_UNREACHABLE_RETURN(simde_mm_undefined_pd());
  }

  return simde__m128d_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
#  define simde_mm_round_sd(a, b, rounding) _mm_round_sd(a, b, rounding)
#elif SIMDE_NATURAL_VECTOR_SIZE_GE(128) && defined(SIMDE_FAST_EXCEPTIONS)
#  define simde_mm_round_sd(a, b, rounding) simde_mm_move_sd(a, simde_mm_round_pd(b, rounding))
#elif SIMDE_NATURAL_VECTOR_SIZE_GE(128)
  #define simde_mm_round_sd(a, b, rounding) simde_mm_move_sd(a, simde_mm_round_pd(simde_x_mm_broadcastlow_pd(b), rounding))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_round_sd
  #define _mm_round_sd(a, b, rounding) simde_mm_round_sd(a, b, rounding)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_round_ss (simde__m128 a, simde__m128 b, int rounding)
    SIMDE_REQUIRE_CONSTANT_RANGE(rounding, 0, 15) {
  simde__m128_private
    r_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  switch (rounding & ~SIMDE_MM_FROUND_NO_EXC) {
    #if defined(simde_math_nearbyintf)
      case SIMDE_MM_FROUND_TO_NEAREST_INT:
      case SIMDE_MM_FROUND_CUR_DIRECTION:
        r_.f32[0] = simde_math_nearbyintf(b_.f32[0]);
        break;
    #endif

    #if defined(simde_math_floorf)
      case SIMDE_MM_FROUND_TO_NEG_INF:
        r_.f32[0] = simde_math_floorf(b_.f32[0]);
        break;
    #endif

    #if defined(simde_math_ceilf)
      case SIMDE_MM_FROUND_TO_POS_INF:
        r_.f32[0] = simde_math_ceilf(b_.f32[0]);
        break;
    #endif

    #if defined(simde_math_truncf)
      case SIMDE_MM_FROUND_TO_ZERO:
        r_.f32[0] = simde_math_truncf(b_.f32[0]);
        break;
    #endif

    default:
      HEDLEY_UNREACHABLE_RETURN(simde_mm_undefined_pd());
  }

  return simde__m128_from_private(r_);
}
#if defined(SIMDE_X86_SSE4_1_NATIVE)
  #define simde_mm_round_ss(a, b, rounding) _mm_round_ss(a, b, rounding)
#elif SIMDE_NATURAL_VECTOR_SIZE > 0 && defined(SIMDE_FAST_EXCEPTIONS)
  #define simde_mm_round_ss(a, b, rounding) simde_mm_move_ss((a), simde_mm_round_ps((b), (rounding)))
#elif SIMDE_NATURAL_VECTOR_SIZE > 0
  #define simde_mm_round_ss(a, b, rounding) simde_mm_move_ss((a), simde_mm_round_ps(simde_x_mm_broadcastlow_ps(b), (rounding)))
#endif
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_round_ss
  #define _mm_round_ss(a, b, rounding) simde_mm_round_ss(a, b, rounding)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_stream_load_si128 (const simde__m128i* mem_addr) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_stream_load_si128(HEDLEY_CONST_CAST(simde__m128i*, mem_addr));
  #elif HEDLEY_HAS_BUILTIN(__builtin_nontemporal_load) && ( \
      defined(SIMDE_ARM_NEON_A32V7_NATIVE) || defined(SIMDE_VECTOR_SUBSCRIPT) || \
      defined(SIMDE_WASM_SIMD128_NATIVE) || defined(SIMDE_POWER_ALTIVEC_P6_NATIVE) || \
      defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE))
    return __builtin_nontemporal_load(mem_addr);
  #else
    return simde_mm_load_si128(mem_addr);
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_stream_load_si128
  #define _mm_stream_load_si128(mem_addr) simde_mm_stream_load_si128(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_test_all_ones (simde__m128i a) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_test_all_ones(a);
  #else
    simde__m128i_private a_ = simde__m128i_to_private(a);
    int r;

    #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
      r = vec_all_eq(a_.altivec_i32, vec_splats(~0));
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r = ((vgetq_lane_s64(a_.neon_i64, 0) & vgetq_lane_s64(a_.neon_i64, 1)) == ~HEDLEY_STATIC_CAST(int64_t, 0));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r = HEDLEY_STATIC_CAST(unsigned long long, wasm_i64x2_extract_lane(a_.wasm_v128, 0) & wasm_i64x2_extract_lane(a_.wasm_v128, 1)) == 0xFFFFFFFFFFFFFFFFull;
    #else
      int_fast32_t r_ = ~HEDLEY_STATIC_CAST(int_fast32_t, 0);

      SIMDE_VECTORIZE_REDUCTION(&:r_)
      for (size_t i = 0 ; i < (sizeof(a_.i32f) / sizeof(a_.i32f[0])) ; i++) {
        r_ &= a_.i32f[i];
      }

      r = (r_ == ~HEDLEY_STATIC_CAST(int_fast32_t, 0));
    #endif

    return r;
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_test_all_ones
  #define _mm_test_all_ones(a) simde_mm_test_all_ones(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_test_all_zeros (simde__m128i a, simde__m128i mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_test_all_zeros(a, mask);
  #else
    simde__m128i_private tmp_ = simde__m128i_to_private(simde_mm_and_si128(a, mask));
    int r;

    #if defined(SIMDE_POWER_ALTIVEC_P6_NATIVE)
      r = vec_all_eq(tmp_.altivec_i32, vec_splats(0));
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r = !(vgetq_lane_s64(tmp_.neon_i64, 0) | vgetq_lane_s64(tmp_.neon_i64, 1));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r = (wasm_i64x2_extract_lane(tmp_.wasm_v128, 0) | wasm_i64x2_extract_lane(tmp_.wasm_v128, 1)) == 0;
    #else
      int_fast32_t r_ = HEDLEY_STATIC_CAST(int_fast32_t, 0);

      SIMDE_VECTORIZE_REDUCTION(|:r_)
      for (size_t i = 0 ; i < (sizeof(tmp_.i32f) / sizeof(tmp_.i32f[0])) ; i++) {
        r_ |= tmp_.i32f[i];
      }

      r = !r_;
    #endif

    return r;
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_test_all_zeros
  #define _mm_test_all_zeros(a, mask) simde_mm_test_all_zeros(a, mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_test_mix_ones_zeros (simde__m128i a, simde__m128i mask) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_test_mix_ones_zeros(a, mask);
  #else
    simde__m128i_private
      a_ = simde__m128i_to_private(a),
      mask_ = simde__m128i_to_private(mask);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int64x2_t s640 = vandq_s64(a_.neon_i64, mask_.neon_i64);
      int64x2_t s641 = vandq_s64(vreinterpretq_s64_s32(vmvnq_s32(vreinterpretq_s32_s64(a_.neon_i64))), mask_.neon_i64);
      return (((vgetq_lane_s64(s640, 0) | vgetq_lane_s64(s640, 1)) & (vgetq_lane_s64(s641, 0) | vgetq_lane_s64(s641, 1)))!=0);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_v128_and(a_.wasm_v128, mask_.wasm_v128);
      long long c0 = wasm_i64x2_extract_lane(m, 0);
      long long c1 = wasm_i64x2_extract_lane(m, 1);
      long long ones = c0 | c1;
      long long zeros = ~(c0 & c1);
      return ones && zeros;
    #else
      for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++)
        if (((a_.u64[i] & mask_.u64[i]) != 0) && ((~a_.u64[i] & mask_.u64[i]) != 0))
          return 1;

      return 0;
    #endif
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_test_mix_ones_zeros
  #define _mm_test_mix_ones_zeros(a, mask) simde_mm_test_mix_ones_zeros(a, mask)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testc_si128 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_testc_si128(a, b);
  #else
    simde__m128i_private
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int64x2_t s64 = vbicq_s64(b_.neon_i64, a_.neon_i64);
      return !(vgetq_lane_s64(s64, 0) | vgetq_lane_s64(s64, 1));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_v128_andnot(b_.wasm_v128, a_.wasm_v128);
      return (wasm_i64x2_extract_lane(m, 0) | wasm_i64x2_extract_lane(m, 1)) == 0;
    #else
      int_fast32_t r = 0;

      SIMDE_VECTORIZE_REDUCTION(|:r)
      for (size_t i = 0 ; i < (sizeof(a_.i32f) / sizeof(a_.i32f[0])) ; i++) {
        r |= ~a_.i32f[i] & b_.i32f[i];
      }

      return HEDLEY_STATIC_CAST(int, !r);
    #endif
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_testc_si128
  #define _mm_testc_si128(a, b) simde_mm_testc_si128(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testnzc_si128 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_testnzc_si128(a, b);
  #else
    simde__m128i_private
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int64x2_t s640 = vandq_s64(b_.neon_i64, a_.neon_i64);
      int64x2_t s641 = vbicq_s64(b_.neon_i64, a_.neon_i64);
      return !( !(vgetq_lane_s64(s641, 0) || vgetq_lane_s64(s641, 1)) \
             || !(vgetq_lane_s64(s640, 0) || vgetq_lane_s64(s640, 1)) );
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m1 = wasm_v128_and(a_.wasm_v128, b_.wasm_v128);
      v128_t m2 = wasm_v128_andnot(b_.wasm_v128, a_.wasm_v128);
      return (wasm_i64x2_extract_lane(m1, 0) | wasm_i64x2_extract_lane(m1, 1)) \
        && (wasm_i64x2_extract_lane(m2, 0) | wasm_i64x2_extract_lane(m2, 1));
    #else
      for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
        if (((a_.u64[i] & b_.u64[i]) != 0) && ((~a_.u64[i] & b_.u64[i]) != 0))
          return 1;
      }

      return 0;
    #endif
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_testnzc_si128
  #define _mm_testnzc_si128(a, b) simde_mm_testnzc_si128(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
int
simde_mm_testz_si128 (simde__m128i a, simde__m128i b) {
  #if defined(SIMDE_X86_SSE4_1_NATIVE)
    return _mm_testz_si128(a, b);
  #else
    simde__m128i_private
      a_ = simde__m128i_to_private(a),
      b_ = simde__m128i_to_private(b);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      int64x2_t s64 = vandq_s64(a_.neon_i64, b_.neon_i64);
      return !(vgetq_lane_s64(s64, 0) | vgetq_lane_s64(s64, 1));
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      v128_t m = wasm_v128_and(a_.wasm_v128, b_.wasm_v128);
      return (wasm_i64x2_extract_lane(m, 0) | wasm_i64x2_extract_lane(m, 1)) == 0;
    #elif defined(SIMDE_HAVE_INT128_)
      if ((a_.u128[0] & b_.u128[0]) == 0) {
        return 1;
      }
      return 0;
    #else
      for (size_t i = 0 ; i < (sizeof(a_.u64) / sizeof(a_.u64[0])) ; i++) {
        if ((a_.u64[i] & b_.u64[i]) > 0)
          return 0;
      }
    #endif

    return 1;
  #endif
}
#if defined(SIMDE_X86_SSE4_1_ENABLE_NATIVE_ALIASES)
  #undef _mm_testz_si128
  #define _mm_testz_si128(a, b) simde_mm_testz_si128(a, b)
#endif

SIMDE_END_DECLS_

HEDLEY_DIAGNOSTIC_POP

#endif /* !defined(SIMDE_X86_SSE4_1_H) */
