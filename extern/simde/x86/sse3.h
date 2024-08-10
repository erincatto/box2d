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

#if !defined(SIMDE_X86_SSE3_H)
#define SIMDE_X86_SSE3_H

#include "sse2.h"

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DISABLE_UNWANTED_DIAGNOSTICS
SIMDE_BEGIN_DECLS_

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_deinterleaveeven_epi16 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_i16 = vuzp1q_s16(a_.neon_i16, b_.neon_i16);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int16x8x2_t t = vuzpq_s16(a_.neon_i16, b_.neon_i16);
    r_.neon_i16 = t.val[0];
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i16x8_shuffle(a_.wasm_v128, b_.wasm_v128, 0, 2, 4, 6, 8, 10, 12, 14);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i16 = SIMDE_SHUFFLE_VECTOR_(16, 16, a_.i16, b_.i16, 0, 2, 4, 6, 8, 10, 12, 14);
  #else
    const size_t halfway_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.i16[i] = a_.i16[2 * i];
      r_.i16[i + halfway_point] = b_.i16[2 * i];
    }
  #endif

  return simde__m128i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_deinterleaveodd_epi16 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_i16 = vuzp2q_s16(a_.neon_i16, b_.neon_i16);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int16x8x2_t t = vuzpq_s16(a_.neon_i16, b_.neon_i16);
    r_.neon_i16 = t.val[1];
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i16x8_shuffle(a_.wasm_v128, b_.wasm_v128, 1, 3, 5, 7, 9, 11, 13, 15);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i16 = SIMDE_SHUFFLE_VECTOR_(16, 16, a_.i16, b_.i16, 1, 3, 5, 7, 9, 11, 13, 15);
  #else
    const size_t halfway_point = (sizeof(r_.i16) / sizeof(r_.i16[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.i16[i] = a_.i16[2 * i + 1];
      r_.i16[i + halfway_point] = b_.i16[2 * i + 1];
    }
  #endif

  return simde__m128i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_deinterleaveeven_epi32 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_i32 = vuzp1q_s32(a_.neon_i32, b_.neon_i32);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int32x4x2_t t = vuzpq_s32(a_.neon_i32, b_.neon_i32);
    r_.neon_i32 = t.val[0];
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i32x4_shuffle(a_.wasm_v128, b_.wasm_v128, 0, 2, 4, 6);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.i32, b_.i32, 0, 2, 4, 6);
  #else
    const size_t halfway_point = (sizeof(r_.i32) / sizeof(r_.i32[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.i32[i] = a_.i32[2 * i];
      r_.i32[i + halfway_point] = b_.i32[2 * i];
    }
  #endif

  return simde__m128i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_x_mm_deinterleaveodd_epi32 (simde__m128i a, simde__m128i b) {
  simde__m128i_private
    r_,
    a_ = simde__m128i_to_private(a),
    b_ = simde__m128i_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_i32 = vuzp2q_s32(a_.neon_i32, b_.neon_i32);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    int32x4x2_t t = vuzpq_s32(a_.neon_i32, b_.neon_i32);
    r_.neon_i32 = t.val[1];
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i32x4_shuffle(a_.wasm_v128, b_.wasm_v128, 1, 3, 5, 7);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.i32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.i32, b_.i32, 1, 3, 5, 7);
  #else
    const size_t halfway_point = (sizeof(r_.i32) / sizeof(r_.i32[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.i32[i] = a_.i32[2 * i + 1];
      r_.i32[i + halfway_point] = b_.i32[2 * i + 1];
    }
  #endif

  return simde__m128i_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_x_mm_deinterleaveeven_ps (simde__m128 a, simde__m128 b) {
  simde__m128_private
    r_,
    a_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_f32 = vuzp1q_f32(a_.neon_f32, b_.neon_f32);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    float32x4x2_t t = vuzpq_f32(a_.neon_f32, b_.neon_f32);
    r_.neon_f32 = t.val[0];
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i32x4_shuffle(a_.wasm_v128, b_.wasm_v128, 0, 2, 4, 6);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.f32, b_.f32, 0, 2, 4, 6);
  #else
    const size_t halfway_point = (sizeof(r_.f32) / sizeof(r_.f32[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.f32[i] = a_.f32[2 * i];
      r_.f32[i + halfway_point] = b_.f32[2 * i];
    }
  #endif

  return simde__m128_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_x_mm_deinterleaveodd_ps (simde__m128 a, simde__m128 b) {
  simde__m128_private
    r_,
    a_ = simde__m128_to_private(a),
    b_ = simde__m128_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_f32 = vuzp2q_f32(a_.neon_f32, b_.neon_f32);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    float32x4x2_t t = vuzpq_f32(a_.neon_f32, b_.neon_f32);
    r_.neon_f32 = t.val[1];
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i32x4_shuffle(a_.wasm_v128, b_.wasm_v128, 1, 3, 5, 7);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.f32, b_.f32, 1, 3, 5, 7);
  #else
    const size_t halfway_point = (sizeof(r_.f32) / sizeof(r_.f32[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.f32[i] = a_.f32[2 * i + 1];
      r_.f32[i + halfway_point] = b_.f32[2 * i + 1];
    }
  #endif

  return simde__m128_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_x_mm_deinterleaveeven_pd (simde__m128d a, simde__m128d b) {
  simde__m128d_private
    r_,
    a_ = simde__m128d_to_private(a),
    b_ = simde__m128d_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_f64 = vuzp1q_f64(a_.neon_f64, b_.neon_f64);
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i64x2_shuffle(a_.wasm_v128, b_.wasm_v128, 0, 2);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 16, a_.f64, b_.f64, 0, 2);
  #else
    const size_t halfway_point = (sizeof(r_.f64) / sizeof(r_.f64[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.f64[i] = a_.f64[2 * i];
      r_.f64[i + halfway_point] = b_.f64[2 * i];
    }
  #endif

  return simde__m128d_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_x_mm_deinterleaveodd_pd (simde__m128d a, simde__m128d b) {
  simde__m128d_private
    r_,
    a_ = simde__m128d_to_private(a),
    b_ = simde__m128d_to_private(b);

  #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    r_.neon_f64 = vuzp2q_f64(a_.neon_f64, b_.neon_f64);
  #elif defined(SIMDE_WASM_SIMD128_NATIVE)
    r_.wasm_v128 = wasm_i64x2_shuffle(a_.wasm_v128, b_.wasm_v128, 1, 3);
  #elif defined(SIMDE_SHUFFLE_VECTOR_)
    r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 16, a_.f64, b_.f64, 1, 3);
  #else
    const size_t halfway_point = (sizeof(r_.f64) / sizeof(r_.f64[0])) / 2;
    for(size_t i = 0 ; i < halfway_point ; i++) {
      r_.f64[i] = a_.f64[2 * i + 1];
      r_.f64[i + halfway_point] = b_.f64[2 * i + 1];
    }
  #endif

  return simde__m128d_from_private(r_);
}

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_addsub_pd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_addsub_pd(a, b);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      float64x2_t rs = vsubq_f64(a_.neon_f64, b_.neon_f64);
      float64x2_t ra = vaddq_f64(a_.neon_f64, b_.neon_f64);
      return vcombine_f64(vget_low_f64(rs), vget_high_f64(ra));
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 16, a_.f64 - b_.f64, a_.f64 + b_.f64, 0, 3);
    #else
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i += 2) {
        r_.f64[  i  ] = a_.f64[  i  ] - b_.f64[  i  ];
        r_.f64[1 + i] = a_.f64[1 + i] + b_.f64[1 + i];
      }
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_addsub_pd(a, b) simde_mm_addsub_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_addsub_ps (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_addsub_ps(a, b);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      float32x4_t rs = vsubq_f32(a_.neon_f32, b_.neon_f32);
      float32x4_t ra = vaddq_f32(a_.neon_f32, b_.neon_f32);
      return vtrn2q_f32(vreinterpretq_f32_s32(vrev64q_s32(vreinterpretq_s32_f32(rs))), ra);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.f32 - b_.f32, a_.f32 + b_.f32, 0, 5, 2, 7);
    #else
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i += 2) {
        r_.f32[  i  ] = a_.f32[  i  ] - b_.f32[  i  ];
        r_.f32[1 + i] = a_.f32[1 + i] + b_.f32[1 + i];
      }
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_addsub_ps(a, b) simde_mm_addsub_ps((a), (b))
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_hadd_pd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_hadd_pd(a, b);
  #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    return simde__m128d_from_neon_f64(vpaddq_f64(simde__m128d_to_neon_f64(a), simde__m128d_to_neon_f64(b)));
  #else
    return simde_mm_add_pd(simde_x_mm_deinterleaveeven_pd(a, b), simde_x_mm_deinterleaveodd_pd(a, b));
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadd_pd(a, b) simde_mm_hadd_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_hadd_ps (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_hadd_ps(a, b);
  #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
    return simde__m128_from_neon_f32(vpaddq_f32(simde__m128_to_neon_f32(a), simde__m128_to_neon_f32(b)));
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    float32x4x2_t t = vuzpq_f32(simde__m128_to_neon_f32(a), simde__m128_to_neon_f32(b));
    return simde__m128_from_neon_f32(vaddq_f32(t.val[0], t.val[1]));
  #else
    return simde_mm_add_ps(simde_x_mm_deinterleaveeven_ps(a, b), simde_x_mm_deinterleaveodd_ps(a, b));
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hadd_ps(a, b) simde_mm_hadd_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_hsub_pd (simde__m128d a, simde__m128d b) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_hsub_pd(a, b);
  #else
    return simde_mm_sub_pd(simde_x_mm_deinterleaveeven_pd(a, b), simde_x_mm_deinterleaveodd_pd(a, b));
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsub_pd(a, b) simde_mm_hsub_pd(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_hsub_ps (simde__m128 a, simde__m128 b) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_hsub_ps(a, b);
  #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
    float32x4x2_t t = vuzpq_f32(simde__m128_to_neon_f32(a), simde__m128_to_neon_f32(b));
    return simde__m128_from_neon_f32(vaddq_f32(t.val[0], vnegq_f32(t.val[1])));
  #else
    return simde_mm_sub_ps(simde_x_mm_deinterleaveeven_ps(a, b), simde_x_mm_deinterleaveodd_ps(a, b));
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_hsub_ps(a, b) simde_mm_hsub_ps(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i
simde_mm_lddqu_si128 (simde__m128i const* mem_addr) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_lddqu_si128(mem_addr);
  #else
    simde__m128i_private r_;

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i32 = vld1q_s32(HEDLEY_REINTERPRET_CAST(int32_t const*, mem_addr));
    #else
      simde_memcpy(&r_, mem_addr, sizeof(r_));
    #endif

    return simde__m128i_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_lddqu_si128(mem_addr) simde_mm_lddqu_si128(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_loaddup_pd (simde_float64 const* mem_addr) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_loaddup_pd(mem_addr);
  #else
    simde__m128d_private r_;

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_f64 = vdupq_n_f64(*mem_addr);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_i64 = vdupq_n_s64(*HEDLEY_REINTERPRET_CAST(int64_t const*, mem_addr));
    #else
      r_.f64[0] = *mem_addr;
      r_.f64[1] = *mem_addr;
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_loaddup_pd(mem_addr) simde_mm_loaddup_pd(mem_addr)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_movedup_pd (simde__m128d a) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_movedup_pd(a);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_f64 = vdupq_laneq_f64(a_.neon_f64, 0);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i64x2_shuffle(a_.wasm_v128, a_.wasm_v128, 0, 0);
    #elif defined(SIMDE_VECTOR_SUBSCRIPT_OPS) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f64 = SIMDE_SHUFFLE_VECTOR_(64, 16, a_.f64, a_.f64, 0, 0);
    #else
      r_.f64[0] = a_.f64[0];
      r_.f64[1] = a_.f64[0];
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_movedup_pd(a) simde_mm_movedup_pd(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_movehdup_ps (simde__m128 a) {
  #if defined(SIMDE_X86_SSE3_NATIVE)
    return _mm_movehdup_ps(a);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_f32 = vtrn2q_f32(a_.neon_f32, a_.neon_f32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_shuffle(a_.wasm_v128, a_.wasm_v128, 1, 1, 3, 3);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.f32, a_.f32, 1, 1, 3, 3);
    #else
      r_.f32[0] = a_.f32[1];
      r_.f32[1] = a_.f32[1];
      r_.f32[2] = a_.f32[3];
      r_.f32[3] = a_.f32[3];
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_movehdup_ps(a) simde_mm_movehdup_ps(a)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_moveldup_ps (simde__m128 a) {
  #if defined(SIMDE__SSE3_NATIVE)
    return _mm_moveldup_ps(a);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_f32 = vtrn1q_f32(a_.neon_f32, a_.neon_f32);
    #elif defined(SIMDE_WASM_SIMD128_NATIVE)
      r_.wasm_v128 = wasm_i32x4_shuffle(a_.wasm_v128, a_.wasm_v128, 0, 0, 2, 2);
    #elif (SIMDE_NATURAL_VECTOR_SIZE > 0) && defined(SIMDE_SHUFFLE_VECTOR_)
      r_.f32 = SIMDE_SHUFFLE_VECTOR_(32, 16, a_.f32, a_.f32, 0, 0, 2, 2);
    #else
      r_.f32[0] = a_.f32[0];
      r_.f32[1] = a_.f32[0];
      r_.f32[2] = a_.f32[2];
      r_.f32[3] = a_.f32[2];
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_SSE3_ENABLE_NATIVE_ALIASES)
#  define _mm_moveldup_ps(a) simde_mm_moveldup_ps(a)
#endif

SIMDE_END_DECLS_

HEDLEY_DIAGNOSTIC_POP

#endif /* !defined(SIMDE_X86_SSE3_H) */
