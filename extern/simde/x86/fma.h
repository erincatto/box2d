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
 *   2019      Evan Nemerson <evan@nemerson.com>
 */

#if !defined(SIMDE_X86_FMA_H)
#define SIMDE_X86_FMA_H

#include "avx.h"

#if !defined(SIMDE_X86_FMA_NATIVE) && defined(SIMDE_ENABLE_NATIVE_ALIASES)
#  define SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES
#endif

HEDLEY_DIAGNOSTIC_PUSH
SIMDE_DISABLE_UNWANTED_DIAGNOSTICS
SIMDE_BEGIN_DECLS_

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fmadd_pd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmadd_pd(a, b, c);
  #else
    simde__m128d_private
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b),
      c_ = simde__m128d_to_private(c),
      r_;

    #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE) || defined(SIMDE_ZARCH_ZVECTOR_13_NATIVE)
      r_.altivec_f64 = vec_madd(a_.altivec_f64, b_.altivec_f64, c_.altivec_f64);
    #elif defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_f64 = vfmaq_f64(c_.neon_f64, b_.neon_f64, a_.neon_f64);
    #elif defined(simde_math_fma) && (defined(__FP_FAST_FMA) || defined(FP_FAST_FMA))
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = simde_math_fma(a_.f64[i], b_.f64[i], c_.f64[i]);
      }
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = (a_.f64[i] * b_.f64[i]) + c_.f64[i];
      }
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmadd_pd
  #define _mm_fmadd_pd(a, b, c) simde_mm_fmadd_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_fmadd_pd (simde__m256d a, simde__m256d b, simde__m256d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmadd_pd(a, b, c);
  #else
    return simde_mm256_add_pd(simde_mm256_mul_pd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmadd_pd
  #define _mm256_fmadd_pd(a, b, c) simde_mm256_fmadd_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fmadd_ps (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmadd_ps(a, b, c);
  #else
    simde__m128_private
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b),
      c_ = simde__m128_to_private(c),
      r_;

    #if defined(SIMDE_POWER_ALTIVEC_P7_NATIVE)
      r_.altivec_f32 = vec_madd(a_.altivec_f32, b_.altivec_f32, c_.altivec_f32);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE) && defined(SIMDE_ARCH_ARM_FMA)
      r_.neon_f32 = vfmaq_f32(c_.neon_f32, b_.neon_f32, a_.neon_f32);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_f32 = vmlaq_f32(c_.neon_f32, b_.neon_f32, a_.neon_f32);
    #elif defined(simde_math_fmaf) && (defined(__FP_FAST_FMAF) || defined(FP_FAST_FMAF))
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = simde_math_fmaf(a_.f32[i], b_.f32[i], c_.f32[i]);
      }
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = (a_.f32[i] * b_.f32[i]) + c_.f32[i];
      }
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmadd_ps
  #define _mm_fmadd_ps(a, b, c) simde_mm_fmadd_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_fmadd_ps (simde__m256 a, simde__m256 b, simde__m256 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmadd_ps(a, b, c);
  #elif SIMDE_NATURAL_VECTOR_SIZE_LE(128)
    simde__m256_private
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b),
      c_ = simde__m256_to_private(c),
      r_;

    for (size_t i = 0 ; i < (sizeof(r_.m128) / sizeof(r_.m128[0])) ; i++) {
      r_.m128[i] = simde_mm_fmadd_ps(a_.m128[i], b_.m128[i], c_.m128[i]);
    }

    return simde__m256_from_private(r_);
  #else
    return simde_mm256_add_ps(simde_mm256_mul_ps(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmadd_ps
  #define _mm256_fmadd_ps(a, b, c) simde_mm256_fmadd_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fmadd_sd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fmadd_sd(a, b, c);
  #else
    return simde_mm_add_sd(simde_mm_mul_sd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmadd_sd
  #define _mm_fmadd_sd(a, b, c) simde_mm_fmadd_sd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fmadd_ss (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fmadd_ss(a, b, c);
  #else
    return simde_mm_add_ss(simde_mm_mul_ss(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmadd_ss
  #define _mm_fmadd_ss(a, b, c) simde_mm_fmadd_ss(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fmaddsub_pd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmaddsub_pd(a, b, c);
  #else
    return simde_mm_addsub_pd(simde_mm_mul_pd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmaddsub_pd
  #define _mm_fmaddsub_pd(a, b, c) simde_mm_fmaddsub_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_fmaddsub_pd (simde__m256d a, simde__m256d b, simde__m256d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmaddsub_pd(a, b, c);
  #else
    return simde_mm256_addsub_pd(simde_mm256_mul_pd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmaddsub_pd
  #define _mm256_fmaddsub_pd(a, b, c) simde_mm256_fmaddsub_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fmaddsub_ps (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmaddsub_ps(a, b, c);
  #else
    return simde_mm_addsub_ps(simde_mm_mul_ps(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmaddsub_ps
  #define _mm_fmaddsub_ps(a, b, c) simde_mm_fmaddsub_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_fmaddsub_ps (simde__m256 a, simde__m256 b, simde__m256 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmaddsub_ps(a, b, c);
  #else
    return simde_mm256_addsub_ps(simde_mm256_mul_ps(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmaddsub_ps
  #define _mm256_fmaddsub_ps(a, b, c) simde_mm256_fmaddsub_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fmsub_pd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmsub_pd(a, b, c);
  #else
    return simde_mm_sub_pd(simde_mm_mul_pd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmsub_pd
  #define _mm_fmsub_pd(a, b, c) simde_mm_fmsub_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_fmsub_pd (simde__m256d a, simde__m256d b, simde__m256d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmsub_pd(a, b, c);
  #else
    return simde_mm256_sub_pd(simde_mm256_mul_pd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmsub_pd
  #define _mm256_fmsub_pd(a, b, c) simde_mm256_fmsub_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fmsub_ps (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmsub_ps(a, b, c);
  #else
    return simde_mm_sub_ps(simde_mm_mul_ps(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmsub_ps
  #define _mm_fmsub_ps(a, b, c) simde_mm_fmsub_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_fmsub_ps (simde__m256 a, simde__m256 b, simde__m256 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmsub_ps(a, b, c);
  #else
    return simde_mm256_sub_ps(simde_mm256_mul_ps(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmsub_ps
  #define _mm256_fmsub_ps(a, b, c) simde_mm256_fmsub_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fmsub_sd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fmsub_sd(a, b, c);
  #else
    return simde_mm_sub_sd(simde_mm_mul_sd(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmsub_sd
  #define _mm_fmsub_sd(a, b, c) simde_mm_fmsub_sd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fmsub_ss (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fmsub_ss(a, b, c);
  #else
    return simde_mm_sub_ss(simde_mm_mul_ss(a, b), c);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmsub_ss
  #define _mm_fmsub_ss(a, b, c) simde_mm_fmsub_ss(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fmsubadd_pd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmsubadd_pd(a, b, c);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b),
      c_ = simde__m128d_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i += 2) {
      r_.f64[  i  ] = (a_.f64[  i  ] * b_.f64[  i  ]) + c_.f64[  i  ];
      r_.f64[i + 1] = (a_.f64[i + 1] * b_.f64[i + 1]) - c_.f64[i + 1];
    }

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmsubadd_pd
  #define _mm_fmsubadd_pd(a, b, c) simde_mm_fmsubadd_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_fmsubadd_pd (simde__m256d a, simde__m256d b, simde__m256d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmsubadd_pd(a, b, c);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b),
      c_ = simde__m256d_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i += 2) {
      r_.f64[  i  ] = (a_.f64[  i  ] * b_.f64[  i  ]) + c_.f64[  i  ];
      r_.f64[i + 1] = (a_.f64[i + 1] * b_.f64[i + 1]) - c_.f64[i + 1];
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmsubadd_pd
  #define _mm256_fmsubadd_pd(a, b, c) simde_mm256_fmsubadd_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fmsubadd_ps (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fmsubadd_ps(a, b, c);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b),
      c_ = simde__m128_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i += 2) {
      r_.f32[  i  ] = (a_.f32[  i  ] * b_.f32[  i  ]) + c_.f32[  i  ];
      r_.f32[i + 1] = (a_.f32[i + 1] * b_.f32[i + 1]) - c_.f32[i + 1];
    }

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fmsubadd_ps
  #define _mm_fmsubadd_ps(a, b, c) simde_mm_fmsubadd_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_fmsubadd_ps (simde__m256 a, simde__m256 b, simde__m256 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fmsubadd_ps(a, b, c);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b),
      c_ = simde__m256_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i += 2) {
      r_.f32[  i  ] = (a_.f32[  i  ] * b_.f32[  i  ]) + c_.f32[  i  ];
      r_.f32[i + 1] = (a_.f32[i + 1] * b_.f32[i + 1]) - c_.f32[i + 1];
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fmsubadd_ps
  #define _mm256_fmsubadd_ps(a, b, c) simde_mm256_fmsubadd_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fnmadd_pd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fnmadd_pd(a, b, c);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b),
      c_ = simde__m128d_to_private(c);

    #if defined(SIMDE_ARM_NEON_A64V8_NATIVE)
      r_.neon_f64 = vfmsq_f64(c_.neon_f64, a_.neon_f64, b_.neon_f64);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
        r_.f64[i] = -(a_.f64[i] * b_.f64[i]) + c_.f64[i];
      }
    #endif

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmadd_pd
  #define _mm_fnmadd_pd(a, b, c) simde_mm_fnmadd_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_fnmadd_pd (simde__m256d a, simde__m256d b, simde__m256d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fnmadd_pd(a, b, c);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b),
      c_ = simde__m256d_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = -(a_.f64[i] * b_.f64[i]) + c_.f64[i];
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fnmadd_pd
  #define _mm256_fnmadd_pd(a, b, c) simde_mm256_fnmadd_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fnmadd_ps (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fnmadd_ps(a, b, c);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b),
      c_ = simde__m128_to_private(c);

    #if defined(SIMDE_ARM_NEON_A32V7_NATIVE) && defined(SIMDE_ARCH_ARM_FMA)
      r_.neon_f32 = vfmsq_f32(c_.neon_f32, a_.neon_f32, b_.neon_f32);
    #elif defined(SIMDE_ARM_NEON_A32V7_NATIVE)
      r_.neon_f32 = vmlsq_f32(c_.neon_f32, a_.neon_f32, b_.neon_f32);
    #else
      SIMDE_VECTORIZE
      for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
        r_.f32[i] = -(a_.f32[i] * b_.f32[i]) + c_.f32[i];
      }
    #endif

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmadd_ps
  #define _mm_fnmadd_ps(a, b, c) simde_mm_fnmadd_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_fnmadd_ps (simde__m256 a, simde__m256 b, simde__m256 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fnmadd_ps(a, b, c);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b),
      c_ = simde__m256_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = -(a_.f32[i] * b_.f32[i]) + c_.f32[i];
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fnmadd_ps
  #define _mm256_fnmadd_ps(a, b, c) simde_mm256_fnmadd_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fnmadd_sd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fnmadd_sd(a, b, c);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b),
      c_ = simde__m128d_to_private(c);

    r_ = a_;
    r_.f64[0] = -(a_.f64[0] * b_.f64[0]) + c_.f64[0];

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmadd_sd
  #define _mm_fnmadd_sd(a, b, c) simde_mm_fnmadd_sd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fnmadd_ss (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fnmadd_ss(a, b, c);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b),
      c_ = simde__m128_to_private(c);

    r_ = a_;
    r_.f32[0] = -(a_.f32[0] * b_.f32[0]) + c_.f32[0];

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmadd_ss
  #define _mm_fnmadd_ss(a, b, c) simde_mm_fnmadd_ss(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fnmsub_pd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fnmsub_pd(a, b, c);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b),
      c_ = simde__m128d_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = -(a_.f64[i] * b_.f64[i]) - c_.f64[i];
    }

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmsub_pd
  #define _mm_fnmsub_pd(a, b, c) simde_mm_fnmsub_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256d
simde_mm256_fnmsub_pd (simde__m256d a, simde__m256d b, simde__m256d c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fnmsub_pd(a, b, c);
  #else
    simde__m256d_private
      r_,
      a_ = simde__m256d_to_private(a),
      b_ = simde__m256d_to_private(b),
      c_ = simde__m256d_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f64) / sizeof(r_.f64[0])) ; i++) {
      r_.f64[i] = -(a_.f64[i] * b_.f64[i]) - c_.f64[i];
    }

    return simde__m256d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fnmsub_pd
  #define _mm256_fnmsub_pd(a, b, c) simde_mm256_fnmsub_pd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fnmsub_ps (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm_fnmsub_ps(a, b, c);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b),
      c_ = simde__m128_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = -(a_.f32[i] * b_.f32[i]) - c_.f32[i];
    }

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmsub_ps
  #define _mm_fnmsub_ps(a, b, c) simde_mm_fnmsub_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m256
simde_mm256_fnmsub_ps (simde__m256 a, simde__m256 b, simde__m256 c) {
  #if defined(SIMDE_X86_FMA_NATIVE)
    return _mm256_fnmsub_ps(a, b, c);
  #else
    simde__m256_private
      r_,
      a_ = simde__m256_to_private(a),
      b_ = simde__m256_to_private(b),
      c_ = simde__m256_to_private(c);

    SIMDE_VECTORIZE
    for (size_t i = 0 ; i < (sizeof(r_.f32) / sizeof(r_.f32[0])) ; i++) {
      r_.f32[i] = -(a_.f32[i] * b_.f32[i]) - c_.f32[i];
    }

    return simde__m256_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm256_fnmsub_ps
  #define _mm256_fnmsub_ps(a, b, c) simde_mm256_fnmsub_ps(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128d
simde_mm_fnmsub_sd (simde__m128d a, simde__m128d b, simde__m128d c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fnmsub_sd(a, b, c);
  #else
    simde__m128d_private
      r_,
      a_ = simde__m128d_to_private(a),
      b_ = simde__m128d_to_private(b),
      c_ = simde__m128d_to_private(c);

    r_ = a_;
    r_.f64[0] = -(a_.f64[0] * b_.f64[0]) - c_.f64[0];

    return simde__m128d_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmsub_sd
  #define _mm_fnmsub_sd(a, b, c) simde_mm_fnmsub_sd(a, b, c)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128
simde_mm_fnmsub_ss (simde__m128 a, simde__m128 b, simde__m128 c) {
  #if defined(SIMDE_X86_FMA_NATIVE) && !defined(SIMDE_BUG_MCST_LCC_FMA_WRONG_RESULT)
    return _mm_fnmsub_ss(a, b, c);
  #else
    simde__m128_private
      r_,
      a_ = simde__m128_to_private(a),
      b_ = simde__m128_to_private(b),
      c_ = simde__m128_to_private(c);

    r_ = simde__m128_to_private(a);
    r_.f32[0] = -(a_.f32[0] * b_.f32[0]) - c_.f32[0];

    return simde__m128_from_private(r_);
  #endif
}
#if defined(SIMDE_X86_FMA_ENABLE_NATIVE_ALIASES)
  #undef _mm_fnmsub_ss
  #define _mm_fnmsub_ss(a, b, c) simde_mm_fnmsub_ss(a, b, c)
#endif

SIMDE_END_DECLS_

HEDLEY_DIAGNOSTIC_POP

#endif /* !defined(SIMDE_X86_FMA_H) */
