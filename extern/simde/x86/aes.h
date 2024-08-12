/* MIT License
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
 */

#if !defined(SIMDE_X86_AES_H)
#define SIMDE_X86_AES_H

/*
 * Advanced Encryption Standard
 * @author Dani Huertas
 * @email huertas.dani@gmail.com
 *
 * Based on the document FIPS PUB 197
 */

#include "sse2.h"

/*
 * Multiplication in GF(2^8)
 * http://en.wikipedia.org/wiki/Finite_field_arithmetic
 * Irreducible polynomial m(x) = x8 + x4 + x3 + x + 1
 *
 * NOTE: This function can be easily replaced with a look up table for a speed
 *       boost, at the expense of an increase in memory size.

SIMDE_FUNCTION_ATTRIBUTES
uint8_t gmult(uint8_t a, uint8_t b) {
  uint8_t p = 0, i = 0, hbs = 0;

  for (i = 0; i < 8; i++) {
    if (b & 1) {
      p ^= a;
    }

    hbs = a & 0x80;
    a <<= 1;
    if (hbs) a ^= 0x1b; // 0000 0001 0001 1011
    b >>= 1;
  }

  return (uint8_t)p;
}
 */

#if !(defined(SIMDE_ARM_NEON_A32V7_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO))

#include "../simde-aes.h"

/*
 * Transformation in the Cipher and Inverse Cipher in which a Round
 * Key is added to the State using an XOR operation. The length of a
 * Round Key equals the size of the State (i.e., for Nb = 4, the Round
 * Key length equals 128 bits/16 bytes).
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_add_round_key(uint8_t *state, simde__m128i_private w, uint8_t r) {

  int Nb = simde_x_aes_Nb;
  uint8_t c;

  for (c = 0; c < Nb; c++) {
    state[Nb*0+c] = state[Nb*0+c]^w.u8[4*Nb*r+4*c+0];
    state[Nb*1+c] = state[Nb*1+c]^w.u8[4*Nb*r+4*c+1];
    state[Nb*2+c] = state[Nb*2+c]^w.u8[4*Nb*r+4*c+2];
    state[Nb*3+c] = state[Nb*3+c]^w.u8[4*Nb*r+4*c+3];
  }
}

/*
 * Transformation in the Cipher that takes all of the columns of the
 * State and mixes their data (independently of one another) to
 * produce new columns.
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_mix_columns(uint8_t *state) {

  int Nb = simde_x_aes_Nb;
  // uint8_t k[] = {0x02, 0x01, 0x01, 0x03}; // a(x) = {02} + {01}x + {01}x2 + {03}x3
  uint8_t i, j, col[4], res[4];

  for (j = 0; j < Nb; j++) {
    for (i = 0; i < 4; i++) {
      col[i] = state[Nb*i+j];
    }

    //coef_mult(k, col, res);
    simde_x_aes_coef_mult_lookup(0, col, res);

    for (i = 0; i < 4; i++) {
      state[Nb*i+j] = res[i];
    }
  }
}

/*
 * Transformation in the Inverse Cipher that is the inverse of
 * MixColumns().
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_inv_mix_columns(uint8_t *state) {

  int Nb = simde_x_aes_Nb;
  // uint8_t k[] = {0x0e, 0x09, 0x0d, 0x0b}; // a(x) = {0e} + {09}x + {0d}x2 + {0b}x3
  uint8_t i, j, col[4], res[4];

  for (j = 0; j < Nb; j++) {
    for (i = 0; i < 4; i++) {
      col[i] = state[Nb*i+j];
    }

    //coef_mult(k, col, res);
    simde_x_aes_coef_mult_lookup(4, col, res);

    for (i = 0; i < 4; i++) {
      state[Nb*i+j] = res[i];
    }
  }
}

/*
 * Transformation in the Cipher that processes the State by cyclically
 * shifting the last three rows of the State by different offsets.
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_shift_rows(uint8_t *state) {

  int Nb = simde_x_aes_Nb;
  uint8_t i, k, s, tmp;

  for (i = 1; i < 4; i++) {
    // shift(1,4)=1; shift(2,4)=2; shift(3,4)=3
    // shift(r, 4) = r;
    s = 0;
    while (s < i) {
      tmp = state[Nb*i+0];

      for (k = 1; k < Nb; k++) {
        state[Nb*i+k-1] = state[Nb*i+k];
      }

      state[Nb*i+Nb-1] = tmp;
      s++;
    }
  }
}

/*
 * Transformation in the Inverse Cipher that is the inverse of
 * ShiftRows().
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_inv_shift_rows(uint8_t *state) {

  uint8_t Nb = simde_x_aes_Nb;
  uint8_t i, k, s, tmp;

  for (i = 1; i < 4; i++) {
    s = 0;
    while (s < i) {
      tmp = state[Nb*i+Nb-1];

      for (k = Nb-1; k > 0; k--) {
        state[Nb*i+k] = state[Nb*i+k-1];
      }

      state[Nb*i+0] = tmp;
      s++;
    }
  }
}

/*
 * Transformation in the Cipher that processes the State using a non
 * linear byte substitution table (S-box) that operates on each of the
 * State bytes independently.
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_sub_bytes(uint8_t *state) {

  int Nb = simde_x_aes_Nb;
  uint8_t i, j;

  for (i = 0; i < 4; i++) {
    for (j = 0; j < Nb; j++) {
      // s_box row: yyyy ----
      // s_box col: ---- xxxx
      // s_box[16*(yyyy) + xxxx] == s_box[yyyyxxxx]
      state[Nb*i+j] = simde_x_aes_s_box[state[Nb*i+j]];
    }
  }
}

/*
 * Transformation in the Inverse Cipher that is the inverse of
 * SubBytes().
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_inv_sub_bytes(uint8_t *state) {

  int Nb = simde_x_aes_Nb;
  uint8_t i, j;

  for (i = 0; i < 4; i++) {
    for (j = 0; j < Nb; j++) {
      state[Nb*i+j] = simde_x_aes_inv_s_box[state[Nb*i+j]];
    }
  }
}

/*
 * Performs the AES cipher operation
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_enc(simde__m128i_private in, simde__m128i_private *out, simde__m128i_private w, int is_last) {

  int Nb = simde_x_aes_Nb;
  uint8_t state[4*simde_x_aes_Nb];
  uint8_t r = 0, i, j;

  for (i = 0; i < 4; i++) {
    for (j = 0; j < Nb; j++) {
      state[Nb*i+j] = in.u8[i+4*j];
    }
  }

  simde_x_aes_sub_bytes(state);
  simde_x_aes_shift_rows(state);

  if (!is_last)
    simde_x_aes_mix_columns(state);

  simde_x_aes_add_round_key(state, w, r);

  for (i = 0; i < 4; i++) {
    for (j = 0; j < Nb; j++) {
      out->u8[i+4*j] = state[Nb*i+j];
    }
  }
}

/*
 * Performs the AES inverse cipher operation
 */
SIMDE_FUNCTION_ATTRIBUTES
void simde_x_aes_dec(simde__m128i_private in, simde__m128i_private *out, simde__m128i_private w, int is_last) {

  int Nb = simde_x_aes_Nb;
  uint8_t state[4*simde_x_aes_Nb];
  uint8_t r = 0, i, j;

  for (i = 0; i < 4; i++) {
    for (j = 0; j < Nb; j++) {
      state[Nb*i+j] = in.u8[i+4*j];
    }
  }

  simde_x_aes_inv_shift_rows(state);
  simde_x_aes_inv_sub_bytes(state);

  if (!is_last)
    simde_x_aes_inv_mix_columns(state);

  simde_x_aes_add_round_key(state, w, r);

  for (i = 0; i < 4; i++) {
    for (j = 0; j < Nb; j++) {
      out->u8[i+4*j] = state[Nb*i+j];
    }
  }
}
#endif // if !(defined(SIMDE_ARM_NEON_A32V7_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO))

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i simde_mm_aesenc_si128(simde__m128i a, simde__m128i round_key) {
  #if defined(SIMDE_X86_AES_NATIVE)
    return _mm_aesenc_si128(a, round_key);
  #else
    simde__m128i_private result_;
    simde__m128i_private a_ = simde__m128i_to_private(a);
    simde__m128i_private round_key_ = simde__m128i_to_private(round_key);
    #if defined(SIMDE_ARM_NEON_A32V8_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO)
      result_.neon_u8 = veorq_u8(
        vaesmcq_u8(vaeseq_u8(a_.neon_u8, vdupq_n_u8(0))),
        round_key_.neon_u8);
    #else
      simde_x_aes_enc(a_, &result_, round_key_, 0);
    #endif
    return simde__m128i_from_private(result_);
  #endif
}
#if defined(SIMDE_X86_AES_ENABLE_NATIVE_ALIASES)
  #define _mm_aesenc_si128(a, b) simde_mm_aesenc_si128(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i simde_mm_aesdec_si128(simde__m128i a, simde__m128i round_key) {
  #if defined(SIMDE_X86_AES_NATIVE)
    return _mm_aesdec_si128(a, round_key);
  #else
    simde__m128i_private result_;
    simde__m128i_private a_ = simde__m128i_to_private(a);
    simde__m128i_private round_key_ = simde__m128i_to_private(round_key);
    #if defined(SIMDE_ARM_NEON_A32V8_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO)
      result_.neon_u8 = veorq_u8(
        vaesimcq_u8(vaesdq_u8(a_.neon_u8, vdupq_n_u8(0))),
        round_key_.neon_u8);
    #else
      simde_x_aes_dec(a_, &result_, round_key_, 0);
    #endif
    return simde__m128i_from_private(result_);
  #endif
}
#if defined(SIMDE_X86_AES_ENABLE_NATIVE_ALIASES)
  #define _mm_aesdec_si128(a, b) simde_mm_aesdec_si128(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i simde_mm_aesenclast_si128(simde__m128i a, simde__m128i round_key) {
  #if defined(SIMDE_X86_AES_NATIVE)
    return _mm_aesenclast_si128(a, round_key);
  #else
    simde__m128i_private result_;
    simde__m128i_private a_ = simde__m128i_to_private(a);
    simde__m128i_private round_key_ = simde__m128i_to_private(round_key);
    #if defined(SIMDE_ARM_NEON_A32V8_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO)
      result_.neon_u8 = vaeseq_u8(a_.neon_u8, vdupq_n_u8(0));
      result_.neon_i32 = veorq_s32(result_.neon_i32, round_key_.neon_i32); // _mm_xor_si128
    #else
      simde_x_aes_enc(a_, &result_, round_key_, 1);
    #endif
    return simde__m128i_from_private(result_);
  #endif
}
#if defined(SIMDE_X86_AES_ENABLE_NATIVE_ALIASES)
  #define _mm_aesenclast_si128(a, b) simde_mm_aesenclast_si128(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i simde_mm_aesdeclast_si128(simde__m128i a, simde__m128i round_key) {
  #if defined(SIMDE_X86_AES_NATIVE)
    return _mm_aesdeclast_si128(a, round_key);
  #else
    simde__m128i_private result_;
    simde__m128i_private a_ = simde__m128i_to_private(a);
    simde__m128i_private round_key_ = simde__m128i_to_private(round_key);
    #if defined(SIMDE_ARM_NEON_A32V8_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO)
      result_.neon_u8 = veorq_u8(
        vaesdq_u8(a_.neon_u8, vdupq_n_u8(0)),
        round_key_.neon_u8);
    #else
      simde_x_aes_dec(a_, &result_, round_key_, 1);
    #endif
    return simde__m128i_from_private(result_);
  #endif
}
#if defined(SIMDE_X86_AES_ENABLE_NATIVE_ALIASES)
  #define _mm_aesdeclast_si128(a, b) simde_mm_aesdeclast_si128(a, b)
#endif

SIMDE_FUNCTION_ATTRIBUTES
simde__m128i simde_mm_aesimc_si128(simde__m128i a) {
  #if defined(SIMDE_X86_AES_NATIVE)
    return _mm_aesimc_si128(a);
  #else
    simde__m128i_private result_ = simde__m128i_to_private(simde_mm_setzero_si128());
    simde__m128i_private a_ = simde__m128i_to_private(a);

    #if defined(SIMDE_ARM_NEON_A32V8_NATIVE) && defined(SIMDE_ARCH_ARM_CRYPTO)
      result_.neon_u8 = vaesimcq_u8(a_.neon_u8);
    #else
      int Nb = simde_x_aes_Nb;
      // uint8_t k[] = {0x0e, 0x09, 0x0d, 0x0b}; // a(x) = {0e} + {09}x + {0d}x2 + {0b}x3
      uint8_t i, j, col[4], res[4];

      for (j = 0; j < Nb; j++) {
        for (i = 0; i < 4; i++) {
          col[i] = a_.u8[Nb*j+i];
        }

        //coef_mult(k, col, res);
        simde_x_aes_coef_mult_lookup(4, col, res);

        for (i = 0; i < 4; i++) {
          result_.u8[Nb*j+i] = res[i];
        }
      }
    #endif
    return simde__m128i_from_private(result_);
  #endif
}
#if defined(SIMDE_X86_AES_ENABLE_NATIVE_ALIASES)
  #define _mm_aesimc_si128(a) simde_mm_aesimc_si128(a)
#endif

#undef simde_x_aes_Nb

#endif /* !defined(SIMDE_X86_AES_H) */
