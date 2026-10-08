// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#define B2_SIMD_WIDTH 8

#include "core.h"

#if defined( B2_SIMD_AVX2 )

#if defined( _MSC_VER ) && !defined( __clang__ ) && !defined( __AVX2__ )
#error "MSVC must compile this file with /arch:AVX2, or define BOX2D_DISABLE_SIMD"
#endif

#include <immintrin.h>

B2_AVX2_BEGIN

#include "contact_solver_wide.inl"

B2_AVX2_END

#endif
