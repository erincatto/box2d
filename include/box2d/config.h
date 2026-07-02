// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// Box2D compile-time options.
//
// Normally set by the CMake BOX2D_* options. If you build Box2D without its
// CMake (dropping the sources into another project), set them here so the
// library and your code agree. Edit this file, or keep your settings elsewhere
// and point Box2D at them from your build:
//
//   #define BOX2D_USER_CONFIG "my_box2d_config.h"
//
// A define passed on the compiler command line still wins over this file.

// Large world mode. Stores world positions in double precision. Affects ABI.
//#define BOX2D_DOUBLE_PRECISION

// Build the scalar fallback instead of SSE2/NEON.
//#define BOX2D_DISABLE_SIMD

// Enable internal validation in debug builds.
//#define BOX2D_VALIDATE

// Decorate the public API with your own export macro instead of Box2D's
// box2d_EXPORTS/BOX2D_DLL scheme, for example when compiling Box2D into another
// shared library. A single value cannot switch between dllexport and dllimport,
// so this suits embedding more than shipping Box2D as its own DLL.
//#define BOX2D_EXPORT MYENGINE_API
