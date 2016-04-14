//========================================================================
// GLFW 3.1 - www.glfw.org
//------------------------------------------------------------------------
// Copyright (c) 2010 Camilla Berglund <elmindreda@elmindreda.org>
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would
//    be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not
//    be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source
//    distribution.
//
//========================================================================
// As glfw_config.h.in, this file is used by CMake to produce the
// glfw_config.h configuration header file.  If you are adding a feature
// requiring conditional compilation, this is where to add the macro.
//========================================================================
// As glfw_config.h, this file defines compile-time option macros for a
// specific platform and development environment.  If you are using the
// GLFW CMake files, modify glfw_config.h.in instead of this file.  If you
// are using your own build system, make this file define the appropriate
// macros in whatever way is suitable.
//========================================================================

#if defined(WIN32)
#define _GLFW_WIN32
#define _GLFW_WGL
#endif

#if defined(__APPLE__)
#define _GLFW_COCOA
#define _GLFW_NSGL
//#define _GLFW_USE_CHDIR
#define _GLFW_USE_MENUBAR
#endif

#define _GLFW_USE_OPENGL
