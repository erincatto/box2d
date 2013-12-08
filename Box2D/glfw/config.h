//========================================================================
// GLFW 3.0 - www.glfw.org
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
// As config.h.in, this file is used by CMake to produce the config.h shared
// configuration header file.  If you are adding a feature requiring
// conditional compilation, this is the proper place to add the macros.
//========================================================================
// As config.h, this file defines compile-time build options and macros for
// all platforms supported by GLFW.  As this is a generated file, don't modify
// it.  Instead, you should modify the config.h.in file.
//========================================================================

// GLFW doesn't like to be embedded as source, but that is what I'm doing.
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

// TODO get defines for other platforms

#define _GLFW_USE_OPENGL
#define _GLFW_VERSION_FULL "3.0.3"

