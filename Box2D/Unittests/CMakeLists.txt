# Copyright (c) 2013 Google, Inc.
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
cmake_minimum_required(VERSION 2.8)

# Set some variables describing file locations.
#
# NOTE: This configuration assumes that gtest was built installed in the "inst"
# directory. Please see http://blah for instructions.
#
# Also keep in mind that this will need to be cleaned up were we ever to make
# this code public domain, we will wish to remove this relative path and
# replace it with something less google-specific.
set(GUNIT_INCDIR "${CMAKE_CURRENT_SOURCE_DIR}/../googletest/include")
set(GTEST_LIBDIR "${CMAKE_CURRENT_SOURCE_DIR}/../googletest")

# Include helper functions and macros used by Google Test.
include(${GTEST_LIBDIR}/cmake/internal_utils.cmake)
config_compiler_and_linker()
string(REPLACE "-W4" "-W3" cxx_default "${cxx_default}")
string(REPLACE "-Wshadow" "" cxx_default "${cxx_default}")
string(REPLACE "-Wextra" "" cxx_default "${cxx_default}")

# This is the directory into which the executables are built.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${GUNIT_INCDIR} "${CMAKE_CURRENT_SOURCE_DIR}"
                                    "${CMAKE_CURRENT_SOURCE_DIR}/..")

# These are support sources that are used for unit tests.
set(SUPPORT_SRCS BodyTracker.cpp)

# Common libraries for tests.
if(NOT MSVC)
  find_package(Threads)
endif()
set(COMMON_LIBS "gtest;Box2D;${CMAKE_THREAD_LIBS_INIT}")

# This define allows for test-specific code. For example, you can use it with
# the FRIEND_TEST macro to expose private member functions for testing.
add_definitions(-DBOX2D_UNIT_TESTS=1)

# PUT ADDITIONAL UNIT TEST BINARIES BELOW!
# The commands should be of the form:
#
# test_executable(<test-name>)
#
# Where <test-name> is the name of the output test executable and the basename
# of the source file for the test.  For example, test_executable(CallbackTests)
# generates an executable called CallbackTests which is the result of compiling
# CallbackTests.cpp along with any files specified by the SUPPORT_SRCS variable.

function(test_executable name)
  cxx_executable_with_flags(${name}Tests "${cxx_default}" "${COMMON_LIBS}"
                            ${name}/${name}Tests.cpp ${SUPPORT_SRCS})
endfunction()

test_executable(BlockAllocator)
test_executable(BodyContacts)
test_executable(Callback)
test_executable(Color)
test_executable(Common)
test_executable(Confinement)
test_executable(Conservation)
test_executable(FreeList)
test_executable(Function)
test_executable(HelloWorld)
test_executable(IntrusiveList)
test_executable(SlabAllocator)
test_executable(TrackedBlock)
