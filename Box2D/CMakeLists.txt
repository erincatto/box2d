cmake_minimum_required(VERSION 2.8)

project(Box2D)
find_package (Threads)

if(UNIX)
  set(BOX2D_INSTALL_BY_DEFAULT ON)
else(UNIX)
  set(BOX2D_INSTALL_BY_DEFAULT OFF)
endif(UNIX)

# Box2D build configuration.
# Force gtest to compile against the shared CRT.
if(MSVC)
  set(gtest_force_shared_crt ON CACHE INTERNAL "")
endif()
# Disable install of Box2D by default on all platforms.
set(BOX2D_INSTALL_BY_DEFAULT OFF)

option(BOX2D_INSTALL "Install Box2D libs, includes, and CMake scripts" ${BOX2D_INSTALL_BY_DEFAULT})
option(BOX2D_INSTALL_DOC "Install Box2D documentation" OFF)
option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)
option(BOX2D_BUILD_EXAMPLES "Build Box2D examples" ON)
option(BOX2D_BUILD_UNITTESTS "Build Box2D Unit Tests" ON)
# NOTE: Code coverage only works on Linux & OSX.
option(BOX2D_CODE_COVERAGE "Enable the code coverage build option." OFF)

set(BOX2D_VERSION 2.3.2)
set(LIB_INSTALL_DIR lib${LIB_SUFFIX})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
   "MinSizeRel" "RelWithDebInfo")
endif()

# Detect clang
if(CMAKE_CXX_COMPILER MATCHES ".*clang")
  set(CMAKE_COMPILER_IS_CLANGXX 1)
  # call find_package for Threads. Without this, you will get this error:
  # set_target_properties Can not find target to add properties to: Threads::Threads
  find_package(Threads)
endif()

add_definitions( -DBOX2D_EXTERNAL_LANGUAGE_API=1 )

# Enable / disable debug code depending upon the build configuration.
set(DEBUG_FLAGS "-DDEBUG=1")
set(RELEASE_FLAGS "-DDEBUG=0")
file(WRITE "flags.cmake" "")
foreach(FLAGS_TYPE C CXX)
  # Enable debug code when CMAKE_BUILD_TYPE=Debug.
  set(FLAGS_VARIABLE CMAKE_${FLAGS_TYPE}_FLAGS_DEBUG)
  file(APPEND "flags.cmake"
       "set(${FLAGS_VARIABLE} \"\${${FLAGS_VARIABLE}} ${DEBUG_FLAGS}\")\n")
  # Disable debug code when CMAKE_BUILD_TYPE is a release config.
  foreach(BUILD_TYPE_SUFFIX RELEASE RELWITHDEBINFO MINSIZEREL)
    set(FLAGS_VARIABLE CMAKE_${FLAGS_TYPE}_FLAGS_${BUILD_TYPE_SUFFIX})
    file(APPEND "flags.cmake"
         "set(${FLAGS_VARIABLE} \"\${${FLAGS_VARIABLE}} ${RELEASE_FLAGS}\")\n")
  endforeach(BUILD_TYPE_SUFFIX)
  # If no build configuration is specified, default to release flags to align
  # with cmake's default build configuration for makefiles.
  if(NOT CMAKE_BUILD_TYPE AND UNIX AND NOT APPLE)
    set(FLAGS_VARIABLE CMAKE_${FLAGS_TYPE}_FLAGS)
    file(APPEND "flags.cmake"
         "set(${FLAGS_VARIABLE} \"\${${FLAGS_VARIABLE}} ${RELEASE_FLAGS}\")\n")
  endif(NOT CMAKE_BUILD_TYPE AND UNIX AND NOT APPLE)
endforeach(FLAGS_TYPE)
include("flags.cmake")

set(C_FLAGS_WARNINGS "")
if(MSVC)
  set(C_FLAGS_WARNINGS "/W4 /WX")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR
       CMAKE_COMPILER_IS_CLANGXX)
  # Set the maximum warning level for gcc.
  set(C_FLAGS_WARNINGS "-Wall -Werror -Wno-long-long -Wno-variadic-macros")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_WARNINGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C_FLAGS_WARNINGS}")

# Configure flags for gcov when the code coverage config is enabled.
if (BOX2D_CODE_COVERAGE AND
    (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR
     CMAKE_COMPILER_IS_CLANGXX))
  set(GCOV_FLAGS "-fprofile-arcs -ftest-coverage")
  set(GCOV_COMPILER_FLAGS "-g ${GCOV_FLAGS}")
  # DEBUG and RELEASE flags are set explicitly here as CMAKE_C*_FLAGS get
  # used to perform check compiles for cmake's configure step which fails on
  # OSX when gcov flags are specified.
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${GCOV_COMPILER_FLAGS}")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${GCOV_COMPILER_FLAGS}")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${GCOV_COMPILER_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} ${GCOV_COMPILER_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCOV_FLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${GCOV_FLAGS}")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${GCOV_FLAGS}")
endif()

# The XCode generator ignores CMAKE_C_FLAGS AND CMAKE_CXX_FLAGS so flags
# need to be added with target properties.
# The following functions track the list of targets so that it's possible to
# add flags to all targets using target properties.
set(ALL_TARGETS "" CACHE INTERNAL "")
# Override add_library() and adding the target to the ALL_TARGETS variable.
function(add_library name)
  _add_library(${name} ${ARGN})
  set(ALL_TARGETS "${ALL_TARGETS}" "${name}" CACHE INTERNAL "")
endfunction(add_library)

# Override add_executable() and adding the target to the ALL_TARGETS variable.
function(add_executable name)
  _add_executable(${name} ${ARGN})
  set(ALL_TARGETS "${ALL_TARGETS}" "${name}" CACHE INTERNAL "")
endfunction(add_executable)

# The Box2D library.
add_subdirectory(Box2D)

if(BOX2D_BUILD_EXAMPLES)
  # HelloWorld console example.
  add_subdirectory(HelloWorld)

  # Testbed and dependencies.
  find_package(OpenGL REQUIRED)
  add_subdirectory(glew)
  add_subdirectory(glfw)
  add_subdirectory(Testbed)
endif(BOX2D_BUILD_EXAMPLES)

if(BOX2D_INSTALL_DOC)
  install(DIRECTORY Documentation DESTINATION share/doc/Box2D PATTERN ".svn" EXCLUDE)
endif(BOX2D_INSTALL_DOC)

# Set the warning flags for all targets when using the Xcode generator.
if(CMAKE_COMPILER_IS_CLANGXX)
  foreach(target ${ALL_TARGETS})
    set_target_properties(${target} PROPERTIES XCODE_ATTRIBUTE_WARNING_CFLAGS
                          "${C_FLAGS_WARNINGS}")
  endforeach(target)
endif()

# Create a subdirectory for each build configuration when building using
# makefiles.  This makes it possible to keep multiple build artifacts
set(OUTPUT_SUBDIR "")
if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
  set(OUTPUT_SUBDIR "${CMAKE_BUILD_TYPE}/")
endif()

# Override / reset the output directory for all targets.
set(LIB_OUTPUT_DIR "${OUTPUT_SUBDIR}")
set(EXE_OUTPUT_DIR "${OUTPUT_SUBDIR}")
foreach(target ${ALL_TARGETS})
  set_target_properties(
    ${target} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${LIB_OUTPUT_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${LIB_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${EXE_OUTPUT_DIR}")
endforeach(target)
