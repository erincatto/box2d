cmake_minimum_required(VERSION 2.8)

project(Box2D)

option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)
option(BOX2D_BUILD_EXAMPLES "Build Box2D examples" ON)

set(BOX2D_VERSION 2.1.0)

# The Box2D library.
add_subdirectory(Box2D)


if(BOX2D_BUILD_EXAMPLES)
  # HelloWorld console example.
  add_subdirectory(HelloWorld)

  # Testbed and dependencies.
  find_package(OpenGL REQUIRED)
  add_subdirectory(freeglut)
  add_subdirectory(glui)
  add_subdirectory(Testbed)
endif(BOX2D_BUILD_EXAMPLES)

if(UNIX)
  install(DIRECTORY Documentation DESTINATION usr/share/doc/Box2D PATTERN ".svn" EXCLUDE)
endif(UNIX)