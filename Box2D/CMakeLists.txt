cmake_minimum_required(VERSION 2.6)

project(Box2D)

if(UNIX)
	set(BOX2D_INSTALL_BY_DEFAULT ON)
else(UNIX)
	set(BOX2D_INSTALL_BY_DEFAULT OFF)
endif(UNIX)

option(BOX2D_INSTALL "Install Box2D libs, includes, and CMake scripts" ${BOX2D_INSTALL_BY_DEFAULT})
option(BOX2D_INSTALL_DOC "Install Box2D documentation" OFF)
option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)
option(BOX2D_BUILD_EXAMPLES "Build Box2D examples" ON)
option(BOX2D_DISABLE_BUNDLED_GLFW "Build Testbed without bundled GLFW" OFF)
option(BOX2D_DISABLE_BUNDLED_GLEW "Build Testbed without bundled GLEW" OFF)

set(BOX2D_VERSION 2.3.2)
set(LIB_INSTALL_DIR lib${LIB_SUFFIX})

# The Box2D library.
add_subdirectory(Box2D)

if(BOX2D_BUILD_EXAMPLES)
  # HelloWorld console example.
  add_subdirectory(HelloWorld)

  if(NOT BOX2D_DISABLE_BUNDLED_GLEW)
      add_subdirectory(glew)
  endif()

  if(NOT BOX2D_DISABLE_BUNDLED_GLFW)
      add_subdirectory(glfw-3.1.1)
  endif()

  add_subdirectory(Testbed)
endif(BOX2D_BUILD_EXAMPLES)

if(BOX2D_INSTALL_DOC)
  install(DIRECTORY Documentation DESTINATION share/doc/Box2D PATTERN ".svn" EXCLUDE)
endif(BOX2D_INSTALL_DOC)
