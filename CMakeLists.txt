cmake_minimum_required(VERSION 3.8)

# https://cmake.org/cmake/help/latest/command/project.html
project(box2d VERSION 2.4.0)

# set(CMAKE_CONFIGURATION_TYPES "Debug;RelWithDebInfo" CACHE STRING "" FORCE)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(src)

option(BOX2D_BUILD_UNIT_TESTS "Build the Box2D unit tests" ON)
option(BOX2D_BUILD_TESTBED "Build the Box2D testbed" ON)
option(BOX2D_BUILD_DOCS "Build the Box2D documentation" OFF)

if (BOX2D_BUILD_DOCS)
    set(DOXYGEN_SKIP_DOT TRUE)
    find_package(Doxygen)
endif()

if (DOXYGEN_FOUND AND BOX2D_BUILD_DOCS)
    add_subdirectory(docs)
endif()

if (BOX2D_BUILD_UNIT_TESTS)
	add_subdirectory(unit-test)
endif()

if (BOX2D_BUILD_TESTBED)
	add_subdirectory(extern/glad)
	add_subdirectory(extern/glfw)
	add_subdirectory(extern/imgui)
	add_subdirectory(extern/sajson)
	add_subdirectory(testbed)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT testbed)
		set_property(TARGET testbed PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/testbed")
	endif()
endif()
