cmake_minimum_required(VERSION 3.22)
include(FetchContent)

project(box2d
	VERSION 3.0.0
	DESCRIPTION "A 2D physics engine for games"
	HOMEPAGE_URL "https://box2d.org"
	LANGUAGES C CXX
)

# stuff to help debug cmake
# message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")
# message(STATUS "library postfix: ${CMAKE_DEBUG_POSTFIX}")
message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

if (MSVC OR APPLE)
	option(BOX2D_SANITIZE "Enable sanitizers for some builds" OFF)
	if(BOX2D_SANITIZE)
		message(STATUS "Box2D Sanitize")
		# sanitizers need to apply to all compiled libraries to work well
		if(MSVC)
			# address sanitizer only in the debug build
			add_compile_options("$<$<CONFIG:Debug>:/fsanitize=address>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL:NO>")
		elseif(APPLE)
			# more sanitizers on Apple clang
			# add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
			add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
			add_link_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
		endif()
	endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
	option(BOX2D_AVX2 "Enable AVX2 (faster)" ON)
endif()

if(PROJECT_IS_TOP_LEVEL)
	# Needed for samples.exe to find box2d.dll
	# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
endif()

# C++17 needed for imgui
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# The Box2D library uses simde https://github.com/simd-everywhere/simde
add_subdirectory(extern/simde)
add_subdirectory(src)

# This hides samples, test, and doxygen from apps that use box2d via FetchContent
if(PROJECT_IS_TOP_LEVEL)
	option(BOX2D_SAMPLES "Build the Box2D samples" ON)
	option(BOX2D_BENCHMARKS "Build the Box2D benchmarks" OFF)
	option(BOX2D_DOCS "Build the Box2D documentation" OFF)
	option(BOX2D_PROFILE "Enable profiling with Tracy" OFF)
	option(BOX2D_VALIDATE "Enable heavy validation" ON)
	option(BOX2D_UNIT_TESTS "Build the Box2D unit tests" ON)

	if(MSVC AND WIN32)
		# Enable edit and continue only in debug due to perf hit in release
		# add_link_options($<$<CONFIG:Debug>:/INCREMENTAL>)
		# add_compile_options($<$<CONFIG:Debug>:/ZI>)
		# add_compile_options(/fsanitize=address)
		# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/GS- /Gy /O2 /Oi /Ot")
		# set(CMAKE_CXX_FLAGS_RELEASE "/GS- /Gy /O2 /Oi /Ot")
		# set(CMAKE_C_FLAGS_RELWITHDEBINFO "/GS- /Gy /O2 /Oi /Ot")
		# set(CMAKE_C_FLAGS_RELEASE "/GS- /Gy /O2 /Oi /Ot")
	endif()

	# enkiTS is used by all the test apps, but not directly by the Box2D library
	if(BOX2D_UNIT_TESTS OR BOX2D_SAMPLES OR BOX2D_BENCHMARKS)
		SET(ENKITS_BUILD_EXAMPLES OFF CACHE BOOL "Build enkiTS examples")

		# Used in tests and samples
		FetchContent_Declare(
			enkits
			GIT_REPOSITORY https://github.com/dougbinks/enkiTS.git
			GIT_TAG master
			GIT_SHALLOW TRUE
			GIT_PROGRESS TRUE
		)
		FetchContent_MakeAvailable(enkits)
	endif()

	# Tests need static linkage because they test internal Box2D functions
	if(NOT BUILD_SHARED_LIBS AND BOX2D_UNIT_TESTS)
		message(STATUS "Adding Box2D unit tests")
		add_subdirectory(test)
	else()
		message(STATUS "Skipping Box2D unit tests")
	endif()

	if(BOX2D_SAMPLES)
		add_subdirectory(samples)

		# default startup project for Visual Studio
		if(MSVC)
			set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
			set_property(TARGET samples PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
		endif()
	endif()

	if(BOX2D_BENCHMARKS)
		add_subdirectory(benchmark)
	endif()

	if(BOX2D_DOCS)
		add_subdirectory(docs)
	endif()
endif()

# # Building on clang in windows
# cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 -T ClangCL
# https://clang.llvm.org/docs/UsersManual.html#clang-cl
