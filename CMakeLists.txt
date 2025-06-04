cmake_minimum_required(VERSION 3.22)
include(FetchContent)
include(CMakeDependentOption)

project(box2d
	VERSION 3.1.1
	DESCRIPTION "A 2D physics engine for games"
	HOMEPAGE_URL "https://box2d.org"
	LANGUAGES C CXX
)

# stuff to help debug cmake
# message(STATUS "cmake tool chain: ${CMAKE_TOOLCHAIN_FILE}")
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
			add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
			add_link_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
		endif()
	endif()
endif()

# Deterministic math
# https://box2d.org/posts/2024/08/determinism/
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

option(BOX2D_DISABLE_SIMD "Disable SIMD math (slower)" OFF)
option(BOX2D_COMPILE_WARNING_AS_ERROR "Compile warnings as errors" OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
	cmake_dependent_option(BOX2D_AVX2 "Enable AVX2" OFF "NOT BOX2D_DISABLE_SIMD" OFF)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Needed for samples.exe to find box2d.dll
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

add_subdirectory(src)

# This hides samples, test, and doxygen from apps that use box2d via FetchContent
if(PROJECT_IS_TOP_LEVEL)
	option(BOX2D_SAMPLES "Build the Box2D samples" ON)
	option(BOX2D_BENCHMARKS "Build the Box2D benchmarks" OFF)
	option(BOX2D_DOCS "Build the Box2D documentation" OFF)
	option(BOX2D_PROFILE "Enable profiling with Tracy" OFF)
	option(BOX2D_VALIDATE "Enable heavy validation" ON)
	option(BOX2D_UNIT_TESTS "Build the Box2D unit tests" ON)
	
	if(BOX2D_UNIT_TESTS OR BOX2D_SAMPLES OR BOX2D_BENCHMARKS)

		if(MSVC AND NOT BOX2D_SANITIZE)
			# enable hot reloading
			add_compile_options("$<$<CONFIG:Debug>:/ZI>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL>")
		endif()

		# Emscripten pthread support for enkiTS
		if(EMSCRIPTEN)
			set(EMSCRIPTEN_PTHREADS_COMPILER_FLAGS "-pthread -s USE_PTHREADS=1")
			set(EMSCRIPTEN_PTHREADS_LINKER_FLAGS "${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS} -s ALLOW_MEMORY_GROWTH")
			string(APPEND CMAKE_C_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
			string(APPEND CMAKE_CXX_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
			string(APPEND CMAKE_EXE_LINKER_FLAGS " ${EMSCRIPTEN_PTHREADS_LINKER_FLAGS}")
		endif()

		# Task system used in tests and samples
		set(ENKITS_BUILD_EXAMPLES OFF CACHE BOOL "Build enkiTS examples")
		FetchContent_Declare(
			enkits
			GIT_REPOSITORY https://github.com/dougbinks/enkiTS.git
			GIT_TAG master
			GIT_SHALLOW TRUE
			GIT_PROGRESS TRUE
		)
		FetchContent_MakeAvailable(enkits)

		add_subdirectory(shared)
	endif()

	# Tests need static linkage because they test internal Box2D functions
	if(NOT BUILD_SHARED_LIBS AND BOX2D_UNIT_TESTS)
		message(STATUS "Adding Box2D unit tests")
		add_subdirectory(test)
		set_target_properties(test PROPERTIES XCODE_GENERATE_SCHEME TRUE)
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

		if(APPLE)
			set_target_properties(samples PROPERTIES
				XCODE_GENERATE_SCHEME TRUE
				XCODE_SCHEME_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
		endif()
	endif()

	if(BOX2D_BENCHMARKS)
		add_subdirectory(benchmark)
		set_target_properties(benchmark PROPERTIES XCODE_GENERATE_SCHEME TRUE)
	endif()

	if(BOX2D_DOCS)
		add_subdirectory(docs)
	endif()
endif()

# # Building on clang in windows
# cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 -T ClangCL
# https://clang.llvm.org/docs/UsersManual.html#clang-cl
