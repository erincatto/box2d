include(GNUInstallDirs)
include(FetchContent)

set(BOX2D_SOURCE_FILES
	aabb.c
	aabb.h
	arena_allocator.c
	arena_allocator.h
	atomic.h
	bitset.c
	bitset.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	constraint_graph.c
	constraint_graph.h
	contact.c
	contact.h
	contact_solver.c
	contact_solver.h
	container.h
	core.c
	core.h
	ctz.h
	distance.c
	distance_joint.c
	dynamic_tree.c
	dynamic_tree.h
	geometry.c
	hull.c
	id_pool.c
	id_pool.h
	island.c
	island.h
	joint.c
	joint.h
	manifold.c
	math_functions.c
	motor_joint.c
	mover_joint.c
	mover.c
	parallel_for.c
	parallel_for.h
	physics_world.c
	physics_world.h
	pogo_joint.c
	prismatic_joint.c
	recording.c
	recording.h
	recording_ops.inl
	recording_replay.c
	recording_replay.h
	revolute_joint.c
	scheduler.c
	scheduler.h
	sensor.c
	sensor.h
	shape.c
	shape.h
	solver.c
	solver.h
	solver_set.c
	solver_set.h
	table.c
	table.h
	timer.c
	types.c
	weld_joint.c
	wheel_joint.c
	world_snapshot.c
	world_snapshot.h
)

set(BOX2D_API_FILES
	../include/box2d/base.h
	../include/box2d/box2d.h
	../include/box2d/collision.h
	../include/box2d/config.h
	../include/box2d/constants.h
	../include/box2d/id.h
	../include/box2d/math_functions.h
	../include/box2d/types.h
)

# Hide internal functions
# https://gcc.gnu.org/wiki/Visibility
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES})

target_include_directories(box2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_DEBUG_POSTFIX "d")

# Box2D uses C17 for _Static_assert and anonymous unions
set_target_properties(box2d PROPERTIES
	C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS YES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
	DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
)

if (BOX2D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(box2d PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

if (BOX2D_PROFILE)
    target_compile_definitions(box2d PRIVATE BOX2D_PROFILE)
    set(TRACY_DELAYED_INIT ON CACHE BOOL "Enable delayed init for Tracy" FORCE)
    set(TRACY_MANUAL_LIFETIME ON CACHE BOOL "Enable manual lifetime for Tracy" FORCE)

    FetchContent_Declare(
        tracy
        GIT_REPOSITORY https://github.com/wolfpld/tracy.git
        GIT_TAG v0.13.1
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(tracy)

    target_link_libraries(box2d PUBLIC TracyClient)
endif()

if (BOX2D_VALIDATE)
	# Only effective when NDEBUG is undefined, see base.h
	message(STATUS "Box2D validation ON (debug builds)")
	target_compile_definitions(box2d PRIVATE BOX2D_VALIDATE)
endif()

if (BOX2D_DISABLE_SIMD)
	message(STATUS "Box2D SIMD disabled")
	target_compile_definitions(box2d PRIVATE BOX2D_DISABLE_SIMD)
endif()

if (BOX2D_DOUBLE_PRECISION)
	message(STATUS "Box2D double precision ON")
	# PUBLIC: consumers must see the same precision mode in the headers or they cannot match the ABI
	target_compile_definitions(box2d PUBLIC BOX2D_DOUBLE_PRECISION)
endif()

# Enable asserts in release with debug info. RelWithDebInfo defines NDEBUG, so the
# optimized test runs would otherwise lose assertions.
target_compile_definitions(box2d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B2_ENABLE_ASSERT>")

if (MSVC)
	message(STATUS "Box2D on MSVC")	
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Box2D symbols
		target_compile_definitions(box2d INTERFACE BOX2D_DLL)
	endif()

	# Visual Studio won't load the natvis unless it is in the project
	target_sources(box2d PRIVATE box2d.natvis)

	if (CMAKE_C_COMPILER_ID STREQUAL "Clang")
		# clang-cl: /Wall maps to -Weverything which is far too aggressive
		target_compile_options(box2d PRIVATE /W4 -Wmissing-prototypes)
	else()
		# All Warnings with some disabled
		# 4820 - padding between members
		# 5045 - Spectre mitigation
		# 4061 - not handling all cases for an enum in a switch block
		# 4711 - function inlined without being marked for inline
		# 4710 - inline functions that are not inlined
		target_compile_options(box2d PRIVATE /Wall /wd4820 /wd5045 /wd4061 /wd4711 /wd4710)
	endif()

	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")
		target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
		target_compile_options(box2d PRIVATE /arch:AVX2)
	endif()

elseif (MINGW)
	message(STATUS "Box2D on MinGW")
	target_compile_options(box2d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")	
		target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
		target_compile_options(box2d PRIVATE -mavx2)
	endif()
elseif (APPLE)
	message(STATUS "Box2D on Apple")
	target_compile_options(box2d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
elseif (EMSCRIPTEN)
	message(STATUS "Box2D on Emscripten")
	if (NOT BOX2D_DISABLE_SIMD)
		target_compile_options(box2d PRIVATE -msimd128 -msse2)
	endif()
elseif (UNIX)
	message(STATUS "Box2D using Unix")
	target_compile_options(box2d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
	if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
		# raspberry pi
		# -mfpu=neon
		# target_compile_options(box2d PRIVATE)
	else()
		if (BOX2D_AVX2)
			message(STATUS "Box2D using AVX2")
			target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
			target_compile_options(box2d PRIVATE -mavx2)
		endif()
	endif()
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_API_FILES})

add_library(box2d::box2d ALIAS box2d)

# libm is required on many Unix toolchains for sqrtf, etc.
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
  target_link_libraries(box2d PUBLIC m)
endif()

install(
  TARGETS box2d
  EXPORT box2dConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  EXPORT box2dConfig
  NAMESPACE box2d::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)

install(
  FILES ${BOX2D_API_FILES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/box2d
)

export(
  EXPORT box2dConfig
  NAMESPACE box2d::
  FILE "${PROJECT_BINARY_DIR}/box2dConfig.cmake"
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/box2dConfigVersion.cmake"
  VERSION ${box2d_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${PROJECT_BINARY_DIR}/box2dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)

# Anchor every path to the .pc file itself so the install tree can be relocated,
# by cmake --install --prefix or by moving it afterwards
file(RELATIVE_PATH PKGCONFIG_PREFIX_REL "${CMAKE_INSTALL_FULL_LIBDIR}/pkgconfig" "${CMAKE_INSTALL_PREFIX}")
string(REGEX REPLACE "/$" "" PKGCONFIG_PREFIX_REL "${PKGCONFIG_PREFIX_REL}")
file(RELATIVE_PATH PKGCONFIG_LIBDIR_REL "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_FULL_LIBDIR}")
file(RELATIVE_PATH PKGCONFIG_INCLUDEDIR_REL "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_FULL_INCLUDEDIR}")

# The file has to carry every PUBLIC define the exported target carries, or consumers
# compile against different headers than the library was built with. Keep in sync
# with the defines above.
set(PKGCONFIG_CFLAGS "")
if (BOX2D_DOUBLE_PRECISION)
	string(APPEND PKGCONFIG_CFLAGS " -DBOX2D_DOUBLE_PRECISION")
endif()
if (WIN32 AND BUILD_SHARED_LIBS)
	# base.h keys the dllimport off _WIN32, so MinGW needs this as much as MSVC
	string(APPEND PKGCONFIG_CFLAGS " -DBOX2D_DLL")
endif()
string(APPEND PKGCONFIG_CFLAGS "$<$<CONFIG:RELWITHDEBINFO>: -DB2_ENABLE_ASSERT>")

set(PKGCONFIG_LIBNAME "box2d$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>")

# Plain pkg-config --libs skips Libs.private, and Box2D builds static by default, so
# public link deps belong on the main line unless we are actually building a shared lib
set(PKGCONFIG_LIBS "")
set(PKGCONFIG_LIBS_PRIVATE "")
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
	if(BUILD_SHARED_LIBS)
		set(PKGCONFIG_LIBS_PRIVATE "-lm")
	else()
		set(PKGCONFIG_LIBS " -lm")
	endif()
endif()

configure_file(box2d.pc.in box2d.pc.gen @ONLY)

# The library name and the assert define vary by config, so finish the substitution
# per config instead of once at configure time
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/$<CONFIG>/box2d.pc"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/box2d.pc.gen"
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/$<CONFIG>/box2d.pc"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
)
