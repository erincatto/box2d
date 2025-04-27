include(GNUInstallDirs)

set(BOX2D_SOURCE_FILES
	aabb.c
	aabb.h
	arena_allocator.c
	arena_allocator.h
	array.c
	array.h
	atomic.h
	bitset.c
	bitset.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	constants.h
	constraint_graph.c
	constraint_graph.h
	contact.c
	contact.h
	contact_solver.c
	contact_solver.h
	core.c
	core.h
	ctz.h
	distance.c
	distance_joint.c
	dynamic_tree.c
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
	mouse_joint.c
	mover.c
	prismatic_joint.c
	revolute_joint.c
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
	world.c
	world.h
)

set(BOX2D_API_FILES
	../include/box2d/base.h
	../include/box2d/box2d.h
	../include/box2d/collision.h
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

	FetchContent_Declare(
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG v0.11.1
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(tracy)

	target_link_libraries(box2d PUBLIC TracyClient)
endif()

if (BOX2D_VALIDATE)
	message(STATUS "Box2D validation ON")	
	target_compile_definitions(box2d PRIVATE BOX2D_VALIDATE)
endif()

if (BOX2D_DISABLE_SIMD)
	message(STATUS "Box2D SIMD disabled")	
	target_compile_definitions(box2d PRIVATE BOX2D_DISABLE_SIMD)
endif()

if (MSVC)
	message(STATUS "Box2D on MSVC")	
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Box2D symbols
		target_compile_definitions(box2d INTERFACE BOX2D_DLL)
	endif()

	# Visual Studio won't load the natvis unless it is in the project
	target_sources(box2d PRIVATE box2d.natvis)

	# Enable asserts in release with debug info
	target_compile_definitions(box2d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B2_ENABLE_ASSERT>")

	# Warnings
	# 4710 - warn about inline functions that are not inlined
	target_compile_options(box2d PRIVATE /Wall /wd4820 /wd5045 /wd4061 /wd4711 /wd4710)

	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")	
		target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
		target_compile_options(box2d PRIVATE /arch:AVX2)
	endif()

	if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		target_compile_options(box2d PRIVATE -Wmissing-prototypes)
	endif()

elseif (MINGW)
	message(STATUS "Box2D on MinGW")
	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")	
		target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
		target_compile_options(box2d PRIVATE -mavx2)
	else()
	endif()
elseif (APPLE)
	message(STATUS "Box2D on Apple")
	target_compile_options(box2d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic)
elseif (EMSCRIPTEN)
	message(STATUS "Box2D on Emscripten")
	if (NOT BOX2D_DISABLE_SIMD)
		target_compile_options(box2d PRIVATE -msimd128 -msse2)
	endif()
elseif (UNIX)
	message(STATUS "Box2D using Unix")
	target_compile_options(box2d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
	if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# raspberry pi
		# -mfpu=neon
		# target_compile_options(box2d PRIVATE)
	else()
		if (BOX2D_AVX2)
			message(STATUS "Box2D using AVX2")
			target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
			target_compile_options(box2d PRIVATE -mavx2)
		else()
		endif()
	endif()
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_API_FILES})

add_library(box2d::box2d ALIAS box2d)

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

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  VERSION ${BOX2D_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)
