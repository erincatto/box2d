set(BOX2D_SOURCE_FILES
	aabb.c
	aabb.h
	array.c
	array.h
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
	prismatic_joint.c
	revolute_joint.c
	shape.c
	shape.h
	solver.c
	solver.h
	solver_set.c
	solver_set.h
	stack_allocator.c
	stack_allocator.h
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
# todo need to investigate this more
# https://gcc.gnu.org/wiki/Visibility
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_API_FILES})

# Generate box2d_export.h to handles shared library builds
# turned this off to make Box2D easier to use without cmake
# include(GenerateExportHeader)
# generate_export_header(box2d)

target_include_directories(box2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_DEBUG_POSTFIX "d")

# Box2D uses C17
set_target_properties(box2d PROPERTIES
	C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS YES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
	DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
)

if (BOX2D_PROFILE)
	target_compile_definitions(box2d PRIVATE BOX2D_PROFILE)

	FetchContent_Declare(
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG master
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(tracy)

	target_link_libraries(box2d PUBLIC TracyClient)
endif()

if (BOX2D_VALIDATE)
	target_compile_definitions(box2d PRIVATE BOX2D_VALIDATE)
endif()

if (BOX2D_ENABLE_SIMD)
	target_compile_definitions(box2d PRIVATE BOX2D_ENABLE_SIMD)
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

	# Atomics are still considered experimental in Visual Studio 17.8
	target_compile_options(box2d PRIVATE /experimental:c11atomics)

	# All warnings
	target_compile_options(box2d PRIVATE /W4)

	if (BOX2D_AVX2)
		message(STATUS "Box2D using AVX2")	
		target_compile_definitions(box2d PRIVATE BOX2D_AVX2)
		target_compile_options(box2d PRIVATE /arch:AVX2)
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
elseif (EMSCRIPTEN)
	message(STATUS "Box2D on Emscripten")
	target_compile_options(box2d PRIVATE -msimd128 -msse2)
elseif (UNIX)
	message(STATUS "Box2D using Unix")
	if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# raspberry pi
		# -mfpu=neon
		# target_compile_options(box2d PRIVATE)
	else()
		# x64
		if (BOX2D_AVX2)
			message(STATUS "Box2D using AVX2")
			# FMA -mfma -mavx -march=native
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

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)

