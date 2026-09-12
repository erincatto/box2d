# Box2D code shared by samples, benchmarks, and unit tests

set(BOX2D_SHARED_FILES
	benchmarks.c
	benchmarks.h
	determinism.c
	determinism.h
	human.c
	human.h
	utils.c
	utils.h
)

add_library(shared STATIC ${BOX2D_SHARED_FILES})

set_target_properties(shared PROPERTIES
	C_STANDARD 17
)

if (BOX2D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(shared PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

# Special access to Box2D internals for testing. The internal headers select the SIMD path
# from the build configuration, so the tests must see the same selection as the library.
target_include_directories(shared PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_compile_definitions(shared PRIVATE ${BOX2D_SIMD_DEFINITIONS})
target_compile_options(shared PRIVATE ${BOX2D_SIMD_OPTIONS})

target_include_directories(shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(shared PRIVATE box2d)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX2D_SHARED_FILES})