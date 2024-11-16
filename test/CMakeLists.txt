# Box2D unit test app

set(BOX2D_TEST_FILES
    main.c
    test_bitset.c
    test_collision.c
    test_determinism.c
    test_distance.c
    test_id.c
    test_macros.h
    test_math.c
    test_shape.c
    test_table.c
    test_world.c
)

add_executable(test ${BOX2D_TEST_FILES})

set_target_properties(test PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

# Special access to Box2D internals for testing
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/src)

if (MSVC)
	target_compile_options(test PRIVATE /experimental:c11atomics)
endif()

target_link_libraries(test PRIVATE box2d enkiTS)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX2D_TEST_FILES})
