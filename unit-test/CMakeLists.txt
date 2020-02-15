project(unit-test LANGUAGES CXX)

set (UNIT_TEST_SOURCE_FILES hello_world.cpp)

add_executable(unit-test ${UNIT_TEST_SOURCE_FILES})
set_target_properties(unit-test PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
target_link_libraries(unit-test PUBLIC box2d)
