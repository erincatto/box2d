project(test LANGUAGES CXX)

set (TEST_SOURCE_FILES hello_world.cpp)

add_executable(test ${TEST_SOURCE_FILES})
set_target_properties(test PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
target_link_libraries(test PUBLIC box2d)
