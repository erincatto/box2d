add_executable(unit_test
    doctest.h
    hello_world.cpp
)

set_target_properties(unit_test PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
target_link_libraries(unit_test PUBLIC box2d)

# Place the test executable at the project binary directory instead of in the nested subfolder
set_target_properties(unit_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES doctest.h hello_world.cpp)
