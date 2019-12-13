project(test LANGUAGES CXX)

set (TEST_SOURCE_FILES
	hello_world.cpp)

add_executable(test ${TEST_SOURCE_FILES})
target_link_libraries(test PUBLIC box2d)
