# Hello World examples
include_directories (${Box2D_SOURCE_DIR})
add_executable(HelloWorld HelloWorld.cpp)
link_directories (${Box2D_BINARY_DIR}/Box2D)
target_link_libraries (HelloWorld Box2D) 
