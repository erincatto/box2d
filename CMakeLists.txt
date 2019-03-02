# cmake for Box2D

cmake_minimum_required(VERSION 3.12)

project(Box2D)

include(build.cmake)
include(doxygen.cmake)

add_subdirectory(Box2D)
add_library(Box2D::Box2D ALIAS Box2D)


# tests and examples
if (BUILD_TESTING)
  add_executable(HelloWorld "HelloWorld/HelloWorld.cpp")
  target_link_libraries(HelloWorld PRIVATE Box2D::Box2D)
  target_compile_features(HelloWorld PRIVATE cxx_std_11)


endif()
