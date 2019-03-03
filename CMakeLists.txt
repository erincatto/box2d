# cmake for Box2D

cmake_minimum_required(VERSION 3.12)

project(Box2D VERSION 2.3.1)

include(cmake/build.cmake)
include(cmake/doxygen.cmake)

add_subdirectory(Box2D)

# tests and examples
if (BUILD_TESTING)
  add_executable(HelloWorld "HelloWorld/HelloWorld.cpp")
  target_link_libraries(HelloWorld PRIVATE Box2D)
  target_compile_features(HelloWorld PRIVATE cxx_std_11)
endif()

include(cmake/install.cmake)
include(cmake/uninstall.cmake)
