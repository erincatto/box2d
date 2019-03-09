# cmake for Box2D

cmake_minimum_required(VERSION 3.12)

if(DEFINED PROJECT_NAME)
  set(IS_SUBPROJECT ON)
endif()

project(Box2D VERSION 2.3.1)

if(NOT IS_SUBPROJECT)
  include(cmake/build.cmake)
  include(cmake/doxygen.cmake)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(Box2D)

# tests and examples
if (NOT IS_SUBPROJECT AND BUILD_TESTING)
  add_executable(HelloWorld "HelloWorld/HelloWorld.cpp")
  target_link_libraries(HelloWorld PRIVATE Box2D)
  target_compile_features(HelloWorld PRIVATE cxx_std_11)
endif()

if(NOT IS_SUBPROJECT)
  include(cmake/install.cmake)
  include(cmake/uninstall.cmake)
endif()
