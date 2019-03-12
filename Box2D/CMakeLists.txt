# cmake for Box2D

set(LIST_SUBDIRS Collision Common Dynamics Rope)
message(STATUS "SUBDIRECTORIES: ${LIST_SUBDIRS}")

foreach(DIR ${LIST_SUBDIRS})
  add_subdirectory("${DIR}")
endforeach()

add_library(${PROJECT_NAME}
  $<TARGET_OBJECTS:Collision>
  $<TARGET_OBJECTS:Common>
  $<TARGET_OBJECTS:Dynamics>
  $<TARGET_OBJECTS:Rope>
  )

add_library(Box2D::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
    $<INSTALL_INTERFACE:include>
  )

set_target_properties(Box2D
  PROPERTIES
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}"
  )
