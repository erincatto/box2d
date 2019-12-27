project(glad C)

set(GLAD_SOURCE_FILES
  src/gl.c)

set(GLAD_HEADER_FILES
  include/glad/gl.h
  include/KHR/khrplatform.h)

add_library(glad STATIC ${GLAD_SOURCE_FILES} ${GLAD_HEADER_FILES})
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
