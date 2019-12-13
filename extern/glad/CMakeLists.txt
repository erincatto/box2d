project(glad C)

set(GLAD_SOURCE_FILES
  glad.c)

set(GLAD_HEADER_FILES
  glad.h
  khrplatform.h)

add_library(glad STATIC ${GLAD_SOURCE_FILES} ${GLAD_HEADER_FILES})
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
