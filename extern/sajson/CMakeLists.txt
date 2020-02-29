add_library(sajson STATIC sajson.cpp sajson.h)
target_include_directories(sajson PUBLIC ..)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES sajson.cpp sajson.h)
