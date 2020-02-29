add_library(sajson STATIC sajson.cpp sajson.h)
target_include_directories(sajson PUBLIC ..)

set_target_properties(sajson PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES sajson.cpp sajson.h)
