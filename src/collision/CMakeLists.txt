set(DYNTREE_SOURCE_FILES
	tree.cpp
	utils.cpp)

set(DYNTREE_HEADER_FILES
	../include/dynamic-tree/utils.h
	../include/dynamic-tree/tree.h)

add_library(dynamic-tree STATIC ${DYNTREE_SOURCE_FILES} ${DYNTREE_HEADER_FILES})
target_include_directories(dynamic-tree PUBLIC ../include)
