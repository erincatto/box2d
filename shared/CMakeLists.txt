# Box2D code shared by samples, benchmarks, and unit tests

set(BOX2D_SHARED_FILES
	benchmarks.c
	benchmarks.h
	human.c
	human.h
	random.c
	random.h
)

add_library(shared ${BOX2D_SHARED_FILES})

set_target_properties(shared PROPERTIES
	C_STANDARD 17
)

target_include_directories(shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(shared PRIVATE box2d)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX2D_SHARED_FILES})