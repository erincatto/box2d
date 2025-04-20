# Box2D benchmark app

set(BOX2D_BENCHMARK_FILES
	main.c
)
add_executable(benchmark ${BOX2D_BENCHMARK_FILES})

set_target_properties(benchmark PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
	COMPILE_WARNING_AS_ERROR ON
)

target_link_libraries(benchmark PRIVATE box2d shared enkiTS)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX2D_BENCHMARK_FILES})