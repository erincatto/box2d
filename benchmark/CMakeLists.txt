# Box2D benchmark app

set(BOX2D_BENCHMARK_FILES
	main.c
)
add_executable(benchmark ${BOX2D_BENCHMARK_FILES})

set_target_properties(benchmark PROPERTIES C_STANDARD 17)

if (MSVC AND CMAKE_C_COMPILER_ID MATCHES "MSVC")
	target_compile_options(benchmark PRIVATE /experimental:c11atomics)
endif()

target_link_libraries(benchmark PRIVATE box2d shared enkiTS)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX2D_BENCHMARK_FILES})