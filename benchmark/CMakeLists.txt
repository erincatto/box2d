# Box2D benchmark app

add_executable(benchmark
    main.c
)

set_target_properties(benchmark PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

if (MSVC)
	target_compile_options(benchmark PRIVATE /experimental:c11atomics)
endif()

target_link_libraries(benchmark PRIVATE box2d shared enkiTS)
