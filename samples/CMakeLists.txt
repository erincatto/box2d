# Box2D samples app

# glad for OpenGL API
set(GLAD_DIR ${CMAKE_SOURCE_DIR}/extern/glad)

add_library(
	glad STATIC
	${GLAD_DIR}/src/glad.c
	${GLAD_DIR}/include/glad/glad.h
	${GLAD_DIR}/include/KHR/khrplatform.h
)
target_include_directories(glad PUBLIC ${GLAD_DIR}/include)

# glfw for windowing and input
set(GLFW_BUILD_DOCS OFF CACHE BOOL "GLFW Docs")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "GLFW Examples")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "GLFW Tests")
set(GLFW_INSTALL OFF CACHE BOOL "GLFW Install")

FetchContent_Declare(
	glfw
	GIT_REPOSITORY https://github.com/glfw/glfw.git
	GIT_TAG 3.4
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(glfw)

# imgui and glfw backend for GUI
# https://gist.github.com/jeffamstutz/992723dfabac4e3ffff265eb71a24cd9
# Modified to pin to a specific imgui release
FetchContent_Populate(imgui
	URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.3.zip
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/build/imgui
)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/build/imgui)

add_library(imgui STATIC
	${IMGUI_DIR}/imconfig.h
	${IMGUI_DIR}/imgui.h

	${IMGUI_DIR}/imgui.cpp
	${IMGUI_DIR}/imgui_draw.cpp
	${IMGUI_DIR}/imgui_demo.cpp
	${IMGUI_DIR}/imgui_tables.cpp
	${IMGUI_DIR}/imgui_widgets.cpp

	${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
	${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_link_libraries(imgui PUBLIC glfw glad)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
# The sample app also uses stb_truetype and this keeps the symbols separate
target_compile_definitions(imgui PRIVATE IMGUI_STB_NAMESPACE=imgui_stb)
set_target_properties(imgui PROPERTIES
	CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# jsmn for json
set(JSMN_DIR ${CMAKE_SOURCE_DIR}/extern/jsmn)

add_library(jsmn INTERFACE ${JSMN_DIR}/jsmn.h)
target_include_directories(jsmn INTERFACE ${JSMN_DIR})

set(BOX2D_SAMPLE_FILES
	car.cpp
	car.h
	donut.cpp
	donut.h
	doohickey.cpp
	doohickey.h
	draw.cpp
	draw.h
	main.cpp
	sample.cpp
	sample.h
	sample_benchmark.cpp
	sample_bodies.cpp
	sample_character.cpp
	sample_collision.cpp
	sample_continuous.cpp
	sample_determinism.cpp
	sample_events.cpp
	sample_geometry.cpp
	sample_joints.cpp
	sample_robustness.cpp
	sample_shapes.cpp
	sample_stacking.cpp
	sample_world.cpp
	shader.cpp
	shader.h)

add_executable(samples ${BOX2D_SAMPLE_FILES})

set_target_properties(samples PROPERTIES
	CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if (BOX2D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(samples PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

target_include_directories(samples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${JSMN_DIR})
target_link_libraries(samples PUBLIC box2d shared imgui glfw glad enkiTS)

# target_compile_definitions(samples PRIVATE "$<$<CONFIG:DEBUG>:SAMPLES_DEBUG>")
# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc
add_custom_command(
	TARGET samples POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
			${CMAKE_CURRENT_SOURCE_DIR}/data/
			${CMAKE_CURRENT_BINARY_DIR}/data/)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${BOX2D_SAMPLE_FILES})
