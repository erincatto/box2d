project(samples LANGUAGES CXX)

set (SAMPLE_SOURCE_FILES
	draw.cpp
	draw.h
	imgui_impl_glfw.cpp
	imgui_impl_glfw.h
	imgui_impl_opengl3.cpp
	imgui_impl_opengl3.h
	main.cpp
	settings.h
	settings.cpp
	test.cpp
	test.h
	tests/add_pair.cpp
#	tests/apply_forces.cpp
#	tests/body_types.cpp
#	tests/box_stack.cpp
#	tests/breakable.cpp
#	tests/bridge.cpp
#	tests/bullet_test.cpp
#	tests/cantilever.cpp
#	tests/car.cpp
#	tests/chain.cpp
#	tests/chain_problem.cpp
#	tests/character_collision.cpp
#	tests/collision_filtering.cpp
#	tests/collision_processing.cpp
#	tests/compound_shapes.cpp
#	tests/confined.cpp
#	tests/continuous_test.cpp
#	tests/convex_hull.cpp
#	tests/conveyor_belt.cpp
#	tests/distance_joint.cpp
#	tests/distance_test.cpp
#	tests/dominos.cpp
#	tests/dump_shell.cpp
#	tests/dynamic_tree.cpp
#	tests/edge_shapes.cpp
#	tests/edge_test.cpp
#	tests/friction.cpp
#	tests/gears.cpp
#	tests/heavy_on_light_1.cpp
#	tests/heavy_on_light_2.cpp
#	tests/mobile_balanced.cpp
#	tests/mobile_unbalanced.cpp
#	tests/motor_joint.cpp
#	tests/pinball.cpp
#	tests/platformer.cpp
#	tests/polygon_collision.cpp
#	tests/polygon_shapes.cpp
#	tests/prismatic_joint.cpp
#	tests/pulleys.cpp
#	tests/pyramid.cpp
#	tests/ray_cast.cpp
#	tests/restitution.cpp
#	tests/revolute_joint.cpp
#	tests/rope.cpp
#	tests/rope_joint.cpp
#	tests/sensor.cpp
#	tests/shape_cast.cpp
#	tests/shape_editing.cpp
#	tests/skier.cpp
#	tests/slider_crank_1.cpp
#	tests/slider_crank_2.cpp
#	tests/sphere_stack.cpp
#	tests/theo_jansen.cpp
#	tests/tiles.cpp
#	tests/time_of_impact.cpp
#	tests/tumbler.cpp
)

add_executable(samples ${SAMPLE_SOURCE_FILES})
target_include_directories(samples PUBLIC ${OPENGL_INCLUDE_DIR})
target_link_libraries(samples PUBLIC box2d glfw imgui glad sajson ${OPENGL_LIBRARIES})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SAMPLE_SOURCE_FILES})
