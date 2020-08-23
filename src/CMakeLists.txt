set(BOX2D_SOURCE_FILES
	collision/b2_broad_phase.cpp
	collision/b2_chain_shape.cpp
	collision/b2_circle_shape.cpp
	collision/b2_collide_circle.cpp
	collision/b2_collide_edge.cpp
	collision/b2_collide_polygon.cpp
	collision/b2_collision.cpp
	collision/b2_distance.cpp
	collision/b2_dynamic_tree.cpp
	collision/b2_edge_shape.cpp
	collision/b2_polygon_shape.cpp
	collision/b2_time_of_impact.cpp
	common/b2_block_allocator.cpp
	common/b2_draw.cpp
	common/b2_math.cpp
	common/b2_settings.cpp
	common/b2_stack_allocator.cpp
	common/b2_timer.cpp
	dynamics/b2_body.cpp
	dynamics/b2_chain_circle_contact.cpp
	dynamics/b2_chain_circle_contact.h
	dynamics/b2_chain_polygon_contact.cpp
	dynamics/b2_chain_polygon_contact.h
	dynamics/b2_circle_contact.cpp
	dynamics/b2_circle_contact.h
	dynamics/b2_contact.cpp
	dynamics/b2_contact_manager.cpp
	dynamics/b2_contact_solver.cpp
	dynamics/b2_contact_solver.h
	dynamics/b2_distance_joint.cpp
	dynamics/b2_edge_circle_contact.cpp
	dynamics/b2_edge_circle_contact.h
	dynamics/b2_edge_polygon_contact.cpp
	dynamics/b2_edge_polygon_contact.h
	dynamics/b2_fixture.cpp
	dynamics/b2_friction_joint.cpp
	dynamics/b2_gear_joint.cpp
	dynamics/b2_island.cpp
	dynamics/b2_island.h
	dynamics/b2_joint.cpp
	dynamics/b2_motor_joint.cpp
	dynamics/b2_mouse_joint.cpp
	dynamics/b2_polygon_circle_contact.cpp
	dynamics/b2_polygon_circle_contact.h
	dynamics/b2_polygon_contact.cpp
	dynamics/b2_polygon_contact.h
	dynamics/b2_prismatic_joint.cpp
	dynamics/b2_pulley_joint.cpp
	dynamics/b2_revolute_joint.cpp
	dynamics/b2_rope_joint.cpp
	dynamics/b2_weld_joint.cpp
	dynamics/b2_wheel_joint.cpp
	dynamics/b2_world.cpp
	dynamics/b2_world_callbacks.cpp
	rope/b2_rope.cpp)

set(BOX2D_HEADER_FILES
	../include/box2d/b2_block_allocator.h
	../include/box2d/b2_body.h
	../include/box2d/b2_broad_phase.h
	../include/box2d/b2_chain_shape.h
	../include/box2d/b2_circle_shape.h
	../include/box2d/b2_collision.h
	../include/box2d/b2_contact.h
	../include/box2d/b2_contact_manager.h
	../include/box2d/b2_distance.h
	../include/box2d/b2_distance_joint.h
	../include/box2d/b2_draw.h
	../include/box2d/b2_dynamic_tree.h
	../include/box2d/b2_edge_shape.h
	../include/box2d/b2_fixture.h
	../include/box2d/b2_friction_joint.h
	../include/box2d/b2_gear_joint.h
	../include/box2d/b2_growable_stack.h
	../include/box2d/b2_joint.h
	../include/box2d/b2_math.h
	../include/box2d/b2_motor_joint.h
	../include/box2d/b2_mouse_joint.h
	../include/box2d/b2_polygon_shape.h
	../include/box2d/b2_prismatic_joint.h
	../include/box2d/b2_pulley_joint.h
	../include/box2d/b2_revolute_joint.h
	../include/box2d/b2_rope.h
	../include/box2d/b2_rope_joint.h
	../include/box2d/b2_settings.h
	../include/box2d/b2_shape.h
	../include/box2d/b2_stack_allocator.h
	../include/box2d/b2_time_of_impact.h
	../include/box2d/b2_timer.h
	../include/box2d/b2_time_step.h
	../include/box2d/b2_weld_joint.h
	../include/box2d/b2_wheel_joint.h
	../include/box2d/b2_world.h
	../include/box2d/b2_world_callbacks.h
	../include/box2d/box2d.h)

add_library(box2d STATIC ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})
target_include_directories(box2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
set_target_properties(box2d PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
  set_target_properties(box2d PROPERTIES
    COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMPILE_PDB_NAME box2d
  )

  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/box2d.pdb"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    OPTIONAL
  )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX2D_HEADER_FILES})

install(
  TARGETS box2d
  EXPORT box2dConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  EXPORT box2dConfig
  NAMESPACE box2d::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)
