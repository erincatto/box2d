cmake_minimum_required(VERSION 3.8)

project(testbed LANGUAGES CXX)

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
    tests/apply_forces.cpp
    tests/body_types.cpp
    tests/box_stack.cpp
    tests/breakable.cpp
    tests/bridge.cpp
    tests/bullet_test.cpp
    tests/cantilever.cpp
    tests/car.cpp
    tests/chain.cpp
    tests/chain_problem.cpp
    tests/character_collision.cpp
    tests/circle_stack.cpp
    tests/collision_filtering.cpp
    tests/collision_processing.cpp
    tests/compound_shapes.cpp
    tests/confined.cpp
    tests/continuous_test.cpp
    tests/convex_hull.cpp
    tests/conveyor_belt.cpp
    tests/distance_joint.cpp
    tests/distance_test.cpp
    tests/dominos.cpp
    tests/dump_loader.cpp
    tests/dynamic_tree.cpp
    tests/edge_shapes.cpp
    tests/edge_test.cpp
    tests/friction.cpp
    tests/gear_joint.cpp
    tests/heavy1.cpp
    tests/heavy2.cpp
    tests/mobile_balanced.cpp
    tests/mobile_unbalanced.cpp
    tests/motor_joint.cpp
    tests/pinball.cpp
    tests/platformer.cpp
    tests/polygon_collision.cpp
    tests/polygon_shapes.cpp
    tests/prismatic_joint.cpp
    tests/pulley_joint.cpp
    tests/pyramid.cpp
    tests/ray_cast.cpp
    tests/restitution.cpp
    tests/revolute_joint.cpp
    tests/rope.cpp
    tests/rope_joint.cpp
    tests/sensor.cpp
    tests/shape_cast.cpp
    tests/shape_editing.cpp
    tests/skier.cpp
    tests/slider_crank_1.cpp
    tests/slider_crank_2.cpp
    tests/theo_jansen.cpp
    tests/tiles.cpp
    tests/time_of_impact.cpp
    tests/tumbler.cpp
)

set (SAMPLE_RESOURCE_FILES
    data/DroidSans.ttf
)

add_subdirectory(extern/glad)
add_subdirectory(extern/glfw)
add_subdirectory(extern/imgui)
add_subdirectory(extern/sajson)

add_executable(testbed ${SAMPLE_SOURCE_FILES} ${SAMPLE_RESOURCE_FILES})

option(USE_BOX2D_LIB "exist box2d lib used" OFF)

if ( ${USE_BOX2D_LIB} )
    set(BOX2D_DIR "Box2D-libary-dir" CACHE PATH "")
endif()

if(${USE_BOX2D_LIB})
    target_include_directories(testbed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${BOX2D_DIR}/include)
    target_link_libraries(testbed PUBLIC ${BOX2D_DIR}/lib/libbox2d.a glfw imgui sajson glad)
else()
    target_include_directories(testbed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(testbed PUBLIC box2d glfw imgui sajson glad)
endif()

set_target_properties(testbed PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
if(APPLE)
    set_target_properties(testbed PROPERTIES
        MACOSX_BUNDLE_INFO_PLIST            "${CMAKE_CURRENT_SOURCE_DIR}/MacOSXBundleInfo.plist.in"
        MACOSX_BUNDLE                        TRUE
        MACOSX_BUNDLE_BUNDLE_NAME           "testbed"
        MACOSX_BUNDLE_GUI_IDENTIFIER        "org.box2d.testbed"
    )

    # Reflect resources directory tree inside bundle
    foreach (SAMPLE_RESOURCE_FILE ${SAMPLE_RESOURCE_FILES})
        get_filename_component(SAMPLE_RESOURCE_DIR ${SAMPLE_RESOURCE_FILE} DIRECTORY)
        set_source_files_properties(${SAMPLE_RESOURCE_FILE} PROPERTIES
            MACOSX_PACKAGE_LOCATION Resources/${SAMPLE_RESOURCE_DIR}
        )
    endforeach()
endif()
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SAMPLE_SOURCE_FILES} ${SAMPLE_RESOURCE_FILES})

if (NOT ${USE_BOX2D_LIB} )
INSTALL(DIRECTORY ../samples 
        DESTINATION "${CMAKE_INSTALL_PREFIX}/samples"
)
endif()
INSTALL(DIRECTORY data 
        DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
)
INSTALL(TARGETS testbed 
        RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"             
)