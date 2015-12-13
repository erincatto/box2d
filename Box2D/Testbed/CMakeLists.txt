# OpenGL
find_package(OpenGL REQUIRED)

# glfw
add_subdirectory(glfw)
set(GLFW_INCLUDE_DIR glfw/include)

# imgui
set(IMGUI 
	imgui/imgui.h 
	imgui/imgui.cpp 
	imgui/imgui_draw.cpp 
	imgui/imgui_demo.cpp 
	imgui/stb_truetype.h
	#imgui/examples/opengl3_example/imgui_impl_glfw_gl3.h
	#imgui/examples/opengl3_example/imgui_impl_glfw_gl3.cpp
	#imgui/examples/libs/gl3w/GL/gl3w.h
	#imgui/examples/libs/gl3w/GL/gl3w.c
	glfw/deps/glad/glad.h
	glfw/deps/glad.c
)
set(IMGUI_INCLUDE_DIR 
	imgui
	#imgui/examples/opengl3_example
	#imgui/examples/libs/gl3w
	glfw/deps
)

# Testbed

# Define the framework files.
set(Testbed_Framework_SRCS
	Framework/DebugDraw.cpp
	Framework/DebugDraw.h
	Framework/Main.cpp
	Framework/Test.cpp
	Framework/Test.h
)

#define the test files.
set(Testbed_Tests_SRCS
	Tests/TestEntries.cpp
	Tests/AddPair.h
	Tests/ApplyForce.h
	Tests/BodyTypes.h
	Tests/Breakable.h
	Tests/Bridge.h
	Tests/BulletTest.h
	Tests/Cantilever.h
	Tests/Car.h
	Tests/Chain.h
	Tests/CharacterCollision.h
	Tests/CollisionFiltering.h
	Tests/CollisionProcessing.h
	Tests/CompoundShapes.h
	Tests/Confined.h
	Tests/ContinuousTest.h
	Tests/DistanceTest.h
	Tests/Dominos.h
	Tests/DumpShell.h
	Tests/DynamicTreeTest.h
	Tests/EdgeShapes.h
	Tests/EdgeTest.h
	Tests/Gears.h
	Tests/OneSidedPlatform.h
	Tests/Pinball.h
	Tests/PolyCollision.h
	Tests/PolyShapes.h
	Tests/Prismatic.h
	Tests/Pulleys.h
	Tests/Pyramid.h
	Tests/RayCast.h
	Tests/Revolute.h
	Tests/Rope.h
	Tests/RopeJoint.h
	Tests/SensorTest.h
	Tests/ShapeEditing.h
	Tests/SliderCrank.h
	Tests/SphereStack.h
	Tests/TheoJansen.h
	Tests/Tiles.h
	Tests/TimeOfImpact.h
	Tests/VaryingFriction.h
	Tests/VaryingRestitution.h
	Tests/VerticalStack.h
	Tests/Web.h
)

# These are used to create visual studio folders.
source_group(Framework FILES ${Testbed_Framework_SRCS})
source_group(Tests FILES ${Testbed_Tests_SRCS})

include_directories(
	${Box2D_SOURCE_DIR}
	${IMGUI_INCLUDE_DIR}
	${GLFW_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
)

add_executable(Testbed
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
	${IMGUI}
)

target_link_libraries(Testbed Box2D glfw ${GLFW_LIBRARIES})
