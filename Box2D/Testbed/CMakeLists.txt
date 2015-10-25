# Some flags for Freeglut and GLUI.
add_definitions(-DGLEW_STATIC -D_CRT_SECURE_NO_WARNINGS)

list(APPEND Testbed_LIBS Box2D)
list(APPEND Testbed_INCLUDES ${Box2D_SOURCE_DIR})

if(BOX2D_DISABLE_BUNDLED_GLFW)
	add_definitions(-DBOX2D_DISABLE_BUNDLED_GLFW)
	find_package(PkgConfig REQUIRED)
	pkg_search_module(GLFW REQUIRED glfw3)

	list(APPEND Testbed_LIBS ${GLFW_LIBRARIES})
	list(APPEND Testbed_INCLUDES ${GLFW_INCLUDE_DIRS})
else()
	list(APPEND Testbed_LIBS glfw ${GLFW_LIBRARIES})
	list(APPEND Testbed_INCLUDES ${GLFW_INCLUDE_DIRS})
endif()

if(BOX2D_DISABLE_BUNDLED_GLEW)
	add_definitions(-DBOX2D_DISABLE_BUNDLED_GLEW)
	list(APPEND Testbed_LIBS glew)
else()
	find_package(GLEW)
	list(APPEND Testbed_LIBS ${GLEW_LIBRARIES})
	list(APPEND Testbed_INCLUDES ${GLEW_INCLUDE_DIRS})
endif()

list(APPEND Testbed_LIBS GL)

# Define the framework files.
set(Testbed_Framework_SRCS
	Framework/DebugDraw.cpp
	Framework/imgui.cpp
	Framework/Main.cpp
	Framework/RenderGL3.cpp
	Framework/Test.cpp
)

# Define the test files.
set(Testbed_Tests_SRCS
	Tests/TestEntries.cpp
)

# These are used to create visual studio folders.
source_group(Framework FILES ${Testbed_Framework_SRCS})
source_group(Tests FILES ${Testbed_Tests_SRCS})

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	link_directories(/usr/X11/lib)
	list(APPEND Testbed_LIBS GL GLU X11)
	list(APPEND Testbed_INCLUDES /usr/X11/include)
elseif(WIN32)
	list(APPEND Testbed_LIBS winmm)
endif(APPLE)

add_executable(Testbed
	${Testbed_Framework_SRCS}
	${Testbed_Tests_SRCS}
)

include_directories(${Testbed_INCLUDES})

target_link_libraries(Testbed ${Testbed_LIBS})

file(COPY ../Build/Data DESTINATION ..)
