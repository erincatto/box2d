add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS )

set(Framework
	Framework/Main.cpp
	Framework/Render.cpp
	Framework/Test.cpp
)

set(Tests
	Tests/TestEntries.cpp
)

include_directories (
	${OPENGL_INCLUDE_DIR}
	${Box2D_SOURCE_DIR}
)

link_directories (
	${Box2D_BINARY_DIR}
) 

add_executable(Testbed
	${Framework}
	${Tests}
)

target_link_libraries (
	Testbed
	Box2D
	freeglut
	glui
	${OPENGL_LIBRARIES}
) 

