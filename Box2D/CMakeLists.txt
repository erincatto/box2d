cmake_minimum_required(VERSION 2.8)

project(Box2D)

# The Box2D library.
add_subdirectory(Box2D)

# HelloWorld console example.
add_subdirectory(HelloWorld)

# Testbed and dependencies.
find_package(OpenGL REQUIRED)
add_subdirectory(freeglut)
add_subdirectory(glui)
add_subdirectory(Testbed)