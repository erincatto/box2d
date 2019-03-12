# cmake for Collisions

project(Collision)

set(Collision_SRC
  "b2BroadPhase.cpp"
  "b2CollideCircle.cpp"
  "b2CollideEdge.cpp"
  "b2CollidePolygon.cpp"
  "b2Collision.cpp"
  "b2Distance.cpp"
  "b2DynamicTree.cpp"
  "b2TimeOfImpact.cpp"
  )

set(Shapes_SRC
  "Shapes/b2ChainShape.cpp"
  "Shapes/b2CircleShape.cpp"
  "Shapes/b2EdgeShape.cpp"
  "Shapes/b2PolygonShape.cpp"
  )

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} OBJECT ${Collision_SRC} ${Shapes_SRC})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
