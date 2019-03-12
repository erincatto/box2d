# cmake for Rope

project(Rope)

set(Rope_SRC
  "b2Rope.cpp"
  )

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} OBJECT ${Rope_SRC})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
