# cmake for Rope

project(Rope)

set(Rope_SRC
  "b2Rope.cpp"
  )

add_library(${PROJECT_NAME} OBJECT ${Rope_SRC})
target_compile_options(${PROJECT_NAME} PRIVATE "-std=c++11")
