# cmake for Common

project(Common)

set(Common_SRC
    "b2BlockAllocator.cpp"
    "b2Draw.cpp"
    "b2Math.cpp"
    "b2Settings.cpp"
    "b2StackAllocator.cpp"
    "b2Timer.cpp"
  )

add_library(${PROJECT_NAME} OBJECT ${Common_SRC})
target_compile_options(${PROJECT_NAME} PRIVATE "-std=c++11")
