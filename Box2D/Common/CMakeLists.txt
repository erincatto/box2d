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

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} OBJECT ${Common_SRC})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
