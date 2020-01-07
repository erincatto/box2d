project(sajson CXX)

add_library(sajson INTERFACE)
# target_sources(sajson INTERFACE sajson.h)
target_include_directories(sajson INTERFACE ..)
