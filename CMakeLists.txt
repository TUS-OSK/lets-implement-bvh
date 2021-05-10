cmake_minimum_required(VERSION 3.12)
project(lets-implement-bvh LANGUAGES CXX)

# extern
add_subdirectory("extern")

# bvh
add_library(bvh INTERFACE)
target_compile_features(bvh INTERFACE cxx_std_17)
target_compile_options(bvh INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -march=native>
)
target_include_directories(bvh INTERFACE "include")

# example
add_subdirectory("example")
