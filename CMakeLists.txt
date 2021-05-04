cmake_minimum_required(VERSION 3.12)
project(lets-implement-bvh LANGUAGES CXX)

# extern
add_subdirectory("extern")

# bvh
add_library(bvh INTERFACE)
target_compile_features(bvh INTERFACE cxx_std_17)
target_include_directories(bvh INTERFACE "include")

# example
add_subdirectory("example")
