cmake_minimum_required(VERSION 3.12)
project(lets-implement-bvh LANGUAGES CXX)

# extern
add_subdirectory("extern")

add_executable(main "src/main.cpp")
target_compile_features(main PRIVATE cxx_std_17)
target_include_directories(main PRIVATE "include")
target_link_libraries(main PRIVATE tinyobjloader)