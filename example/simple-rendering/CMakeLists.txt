add_executable(simple-rendering "main.cpp")
target_include_directories(simple-rendering PRIVATE ".")
target_link_libraries(simple-rendering PRIVATE bvh)
target_link_libraries(simple-rendering PRIVATE tinyobjloader)