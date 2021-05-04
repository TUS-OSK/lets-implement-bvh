add_executable(simple-example "main.cpp")
target_link_libraries(simple-example PRIVATE bvh)
target_link_libraries(simple-example PRIVATE tinyobjloader)