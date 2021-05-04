add_executable(main "main.cpp")
target_compile_features(main PRIVATE cxx_std_17)
target_link_libraries(main PRIVATE bvh)
target_link_libraries(main PRIVATE tinyobjloader)