# OpenMP
find_package(OpenMP)

add_executable(path-tracing "main.cpp")
target_link_libraries(path-tracing PRIVATE bvh)
target_link_libraries(path-tracing PRIVATE tinyobjloader)
target_include_directories(path-tracing PRIVATE "../common")

if(OpenMP_CXX_FOUND)
  target_link_libraries(path-tracing PRIVATE OpenMP::OpenMP_CXX)
else()
  message(WARNING "failed to find OpenMP")
endif()