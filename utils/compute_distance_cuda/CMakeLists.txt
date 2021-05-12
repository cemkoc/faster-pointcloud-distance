add_executable(compute_distance_cuda main.cpp)

target_link_libraries(compute_distance_cuda
  faster_cloud_distance
)
