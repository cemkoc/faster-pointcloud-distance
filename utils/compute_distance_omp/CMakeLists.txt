add_executable(compute_distance_omp main.cpp)

target_link_libraries(compute_distance_omp
  faster_cloud_distance
)
