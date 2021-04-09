add_executable(read_pointcloud main.cpp)

target_link_libraries(read_pointcloud
  faster_cloud_distance
)