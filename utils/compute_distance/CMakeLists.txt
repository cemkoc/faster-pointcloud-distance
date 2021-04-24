add_executable(compute_distance main.cpp)

target_link_libraries(compute_distance
  faster_cloud_distance
  ${Boost_LIBRARIES}
)
