add_library(faster_cloud_distance
        src/distance.cpp
        include/cloud_distance/io.h
        include/cloud_distance/distance.h)

target_include_directories(faster_cloud_distance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include/mylib
)

target_link_libraries(faster_cloud_distance
        pcl_io
        pcl_common
        pcl_kdtree)
