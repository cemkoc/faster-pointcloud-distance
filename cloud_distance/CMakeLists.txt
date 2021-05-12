add_library(faster_cloud_distance
        src/distance.cpp
        src/distance.cu
        include/cloud_distance/io.h
        include/cloud_distance/distance.h
        include/cloud_distance/distance.cuh)

target_include_directories(faster_cloud_distance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # <prefix>/include/mylib
)

target_link_libraries(faster_cloud_distance ${CUDA_LIBRARIES})

target_link_libraries(faster_cloud_distance
      PUBLIC
        pcl_io
        pcl_common
        pcl_kdtree
        pcl_gpu_octree
        ${Boost_LIBRARIES}
        OpenMP::OpenMP_CXX
)
