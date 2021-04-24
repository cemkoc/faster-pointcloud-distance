
set(SUBSYS_NAME cloud_distance)
set(SUBSYS_DESC "Point cloud faster cloud distance library")
set(SUBSYS_DEPS common kdtree io octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

if(NOT build)
  return()
endif()

set(incs
    include/pcl/cloud_distance/distance.h
    include/pcl/cloud_distance/io.h
    include/pcl/cloud_distance/distance.cuh)

set(srcs src/distance.cpp
         src/distance.cu)

set(LIB_NAME "pcl_${SUBSYS_NAME}")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries("${LIB_NAME}" pcl_common pcl_kdtree pcl_io pcl_octree pcl_gpu_octree pcl_gpu_containers)

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})


PCL_ADD_EXECUTABLE(pcl_cloud_distance_compute_distance COMPONENT "${SUBSYS_NAME}_utils" SOURCES utils/compute_distance.cpp)
target_link_libraries(pcl_cloud_distance_compute_distance
  pcl_cloud_distance  
)
target_compile_definitions(pcl_cloud_distance_compute_distance PUBLIC -DBOOST_NO_CXX11_SCOPED_ENUMS)

PCL_ADD_EXECUTABLE(pcl_cloud_distance_read_point_cloud COMPONENT "${SUBSYS_NAME}_utils" SOURCES utils/read_point_cloud.cpp)
target_link_libraries(pcl_cloud_distance_read_point_cloud
  pcl_cloud_distance
)
target_compile_definitions(pcl_cloud_distance_read_point_cloud PUBLIC -DBOOST_NO_CXX11_SCOPED_ENUMS)

#PCL_ADD_EXECUTABLE(pcl_cloud_distance_compute_distance_cuda COMPONENT "${SUBSYS_NAME}_utils" SOURCES utils/compute_distance_cuda.cu)
#target_link_libraries(pcl_cloud_distance_compute_distance_cuda
#  pcl_cloud_distance
#)

#target_compile_definitions(pcl_cloud_distance_compute_distance_cuda PUBLIC -DBOOST_NO_CXX11_SCOPED_ENUMS)
#set_target_properties(pcl_cloud_distance_compute_distance_cuda PROPERTIES LINKER_LANGUAGE CXX)
#set_target_properties(pcl_cloud_distance_compute_distance_cuda PROPERTIES LINKER_LANGUAGE C)