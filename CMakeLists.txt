cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_language(C CXX)

project(faster_pcl_distance C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/modules")

find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(UPCXX)

add_subdirectory(cloud_distance)
add_subdirectory(utils)