cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_language(C CXX)

project(faster_pcl_distance C CXX)
find_package(OpenMP REQUIRED)
find_package(UPCXX)

add_subdirectory(cloud_distance)
add_subdirectory(utils)