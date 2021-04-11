//
// Created by Cem Koc on 4/8/21.
//

#pragma once
#include <iostream>
#include <cmath>

#include <pcl/io/ply_io.h>

namespace cloud {
namespace distance {

inline void faster_pointcloud_distance_hello() {
    std::cout << "Hello World." << std::endl;
}

size_t read_pointcloud(const std::string& fname);

}  // namespace distance
}  // namespace cloud
