//
// Created by Cem Koc on 4/8/21.
//

#pragma once
#include <iostream>
#include <cmath>

#include <pcl/io/ply_io.h>
#include <pcl/impl/point_types.hpp>

namespace cloud {
namespace distance {

pcl::PointCloud<pcl::PointXYZ> read_pointcloud(const std::string& fname);

}  // namespace distance
}  // namespace cloud
