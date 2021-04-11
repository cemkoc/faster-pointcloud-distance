//
// Created by Cem Koc on 4/8/21.
//

#include "cloud_distance/sample.h"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

namespace cloud {
namespace distance {

size_t read_pointcloud(const std::string& fname) {
  std::cout << fname << std::endl;
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::io::loadPLYFile(fname, point_cloud);

  for (auto point_iter=point_cloud.begin(); point_iter != point_cloud.end(); ++point_iter) {
    pcl::PointXYZ p = *point_iter;
    std::cout << "Point (X Y Z): " << p.x << " " <<  p.y << " " << p.z << std::endl;

  }
  return point_cloud.size();
}

}
}
