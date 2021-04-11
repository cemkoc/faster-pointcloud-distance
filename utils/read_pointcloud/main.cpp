//
// Created by Cem Koc on 4/9/21.
//

#include <iostream>
#include "cloud_distance/io.h"

int main(int argc, char* argv[]) {

  pcl::PointCloud<pcl::PointXYZ> pointcloud_a = distance::io::read_pointcloud(std::string(argv[1]));
  pcl::PointCloud<pcl::PointXYZ> pointcloud_b = distance::io::read_pointcloud(std::string(argv[2]));

  std::cout << "First Point Cloud size: " << pointcloud_a.size() << std::endl;
  std::cout << "Second Point Cloud size: " << pointcloud_b.size() << std::endl;

  return 0;
}
