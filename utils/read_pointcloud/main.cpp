//
// Created by Cem Koc on 4/9/21.
//

#include <iostream>
#include "cloud_distance/io.h"

int main(int argc, char* argv[]) {

  pcl::PointCloud<pcl::PointXYZ> pcl1 = cloud::distance::read_pointcloud(std::string(argv[1]));
  pcl::PointCloud<pcl::PointXYZ> pcl2 = cloud::distance::read_pointcloud(std::string(argv[2]));

  std::cout << "First Point Cloud size: " << pcl1.size() << std::endl;
  std::cout << "Second Point Cloud size: " << pcl2.size() << std::endl;
  return 0;
}
