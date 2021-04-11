//
// Created by Cem Koc on 4/9/21.
//

#include <iostream>
#include "cloud_distance/sample.h"

  int main(int argc, char* argv[]) {

  size_t point_cloud_size = cloud::distance::read_pointcloud(std::string(argv[1]));
  std::cout << "Point Cloud size: " << point_cloud_size << std::endl;
  return 0;
}
