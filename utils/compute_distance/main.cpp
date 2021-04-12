//
// Created by Cem Koc on 4/9/21.
//

#include <iostream>
#include "cloud_distance/io.h"
#include "cloud_distance/distance.h"

int main(int argc, char* argv[]) {
  std::cout << "Reading two point clouds." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  *cloud_a_ptr = distance::io::read_pointcloud(std::string(argv[1]));
  *cloud_b_ptr = distance::io::read_pointcloud(std::string(argv[2]));

  distance::Distance dist;
  double distance = dist.compute_distance(cloud_a_ptr, cloud_b_ptr);
  std::cout << "Distance between: " << distance << std::endl;

  return 0;
}