//
// Created by Cem Koc on 4/9/21.
//

#include <chrono>
#include <iostream>
#include "cloud_distance/io.h"
#include "cloud_distance/distance.cuh"

int main(int argc, char* argv[]) {


  std::cout << "Reading two point clouds." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  *cloud_a_ptr = distance::io::read_pointcloud(std::string(argv[1]));
  *cloud_b_ptr = distance::io::read_pointcloud(std::string(argv[2]));

  distance::DistanceCuda dist;
  double distance;

  // distance = dist.compute_distance(cloud_a_ptr, cloud_b_ptr);
  // std::cout << "reference distance " << distance << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  // all-pairs Chamfer distance
  int numtorun = 1;
  for (int i = 0; i < numtorun; ++i) {
    distance = dist.compute_distance_radius(cloud_a_ptr, cloud_b_ptr, 1);
  }


  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "[All-pairs] Chamfer distance between: " << distance << std::endl;


  double time_diff = std::chrono::duration<double>(end - start).count() / numtorun;
  printf("Time: %lf seconds\n ", time_diff);

  return 0;
}
