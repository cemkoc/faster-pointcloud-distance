//
// Created by Cem Koc on 4/9/21.
//

#include <chrono>
#include <iostream>
#include "cloud_distance/io.h"
#include "cloud_distance/distance_omp.h"

int main(int argc, char* argv[]) {

  // int total = distance::omp::test_omp_cem();
  // std::cout << "Total after OMP Block " << total << std::endl;

  std::cout << "Reading two point clouds." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  *cloud_a_ptr = distance::io::read_pointcloud(std::string(argv[1]));
  *cloud_b_ptr = distance::io::read_pointcloud(std::string(argv[2]));

  distance::omp::Distance dist;
  double distance;
  int numruns = 5;

  auto start = std::chrono::high_resolution_clock::now();
  distance = dist.compute_distance(cloud_a_ptr, cloud_b_ptr);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "[All-pairs] Chamfer distance between: " << distance << std::endl;
  double time_diff = std::chrono::duration<double>(end - start).count();
  printf("Time: %lf seconds\n ", time_diff);

  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < numruns; ++i)
    distance = dist.compute_distance(cloud_a_ptr, cloud_b_ptr, 100);
  end = std::chrono::high_resolution_clock::now();
  std::cout << "[kd] Chamfer distance between: " << distance << std::endl;
  time_diff = std::chrono::duration<double>(end - start).count();
  printf("Time: %lf seconds\n ", time_diff / numruns);

  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < numruns; ++i)
    distance = dist.compute_distance_oct(cloud_a_ptr, cloud_b_ptr);
  end = std::chrono::high_resolution_clock::now();
  std::cout << "[oct] Chamfer distance between: " << distance << std::endl;
  time_diff = std::chrono::duration<double>(end - start).count();
  printf("Time: %lf seconds\n ", time_diff / numruns);

  // localized Chamfer
//  distance = dist.compute_distance(cloud_a_ptr, cloud_b_ptr, 1000);
//
//  auto end = std::chrono::high_resolution_clock::now();
//  std::cout << "[Localized] Chamfer distance between: " << distance << std::endl;

  

  return 0;
}