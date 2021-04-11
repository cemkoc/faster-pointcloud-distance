//
// Created by Cem Koc on 4/11/21.
//

#include "cloud_distance/distance.h"
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>



namespace distance {
double Distance::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr) {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_b;
  kdtree_b.setInputCloud(cloud_b_ptr);

  pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;

  double sum_distances = 0.0;
  for (auto point_iter=cloud_a.begin(); pointer_iter != cloud_a.end(); ++point_iter) {

  }
  int K = 10;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<double> pointNKNSquaredDistance(K);

  std::cout << "K-nearest neighbor search at (" << std::endl;

}

}

}




