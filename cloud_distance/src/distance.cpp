//
// Created by Cem Koc on 4/11/21.
//

#include "cloud_distance/distance.h"
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <cmath>



namespace distance {

/**
 * Serial implementation of Bi-directional Chamfer distance between two point clouds.
 * @param cloud_a_ptr pointer to reference point cloud
 * @param cloud_b_ptr pointer to compared point cloud
 * @return distance
 */
double Distance::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr) {
  double sum_a = 0.0;
  double sum_b = 0.0;
  float min_sofar;

  pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
  pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;

  // Forward direction Chamfer
  for (auto point_iter_a = cloud_a.begin(); point_iter_a != cloud_a.end(); ++point_iter_a) {
    pcl::PointXYZ point_a = *point_iter_a;
    min_sofar = std::numeric_limits<float>::max();
    for (auto point_iter_b = cloud_b.begin(); point_iter_b != cloud_b.end(); ++point_iter_b) {
      pcl::PointXYZ point_b = *point_iter_b;
      float dist = distance::l2_norm_sq(point_a, point_b);
      if (dist < min_sofar) {
        min_sofar = dist;
      }
    }

    sum_a = sum_a + static_cast<double>(min_sofar);
  }

  // Backward direction Chamfer
  for (auto point_iter_b = cloud_b.begin(); point_iter_b != cloud_b.end(); ++point_iter_b) {
    pcl::PointXYZ point_b = *point_iter_b;
    min_sofar = std::numeric_limits<float>::max();
    for (auto point_iter_a = cloud_a.begin(); point_iter_a != cloud_a.end(); ++point_iter_a) {
      pcl::PointXYZ point_a = *point_iter_a;
      float dist = distance::l2_norm_sq(point_b, point_a);
      if (dist < min_sofar) {
        min_sofar = dist;
      }
    }

    sum_b = sum_b + static_cast<double>(min_sofar);
  }

  return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}

/**
 * Serial implementation of bi-directional Chamfer distance with k-nearest neighbors instead of all-pairs.
 * Uses a KD-tree structure for KNN queries.
 *
 * @param cloud_a_ptr
 * @param cloud_b_ptr
 * @param k neighbor number
 * @return
 */
double Distance::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr, int k) {

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_b;
  kdtree_b.setInputCloud(cloud_b_ptr);

  pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;

  double result = 0.0;

  for (auto point_iter=cloud_a.begin(); point_iter != cloud_a.end(); ++point_iter) {

  }
  int K = 10;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<double> pointNKNSquaredDistance(K);

  std::cout << "K-nearest neighbor search at (" << std::endl;

  return result;
}

}




