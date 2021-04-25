#include "cloud_distance/distance_omp.h"
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <cmath>


namespace distance {
namespace omp {

/**
 * Serial implementation of Bi-directional Chamfer distance between two point clouds.
 * @param cloud_a_ptr pointer to reference point cloud
 * @param cloud_b_ptr pointer to compared point cloud
 * @return distance
 */
double Distance::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr) {
  double sum_a = 0.0; // we can try making this an array[NUM_THREADS] and pad it to avoid false sharing
  double sum_b = 0.0; // same thing as above
  float min_sofar;

  pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
  pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;

  // Forward direction Chamfer
  #pragma omp parallel for num_threads(NUM_THREADS) schedule(dynamic) private(min_sofar)
  for (auto point_iter_a = cloud_a.begin(); point_iter_a < cloud_a.end(); ++point_iter_a) {
    pcl::PointXYZ point_a = *point_iter_a;
    min_sofar = std::numeric_limits<float>::max();
    for (pcl::PointCloud<pcl::PointXYZ>::iterator point_iter_b = cloud_b.begin(); point_iter_b != cloud_b.end(); ++point_iter_b) {
      pcl::PointXYZ point_b = *point_iter_b;
      float dist = l2_norm_sq(point_a, point_b);
      if (dist < min_sofar) {
        min_sofar = dist;
      }
    }
    #pragma omp critical
    sum_a = sum_a + static_cast<double>(min_sofar);
  }


  // Backward direction Chamfer
  #pragma omp parallel for num_threads(NUM_THREADS) schedule(dynamic) private(min_sofar)
  for (auto point_iter_b = cloud_b.begin(); point_iter_b < cloud_b.end(); ++point_iter_b) {
    pcl::PointXYZ point_b = *point_iter_b;
    min_sofar = std::numeric_limits<float>::max();
    for (pcl::PointCloud<pcl::PointXYZ>::iterator point_iter_a = cloud_a.begin(); point_iter_a != cloud_a.end(); ++point_iter_a) {
      pcl::PointXYZ point_a = *point_iter_a;
      float dist = l2_norm_sq(point_b, point_a);
      if (dist < min_sofar) {
        min_sofar = dist;
      }
    }
    #pragma omp critical
    sum_b = sum_b + static_cast<double>(min_sofar);
  }

  return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}

/**
 * Serial implementation of bi-directional Chamfer distance with k-nearest neighbors instead of all-pairs.
 * Uses a KD-tree structure for KNN queries.
 *
 * In principle, when comparing two point clouds if you can make assumptions on the orientation
 * than you can simply observe that for each point in cloud A we do not have to iterate over all
 * the points in cloud B and instead can search locally in cloud B. Using a KD-tree we can efficiently
 * do k-nearest neighbor searches for each point in A and only compute the distance for those in the
 * returned set in B.
 *
 * @param cloud_a_ptr
 * @param cloud_b_ptr
 * @param k neighbor number
 * @return
 */
double Distance::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr, int k) {

  printf("Running with K=%d\n", k);
  pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
  pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;

  printf("Building KD-Tree for Cloud B\n");
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_b;
  kdtree_b.setInputCloud(cloud_b_ptr);
  printf("done.\n");
  printf("Building KD-Tree for Cloud A\n");
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_a;
  kdtree_a.setInputCloud(cloud_a_ptr);
  printf("done.\n");

  double result = 0.0;
  double sum_a = 0.0;
  double sum_b = 0.0;
  float minsofar;

  // define two vectors to hold the search result.
  std::vector<int> pointIdxKNNSearch(k); // holds the resultant indices of the neighboring points
  std::vector<float> pointKNNSquaredDistance(k); // holds the resultant squared distances to nearby points

  // forward direction
  // pragma omp for num_threads
  for (auto point_iter_a = cloud_a.begin(); point_iter_a != cloud_a.end(); ++point_iter_a) {
    pcl::PointXYZ point_a = *point_iter_a;
//    std::cout << "K-nearest neighbor search at (" << point_a.x
//              << " " << point_a.y
//              << " " << point_a.z
//              << ") with K=" << k << std::endl;

    if (kdtree_b.nearestKSearch(point_a, k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {

      minsofar = std::numeric_limits<float>::max();

      for (std::size_t i = 0; i < pointIdxKNNSearch.size(); ++i) {
        // get the index of current point in cloud b
        int b_idx = pointIdxKNNSearch[i];
        pcl::PointXYZ point_b;
        point_b.x = cloud_b[b_idx].x;
        point_b.y = cloud_b[b_idx].y;
        point_b.z = cloud_b[b_idx].z;

        // compute l2 distance squared
        float dist = l2_norm_sq(point_a, point_b);
        if (dist < minsofar) {
          minsofar = dist;
        }
      }

      sum_a = sum_a + static_cast<double>(minsofar);
    }
  }

  // empty the vectors
  pointIdxKNNSearch.clear();
  pointKNNSquaredDistance.clear();

  // backward direction
  for (auto point_iter_b = cloud_b.begin(); point_iter_b != cloud_b.end(); ++point_iter_b) {
    pcl::PointXYZ point_b = *point_iter_b;
//    std::cout << "K-nearest neighbor search at (" << point_b.x
//              << " " << point_b.y
//              << " " << point_b.z
//              << ") with K=" << k << std::endl;

    if (kdtree_a.nearestKSearch(point_b, k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {

      minsofar = std::numeric_limits<float>::max();

      for (std::size_t i = 0; i < pointIdxKNNSearch.size(); ++i) {
        // get the index of current point in cloud a
        int a_idx = pointIdxKNNSearch[i];
        pcl::PointXYZ point_a;
        point_a.x = cloud_a[a_idx].x;
        point_a.y = cloud_a[a_idx].y;
        point_a.z = cloud_a[a_idx].z;

        // compute the l2 distance squared
        float dist = l2_norm_sq(point_b, point_a);
        if (dist < minsofar) {
          minsofar = dist;
        }
      }

      sum_b = sum_b + static_cast<double>(minsofar);
    }
  }

  return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}


} // namespace omp
} // namespace distance