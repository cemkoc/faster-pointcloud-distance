//
// Created by Cem Koc on 4/11/21.
//

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

#include <omp.h>

#define NUM_THREADS 4

namespace distance {
namespace omp {

    static inline int test_omp_cem() {
      int total = 0;
      #pragma omp parallel for num_threads(NUM_THREADS)
      for (int i = 0; i < 100; ++i) {
        int tid = omp_get_thread_num();
        total += tid;
      }
      return total;
    }

    static inline float l2_norm_sq(const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
      return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
    }

    class Distance {
      public:
          Distance() {}
          double compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr);
          double compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr, int k);
    };

} // namespace omp
} // namespace distance
