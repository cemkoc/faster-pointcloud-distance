//
// Created by Cem Koc on 4/11/21.
//

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

namespace distance {
    class Distance {
      public:
          Distance() {}

          Distance(int num_neighbors) : k(num_neighbors) {}

          double compute_distance(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> cloud_a_ptr, boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> cloud_b_ptr);

      private:
          int k = 1; // how many neighbors to search when computing distance
    };

} // namespace distance
