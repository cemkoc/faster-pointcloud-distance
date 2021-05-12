#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace distance {
  class DistanceCuda {
  public:
    double
      compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr);
      double
      compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr,
                        int k);
  };
}