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
      // __host__ void compute_kernel(pcl::PointXYZ* cloud_a_ptr, pcl::PointXYZ* cloud_b_ptr,
      //                                int asize, int bsize, double* mins);
  };
}