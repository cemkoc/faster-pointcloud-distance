#include <cmath>
#include <vector>
#include <numeric>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/octree.hpp>
#include <thrust/device_vector.h>
#include <thrust/reduce.h>

#include <pcl/cloud_distance/distance.cuh>

#include <iostream>

#define NUM_THREADS 512

__global__ void compute_kernel(pcl::PointXYZ* cloud_a_ptr, pcl::PointXYZ* cloud_b_ptr,
    int asize, int bsize, double* mins) {
    int point_id = threadIdx.x + blockIdx.x * blockDim.x;
    if (point_id >= asize) {
        return;
    }
    pcl::PointXYZ point_a = cloud_a_ptr[point_id];

    double min_sofar = std::numeric_limits<double>::max();
    for (int i = 0; i < bsize; ++i) {
        pcl::PointXYZ point_b = cloud_b_ptr[i];
        double dist = pow(point_a.x - point_b.x, 2.0) + pow(point_a.y - point_b.y, 2.0) + pow(point_a.z - point_b.z, 2.0);
        if (dist < min_sofar) {
            min_sofar = dist;
        }
    }
    if (point_id % 10000 == 0) {
        printf("reached after\n");
    }
    mins[point_id] = min_sofar;
}

namespace distance {

double DistanceCuda::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr) {
    pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
    pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;

    pcl::PointXYZ* cuda_a_ptr;
    pcl::PointXYZ* cuda_b_ptr;
    double* mins;
    cudaMalloc((void**)&mins, max(cloud_a.points.size(), cloud_b.points.size()) * sizeof(double));
    cudaMalloc((void**)&cuda_a_ptr, cloud_a.points.size() * sizeof(pcl::PointXYZ));
    cudaMalloc((void**)&cuda_b_ptr, cloud_b.points.size() * sizeof(pcl::PointXYZ));
    std::cout << "blah" << std::endl;
    cudaMemcpy(cuda_a_ptr, &(cloud_a.points), cloud_a.points.size() * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

    std::cout << "a alloced" << std::endl;
    cudaMemcpy(cuda_b_ptr, &(cloud_b.points), cloud_b.points.size() * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
    std::cout << "finished allocs" << std::endl;

    int blks = (cloud_a.points.size() + NUM_THREADS - 1) / NUM_THREADS;
    compute_kernel<<<blks, NUM_THREADS>>>(cuda_a_ptr, cuda_b_ptr, cloud_a.points.size(), cloud_b.points.size(), mins);
    cudaDeviceSynchronize();
    std::cout << "first calc done" << std::endl;
    double sum_a = thrust::reduce(thrust::device, mins, mins + cloud_a.points.size(), 0.0);  // TODO THIS NEED TO BE DEVICE

    std::cout << "first red done" << std::endl;

    blks = (cloud_b.points.size() + NUM_THREADS - 1) / NUM_THREADS;
    compute_kernel<<<blks, NUM_THREADS>>>(cuda_b_ptr, cuda_a_ptr, cloud_b.points.size(), cloud_a.points.size(), mins);
    std::cout << "second calc done" << std::endl;
    double sum_b = thrust::reduce(thrust::device, mins, mins + cloud_b.points.size(), 0.0);

    return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}

// double DistanceCuda::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
//                                       pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr, int k) {

//     printf("Running with K=%d\n", k);
//     pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
//     pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;
    
//     thrust::device_vector<float> mins(max(cloud_a.points.size(), cloud_b.points.size()));
//     // can we reuse this for iteration? or do we need to upload 
//     // the points separately
//     pcl::gpu::Octree::PointCloud cloud_a_device;
//     pcl::gpu::Octree::PointCloud cloud_b_device;
//     cloud_a_device.upload(cloud_a.points);
//     cloud_b_device.upload(cloud_b.points);

//     pcl::gpu::Octree::Queries queries_a_device = cloud_a_device;
//     pcl::gpu::Octree::Queries queries_b_device = cloud_b_device;

//     // reuse the same octree for both a and b
//     pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
//     octree_device->setCloud(cloud_b_device);
//     octree_device->build();

//     // can we reuse one of the _x_ind?
//     pcl::gpu::NeighborIndices _a_ind(cloud_a.size(), 1);
//     pcl::gpu::NeighborIndices _b_ind(cloud_b.size(), 1);
//     pcl::gpu::Octree::ResultSqrDists a_res;
//     pcl::gpu::Octree::ResultSqrDists b_res;

//     octree_device.nearestKSearchBatch(queries_a_device, 1, _a_ind, a_res);

//     // can we sum on device instead?
//     std::vector<float> downloaded(cloud_a.size());
//     a_res.download(downloaded);
//     double sum_a = accumulate(downloaded.begin(), downloaded.end(), 0.0);

//     octree_device->setCloud(cloud_a_device);
//     octree_device->build();
//     octree_device.nearestKSearchBatch(queries_b_device, 1, _b_ind, b_res);

//     // can we sum on device instead?
//     downloaded.resize(cloud_b.size());
//     a_res.download(downloaded);
//     double sum_b = accumulate(downloaded.begin(), downloaded.end(), 0.0);

//     /* or should we do this method/ is it possible?
//     compute_kernel(cloud_a_device, octree_device, mins);
//     double sum_a = thrust::reduce(mins.begin(), mins.begin() + cloud_a.points.size(), thrust::plus<float>());

//     octree_device->setCloud(cloud_a_device);
//     octree_device->build();

//     compute_kernel(cloud_b_device, octree_device, mins);
//     double sum_b = thrust::reduce(mins.begin(), mins.begin() + cloud_b.points.size(), thrust::plus<float>());
//     */
//     return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
// }

// // __global__ void compute_kernel(pcl::gpu::Octree::PointCloud cloud_a, pcl::gpu::Octree::Ptr octree_ptr,
// //                                thrust::device_vector& mins) {
// //     int point_id = threadIdx.x + blockIdx.x * blockDim.x;
// //     if (point_id > asize) {
// //         return;
// //     }
// //     pcl::
// // }
}