#include <cmath>
#include <vector>
#include <numeric>
//#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/octree.hpp>
#include <thrust/device_vector.h>
#include <thrust/reduce.h>

#include "cloud_distance/distance.cuh"

#include <iostream>

#define NUM_THREADS 512

__global__ void naive_compute_kernel(pcl::PointXYZ* cloud_a_ptr, pcl::PointXYZ* cloud_b_ptr,
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
    mins[point_id] = min_sofar;
}

__global__ void radius_compute_kernel(int size, int max_res, int* octree_indices, int* res_sizes,
                                      pcl::PointXYZ* octree_points, pcl::PointXYZ* search_points, double* mins) {
    int point_id = threadIdx.x + blockIdx.x * blockDim.x;
    if (point_id >= size) {
        return;
    }
    if (!res_sizes[point_id]) {
        mins[point_id] = 0.0;
        return;
    }

    pcl::PointXYZ point_source = search_points[point_id];
    double min_sofar = std::numeric_limits<double>::max();
    for (int i = 0; i < res_sizes[point_id]; ++i) {
        pcl::PointXYZ radius_point = octree_points[octree_indices[point_id * max_res + i]];
        double dist = pow(point_source.x - radius_point.x, 2.0) + pow(point_source.y - radius_point.y, 2.0) + pow(point_source.z - radius_point.z, 2.0);
        if (dist < min_sofar) {
            min_sofar = dist;
        }
    }
    mins[point_id] = min_sofar;
}

namespace distance {

double DistanceCuda::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr) {
    pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
    pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;

    // in order to upload, we need to essentially unalign the vector rip
    pcl::PointXYZ* local_a_ptr = new pcl::PointXYZ[cloud_a.size()];
    pcl::PointXYZ* local_b_ptr = new pcl::PointXYZ[cloud_b.size()];
    for (int i = 0; i < cloud_a.size(); ++i) {
        local_a_ptr[i] = cloud_a.points[i];
    }
    for (int i = 0; i < cloud_b.size(); ++i) {
        local_b_ptr[i] = cloud_b.points[i];
    }

    pcl::PointXYZ* cuda_a_ptr;
    pcl::PointXYZ* cuda_b_ptr;
    double* mins;
    cudaMalloc((void**)&mins, max(cloud_a.size(), cloud_b.size()) * sizeof(double));
    cudaMalloc((void**)&cuda_a_ptr, cloud_a.size() * sizeof(pcl::PointXYZ));
    cudaMalloc((void**)&cuda_b_ptr, cloud_b.size() * sizeof(pcl::PointXYZ));

    cudaMemcpy(cuda_a_ptr, local_a_ptr, cloud_a.size() * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
    cudaMemcpy(cuda_b_ptr, local_b_ptr, cloud_b.size() * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

    int blks = (cloud_a.size() + NUM_THREADS - 1) / NUM_THREADS;
    naive_compute_kernel<<<blks, NUM_THREADS>>>(cuda_a_ptr, cuda_b_ptr, cloud_a.size(), cloud_b.size(), mins);

    double sum_a = thrust::reduce(thrust::device, mins, mins + cloud_a.size(), 0.0);  // TODO THIS NEED TO BE DEVICE

    blks = (cloud_b.size() + NUM_THREADS - 1) / NUM_THREADS;
    naive_compute_kernel<<<blks, NUM_THREADS>>>(cuda_b_ptr, cuda_a_ptr, cloud_b.size(), cloud_a.size(), mins);
    double sum_b = thrust::reduce(thrust::device, mins, mins + cloud_b.size(), 0.0);

    return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}

double DistanceCuda::compute_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr, int k) {

    printf("Running with K=%d\n", k);
    pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
    pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;
    
    // can we reuse this for iteration? or do we need to upload 
    // the points separately
    pcl::gpu::Octree::PointCloud cloud_a_device;
    pcl::gpu::Octree::PointCloud cloud_b_device;
    cloud_a_device.upload(cloud_a.points);
    cloud_b_device.upload(cloud_b.points);

    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    pcl::gpu::Octree::Queries queries_device;
    pcl::gpu::NeighborIndices _a_ind(cloud_a.size(), 1);
    pcl::gpu::NeighborIndices _b_ind(cloud_b.size(), 1);
    pcl::gpu::Octree::ResultSqrDists a_res;
    pcl::gpu::Octree::ResultSqrDists b_res;

    queries_device.upload(cloud_a.points);
    octree_device->setCloud(cloud_b_device);
    octree_device->build();

    octree_device->nearestKSearchBatch(queries_device, 1, _a_ind, a_res);

    queries_device.upload(cloud_b.points);

    // can we sum on device instead?
    std::vector<float> downloaded(cloud_a.size());
    a_res.download(downloaded);
    double sum_a = accumulate(downloaded.begin(), downloaded.end(), 0.0);

    octree_device->setCloud(cloud_a_device);
    octree_device->build();
    octree_device->nearestKSearchBatch(queries_device, 1, _b_ind, b_res);

    // can we sum on device instead?
    downloaded.resize(cloud_b.size());
    b_res.download(downloaded);
    double sum_b = accumulate(downloaded.begin(), downloaded.end(), 0.0);

    return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}

double DistanceCuda::compute_distance_radius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_a_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_b_ptr) {

    float rad = 3;
    int max_res = 1024;
    printf("Running with Radius=%f\n", rad);
    pcl::PointCloud<pcl::PointXYZ> cloud_a = *cloud_a_ptr;
    pcl::PointCloud<pcl::PointXYZ> cloud_b = *cloud_b_ptr;
    double* mins;
    cudaMalloc((void**)&mins, max(cloud_a.size(), cloud_b.size()) * sizeof(double));

    // can we reuse this for iteration? or do we need to upload 
    // the points separately
    pcl::gpu::Octree::PointCloud cloud_a_device;
    pcl::gpu::Octree::PointCloud cloud_b_device;
    cloud_a_device.upload(cloud_a.points);
    cloud_b_device.upload(cloud_b.points);

    pcl::gpu::Octree::Queries queries_device;
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    pcl::gpu::NeighborIndices a_ind(cloud_a.size(), max_res);
    pcl::gpu::NeighborIndices b_ind(cloud_b.size(), max_res);

    queries_device.upload(cloud_a.points);
    octree_device->setCloud(cloud_b_device);
    octree_device->build();

    octree_device->radiusSearch(queries_device, rad, max_res, a_ind);
    int blks = (cloud_a.size() + NUM_THREADS - 1) / NUM_THREADS;
    radius_compute_kernel<<<blks, NUM_THREADS>>>(cloud_a.size(), max_res, a_ind.data.ptr(), a_ind.sizes.ptr(),
                                                 cloud_b_device.ptr(), cloud_a_device.ptr(), mins);

    double sum_a = thrust::reduce(thrust::device, mins, mins + cloud_a.size(), 0.0);

    queries_device.upload(cloud_b.points);
    octree_device->setCloud(cloud_a_device);
    octree_device->build();

    octree_device->radiusSearch(queries_device, rad, max_res, b_ind);
    blks = (cloud_b.size() + NUM_THREADS - 1) / NUM_THREADS;
    radius_compute_kernel<<<blks, NUM_THREADS>>>(cloud_b.size(), max_res, b_ind.data.ptr(), b_ind.sizes.ptr(),
                                                 cloud_a_device.ptr(), cloud_b_device.ptr(), mins);
                                                 
    double sum_b = thrust::reduce(thrust::device, mins, mins + cloud_b.size(), 0.0);

    return (1.0 / cloud_a.size()) * sum_a + (1.0 / cloud_b.size()) * sum_b;
}

}
