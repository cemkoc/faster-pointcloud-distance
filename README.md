# Faster Point Cloud Distance

This project contains several parallel and serial implementations of Chamfer distance for comparing large point clouds without making any sparsity assumptions. It is intended to handle processing of sparse and dense point clouds at scale.

### Python Instructions (for transforming data)

1. Get virtualenvwrapper
   https://virtualenvwrapper.readthedocs.io/en/latest/install.html#basic-installation

2. Create a virtualenv, go into it and install packages

   a. Create a virtualenv based on a specific python version downloaded from
   [official sources](https://www.python.org/downloads/) and assuming located in `/usr/local/bin/` (which is where it is downloaded by default on MacOS)

   ```shell
   mkvirtualenv -p /usr/local/bin/python3.9 pointcloudenv
   workon pointcloudenv

   (pointcloudenv)
   pip install -r requirements.txt
   ```

### Preparing the data

LiDAR data from iPhones can have a lot of variability especially if the format is LAS with GPS information. This is mostly due to the fact that consumer GPS accuracy is only accurate to +- 10 meters. So even if we affix the iPhone on a tripod, two different LiDAR scans can stil have a different origins which makes it harder to get an accurate distance measurement between two point clouds that do not share the same coordinate frame. One attempt to correct this is after standardizing coordinates (by transforming both scans to ECEF coordinates) we can compute the L2 norm between two bounding box centers and then can translate one of the point clouds by that distance to "center" the data by approximately matching the origins.

Transform the LiDAR LAS scan file to ECEF and write it as PLY file to consume easily through PCL C++ library.

```
python3 scripts/transform_data.py -i /Users/cemkoc/data/point_clouds/room_frameB_22_49_59.las
```

this will write a `.ply` file in the same input directory with the same name after converting to ECEF coordinate system.

### Running Serial Chamfer Distance

Steps:

0. Read two ply files for two point clouds.
1. Get bounding box center and shift it to center two frames.
2. Compute pairwise Chamfer distance using nearest neighbors
3. Output

### Running Parallel Chamfer Distance



### Building on Cori NERSC

### Building on Cori NERSC using CUDA
1. Install CUDA using Spack
```
spack install cuda@9.0.176 %gcc@8.3.0
```
and now build PCL with CUDA support (assuming you are in project root dir)
```
cd pcl
mkdir build-release-cuda && cd build-release-cuda
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_CUDA=ON -DCUDA_TOOLKIT_ROOT_DIR=${HOME}/spack/opt/spack/cray-cnl7-haswell/gcc-8.3.0/cuda-9.0.176 -DBUILD_GPU=ON ..
```