# LiDAR-Iris Vendor Code

This directory vendors the core LiDAR-Iris implementation from:

https://github.com/BigMoWangying/LiDAR-Iris

Upstream paper:

Ying Wang, Zezhou Sun, Cheng-Zhong Xu, Sanjay Sarma, Jian Yang, Hui Kong,
"LiDAR Iris for Loop-Closure Detection", IROS 2020.

The upstream code is distributed under the MIT License. See `LICENSE`.

## Core Files

- `LidarIris.h` / `LidarIris.cpp`: point cloud to LiDAR-Iris image, Log-Gabor feature encoding, descriptor comparison.
- `fftm/fftm.hpp` / `fftm/fftm.cpp`: FFT/log-polar phase correlation utilities used to estimate descriptor image shift.

## Important Entrypoints

- `LidarIris::GetIris(const pcl::PointCloud<pcl::PointXYZ>&)`: converts a point cloud into an 80 x 360 iris image.
- `LidarIris::GetFeature(const cv::Mat1b&)`: creates the binary descriptor and mask.
- `LidarIris::Compare(...)`: estimates alignment shift and returns descriptor distance.

This module is intentionally kept separate from `small_point_lio_pgo` source code so it can later be reused by a generic loop-closure backend.
