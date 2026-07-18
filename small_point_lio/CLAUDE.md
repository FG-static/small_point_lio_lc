# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Small Point-LIO is a high-performance LiDAR-Inertial Odometry (LIO) system, a 2-3x faster re-implementation of [Point-LIO](https://github.com/hku-mars/Point-LIO). It performs tightly-coupled LiDAR-IMU fusion using a 30-dimensional Error-State Kalman Filter (ESKF) to estimate 6-DOF pose at high frequency.

This branch (`ros2`) is a ROS 2 ament_cmake package. There is also a standalone `main` branch without ROS.

## Build Commands

```bash
# Build (from workspace root)
source /opt/ros/jazzy/setup.sh  # or humble
colcon build --packages-select small_point_lio

# Run with default Mid-360 config
ros2 launch small_point_lio small_point_lio.launch.py

# Run with Unitree L2 config
ros2 run small_point_lio small_point_lio_node --ros-args --params-file config/unilidar_l2.yaml

# Save accumulated map to PCD (requires save_pcd: true in config)
ros2 service call /map_save std_srvs/srv/Trigger
```

## Testing

No unit tests. Lint-only via `ament_lint_auto`:
```bash
colcon test --packages-select small_point_lio
```
Lint excludes: `ament_cmake_copyright`, `ament_cmake_uncrustify`.

## Architecture

```
SmallPointLioNode (ROS 2 composable node)
  ├── LidarAdapterBase ── pluggable lidar driver (4 adapters, selected by lidar_type param)
  ├── SmallPointLio ── algorithm core
  │     ├── Preprocess ── point filtering, voxel downsampling, temporal sorting into deques
  │     ├── Estimator ── measurement models (point-to-plane h_point, IMU h_imu), owns SmallIVox
  │     └── ESKF ── 30-dim error-state Kalman filter (predict_state/predict_cov, update_point/update_imu)
  ├── PointcloudMapping ── optional voxel map accumulator for PCD saving
  └── Publishers: /Odometry, /cloud_registered, TF (odom→base_link)
```

**Data flow:** LiDAR adapter + IMU callback feed `handle_once()` which interleaves IMU predict and LiDAR point update steps by timestamp ordering. Each LiDAR point triggers ESKF predict-to-timestamp then point-to-plane update against the SmallIVox voxel map. Registered (non-downsampled) points are published and optionally accumulated for PCD map save.

## Key Source Layout

- `src/small_point_lio_node.{hpp,cpp}` — ROS 2 node: subscriptions, publishers, TF, services
- `src/small_point_lio/small_point_lio.{h,cpp}` — orchestrator, `handle_once()` main loop
- `src/small_point_lio/eskf.h` — ESKF implementation (predict/update, SO3 math)
- `src/small_point_lio/estimator.{h,cpp}` — measurement models, owns SmallIVox
- `src/small_point_lio/parameters.{h,cpp}` — ROS 2 parameter reading
- `src/small_point_lio/preprocess.{h,cpp}` — point cloud + IMU preprocessing
- `src/small_point_lio/small_ivox.h` — incremental voxel map with ankerl hash map and LRU eviction
- `src/lidar_adapter/` — pluggable lidar adapters (Livox CustomMsg, Livox PointCloud2, Mid-360, Unilidar)
- `src/util/voxelgrid_sampling.{h,cpp}` — bit-packed voxel grid downsampling (from small_gicp)
- `src/util/pointcloud_mapping.{h,cpp}` — voxel map accumulator
- `src/io/pcd_io.{h,cpp}` — PCD file I/O (ASCII, binary, binary_compressed; from Open3D)
- `3rdparty/` — vendored: ankerl::unordered_dense, liblzf
- `include/pch.h` — precompiled header (STL, Eigen, OpenMP, ankerl, liblzf, rclcpp)
- `include/param_deliver.h.in` — CMake template injecting ROOT_DIR for PCD save path

## Build System Details

- C++20, ament_cmake
- Compiler flags: `-march=native -ffast-math -fno-math-errno`
- Precompiled header: `include/pch.h`
- `livox_ros_driver2` is optional — conditionally compiled behind `#ifdef HAVE_LIVOX_DRIVER`
- Node registered as composable component: `rclcpp_components_register_node(small_point_lio::SmallPointLioNode)`
- OpenMP enabled for parallel voxel grid downsampling
- `configure_file()` generates `param_deliver.h` from `param_deliver.h.in` to inject source root path

## Adding a New LiDAR Adapter

1. Create a header in `src/lidar_adapter/` inheriting `LidarAdapterBase`
2. Implement `setup_subscription()` which subscribes to the lidar topic and converts messages to `std::vector<common::Point>`
3. Add a new `lidar_type` string case in `src/small_point_lio_node.cpp` to instantiate the adapter
4. No CMake changes needed — sources are globbed automatically

## Configuration

Two example configs in `config/`:
- `mid360.yaml` — Livox Mid-360 with `acc_norm: 1.0` (acceleration in g units)
- `unilidar_l2.yaml` — Unitree L2 with `acc_norm: 9.81` (acceleration in m/s^2)

Key config categories: point cloud filtering, IMU processing, map resolution, LiDAR-IMU extrinsic calibration, ESKF R/Q noise covariances, plane matching thresholds.
