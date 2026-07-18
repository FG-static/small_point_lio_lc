# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 `ament_cmake` C++20 package for Small Point-LIO. Core algorithm code lives in `src/small_point_lio/`, with ROS node integration in `src/small_point_lio_node.{hpp,cpp}`. LiDAR adapters are under `src/lidar_adapter/`, utilities under `src/util/`, PCD I/O under `src/io/`, and vendored code under `3rdparty/`. Runtime configuration is in `config/`, launch files are in `launch/`, and image assets are in `img/`. Generated headers come from `include/param_deliver.h.in`; do not edit generated build-tree copies.

## Build, Test, and Development Commands

Run commands from the ROS 2 workspace root after sourcing ROS:

```bash
source /opt/ros/jazzy/setup.sh      # or the target distro, such as humble
colcon build --packages-select small_point_lio
colcon test --packages-select small_point_lio
ros2 launch small_point_lio small_point_lio.launch.py
ros2 run small_point_lio small_point_lio_node --ros-args --params-file config/unilidar_l2.yaml
```

`colcon build` compiles the package and component node. `colcon test` runs `ament_lint_auto`. The launch command uses the default Mid-360 setup; the `ros2 run` example uses the Unitree L2 config.

## Coding Style & Naming Conventions

Use C++20 and the existing style: 4-space indentation, same-line braces, `snake_case` for functions, variables, parameters, topics, and YAML keys, and PascalCase for classes such as `SmallPointLioNode`. Prefer existing Eigen, ROS 2, and local helper types over new dependencies. CMake enables `-Wall -Wextra -Wno-unused-parameter`; keep new code warning-clean.

## Testing Guidelines

There are currently no unit tests. For behavior changes, add focused ROS 2 or algorithm-level tests when feasible, then run `colcon test --packages-select small_point_lio`. At minimum, build successfully and exercise the relevant launch/config path with representative LiDAR and IMU topics.

## Commit & Pull Request Guidelines

Recent commits use short imperative summaries such as `fix bug`, `clean code`, and `add more lidar adapter`. Keep messages concise but specific, for example `fix point cloud frame transform`. Pull requests should describe the behavior change, list tested commands, mention affected configs or LiDAR adapters, and link related issues.

## Configuration & Runtime Notes

Tune parameters in `config/*.yaml`; keep sensor-specific defaults separated by file. To save a map, set `save_pcd: true`, run the node, then call:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

Reset `save_pcd` afterward to avoid unnecessary memory use.
