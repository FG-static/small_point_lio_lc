# Small Point-LIO LC

**English** | [中文](README_CN.md)

Small Point-LIO LC is a complete LiDAR-Inertial SLAM system built on
[Small Point-LIO](https://github.com/Yancey2023/small_point_lio) (an advanced
Point-LIO implementation with 2-3x speed improvement), extended with loop
closure detection, pose-graph optimization, global map reconstruction, and a
closed-loop local tracking-map feedback mechanism that corrects frontend drift
online.

The system runs as a ROS 2 workspace with three packages:

| Package | Role |
|---|---|
| `small_point_lio` | Frontend: LiDAR-inertial odometry (ESKF + iVox) |
| `small_point_lio_pgo` | Backend: loop closure, PGO, global map, local map feedback |
| `small_point_lio_interfaces` | Versioned messages bridging frontend and backend |

## Architecture

```
LiDAR + IMU
    |
    v
SmallPointLioNode (frontend)
    |-- /Odometry, /cloud_registered
    |-- /pointlio_scan_to_map_correction  (packet-level evidence)
    |-- subscribes /local_tracking_map    (backend feedback)
    |
    v
KeyframeBridge --> /keyframe_candidates
    |
    v
LoopDetectorNode
    |-- LidarIris + CartContext place recognition
    |-- GICP geometric verification
    |-- g2o pose-graph optimization
    |-- map -> odom TF (20 Hz)
    |-- /optimized_keyframes (PGO snapshot)
    |
    +---> MapNode --> /global_map, /save_map
    |
    v
LocalMapFeedbackNode
    |-- monitors correction quality (evidence window)
    |-- local sliding-window pose-graph optimization
    |-- builds corrected tracking cloud
    |-- /local_tracking_map --> frontend hot-swaps iVox
```

### Frontend (small_point_lio)

The frontend maintains three parallel ESKF instances:

- **estimator.kf** -- main filter receiving IMU prediction and point-to-plane
  updates against the iVox tracking map.
- **prediction_only_kf** -- IMU-only shadow since the current map epoch; never
  receives point updates. Used to compute the cumulative scan-to-map correction.
- **packet_prediction_kf** -- IMU-only shadow since the last packet boundary;
  reset to the main filter at every packet boundary.

At each LiDAR packet boundary the frontend publishes a
`ScanToMapCorrection` message containing the corrected pose, both IMU-only
baselines, update statistics (attempted/accepted count, residual RMS), and the
active tracking-map version.

When the backend sends a `LocalTrackingMap`, a background thread rebuilds a
fresh iVox. The swap happens only at a safe packet boundary after verifying
version continuity, lag bounds, and replaying tail-journal points that arrived
after the map cutoff.

### Backend (small_point_lio_pgo)

**KeyframeBridge** synchronises `/Odometry` and `/cloud_registered`,
accumulates clouds over a ~100 ms window, and publishes keyframe candidates
with body-frame de-skewed clouds.

**LoopDetectorNode** performs place recognition using LidarIris (log-polar
image descriptor) and CartContext (Cartesian context, secondary verification),
verifies candidates with bounded GICP (small_gicp), and optimises the pose
graph with g2o (Levenberg-Marquardt). Loop edges use a gravity-preserving
measurement mode by default (constraining x/y/yaw while letting odometry
preserve z/roll/pitch). The node publishes a `map -> odom` TF and full
`OptimizedKeyFrames` snapshots.

**MapNode** stores all keyframe clouds, rebuilds the global map on each PGO
snapshot (re-projecting with optimised poses and voxel downsampling), and
provides a `/save_map` service to export PCD.

**LocalMapFeedbackNode** closes the loop between frontend and backend. It
monitors packet-level correction evidence and triggers a local rebuild when:

- *normal*: accumulated correction exceeds translation/rotation thresholds
  over an evidence window;
- *instant*: a single well-supported packet shows a large correction;
- *pgo*: a new PGO snapshot deforms nearby keyframes beyond a threshold.

The node optimises a sliding window of recent keyframes (~20 active, up to 80
frozen history within 20 m), builds a voxel-downsampled tracking cloud, and
publishes it as a `LocalTrackingMap`. Optimised poses are committed only after
the frontend confirms the map version switch.

## Dependencies

- ROS 2 Jazzy (or Humble)
- Eigen3
- PCL
- OpenCV
- g2o (`libg2o-dev`)
- small_gicp (installed to `/usr/local`)
- livox_ros_driver2 (for Livox LiDARs)

## Build

```bash
source /opt/ros/jazzy/setup.sh
cd <workspace-root>   # the directory containing this README
colcon build --packages-select small_point_lio_interfaces small_point_lio small_point_lio_pgo
source install/setup.bash
```

## Run

### Frontend only

```bash
ros2 launch small_point_lio frontend_bringup.launch.py
```

Uses `small_point_lio/config/mid360.yaml` by default; `use_sim_time` defaults
to `true` for rosbag playback. For other LiDARs, create a config based on the
existing examples and pass it via `ros2 run`:

```bash
ros2 run small_point_lio small_point_lio_node \
  --ros-args --params-file src/small_point_lio/config/unilidar_l2.yaml
```

### Frontend + backend (full SLAM with local map feedback)

Three terminals are needed: frontend, backend, and rosbag playback.

```bash
# Terminal 1: frontend (enables packet correction evidence + map feedback path)
ros2 launch small_point_lio frontend_bringup.launch.py \
  enable_local_map_feedback:=true

# Terminal 2: backend PGO (loop closure + pose graph + local map feedback node)
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py \
  enable_local_map_feedback:=true

# Terminal 3: rosbag playback
ros2 bag play <bag_path> --clock
```

`frontend_bringup.launch.py` accepts `use_sim_time` (default `true`), `rviz`
(default `true`), `save_pcd` (default `false`), and
`enable_local_map_feedback` (default `false`).

### Backend only (loop closure + global map, no frontend feedback)

```bash
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py
```

This runs the keyframe bridge, loop detector, and map node without the local
map feedback node.

## Configuration

### Frontend (`small_point_lio/config/*.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `lidar_topic` / `imu_topic` | `/livox/lidar`, `/livox/imu` | Input topics |
| `lidar_type` | `livox_custom_msg` | Adapter: `livox_custom_msg`, `livox_pointcloud2`, `custom_mid360_driver`, `unilidar` |
| `point_filter_num` | `1` | Keep 1 in N points |
| `min_distance` / `max_distance` | `0.5` / `1000.0` | Radial point filter (m) |
| `space_downsample` | `true` | Voxel downsample input cloud |
| `space_downsample_leaf_size` | `0.5` | Downsample voxel size (m) |
| `gravity` | `[0, 0, -9.810]` | Gravity vector |
| `fix_gravity_direction` | `true` | Calibrate gravity from first 200 IMU samples |
| `map_resolution` | `0.5` | iVox voxel resolution (m) |
| `extrinsic_T` / `extrinsic_R` | -- | LiDAR-to-IMU extrinsic |
| `extrinsic_est_en` | `false` | Online extrinsic estimation |
| `local_map_feedback_enable` | `false` | Enable backend feedback path |

See the YAML files for the full list including Kalman filter covariances.

### Backend loop detector (`PGO/config/backend.yaml`)

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `kf_trans_thresh` / `kf_rot_thresh` | `1.5` / `0.3` | Keyframe selection thresholds (m / rad) |
| `loop_iris_distance_thresh` | `0.35` | LidarIris distance gate |
| `cart_distance_thresh` | `0.40` | CartContext distance gate |
| `loop_gicp_score_thresh` | `0.15` | Bounded GICP score gate |
| `loop_gicp_min_inlier_ratio` | `0.85` | Minimum GICP inlier ratio |
| `pgo_loop_measurement_mode` | `gravity_preserving` | `gravity_preserving` or `full_se3` |
| `gravity_align_enable` | `true` | Align poses to IMU gravity |

### Local map feedback (`PGO/config/local_map_feedback.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `cumulative_trigger_translation_m` | `0.15` | Normal trigger: accumulated drift (m) |
| `cumulative_trigger_rotation_deg` | `2.0` | Normal trigger: accumulated drift (deg) |
| `instant_trigger_translation_m` | `0.25` | Instant trigger: single-packet jump (m) |
| `active_window_keyframes` | `20` | Sliding window size for local optimisation |
| `history_search_radius_m` | `20.0` | Frozen history search radius |
| `map_voxel_leaf_size_m` | `0.20` | Tracking cloud voxel size |
| `trigger_pgo_enable` | `true` | Rebuild on PGO deformation |
| `trigger_instant_enable` | `false` | Rebuild on single-packet jump |
| `trigger_normal_enable` | `false` | Rebuild on accumulated drift |

## Save Map

### Frontend local map

Set `save_pcd: true` in the frontend config, run the node, then:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### Backend global map (PGO-corrected)

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

The PCD is written to the path configured in `PGO/config/map.yaml`
(`map_save_path`).

## Custom Interfaces

### `ScanToMapCorrection`

Published by the frontend at every LiDAR packet boundary. Contains the
corrected pose, two IMU-only predicted poses (packet-level and epoch-level),
update statistics, and the active tracking-map version.

### `LocalTrackingMap`

Published by the backend feedback node. Contains a versioned point cloud for
the frontend to rebuild its iVox, along with the source correction sequence,
PGO graph version, anchor keyframe ID, and trigger reason.

## Project Structure

```
small_point_lio_lc/
├── small_point_lio/                  # Frontend package
│   ├── config/                       # Per-LiDAR YAML configs
│   ├── launch/                       # Launch files
│   ├── src/
│   │   ├── small_point_lio/          # Core algorithm
│   │   │   ├── eskf.h               # Error-state Kalman filter
│   │   │   ├── ieskf.h/.cpp         # Iterated ESKF (frame update)
│   │   │   ├── estimator.h/.cpp     # Measurement models
│   │   │   ├── small_ivox.h         # Incremental voxel map
│   │   │   ├── preprocess.h/.cpp    # Point cloud preprocessing
│   │   │   ├── parameters.h/.cpp    # Parameter loading
│   │   │   └── small_point_lio.h/.cpp  # Main algorithm + feedback
│   │   ├── lidar_adapter/           # LiDAR type adapters
│   │   ├── io/                      # PCD I/O
│   │   ├── util/                    # Mapping utilities
│   │   └── small_point_lio_node.*   # ROS 2 node
│   └── 3rdparty/                    # Vendored: Eigen, unordered_dense, small_gicp
├── PGO/                             # Backend package
│   ├── config/                      # Backend YAML configs
│   ├── launch/                      # PGO launch file
│   ├── include/small_point_lio_pgo/ # Headers
│   ├── src/                         # Node implementations
│   └── third_party/lidar_iris/      # LidarIris descriptor
└── small_point_lio_interfaces/      # Message definitions
    └── msg/
        ├── ScanToMapCorrection.msg
        └── LocalTrackingMap.msg
```

## Third-party

| Project | Description | License |
|---|---|---|
| [Eigen](https://gitlab.com/libeigen/eigen) | Linear algebra | MPL 2.0 |
| [unordered_dense](https://github.com/martinus/unordered_dense) | Fast hashmap | MIT |
| [small_gicp](https://github.com/koide3/small_gicp) | Point cloud registration | MIT |
| [Open3D](https://github.com/isl-org/Open3D) | 3D data processing | MIT |
| [LidarIris](https://github.com/iris-lio/LidarIris) | Place recognition descriptor | -- |
| [g2o](https://github.com/RainerKuemmerle/g2o) | Graph optimization | BSD |

## License

Small Point-LIO frontend: Copyright (C) 2025 Yingjie Huang, MIT License.

PGO backend and interfaces: Copyright (C) 2025 FGoose, MIT License.
