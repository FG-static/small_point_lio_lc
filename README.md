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

All three modes use the same rosbag remap to match the Livox topic names in
the config (`/livox/lidar`, `/livox/imu`):

```bash
ros2 bag play <bag_path> --clock \
  --remap \
    /livox/lidar_192_168_1_185:=/livox/lidar \
    /livox/imu_192_168_1_185:=/livox/imu
```

### Mode 1: Frontend only

Odometry + local map, no loop closure or PGO.  Enable the frontend's own PCD
accumulator since there is no backend to provide a global map.

```bash
# Terminal 1: frontend (save_pcd:=true enables the PCD accumulator)
ros2 launch small_point_lio frontend_bringup.launch.py save_pcd:=true

# Terminal 2: rosbag playback (with remap, see above)
```

After playback, save the accumulated PCD:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### Mode 2: Frontend + backend (no local map feedback)

Loop closure + PGO + global map + map→odom TF, but the backend does **not**
feed corrected maps back to the frontend.  The frontend keeps running on its
own iVox; the backend independently optimizes poses and publishes the
corrected global map.

```bash
# Terminal 1: frontend
ros2 launch small_point_lio frontend_bringup.launch.py

# Terminal 2: backend (no feedback node)
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py

# Terminal 3: rosbag playback (with remap, see above)
```

### Mode 3: Frontend + backend (full SLAM with local map feedback)

Adds the closed-loop local tracking-map feedback: the backend rebuilds a
corrected iVox from PGO-optimized poses and sends it to the frontend, which
hot-swaps its tracking map online.

```bash
# Terminal 1: frontend (enables packet correction evidence + map feedback path)
ros2 launch small_point_lio frontend_bringup.launch.py \
  enable_local_map_feedback:=true

# Terminal 2: backend PGO (loop closure + pose graph + local map feedback node)
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py \
  enable_local_map_feedback:=true

# Terminal 3: rosbag playback (with remap, see above)
```

### Coordinate contract and tilted LiDAR mounting

The body-to-LiDAR mounting calibration lives only in the shared
`small_point_lio/config/body_lidar.yaml` file. The frontend, keyframe bridge,
loop detector, and map node all read this file. The frontend node is also the
only static-TF publisher; no separate `static_transform_publisher` is started.
The file uses the standard ROS 2 parameter format:

```yaml
/**:
  ros__parameters:
    body_frame: "base_link"
    lidar_frame: "livox_frame"
    t_body_lidar: [-0.011, -0.02329, 0.04412]
    rpy_body_lidar: [-0.261, -0.113, 0.0]
```

`t_body_lidar` is the LiDAR origin expressed in the body frame, in metres.
`rpy_body_lidar` is the LiDAR roll, pitch, and yaw relative to the body, in
radians. These values are only an example mounting calibration; measure them
for the actual robot. Gravity can help estimate roll and pitch, but not yaw.

The frontend publishes `body_frame -> lidar_frame` from this file and uses it
consistently for `/Odometry`, `/cloud_registered`, and local-map feedback. The
backend trusts this coordinate contract and must not apply the same mounting
tilt again. Keep these four parameter groups separate:

- `extrinsic_T` / `extrinsic_R` in the frontend sensor YAML are the calibrated
  LiDAR-to-IMU transform used inside the estimator. Do not change
  `extrinsic_R` to compensate for the whole LiDAR/IMU assembly being mounted
  at an angle.
- `body_lidar.yaml` is the physical body-to-LiDAR mounting transform and also
  determines the frontend output basis.
- `base_to_ground_height` is a terrain-seed height, not a TF.
- `map -> odom` is a global PGO correction, not a sensor extrinsic.

The default launches automatically use the installed shared file, so the six
scalar launch arguments are no longer required:

```bash
# Terminal 1: frontend uses the default sensor and mounting configs
ros2 launch small_point_lio frontend_bringup.launch.py

# Terminal 2: backend uses the same default mounting config
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py
```

For a hypothetical recording that needs different sensor parameters and a
different mounting calibration, copy and edit the corresponding YAML files,
then pass the same mounting file to both launches:

```bash
ros2 launch small_point_lio frontend_bringup.launch.py \
  params_file:=/path/to/sensor_params.yaml \
  body_lidar_config:=/path/to/body_lidar.yaml

ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py \
  body_lidar_config:=/path/to/body_lidar.yaml
```

Backend gravity-alignment modes behave as follows:

| Configuration | Runtime behaviour | Intended use |
|---|---|---|
| `enable=true`, `source=manual_rpy`, zero manual RPY | Ready immediately; keeps descriptor world-orientation processing without adding another rotation | Recommended when the frontend already uses `body_lidar.yaml` |
| `enable=true`, `source=manual_rpy`, non-zero manual RPY | Applies a fixed world rotation to backend descriptors, PGO poses, and maps | Backend-only legacy correction; does not fix frontend clouds or terrain maps |
| `enable=true`, `source=imu_average` | Averages IMU acceleration for the configured duration; rejects keyframes until ready; estimates roll/pitch but not yaw | Legacy input whose frontend basis is not aligned; the robot must remain still while sampling |
| `enable=false` | Does not wait or rotate, and bypasses the current descriptor world-orientation path | Debugging only; not equivalent to zero manual RPY and not recommended normally |

The recommended `PGO/config/backend.yaml` settings are therefore
`gravity_align_enable: true`, `gravity_align_source: manual_rpy`, and
`gravity_align_manual_rpy_deg: [0, 0, 0]`. `imu_average` affects only the
backend; it does not correct `/cloud_registered` or the terrain mapper input,
and cannot repair an incorrect `body_lidar.yaml`.

`frontend_bringup.launch.py` accepts `use_sim_time` (default `true`), `rviz`
(default `true`), `save_pcd` (default `false`), and
`enable_local_map_feedback` (default `false`), plus `params_file` and
`body_lidar_config`. Their defaults are `small_point_lio/config/mid360.yaml`
and `small_point_lio/config/body_lidar.yaml`. When bypassing launch, pass both
files and put the shared mounting file last so it overrides any duplicate frame
parameters in a sensor YAML:

```bash
ros2 run small_point_lio small_point_lio_node \
  --ros-args \
  --params-file /path/to/sensor_params.yaml \
  --params-file /path/to/body_lidar.yaml
```

### What each mode starts

| Node | Mode 1 | Mode 2 | Mode 3 |
|---|---|---|---|
| `small_point_lio_node` | ✓ | ✓ | ✓ |
| `keyframe_bridge_node` | — | ✓ | ✓ |
| `loop_detector_node` | — | ✓ | ✓ |
| `map_node` | — | ✓ | ✓ |
| `local_map_feedback_node` | — | — | ✓ |

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

### Shared body-LiDAR extrinsic (`small_point_lio/config/body_lidar.yaml`)

| Parameter | Description |
|---|---|
| `body_frame` / `lidar_frame` | Parent and child frames of the static TF |
| `t_body_lidar` | LiDAR origin `[x, y, z]` expressed in the body frame, in metres |
| `rpy_body_lidar` | LiDAR `[roll, pitch, yaw]` relative to the body, in radians |

The frontend, keyframe bridge, loop detector, and map node all read this file.
It is loaded after the sensor parameters, so its frame values are authoritative.
Do not duplicate these parameters in `backend.yaml`, `bridge.yaml`, or
`map.yaml`.

### Backend loop detector (`PGO/config/backend.yaml`)

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `kf_trans_thresh` / `kf_rot_thresh` | `1.5` / `0.3` | Keyframe selection thresholds (m / rad) |
| `loop_iris_distance_thresh` | `0.40` | LidarIris distance gate |
| `cart_distance_thresh` | `0.40` | CartContext distance gate |
| `loop_gicp_score_thresh` | `0.21` | Bounded GICP score gate |
| `loop_gicp_min_inlier_ratio` | `0.50` | Minimum GICP inlier ratio |
| `loop_gicp_submap_keyframes` | `5` | GICP submap neighbour frame count |
| `loop_gicp_submap_leaf_size` | `0.25` | GICP submap voxel size (m) |
| `loop_gicp_max_iterations` | `20` | GICP max iterations per trial |
| `pgo_loop_measurement_mode` | `gravity_preserving_xyz` | `gravity_preserving` (x/y/yaw), `gravity_preserving_xyz` (x/y/z/yaw), or `full_se3` (all 6 DoF) |
| `gravity_align_enable` | `true` | Keep descriptor/world orientation processing enabled |
| `gravity_align_source` | `manual_rpy` | Identity manual alignment by default; prevents duplicate frontend tilt correction |
| `loop_history_max_current_matches` | `2` | Max loop edges per history keyframe before disabling its LCD retrieval (0 = disable) |
| `loop_sequence_max_history_lag_m` | `3.0` | Max bilateral deviation between current and history travel progress (0 = disable) |
| `loop_sequence_max_anchor_progress_m` | `15.0` | Max pending/anchor travel distance for sequence validity (0 = disable) |
| `loop_descriptor_verify_top_k` | `4` | Top-k candidates sent to GICP after descriptor ranking |

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

### Map node (`PGO/config/map.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `map_leaf_size` | `0.20` | Global map voxel size (m) |
| `keyframe_rebuild_interval` | `20` | Rebuild global map every N keyframes; PGO snapshots always trigger immediate rebuild |
| `save_map_service_name` | `/map_save_lc` | ROS service name for saving PCD |
| `map_save_path` | `.../global_map.pcd` | PCD output path |

## Save Map

### Frontend local map

Set `save_pcd: true` in the frontend config (or pass `save_pcd:=true` to
`frontend_bringup.launch.py`), run the node, then:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### Backend global map (PGO-corrected)

```bash
ros2 service call /map_save_lc std_srvs/srv/Trigger
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
