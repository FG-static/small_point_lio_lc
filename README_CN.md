# Small Point-LIO LC

[English](README.md) | **中文**

Small Point-LIO LC 是一个完整的激光-惯性 SLAM 系统，基于
[Small Point-LIO](https://github.com/Yancey2023/small_point_lio)（Point-LIO
的高性能实现，速度提升 2-3 倍），扩展了回环检测、位姿图优化、全局地图重建，
以及闭环局部跟踪地图反馈机制，可在线修正前端漂移。

系统以 ROS 2 工作空间形式组织，包含三个 package：

| Package | 角色 |
|---|---|
| `small_point_lio` | 前端：激光-惯性里程计（ESKF + iVox） |
| `small_point_lio_pgo` | 后端：回环检测、PGO、全局地图、局部地图反馈 |
| `small_point_lio_interfaces` | 前后端之间的版本化自定义消息 |

## 系统架构

```
LiDAR + IMU
    |
    v
SmallPointLioNode（前端）
    |-- /Odometry, /cloud_registered
    |-- /pointlio_scan_to_map_correction（packet 级修正证据）
    |-- 订阅 /local_tracking_map（后端反馈）
    |
    v
KeyframeBridge --> /keyframe_candidates
    |
    v
LoopDetectorNode
    |-- LidarIris + CartContext 场景识别
    |-- GICP 几何验证
    |-- g2o 位姿图优化
    |-- map -> odom TF（20 Hz）
    |-- /optimized_keyframes（PGO 快照）
    |
    +---> MapNode --> /global_map, /save_map
    |
    v
LocalMapFeedbackNode
    |-- 监控修正质量（证据窗口）
    |-- 局部滑窗位姿图优化
    |-- 构建修正后的跟踪点云
    |-- /local_tracking_map --> 前端热替换 iVox
```

### 前端（small_point_lio）

前端维护三个并行的 ESKF 实例：

- **estimator.kf** — 主滤波器，接收 IMU 预测和针对 iVox 跟踪地图的点到平面更新。
- **prediction_only_kf** — 当前地图 epoch 以来仅 IMU 的影子滤波器；不接收点更新，
  用于计算累计 scan-to-map 修正量。
- **packet_prediction_kf** — 上一个 packet 边界以来仅 IMU 的影子滤波器；
  在每个 packet 边界重置为主滤波器。

在每个 LiDAR packet 边界，前端发布 `ScanToMapCorrection` 消息，包含修正后位姿、
两个 IMU-only 基线位姿、更新统计（尝试/接受数量、残差 RMS）以及当前跟踪地图版本。

当后端发送 `LocalTrackingMap` 时，后台线程重建新的 iVox。替换仅在安全的 packet
边界处进行，需验证版本连续性、lag 界限，并重放 cutoff 之后到达的 tail-journal 点。

### 后端（small_point_lio_pgo）

**KeyframeBridge** 同步 `/Odometry` 和 `/cloud_registered`，在约 100 ms 窗口内
累积点云，发布 body 坐标系去畸变点云的关键帧候选。

**LoopDetectorNode** 使用 LidarIris（log-polar 图像描述子）和 CartContext（笛卡尔
上下文，二级验证）进行场景识别，通过有界 GICP（small_gicp）验证候选，并使用
g2o（Levenberg-Marquardt）优化位姿图。回环边默认使用重力保持测量模式（约束
x/y/yaw，让里程计保持 z/roll/pitch）。节点发布 `map -> odom` TF 和完整的
`OptimizedKeyFrames` 快照。

**MapNode** 存储所有关键帧点云，在每次 PGO 快照后重建全局地图（使用优化位姿
重投影并体素降采样），并提供 `/save_map` 服务导出 PCD。

**LocalMapFeedbackNode** 闭合前端与后端之间的回路。它监控 packet 级修正证据，
在以下情况触发局部重建：

- *normal*：证据窗口内累计修正超过平移/旋转阈值；
- *instant*：单个高质量 packet 出现大幅修正；
- *pgo*：新的 PGO 快照导致附近关键帧形变超过阈值。

节点优化最近关键帧的滑窗（约 20 个活跃帧，20 m 内最多 80 个冻结历史帧），
构建体素降采样的跟踪点云，并发布为 `LocalTrackingMap`。优化位姿仅在前端
确认地图版本切换后才正式提交。

## 依赖

- ROS 2 Jazzy（或 Humble）
- Eigen3
- PCL
- OpenCV
- g2o（`libg2o-dev`）
- small_gicp（安装到 `/usr/local`）
- livox_ros_driver2（Livox 雷达驱动）

## 构建

```bash
source /opt/ros/jazzy/setup.sh
cd <workspace-root>   # 包含本 README 的目录
colcon build --packages-select small_point_lio_interfaces small_point_lio small_point_lio_pgo
source install/setup.bash
```

## 运行

三种模式共用同一条 rosbag remap 命令，将 Livox 话题名映射到配置文件中的
`/livox/lidar`、`/livox/imu`：

```bash
ros2 bag play <bag_path> --clock \
  --remap \
    /livox/lidar_192_168_1_185:=/livox/lidar \
    /livox/imu_192_168_1_185:=/livox/imu
```

### 模式一：仅前端

里程计 + 局部地图，无回环检测和 PGO。由于没有后端提供全局地图，需开启
前端自身的 PCD 累积器。

```bash
# 终端 1：前端（save_pcd:=true 开启 PCD 累积器）
ros2 launch small_point_lio frontend_bringup.launch.py save_pcd:=true

# 终端 2：rosbag 回放（带 remap，见上）
```

回放结束后保存累积的 PCD：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### 模式二：前端 + 后端（无反馈闭环）

回环检测 + PGO + 全局地图 + map→odom TF，但后端**不**将修正地图反馈给
前端。前端继续在自己的 iVox 上运行；后端独立优化位姿并发布修正后的
全局地图。

```bash
# 终端 1：前端
ros2 launch small_point_lio frontend_bringup.launch.py

# 终端 2：后端（无反馈节点）
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py

# 终端 3：rosbag 回放（带 remap，见上）
```

### 模式三：前端 + 后端（完整 SLAM，含反馈闭环）

在前者基础上增加闭环局部跟踪地图反馈：后端用 PGO 优化后的位姿重建 iVox
并发送给前端，前端在线热替换跟踪地图。

```bash
# 终端 1：前端（启用 packet 级修正证据 + 地图反馈通路）
ros2 launch small_point_lio frontend_bringup.launch.py \
  enable_local_map_feedback:=true

# 终端 2：后端 PGO（回环检测 + 位姿图 + 局部地图反馈节点）
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py \
  enable_local_map_feedback:=true

# 终端 3：rosbag 回放（带 remap，见上）
```

`frontend_bringup.launch.py` 支持 `use_sim_time`（默认 `true`）、`rviz`
（默认 `true`）、`save_pcd`（默认 `false`）、`enable_local_map_feedback`
（默认 `false`）等参数。默认使用 `small_point_lio/config/mid360.yaml`。
其他雷达请参照现有示例创建配置文件，通过 `ros2 run` 指定：

```bash
ros2 run small_point_lio small_point_lio_node \
  --ros-args --params-file src/small_point_lio/config/unilidar_l2.yaml
```

### 各模式启动的节点

| 节点 | 模式一 | 模式二 | 模式三 |
|---|---|---|---|
| `small_point_lio_node` | ✓ | ✓ | ✓ |
| `keyframe_bridge_node` | — | ✓ | ✓ |
| `loop_detector_node` | — | ✓ | ✓ |
| `map_node` | — | ✓ | ✓ |
| `local_map_feedback_node` | — | — | ✓ |

## 配置

### 前端（`small_point_lio/config/*.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `lidar_topic` / `imu_topic` | `/livox/lidar`, `/livox/imu` | 输入话题 |
| `lidar_type` | `livox_custom_msg` | 适配器：`livox_custom_msg`、`livox_pointcloud2`、`custom_mid360_driver`、`unilidar` |
| `point_filter_num` | `1` | 每 N 个点取 1 个 |
| `min_distance` / `max_distance` | `0.5` / `1000.0` | 径向点云过滤（m） |
| `space_downsample` | `true` | 体素降采样输入点云 |
| `space_downsample_leaf_size` | `0.5` | 降采样体素大小（m） |
| `gravity` | `[0, 0, -9.810]` | 重力向量 |
| `fix_gravity_direction` | `true` | 用前 200 个 IMU 样本标定重力方向 |
| `map_resolution` | `0.5` | iVox 体素分辨率（m） |
| `extrinsic_T` / `extrinsic_R` | -- | 雷达到 IMU 的外参 |
| `extrinsic_est_en` | `false` | 在线估计外参 |
| `local_map_feedback_enable` | `false` | 启用后端反馈通路 |

完整参数列表（含卡尔曼滤波协方差）请参阅 YAML 文件。

### 后端回环检测（`PGO/config/backend.yaml`）

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `kf_trans_thresh` / `kf_rot_thresh` | `1.5` / `0.3` | 关键帧选取阈值（m / rad） |
| `loop_iris_distance_thresh` | `0.35` | LidarIris 距离门限 |
| `cart_distance_thresh` | `0.40` | CartContext 距离门限 |
| `loop_gicp_score_thresh` | `0.15` | 有界 GICP 分数门限 |
| `loop_gicp_min_inlier_ratio` | `0.85` | GICP 最小内点比例 |
| `pgo_loop_measurement_mode` | `gravity_preserving` | `gravity_preserving` 或 `full_se3` |
| `gravity_align_enable` | `true` | 将位姿对齐到 IMU 重力方向 |

### 局部地图反馈（`PGO/config/local_map_feedback.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `cumulative_trigger_translation_m` | `0.15` | 常规触发：累计漂移（m） |
| `cumulative_trigger_rotation_deg` | `2.0` | 常规触发：累计漂移（deg） |
| `instant_trigger_translation_m` | `0.25` | 瞬时触发：单 packet 跳变（m） |
| `active_window_keyframes` | `20` | 局部优化滑窗大小 |
| `history_search_radius_m` | `20.0` | 冻结历史帧搜索半径 |
| `map_voxel_leaf_size_m` | `0.20` | 跟踪点云体素大小 |
| `trigger_pgo_enable` | `true` | PGO 形变触发重建 |
| `trigger_instant_enable` | `false` | 单 packet 跳变触发重建 |
| `trigger_normal_enable` | `false` | 累计漂移触发重建 |

## 保存地图

### 前端局部地图

在前端配置中设置 `save_pcd: true`，运行节点后调用：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### 后端全局地图（PGO 修正后）

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

PCD 文件写入 `PGO/config/map.yaml` 中配置的路径（`map_save_path`）。

## 自定义消息

### `ScanToMapCorrection`

前端在每个 LiDAR packet 边界发布。包含修正后位姿、两个 IMU-only 预测位姿
（packet 级和 epoch 级）、更新统计以及当前跟踪地图版本。

### `LocalTrackingMap`

后端反馈节点发布。包含供前端重建 iVox 的版本化点云，以及来源 correction 序号、
PGO 图版本、锚点关键帧 ID 和触发原因。

## 项目结构

```
small_point_lio_lc/
├── small_point_lio/                  # 前端 package
│   ├── config/                       # 各雷达 YAML 配置
│   ├── launch/                       # Launch 文件
│   ├── src/
│   │   ├── small_point_lio/          # 核心算法
│   │   │   ├── eskf.h               # 误差状态卡尔曼滤波
│   │   │   ├── ieskf.h/.cpp         # 迭代 ESKF（帧更新）
│   │   │   ├── estimator.h/.cpp     # 观测模型
│   │   │   ├── small_ivox.h         # 增量体素地图
│   │   │   ├── preprocess.h/.cpp    # 点云预处理
│   │   │   ├── parameters.h/.cpp    # 参数加载
│   │   │   └── small_point_lio.h/.cpp  # 主算法 + 反馈
│   │   ├── lidar_adapter/           # 雷达类型适配器
│   │   ├── io/                      # PCD 读写
│   │   ├── util/                    # 建图工具
│   │   └── small_point_lio_node.*   # ROS 2 节点
│   └── 3rdparty/                    # 内嵌：Eigen, unordered_dense, small_gicp
├── PGO/                             # 后端 package
│   ├── config/                      # 后端 YAML 配置
│   ├── launch/                      # PGO launch 文件
│   ├── include/small_point_lio_pgo/ # 头文件
│   ├── src/                         # 节点实现
│   └── third_party/lidar_iris/      # LidarIris 描述子
└── small_point_lio_interfaces/      # 消息定义
    └── msg/
        ├── ScanToMapCorrection.msg
        └── LocalTrackingMap.msg
```

## 第三方依赖

| 项目 | 说明 | 许可证 |
|---|---|---|
| [Eigen](https://gitlab.com/libeigen/eigen) | 线性代数库 | MPL 2.0 |
| [unordered_dense](https://github.com/martinus/unordered_dense) | 高性能哈希表 | MIT |
| [small_gicp](https://github.com/koide3/small_gicp) | 点云配准 | MIT |
| [Open3D](https://github.com/isl-org/Open3D) | 3D 数据处理 | MIT |
| [LidarIris](https://github.com/iris-lio/LidarIris) | 场景识别描述子 | -- |
| [g2o](https://github.com/RainerKuemmerle/g2o) | 图优化 | BSD |

## 许可证

Small Point-LIO 前端：Copyright (C) 2025 Yingjie Huang，MIT 许可证。

PGO 后端及消息接口：Copyright (C) 2025 FGoose，MIT 许可证。
