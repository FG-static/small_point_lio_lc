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

### 坐标契约与倾斜安装雷达

车体与雷达的安装外参只保存在共享文件
`small_point_lio/config/body_lidar.yaml`。前端、关键帧桥接、回环节点和地图
节点都读取这一个文件；前端节点同时是唯一的静态 TF 发布者，不再启动独立的
`static_transform_publisher`。共享文件使用标准 ROS 2 参数格式：

```yaml
/**:
  ros__parameters:
    body_frame: "base_link"
    lidar_frame: "livox_frame"
    t_body_lidar: [-0.011, -0.02329, 0.04412]
    rpy_body_lidar: [-0.261, -0.113, 0.0]
```

`t_body_lidar` 表示雷达原点在车体坐标系中的位置，单位为米；
`rpy_body_lidar` 表示雷达相对车体的 roll、pitch、yaw，单位为弧度。以上数值
只是一个安装标定示例，实际车辆必须填写自己的测量结果。重力只能辅助标定
roll/pitch，不能确定 yaw。

前端使用该文件发布 `body_frame -> lidar_frame`，并统一转换 `/Odometry`、
`/cloud_registered` 和局部地图反馈。后端信任这个坐标契约，不能再重复施加
同一安装倾角。需要严格区分以下四类参数：

- 前端传感器 YAML 中的 `extrinsic_T` / `extrinsic_R` 是估计器内部使用的
  雷达到 IMU 标定外参。不能为了补偿整套雷达/IMU 的倾斜安装而修改
  `extrinsic_R`。
- `body_lidar.yaml` 是车体到雷达的物理安装变换，同时决定前端输出坐标基准。
- `base_to_ground_height` 是地形节点用于寻找脚下地面的几何高度，不是 TF。
- `map -> odom` 是 PGO 的全局轨迹修正，不是传感器安装外参。

默认启动会自动使用安装后的共享文件，不再填写六个 launch 参数：

```bash
# 终端 1：前端读取默认传感器配置和默认共享安装外参
ros2 launch small_point_lio frontend_bringup.launch.py

# 终端 2：后端读取同一个默认共享安装外参
ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py
```

假设某次数据采集需要不同的传感器参数和安装标定，先复制并修改对应 YAML，
然后让前后端显式使用同一路径：

```bash
ros2 launch small_point_lio frontend_bringup.launch.py \
  params_file:=/path/to/sensor_params.yaml \
  body_lidar_config:=/path/to/body_lidar.yaml

ros2 launch small_point_lio_pgo small_point_lio_pgo.launch.py \
  body_lidar_config:=/path/to/body_lidar.yaml
```

后端重力对齐模式的实际行为如下：

| 配置 | 运行方式 | 适用场景 |
|---|---|---|
| `enable=true`, `source=manual_rpy`, 手动 RPY 为零 | 立即就绪；仍按车体世界姿态生成描述子，但不追加旋转 | 推荐模式：前端已用 `body_lidar.yaml` 输出正确基准 |
| `enable=true`, `source=manual_rpy`, 手动 RPY 非零 | 立即对后端描述子、PGO 位姿和地图施加固定世界旋转 | 仅修正后端的兼容模式；不会修正前端点云或地形图 |
| `enable=true`, `source=imu_average` | 启动后平均指定时长的 IMU 加速度；完成前拒绝关键帧，只估计 roll/pitch，不估计 yaw | 兼容前端未对齐的旧数据流；采样期间机器人必须静止 |
| `enable=false` | 不等待、不做世界旋转，并跳过当前描述子的世界方向处理分支 | 调试用途；不等价于零 RPY，正常运行不推荐 |

因此当前推荐保持 `PGO/config/backend.yaml` 中的
`gravity_align_enable: true`、`gravity_align_source: manual_rpy` 和
`gravity_align_manual_rpy_deg: [0, 0, 0]`。`imu_average` 只影响后端，不会修正
`/cloud_registered` 或地形节点输入；不能用它补救错误的 `body_lidar.yaml`。

`frontend_bringup.launch.py` 支持 `use_sim_time`（默认 `true`）、`rviz`
（默认 `true`）、`save_pcd`（默认 `false`）、`enable_local_map_feedback`
（默认 `false`）、`params_file` 和 `body_lidar_config`。默认分别使用
`small_point_lio/config/mid360.yaml` 与 `small_point_lio/config/body_lidar.yaml`。
如果绕过 launch 直接运行节点，两个参数文件都必须传入，且共享安装文件放在
后面以覆盖传感器 YAML 中可能存在的同名 frame 参数：

```bash
ros2 run small_point_lio small_point_lio_node \
  --ros-args \
  --params-file /path/to/sensor_params.yaml \
  --params-file /path/to/body_lidar.yaml
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

### 共享车体-雷达外参（`small_point_lio/config/body_lidar.yaml`）

| 参数 | 说明 |
|---|---|
| `body_frame` / `lidar_frame` | 静态 TF 的父、子坐标系 |
| `t_body_lidar` | 雷达原点在车体坐标系中的 `[x, y, z]`，单位为米 |
| `rpy_body_lidar` | 雷达相对车体的 `[roll, pitch, yaw]`，单位为弧度 |

该文件由前端、关键帧桥接、回环节点和地图节点共同读取，并在传感器参数文件
之后加载，因此其中的 frame 参数具有最终决定权。不要在 `backend.yaml`、
`bridge.yaml` 或 `map.yaml` 中复制这些参数。

### 后端回环检测（`PGO/config/backend.yaml`）

主要参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `kf_trans_thresh` / `kf_rot_thresh` | `1.5` / `0.3` | 关键帧选取阈值（m / rad） |
| `loop_iris_distance_thresh` | `0.40` | LidarIris 距离门限 |
| `cart_distance_thresh` | `0.40` | CartContext 距离门限 |
| `loop_gicp_score_thresh` | `0.21` | 有界 GICP 分数门限 |
| `loop_gicp_min_inlier_ratio` | `0.50` | GICP 最小内点比例 |
| `loop_gicp_submap_keyframes` | `5` | GICP 子图相邻关键帧数 |
| `loop_gicp_submap_leaf_size` | `0.25` | GICP 子图体素大小（m） |
| `loop_gicp_max_iterations` | `20` | GICP 每次试验最大迭代数 |
| `pgo_loop_measurement_mode` | `gravity_preserving_xyz` | `gravity_preserving`（x/y/yaw）、`gravity_preserving_xyz`（x/y/z/yaw）、`full_se3`（全 6 自由度） |
| `gravity_align_enable` | `true` | 保留描述子/世界方向处理通路 |
| `gravity_align_source` | `manual_rpy` | 默认使用单位手动修正，避免重复施加前端倾角 |
| `loop_history_max_current_matches` | `2` | 每个 history 关键帧最大回环边数，超过后停用其 LCD 检索（0 = 关闭） |
| `loop_sequence_max_history_lag_m` | `3.0` | current 与 history 有向里程进度的最大双边偏差（0 = 关闭） |
| `loop_sequence_max_anchor_progress_m` | `15.0` | pending 等待距离和正式序列锚点的有效距离（0 = 不失效） |
| `loop_descriptor_verify_top_k` | `4` | 描述子排序后送入 GICP 验证的 top-k 数量 |

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

### 地图节点（`PGO/config/map.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map_leaf_size` | `0.20` | 全局地图体素大小（m） |
| `keyframe_rebuild_interval` | `20` | 每收到 N 个关键帧才重建一次全局地图；PGO 快照始终立即重建 |
| `save_map_service_name` | `/map_save_lc` | 保存 PCD 的 ROS 服务名 |
| `map_save_path` | `.../global_map.pcd` | PCD 输出路径 |

## 保存地图

### 前端局部地图

在前端配置中设置 `save_pcd: true`（或给 `frontend_bringup.launch.py` 传
`save_pcd:=true`），运行节点后调用：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### 后端全局地图（PGO 修正后）

```bash
ros2 service call /map_save_lc std_srvs/srv/Trigger
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
