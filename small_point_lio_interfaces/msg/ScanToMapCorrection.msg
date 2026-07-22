# 所有位姿都是 header.stamp 时刻的 T_odom_base。
# packet_predicted_pose 保留本 packet 之前的修正，只排除本 packet 点更新；
# epoch_predicted_pose 排除当前 tracking_map_version 生效以来的所有点更新。
std_msgs/Header header
# 本次运行内第几个前端 correction。
uint64 sequence
# 生成本 correction 时使用的 iVox 版本。
uint64 tracking_map_version
# 上一 packet 边界以来仅用 IMU 的预测。
geometry_msgs/Pose packet_predicted_pose
# 当前地图 epoch 以来仅用 IMU 的预测。
geometry_msgs/Pose epoch_predicted_pose
# 本 packet 点更新后的 ESKF 位姿。
geometry_msgs/Pose corrected_pose
# 提交给 ESKF 的点更新数。
uint32 attempted_point_updates
# 被 ESKF 接受的点更新数。
uint32 accepted_point_updates
# 接受残差的 RMS。
float64 residual_rms
# 接受残差的最大绝对值。
float64 residual_max_abs
# 产生当前 active tracking_map_version 的来源 correction 序号；0 表示初始地图。
uint64 active_map_source_correction_sequence
