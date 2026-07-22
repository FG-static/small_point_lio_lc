std_msgs/Header header
# 全局 PGO 优化快照版本。
uint64 graph_version
# 与 poses 一一对应的关键帧 ID。
uint32[] ids
# 对应关键帧的优化位姿。
geometry_msgs/Pose[] poses
