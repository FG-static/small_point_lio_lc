# cloud 使用 header.frame_id 表达（通常为 odom）。只有 source_tracking_map_version
# 仍是前端 active 版本时该更新才有效；header.stamp 是 tail journal 的地图 cutoff。
std_msgs/Header header
# 构建该地图所依据的 correction 序号。
uint64 source_correction_sequence
# 构建开始时的前端 iVox 版本。
uint64 source_tracking_map_version
# 前端切图成功后的目标版本。
uint64 target_tracking_map_version
# 构建采用的全局 PGO 图版本。
uint64 pgo_graph_version
# 局部窗口固定锚点关键帧 ID。
uint32 anchor_candidate_id
# pgo、instant_scan_correction 或 cumulative_scan_correction。
string trigger_reason
# 供前端重建 iVox 的局部点云。
sensor_msgs/PointCloud2 cloud
