# All poses are T_odom_base at header.stamp. The packet prediction retains all
# corrections accepted before this LiDAR packet and excludes only this packet's
# point updates. The epoch prediction excludes point updates since the current
# tracking_map_version became active.
std_msgs/Header header
uint64 sequence
uint64 tracking_map_version
geometry_msgs/Pose packet_predicted_pose
geometry_msgs/Pose epoch_predicted_pose
geometry_msgs/Pose corrected_pose
uint32 attempted_point_updates
uint32 accepted_point_updates
float64 residual_rms
float64 residual_max_abs
# Source correction sequence of the LocalTrackingMap that produced the active
# tracking_map_version. Zero means the initial frontend map.
uint64 active_map_source_correction_sequence
