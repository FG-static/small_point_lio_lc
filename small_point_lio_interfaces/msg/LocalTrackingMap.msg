# cloud is expressed in header.frame_id (normally odom). The update is valid
# only while source_tracking_map_version is still active. header.stamp is the
# source correction cutoff used by the frontend tail-point journal.
std_msgs/Header header
uint64 source_correction_sequence
uint64 source_tracking_map_version
uint64 target_tracking_map_version
uint64 pgo_graph_version
uint32 anchor_candidate_id
string trigger_reason
sensor_msgs/PointCloud2 cloud
