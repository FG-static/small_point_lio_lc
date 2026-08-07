#ifndef SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_
#define SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace small_point_lio_map_tools {

// 与点云时间对应的轻量里程计样本。
struct OdomSample {
    std::int64_t stamp_ns{0};
    double position_x{0.0};
    double position_y{0.0};
    double position_z{0.0};
    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double orientation_w{1.0};
};

// 地形建图节点的最小运行骨架。
class TerrainMappingNode final : public rclcpp::Node {
public:
    TerrainMappingNode();

private:
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    using OdomMsg = nav_msgs::msg::Odometry;

    void loadParams();

    // 输入回调和时间匹配接口。
    void onCloud(const CloudMsg::ConstSharedPtr msg);
    void onOdom(const OdomMsg::ConstSharedPtr msg);
    bool findNearestOdom(
        std::int64_t cloud_stamp_ns,
        OdomSample & matched_odom) const;
    void trimOdomBuffer(std::int64_t newest_stamp_ns);
    std::int64_t stampToNanoseconds(const rclcpp::Time & stamp) const;

    rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;

    mutable std::mutex odom_mutex_;
    std::deque<OdomSample> odom_buffer_;

    std::size_t max_odom_buffer_size_{200};
    std::int64_t max_sync_delay_ns_{50000000LL};

    std::string cloud_topic_{"/cloud_registered"};
    std::string odom_topic_{"/Odometry"};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
};
}  // namespace small_point_lio_map_tools

#endif  // SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_
