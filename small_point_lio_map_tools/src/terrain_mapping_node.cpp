#include "small_point_lio_map_tools/terrain_mapping_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace small_point_lio_map_tools {

TerrainMappingNode::TerrainMappingNode()
    : rclcpp::Node("terrain_mapping_node") {

    loadParams();
    cloud_sub_ = create_subscription<CloudMsg>(
        cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TerrainMappingNode::onCloud, this, std::placeholders::_1));
    odom_sub_ = create_subscription<OdomMsg>(
        odom_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&TerrainMappingNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Terrain mapping input: cloud=%s odom=%s frame=%s base=%s",
        cloud_topic_.c_str(),
        odom_topic_.c_str(),
        odom_frame_.c_str(),
        base_frame_.c_str());
}

void TerrainMappingNode::loadParams() {

    cloud_topic_ = declare_parameter<std::string>("cloud_topic", cloud_topic_);
    odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);

    const auto buffer_size = declare_parameter<std::int64_t>(
        "max_odom_buffer_size",
        static_cast<std::int64_t>(max_odom_buffer_size_));
    if (buffer_size <= 0) {

        RCLCPP_WARN(
            get_logger(),
            "max_odom_buffer_size must be positive; using 1");
    }
    max_odom_buffer_size_ = buffer_size > 0
        ? static_cast<std::size_t>(buffer_size)
        : 1U;

    max_sync_delay_ns_ = declare_parameter<std::int64_t>(
        "max_sync_delay_ns",
        max_sync_delay_ns_);
    if (max_sync_delay_ns_ < 0) {

        RCLCPP_WARN(get_logger(), "max_sync_delay_ns must not be negative; using 0");
        max_sync_delay_ns_ = 0;
    }

    if (cloud_topic_.empty()) cloud_topic_ = "/cloud_registered";
    if (odom_topic_.empty()) odom_topic_ = "/Odometry";
    if (odom_frame_.empty()) odom_frame_ = "odom";
    if (base_frame_.empty()) base_frame_ = "base_link";
}

void TerrainMappingNode::onCloud(const CloudMsg::ConstSharedPtr msg) {

    if (!msg || msg->data.empty() || msg->width == 0 || msg->height == 0) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring empty registered cloud");
        return;
    }

    if (msg->header.frame_id != odom_frame_) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring cloud in frame '%s'; expected '%s'",
            msg->header.frame_id.c_str(),
            odom_frame_.c_str());
        return;
    }

    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto & field : msg->fields) {

        has_x = has_x || field.name == "x";
        has_y = has_y || field.name == "y";
        has_z = has_z || field.name == "z";
    }
    if (!has_x || !has_y || !has_z) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring cloud without x/y/z fields");
        return;
    }

    const auto stamp = stampToNanoseconds(msg->header.stamp);
    OdomSample matched_odom;

    if (!findNearestOdom(stamp, matched_odom)) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "No odometry sample matches cloud stamp %ld within %ld ns",
            static_cast<long>(stamp),
            static_cast<long>(max_sync_delay_ns_));
        return;
    }

    // 阶段 2 从此处使用已同步点云和机器人位姿构建滚动栅格。
    RCLCPP_DEBUG(
        get_logger(),
        "Matched cloud stamp %ld with odometry stamp %ld",
        static_cast<long>(stamp),
        static_cast<long>(matched_odom.stamp_ns));
}

void TerrainMappingNode::onOdom(const OdomMsg::ConstSharedPtr msg) {

    if (!msg) return;

    if (msg->header.frame_id != odom_frame_) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring odometry in frame '%s'; expected '%s'",
            msg->header.frame_id.c_str(),
            odom_frame_.c_str());
        return;
    }
    if (!msg->child_frame_id.empty() && msg->child_frame_id != base_frame_) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring odometry child frame '%s'; expected '%s'",
            msg->child_frame_id.c_str(),
            base_frame_.c_str());
        return;
    }

    const auto & position = msg->pose.pose.position;
    const auto & orientation = msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w)
    ) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring odometry with non-finite pose");
        return;
    }

    OdomSample odom_sample;
    odom_sample.stamp_ns = stampToNanoseconds(msg->header.stamp);
    if (odom_sample.stamp_ns < 0) return;

    odom_sample.position_x = position.x;
    odom_sample.position_y = position.y;
    odom_sample.position_z = position.z;
    odom_sample.orientation_w = orientation.w;
    odom_sample.orientation_x = orientation.x;
    odom_sample.orientation_y = orientation.y;
    odom_sample.orientation_z = orientation.z;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    const bool time_moved_backwards = !odom_buffer_.empty() &&
        odom_sample.stamp_ns < odom_buffer_.back().stamp_ns;
    trimOdomBuffer(odom_sample.stamp_ns);
    odom_buffer_.push_back(odom_sample);

    if (time_moved_backwards) {

        RCLCPP_WARN(
            get_logger(),
            "Odometry time moved backwards; reset synchronization buffer");
    }
}

void TerrainMappingNode::trimOdomBuffer(std::int64_t newest_stamp_ns) {

    // 该函数由持有 odom_mutex_ 的里程计回调调用。
    if (!odom_buffer_.empty() &&
        newest_stamp_ns < odom_buffer_.back().stamp_ns) {
        odom_buffer_.clear();
    }

    const std::size_t keep_before_insert = max_odom_buffer_size_ - 1U;
    while (odom_buffer_.size() > keep_before_insert)
        odom_buffer_.pop_front();
}

std::int64_t TerrainMappingNode::stampToNanoseconds(const rclcpp::Time & stamp) const {

    return stamp.nanoseconds();
}

bool TerrainMappingNode::findNearestOdom(
    std::int64_t cloud_stamp_ns,
    OdomSample &matched_odom
) const {

    if (cloud_stamp_ns < 0 || max_sync_delay_ns_ < 0)
        return false;

    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_buffer_.empty()) return false;

    const auto absoluteDifference = [](
        std::int64_t lhs,
        std::int64_t rhs
    ) -> std::uint64_t {

        if (lhs >= rhs)
            return static_cast<std::uint64_t>(lhs - rhs);
        return static_cast<std::uint64_t>(rhs - lhs);
    };

    const auto lower = std::lower_bound(
        odom_buffer_.cbegin(),
        odom_buffer_.cend(),
        cloud_stamp_ns,
        [](const OdomSample & sample, std::int64_t stamp_ns) {
            return sample.stamp_ns < stamp_ns;
        }
    );

    const OdomSample *nearest = nullptr;

    if (lower == odom_buffer_.cbegin()) {

        nearest = &(*lower);
    } else if (lower == odom_buffer_.cend()) {

        nearest = &odom_buffer_.back();
    } else {

        const OdomSample & before = *(lower - 1);
        const OdomSample & after = *lower;

        const std::uint64_t before_difference = absoluteDifference(
            before.stamp_ns,
            cloud_stamp_ns
        );
        const std::uint64_t after_difference = absoluteDifference(
            after.stamp_ns,
            cloud_stamp_ns
        );

        // 时间相同的情况下固定选择较早的样本，保证结果确定。
        nearest = before_difference <= after_difference ? &before : &after;
    }

    const std::uint64_t difference = absoluteDifference(
        nearest->stamp_ns,
        cloud_stamp_ns
    );

    if (difference > static_cast<std::uint64_t>(max_sync_delay_ns_))
        return false;

    matched_odom = *nearest;
    return true;
}

}  // namespace small_point_lio_map_tools
