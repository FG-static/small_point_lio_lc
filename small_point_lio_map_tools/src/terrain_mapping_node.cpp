#include "small_point_lio_map_tools/terrain_mapping_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

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

    filtered_cloud_publisher_ =
        create_publisher<CloudMsg>("/terrain/filtered_cloud", 10);
    observed_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/terrain/observed_map",
        rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
        get_logger(),
        "Terrain mapping input: cloud=%s odom=%s frame=%s base=%s",
        cloud_topic_.c_str(),
        odom_topic_.c_str(),
        odom_frame_.c_str(),
        base_frame_.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Terrain grid: resolution=%.3f width=%.1f height=%.1f recenter=%.1f",
        resolution_,
        map_width_,
        map_height_,
        recenter_distance_);
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

    resolution_ = declare_parameter<double>("resolution", resolution_);
    map_width_ = declare_parameter<double>("map_width", map_width_);
    map_height_ = declare_parameter<double>("map_height", map_height_);
    recenter_distance_ = declare_parameter<double>(
        "recenter_distance",
        recenter_distance_);
    self_filter_radius_ = declare_parameter<double>(
        "self_filter_radius",
        self_filter_radius_);
    min_point_range_ = declare_parameter<double>(
        "min_point_range",
        min_point_range_);
    z_layer_merge_threshold_ = declare_parameter<double>(
        "z_layer_merge_threshold",
        z_layer_merge_threshold_);
    publish_filtered_cloud_ = declare_parameter<bool>(
        "publish_filtered_cloud",
        publish_filtered_cloud_);
    publish_observed_map_ = declare_parameter<bool>(
        "publish_observed_map",
        publish_observed_map_);

    if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "resolution must be positive; using 0.10 m");
        resolution_ = 0.10;
    }
    if (!std::isfinite(map_width_) || map_width_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "map_width must be positive; using 20.0 m");
        map_width_ = 20.0;
    }
    if (!std::isfinite(map_height_) || map_height_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "map_height must be positive; using 20.0 m");
        map_height_ = 20.0;
    }
    if (!std::isfinite(recenter_distance_) || recenter_distance_ < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "recenter_distance must not be negative; using 2.0 m");
        recenter_distance_ = 2.0;
    }
    if (!std::isfinite(self_filter_radius_) || self_filter_radius_ < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "self_filter_radius must not be negative; using 0.30 m");
        self_filter_radius_ = 0.30;
    }
    if (!std::isfinite(min_point_range_) || min_point_range_ < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "min_point_range must not be negative; using 0.30 m");
        min_point_range_ = 0.30;
    }
    if (!std::isfinite(z_layer_merge_threshold_) ||
        z_layer_merge_threshold_ <= 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "z_layer_merge_threshold must be positive; using 0.10 m");
        z_layer_merge_threshold_ = 0.10;
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
    bool valid_xyz_types = true;
    for (const auto & field : msg->fields) {

        if (field.name == "x") {
            has_x = true;
            valid_xyz_types = valid_xyz_types &&
                field.datatype == sensor_msgs::msg::PointField::FLOAT32;
        } else if (field.name == "y") {
            has_y = true;
            valid_xyz_types = valid_xyz_types &&
                field.datatype == sensor_msgs::msg::PointField::FLOAT32;
        } else if (field.name == "z") {
            has_z = true;
            valid_xyz_types = valid_xyz_types &&
                field.datatype == sensor_msgs::msg::PointField::FLOAT32;
        }
    }
    if (!has_x || !has_y || !has_z) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring cloud without x/y/z fields");
        return;
    }
    if (!valid_xyz_types) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring cloud with non-FLOAT32 x/y/z fields");
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

    std::vector<TerrainPoint> filtered_points;
    if (!processSynchronizedCloud(*msg, matched_odom, filtered_points)) {
        return;
    }

    publishDebugOutputs(*msg, filtered_points);

    RCLCPP_DEBUG(
        get_logger(),
        "Processed cloud stamp %ld with odometry stamp %ld: %zu points",
        static_cast<long>(stamp),
        static_cast<long>(matched_odom.stamp_ns),
        filtered_points.size());
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
        !std::isfinite(orientation.w)) {

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

    bool time_moved_backwards = false;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        time_moved_backwards = !odom_buffer_.empty() &&
            odom_sample.stamp_ns < odom_buffer_.back().stamp_ns;
        trimOdomBuffer(odom_sample.stamp_ns);
        odom_buffer_.push_back(odom_sample);
    }

    if (time_moved_backwards) {

        resetTerrainGrid();
        RCLCPP_WARN(
            get_logger(),
            "Odometry time moved backwards; reset synchronization and terrain map");
    }
}

/**
 * @brief 处理同步的点云数据，将其转换为地形点并插入到地形栅格中。
 * @param cloud 点云数据。
 * @param matched_odom 与点云对应的里程计样本。
 * @param filtered_points 过滤后的地形点列表。
 * @return bool 处理成功返回 true，否则返回 false。
 */
bool TerrainMappingNode::processSynchronizedCloud(
    const CloudMsg &cloud,
    const OdomSample &matched_odom,
    std::vector<TerrainPoint> &filtered_points
) {

    filtered_points.clear();
    const std::size_t input_point_count =
        static_cast<std::size_t>(cloud.width) *
        static_cast<std::size_t>(cloud.height);
    filtered_points.reserve(std::min(input_point_count, std::size_t(100000U)));

    std::lock_guard<std::mutex> lock(terrain_mutex_);
    if (!terrain_grid_initialized_) {

        initializeTerrainGridLocked(
            matched_odom.position_x,
            matched_odom.position_y
        );
    } else {

        recenterTerrainGridLocked(
            matched_odom.position_x,
            matched_odom.position_y
        );
    }

    try {

        // 检查点云所有点
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++ iter_x, ++ iter_y, ++ iter_z) {

            const TerrainPoint point{
                *iter_x,
                *iter_y,
                *iter_z
            };
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z))
                continue;

            // calculate twist
            const double dx = static_cast<double>(point.x) -
                matched_odom.position_x;
            const double dy = static_cast<double>(point.y) -
                matched_odom.position_y;
            const double dz = static_cast<double>(point.z) -
                matched_odom.position_z;
            const double horizontal_range = std::hypot(dx, dy);
            const double point_range = std::sqrt(
                dx * dx + dy * dy + dz * dz);
            if (horizontal_range < self_filter_radius_ ||
                point_range < min_point_range_)
                continue;

            std::size_t cell_index = 0U;
            if (!worldToGridLocked(point.x, point.y, cell_index))
                continue;

            insertTerrainPointLocked(point);
            filtered_points.push_back(point);
        }
    } catch (const std::exception & error) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring malformed point cloud: %s", error.what());
        filtered_points.clear();
        return false;
    }

    return true;
}

void TerrainMappingNode::initializeTerrainGridLocked(
    double center_x,
    double center_y
) {

    terrain_grid_width_cells_ = std::max<std::size_t>(
        1U,
        static_cast<std::size_t>(std::ceil(map_width_ / resolution_))
    );
    terrain_grid_height_cells_ = std::max<std::size_t>(
        1U,
        static_cast<std::size_t>(std::ceil(map_height_ / resolution_))
    );
    terrain_grid_.assign(
        terrain_grid_width_cells_ * terrain_grid_height_cells_,
        TerrainCell{}
    );
    // 初始化栅格地图左下角原点
    terrain_origin_x_ = center_x -
        0.5 * static_cast<double>(terrain_grid_width_cells_) * resolution_;
    terrain_origin_y_ = center_y -
        0.5 * static_cast<double>(terrain_grid_height_cells_) * resolution_;
    terrain_grid_initialized_ = true;
}

void TerrainMappingNode::resetTerrainGrid() {

    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain_grid_.clear();
    terrain_grid_width_cells_ = 0U;
    terrain_grid_height_cells_ = 0U;
    terrain_origin_x_ = 0.0;
    terrain_origin_y_ = 0.0;
    terrain_grid_initialized_ = false;
}

void TerrainMappingNode::recenterTerrainGridLocked(
    double robot_x,
    double robot_y
) {

    if (!terrain_grid_initialized_) {

        initializeTerrainGridLocked(robot_x, robot_y);
        return;
    }

    const double current_center_x = terrain_origin_x_ +
        0.5 * static_cast<double>(terrain_grid_width_cells_) * resolution_;
    const double current_center_y = terrain_origin_y_ +
        0.5 * static_cast<double>(terrain_grid_height_cells_) * resolution_;
    const double offset_x = robot_x - current_center_x;
    const double offset_y = robot_y - current_center_y;
    if (std::hypot(offset_x, offset_y) <= recenter_distance_)
        return;

    // calculate twist
    std::int64_t shift_x = static_cast<std::int64_t>(
        std::llround(offset_x / resolution_));
    std::int64_t shift_y = static_cast<std::int64_t>(
        std::llround(offset_y / resolution_));
    if (shift_x == 0 && offset_x != 0.0)
        shift_x = offset_x > 0.0 ? 1 : -1;
    if (shift_y == 0 && offset_y != 0.0)
        shift_y = offset_y > 0.0 ? 1 : -1;

    const auto width = static_cast<std::int64_t>(terrain_grid_width_cells_);
    const auto height = static_cast<std::int64_t>(terrain_grid_height_cells_);
    if (std::abs(shift_x) >= width || std::abs(shift_y) >= height) {

        // 超出边界，重新初始化栅格地图
        initializeTerrainGridLocked(robot_x, robot_y);
        return;
    }

    // 移动栅格地图
    std::vector<TerrainCell> shifted_grid(terrain_grid_.size());
    for (std::size_t y = 0U; y < terrain_grid_height_cells_; ++ y) {

        for (std::size_t x = 0U; x < terrain_grid_width_cells_; ++ x) {

            const auto new_x = static_cast<std::int64_t>(x) - shift_x;
            const auto new_y = static_cast<std::int64_t>(y) - shift_y;
            if (new_x < 0 || new_x >= width || new_y < 0 || new_y >= height)
                continue;

            const std::size_t old_index =
                y * terrain_grid_width_cells_ + x;
            const std::size_t new_index =
                static_cast<std::size_t>(new_y) * terrain_grid_width_cells_ +
                static_cast<std::size_t>(new_x);
            shifted_grid[new_index] = terrain_grid_[old_index];
        }
    }

    terrain_grid_.swap(shifted_grid);
    terrain_origin_x_ += static_cast<double>(shift_x) * resolution_;
    terrain_origin_y_ += static_cast<double>(shift_y) * resolution_;
}

/**
 * @brief 将世界坐标转换为栅格地图索引
 * @param x 世界坐标 x
 * @param y 世界坐标 y
 * @param cell_index 栅格地图索引
 * @return bool 转换成功返回 true，否则返回 false
 */
bool TerrainMappingNode::worldToGridLocked(
    double x,
    double y,
    std::size_t &cell_index
) const {

    const auto grid_x = static_cast<std::int64_t>(
        std::floor((x - terrain_origin_x_) / resolution_));
    const auto grid_y = static_cast<std::int64_t>(
        std::floor((y - terrain_origin_y_) / resolution_));
    if (grid_x < 0 || grid_y < 0 ||
        grid_x >= static_cast<std::int64_t>(terrain_grid_width_cells_) ||
        grid_y >= static_cast<std::int64_t>(terrain_grid_height_cells_))
        return false;

    cell_index = static_cast<std::size_t>(grid_y) *
        terrain_grid_width_cells_ + static_cast<std::size_t>(grid_x);
    return true;
}

/**
 * @brief 往地图插入地形点
 * @param point 地形点
 */
void TerrainMappingNode::insertTerrainPointLocked(
    const TerrainPoint &point
) {

    std::size_t cell_index = 0U;
    if (!worldToGridLocked(point.x, point.y, cell_index))
        return;

    auto &cell = terrain_grid_[cell_index];
    cell.observed = true;
    ++ cell.point_count;
    cell.min_z = std::min(cell.min_z, point.z);
    cell.max_z = std::max(cell.max_z, point.z);
    cell.z_sum += static_cast<double>(point.z);
    updateZLayersLocked(cell, point.z);
}

/**
 * @brief 更新某个 cell 的所有地形层
 * @param cell 地形单元格
 * @param z 地形点高度
 */
void TerrainMappingNode::updateZLayersLocked(TerrainCell &cell, float z) {

    std::size_t nearest_layer = 0U;
    float nearest_distance = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0U; index < cell.z_layer_count; ++ index) {

        const float distance = std::abs(cell.z_layers[index].center_z - z);
        if (distance < nearest_distance) {

            nearest_distance = distance;
            nearest_layer = index;
        }
    }

    if (cell.z_layer_count < kTerrainMaxZLayers &&
        (cell.z_layer_count == 0U ||
         nearest_distance > static_cast<float>(z_layer_merge_threshold_))) {

        auto & layer = cell.z_layers[cell.z_layer_count++];
        layer.center_z = z;
        layer.point_count = 1U;
        layer.min_z = z;
        layer.max_z = z;
        return;
    }

    // 层数达到上限时合并到最近层，保证每个格子的内存固定。
    auto & layer = cell.z_layers[nearest_layer];
    ++ layer.point_count;
    layer.center_z +=
        (z - layer.center_z) / static_cast<float>(layer.point_count);
    layer.min_z = std::min(layer.min_z, z);
    layer.max_z = std::max(layer.max_z, z);
}

void TerrainMappingNode::publishDebugOutputs(
    const CloudMsg &source_cloud,
    const std::vector<TerrainPoint> &filtered_points
) {

    if (publish_filtered_cloud_ && filtered_cloud_publisher_) {

        CloudMsg filtered_cloud;
        filtered_cloud.header = source_cloud.header;
        filtered_cloud.header.frame_id = odom_frame_;
        sensor_msgs::PointCloud2Modifier modifier(filtered_cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(filtered_points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(filtered_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(filtered_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_cloud, "z");
        for (const auto & point : filtered_points) {

            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            ++ iter_x;
            ++ iter_y;
            ++ iter_z;
        }
        filtered_cloud.is_dense = true;
        filtered_cloud_publisher_->publish(filtered_cloud);
    }

    if (publish_observed_map_ && observed_map_publisher_) {

        nav_msgs::msg::OccupancyGrid observed_map;
        {
            std::lock_guard<std::mutex> lock(terrain_mutex_);
            if (!terrain_grid_initialized_)
                return;

            observed_map.header = source_cloud.header;
            observed_map.header.frame_id = odom_frame_;
            observed_map.info.resolution = static_cast<float>(resolution_);
            observed_map.info.width = static_cast<std::uint32_t>(
                terrain_grid_width_cells_);
            observed_map.info.height = static_cast<std::uint32_t>(
                terrain_grid_height_cells_);
            observed_map.info.origin.position.x = terrain_origin_x_;
            observed_map.info.origin.position.y = terrain_origin_y_;
            observed_map.info.origin.orientation.w = 1.0;
            observed_map.data.assign(
                terrain_grid_.size(),
                static_cast<std::int8_t>(-1));
            for (std::size_t index = 0U; index < terrain_grid_.size(); ++ index) {

                if (terrain_grid_[index].observed)
                    observed_map.data[index] = 0;
            }
        }
        observed_map_publisher_->publish(observed_map);
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

std::int64_t TerrainMappingNode::stampToNanoseconds(
    const rclcpp::Time &stamp
) const {

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
