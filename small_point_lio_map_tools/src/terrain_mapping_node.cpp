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

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

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
    ground_candidate_map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/terrain/ground_candidate_map",
            rclcpp::QoS(1).reliable().transient_local());
    ground_cloud_publisher_ =
        create_publisher<CloudMsg>("/terrain/ground_cloud", 10);
    slope_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/terrain/slope_map",
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
    RCLCPP_INFO(
        get_logger(),
        "Ground support: base_height=%.3f seed_radius=%.2f "
        "seed_tolerance=%.2f max_slope=%.1f max_step=%.2f "
        "update_rate=%.1fHz hold=%.2fs",
        base_to_ground_height_,
        seed_radius_,
        seed_height_tolerance_,
        propagation_max_slope_deg_,
        max_ground_step_,
        ground_update_rate_hz_,
        ground_support_hold_time_sec_);
    RCLCPP_INFO(
        get_logger(),
        "Ground PCA: radius=%.2f min_cells=%zu max_residual=%.3f",
        pca_radius_,
        pca_min_cells_,
        max_plane_residual_);
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
    base_to_ground_height_ = declare_parameter<double>(
        "base_to_ground_height",
        base_to_ground_height_);
    seed_radius_ = declare_parameter<double>("seed_radius", seed_radius_);
    seed_height_tolerance_ = declare_parameter<double>(
        "seed_height_tolerance",
        seed_height_tolerance_);
    const auto seed_min_cells = declare_parameter<std::int64_t>(
        "seed_min_cells",
        static_cast<std::int64_t>(seed_min_cells_));
    const auto ground_layer_min_points = declare_parameter<std::int64_t>(
        "ground_layer_min_points",
        static_cast<std::int64_t>(ground_layer_min_points_));
    propagation_max_slope_deg_ = declare_parameter<double>(
        "propagation_max_slope_deg",
        propagation_max_slope_deg_);
    propagation_height_tolerance_ = declare_parameter<double>(
        "propagation_height_tolerance",
        propagation_height_tolerance_);
    max_ground_step_ = declare_parameter<double>(
        "max_ground_step",
        max_ground_step_);
    ground_update_rate_hz_ = declare_parameter<double>(
        "ground_update_rate_hz",
        ground_update_rate_hz_);
    ground_support_hold_time_sec_ = declare_parameter<double>(
        "ground_support_hold_time_sec",
        ground_support_hold_time_sec_);
    pca_radius_ = declare_parameter<double>("pca_radius", pca_radius_);
    const auto pca_min_cells = declare_parameter<std::int64_t>(
        "pca_min_cells",
        static_cast<std::int64_t>(pca_min_cells_));
    max_plane_residual_ = declare_parameter<double>(
        "max_plane_residual",
        max_plane_residual_);
    publish_filtered_cloud_ = declare_parameter<bool>(
        "publish_filtered_cloud",
        publish_filtered_cloud_);
    publish_observed_map_ = declare_parameter<bool>(
        "publish_observed_map",
        publish_observed_map_);
    publish_ground_candidate_map_ = declare_parameter<bool>(
        "publish_ground_candidate_map",
        publish_ground_candidate_map_);
    publish_ground_cloud_ = declare_parameter<bool>(
        "publish_ground_cloud",
        publish_ground_cloud_);
    publish_slope_map_ = declare_parameter<bool>(
        "publish_slope_map",
        publish_slope_map_);

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
    if (!std::isfinite(base_to_ground_height_) ||
        base_to_ground_height_ < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "base_to_ground_height must not be negative; using 0.0 m");
        base_to_ground_height_ = 0.0;
    }
    if (!std::isfinite(seed_radius_) || seed_radius_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "seed_radius must be positive; using 0.80 m");
        seed_radius_ = 0.80;
    }
    if (!std::isfinite(seed_height_tolerance_) ||
        seed_height_tolerance_ <= 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "seed_height_tolerance must be positive; using 0.15 m");
        seed_height_tolerance_ = 0.15;
    }
    if (seed_min_cells <= 0) {
        RCLCPP_WARN(get_logger(), "seed_min_cells must be positive; using 1");
    }
    seed_min_cells_ = seed_min_cells > 0
        ? static_cast<std::size_t>(seed_min_cells)
        : 1U;
    if (ground_layer_min_points <= 0) {
        RCLCPP_WARN(
            get_logger(),
            "ground_layer_min_points must be positive; using 1");
    }
    ground_layer_min_points_ = ground_layer_min_points > 0
        ? static_cast<std::size_t>(ground_layer_min_points)
        : 1U;
    if (!std::isfinite(propagation_max_slope_deg_) ||
        propagation_max_slope_deg_ < 0.0 ||
        propagation_max_slope_deg_ >= 89.0) {
        RCLCPP_WARN(
            get_logger(),
            "propagation_max_slope_deg must be in [0, 89); using 25 deg");
        propagation_max_slope_deg_ = 25.0;
    }
    if (!std::isfinite(propagation_height_tolerance_) ||
        propagation_height_tolerance_ < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "propagation_height_tolerance must not be negative; using 0.02 m");
        propagation_height_tolerance_ = 0.02;
    }
    if (!std::isfinite(max_ground_step_) || max_ground_step_ <= 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "max_ground_step must be positive; using 0.10 m");
        max_ground_step_ = 0.10;
    }
    if (!std::isfinite(ground_update_rate_hz_) ||
        ground_update_rate_hz_ <= 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "ground_update_rate_hz must be positive; using 10.0 Hz");
        ground_update_rate_hz_ = 10.0;
    }
    if (!std::isfinite(ground_support_hold_time_sec_) ||
        ground_support_hold_time_sec_ < 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "ground_support_hold_time_sec must not be negative; using 0.50 s");
        ground_support_hold_time_sec_ = 0.50;
    }
    if (!std::isfinite(pca_radius_) || pca_radius_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "pca_radius must be positive; using 0.30 m");
        pca_radius_ = 0.30;
    }
    if (pca_min_cells < 3) {
        RCLCPP_WARN(get_logger(), "pca_min_cells must be at least 3; using 3");
    }
    pca_min_cells_ = pca_min_cells >= 3
        ? static_cast<std::size_t>(pca_min_cells)
        : 3U;
    if (!std::isfinite(max_plane_residual_) || max_plane_residual_ <= 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "max_plane_residual must be positive; using 0.05 m");
        max_plane_residual_ = 0.05;
    }
    ground_update_period_ns_ = std::max<std::int64_t>(
        1LL,
        static_cast<std::int64_t>(std::llround(
            1.0e9 / ground_update_rate_hz_)));
    ground_support_hold_time_ns_ = std::max<std::int64_t>(
        0LL,
        static_cast<std::int64_t>(std::llround(
            ground_support_hold_time_sec_ * 1.0e9)));

    if (cloud_topic_.empty()) cloud_topic_ = "/cloud_registered";
    if (odom_topic_.empty()) odom_topic_ = "/Odometry";
    if (odom_frame_.empty()) odom_frame_ = "odom";
    if (base_frame_.empty()) base_frame_ = "base_link";

    if (base_to_ground_height_ == 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "Ground support propagation is disabled until "
            "base_to_ground_height is set");
    }
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
    bool ground_support_updated = false;
    if (!processSynchronizedCloud(
            *msg,
            matched_odom,
            filtered_points,
            &ground_support_updated)) {
        return;
    }

    publishDebugOutputs(*msg, filtered_points, ground_support_updated);

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
    std::vector<TerrainPoint> &filtered_points,
    bool *ground_support_updated
) {

    filtered_points.clear();
    if (ground_support_updated)
        *ground_support_updated = false;
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

        // 检查是否需要更新地面支撑（时间回退或达到更新周期）
        const bool time_moved_backwards =
            last_ground_update_stamp_ns_ >= 0 &&
            matched_odom.stamp_ns < last_ground_update_stamp_ns_;
        const bool update_due =
            last_ground_update_stamp_ns_ < 0 ||
            time_moved_backwards ||
            matched_odom.stamp_ns - last_ground_update_stamp_ns_ >=
                ground_update_period_ns_;
        if (update_due) {

            last_ground_update_stamp_ns_ = matched_odom.stamp_ns;
            if (ground_support_updated)
                *ground_support_updated = true;

            std::size_t seed_count = 0U;
            std::size_t support_count = 0U;
            TerrainPoint expected_ground;
            updateGroundSupportLocked(
                matched_odom,
                seed_count,
                support_count,
                expected_ground);
            const std::size_t ground_fit_count = updateGroundFitsLocked();
            const std::size_t active_support_count =
                static_cast<std::size_t>(std::count_if(
                    terrain_grid_.cbegin(),
                    terrain_grid_.cend(),
                    [](const TerrainCell &cell) {
                        return cell.support_valid;
                    }));
            if (base_to_ground_height_ > 0.0) {

                if (seed_count == 0U) {

                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "No ground seed near expected point [%.2f %.2f %.2f]; "
                        "holding=%zu fitted=%zu",
                        expected_ground.x,
                        expected_ground.y,
                        expected_ground.z,
                        active_support_count,
                        ground_fit_count);
                } else {

                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Ground support: expected_z=%.3f seeds=%zu "
                        "connected=%zu active=%zu fitted=%zu",
                        expected_ground.z,
                        seed_count,
                        support_count,
                        active_support_count,
                        ground_fit_count);
                }
            }
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
    last_ground_update_stamp_ns_ = -1;
}

void TerrainMappingNode::resetTerrainGrid() {

    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain_grid_.clear();
    terrain_grid_width_cells_ = 0U;
    terrain_grid_height_cells_ = 0U;
    terrain_origin_x_ = 0.0;
    terrain_origin_y_ = 0.0;
    terrain_grid_initialized_ = false;
    last_ground_update_stamp_ns_ = -1;
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

/**
 * @brief 根据里程计计算预期地面点位置。
 * @param odom 里程计样本。
 * @param expected_ground 计算得到的地面点位置。
 * @return 计算成功返回 true，否则返回 false。
 */
bool TerrainMappingNode::computeExpectedGroundPoint(
    const OdomSample &odom,
    TerrainPoint &expected_ground
) const {

    if (base_to_ground_height_ <= 0.0)
        return false;

    const double norm = std::sqrt(
        odom.orientation_x * odom.orientation_x +
        odom.orientation_y * odom.orientation_y +
        odom.orientation_z * odom.orientation_z +
        odom.orientation_w * odom.orientation_w);
    if (!std::isfinite(norm) || norm < 1e-9)
        return false;

    const double x = odom.orientation_x / norm;
    const double y = odom.orientation_y / norm;
    const double z = odom.orientation_z / norm;
    const double w = odom.orientation_w / norm;
    const double rotation_xz = 2.0 * (x * z + w * y);
    const double rotation_yz = 2.0 * (y * z - w * x);
    const double rotation_zz = 1.0 - 2.0 * (x * x + y * y);

    expected_ground.x = static_cast<float>(
        odom.position_x - base_to_ground_height_ * rotation_xz);
    expected_ground.y = static_cast<float>(
        odom.position_y - base_to_ground_height_ * rotation_yz);
    expected_ground.z = static_cast<float>(
        odom.position_z - base_to_ground_height_ * rotation_zz);
    return std::isfinite(expected_ground.x) &&
        std::isfinite(expected_ground.y) &&
        std::isfinite(expected_ground.z);
}

bool TerrainMappingNode::selectLayerNearHeight(
    const TerrainCell &cell,
    float reference_z,
    float max_height_difference,
    float &selected_z
) const {

    float best_difference = std::numeric_limits<float>::infinity();
    bool found = false;
    for (std::size_t index = 0U; index < cell.z_layer_count; ++ index) {

        const auto &layer = cell.z_layers[index];
        if (layer.point_count < ground_layer_min_points_)
            continue;

        const float difference = std::abs(layer.center_z - reference_z);
        if (difference <= max_height_difference && difference < best_difference) {
            best_difference = difference;
            selected_z = layer.center_z;
            found = true;
        }
    }
    return found;
}

void TerrainMappingNode::updateGroundSupportLocked(
    const OdomSample &odom,
    std::size_t &seed_count,
    std::size_t &support_count,
    TerrainPoint &expected_ground
) {

    seed_count = 0U;
    support_count = 0U;
    for (auto &cell : terrain_grid_) {

        cell.support_seed = false;
        if (!cell.support_valid)
            continue;

        const bool invalid_stamp = cell.support_update_stamp_ns < 0 ||
            odom.stamp_ns < cell.support_update_stamp_ns;
        const bool expired = !invalid_stamp &&
            odom.stamp_ns - cell.support_update_stamp_ns >
                ground_support_hold_time_ns_;
        if (invalid_stamp || expired) {
            cell.support_valid = false;
            cell.support_z = std::numeric_limits<float>::quiet_NaN();
            cell.support_update_stamp_ns = -1;
        }
    }

    if (!computeExpectedGroundPoint(odom, expected_ground)) // 扩展失败
        return;

    struct SeedCandidate {
        std::size_t index{0U};
        float z{0.0F};
    };
    std::vector<SeedCandidate> candidates;
    std::vector<float> candidate_heights;
    for (std::size_t grid_y = 0U;
         grid_y < terrain_grid_height_cells_;
         ++ grid_y
    ) {

        for (std::size_t grid_x = 0U;
             grid_x < terrain_grid_width_cells_;
             ++ grid_x
        ) {

            const double center_x = terrain_origin_x_ +
                (static_cast<double>(grid_x) + 0.5) * resolution_;
            const double center_y = terrain_origin_y_ +
                (static_cast<double>(grid_y) + 0.5) * resolution_;
            if (std::hypot(
                    center_x - expected_ground.x,
                    center_y - expected_ground.y) > seed_radius_)
                continue;

            const std::size_t index =
                grid_y * terrain_grid_width_cells_ + grid_x;
            float selected_z = 0.0F;
            // 选择最近的高度层作为候选
            if (!selectLayerNearHeight(
                    terrain_grid_[index],
                    expected_ground.z,
                    static_cast<float>(seed_height_tolerance_),
                    selected_z))
                continue;

            // 候选点高度加入列表
            candidates.push_back({index, selected_z});
            candidate_heights.push_back(selected_z);
        }
    }

    if (candidates.size() < seed_min_cells_)
        return;

    std::sort(candidate_heights.begin(), candidate_heights.end());
    const float median_height =
        candidate_heights[candidate_heights.size() / 2U];
    // 基于中位数高度筛选候选点
    std::vector<SeedCandidate> accepted_seeds;
    accepted_seeds.reserve(candidates.size());
    for (const auto &candidate : candidates) {

        if (std::abs(candidate.z - median_height) <= seed_height_tolerance_)
            accepted_seeds.push_back(candidate);
    }
    if (accepted_seeds.size() < seed_min_cells_)
        return;

    // 初始化支撑面刷新状态
    std::vector<std::uint8_t> refreshed_support(
        terrain_grid_.size(),
        std::uint8_t{0U});
    std::vector<std::uint8_t> refreshed_seed(
        terrain_grid_.size(),
        std::uint8_t{0U});
    std::vector<float> refreshed_z(
        terrain_grid_.size(),
        std::numeric_limits<float>::quiet_NaN());

    // 初始化本轮传播队列，旧支撑面只负责显示保持，不参与本轮扩展。
    std::deque<std::size_t> queue;
    for (const auto &seed : accepted_seeds) {

        refreshed_support[seed.index] = 1U;
        refreshed_seed[seed.index] = 1U;
        refreshed_z[seed.index] = seed.z;
        queue.push_back(seed.index);
    }
    seed_count = accepted_seeds.size();
    support_count = seed_count;

    // 定义邻居偏移量和斜率切线
    constexpr int neighbor_offsets[8][2] = {
        {-1, -1}, {0, -1}, {1, -1},
        {-1,  0},          {1,  0},
        {-1,  1}, {0,  1}, {1,  1}
    };
    constexpr double kPi = 3.14159265358979323846;
    const double slope_tangent = std::tan(
        propagation_max_slope_deg_ * kPi / 180.0);
    // 遍历种子队列，传播支持，采用 BFS 算法
    while (!queue.empty()) {

        const std::size_t current_index = queue.front();
        queue.pop_front();
        const std::size_t current_x =
            current_index % terrain_grid_width_cells_;
        const std::size_t current_y =
            current_index / terrain_grid_width_cells_;
        const float current_z = refreshed_z[current_index];

        // 遍历邻居偏移量，传播支持
        for (const auto &offset : neighbor_offsets) {

            const auto next_x = static_cast<std::int64_t>(current_x) + offset[0];
            const auto next_y = static_cast<std::int64_t>(current_y) + offset[1];
            if (next_x < 0 || next_y < 0 ||
                next_x >= static_cast<std::int64_t>(terrain_grid_width_cells_) ||
                next_y >= static_cast<std::int64_t>(terrain_grid_height_cells_))
                continue;

            const std::size_t next_index =
                static_cast<std::size_t>(next_y) * terrain_grid_width_cells_ +
                static_cast<std::size_t>(next_x);
            const auto &next_cell = terrain_grid_[next_index];
            if (refreshed_support[next_index] != 0U)
                continue;

            const double distance = resolution_ * std::hypot(
                static_cast<double>(offset[0]),
                static_cast<double>(offset[1]));
            const float allowed_difference = static_cast<float>(std::min(
                max_ground_step_,
                distance * slope_tangent + propagation_height_tolerance_));
            float selected_z = 0.0F;
            if (!selectLayerNearHeight(
                    next_cell,
                    current_z,
                    allowed_difference,
                    selected_z))
                continue;

            refreshed_support[next_index] = 1U;
            refreshed_z[next_index] = selected_z;
            ++ support_count;
            queue.push_back(next_index);
        }
    }

    // 只刷新本轮重新确认的格子；其余旧结果按保持时间自然过期。
    for (std::size_t index = 0U; index < terrain_grid_.size(); ++ index) {

        if (refreshed_support[index] == 0U)
            continue;

        auto &cell = terrain_grid_[index];
        cell.support_valid = true;
        cell.support_seed = refreshed_seed[index] != 0U;
        cell.support_z = refreshed_z[index];
        cell.support_update_stamp_ns = odom.stamp_ns;
    }
}

std::size_t TerrainMappingNode::updateGroundFitsLocked() {

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (auto &cell : terrain_grid_) {

        cell.ground_valid = false;
        cell.ground_z = nan;
        cell.ground_normal_x = nan;
        cell.ground_normal_y = nan;
        cell.ground_normal_z = nan;
        cell.slope_deg = nan;
        cell.plane_residual = nan;
        cell.ground_confidence = 0.0F;
    }

    std::size_t fitted_count = 0U;
    for (std::size_t index = 0U; index < terrain_grid_.size(); ++ index) {

        // 尝试拟合该单元格的地面平面
        if (terrain_grid_[index].support_valid &&
            fitGroundPlaneAtCellLocked(index))
            ++ fitted_count;
    }
    return fitted_count;
}

/**
 * @brief 尝试拟合指定单元格的地面平面
 * @param cell_index 单元格索引
 * @return 拟合成功返回 true，否则返回 false
 */
bool TerrainMappingNode::fitGroundPlaneAtCellLocked(
    const std::size_t cell_index
) {

    if (cell_index >= terrain_grid_.size() ||
        !terrain_grid_[cell_index].support_valid)
        return false;

    const std::size_t target_x = cell_index % terrain_grid_width_cells_;
    const std::size_t target_y = cell_index / terrain_grid_width_cells_;
    const double target_center_x = terrain_origin_x_ +
        (static_cast<double>(target_x) + 0.5) * resolution_;
    const double target_center_y = terrain_origin_y_ +
        (static_cast<double>(target_y) + 0.5) * resolution_;
    const auto radius_cells = static_cast<std::int64_t>(
        std::ceil(pca_radius_ / resolution_));
    const double radius_squared = pca_radius_ * pca_radius_;

    std::vector<Eigen::Vector3d> samples;
    samples.reserve(static_cast<std::size_t>(
        (2 * radius_cells + 1) * (2 * radius_cells + 1)));
    double min_sample_z = std::numeric_limits<double>::infinity();
    double max_sample_z = -std::numeric_limits<double>::infinity();
    // 遍历邻域内的所有点
    for (std::int64_t offset_y = -radius_cells;
         offset_y <= radius_cells;
         ++ offset_y
    ) {

        for (std::int64_t offset_x = -radius_cells;
             offset_x <= radius_cells;
             ++ offset_x
        ) {

            const auto grid_x = static_cast<std::int64_t>(target_x) + offset_x;
            const auto grid_y = static_cast<std::int64_t>(target_y) + offset_y;
            if (grid_x < 0 || grid_y < 0 ||
                grid_x >= static_cast<std::int64_t>(terrain_grid_width_cells_) ||
                grid_y >= static_cast<std::int64_t>(terrain_grid_height_cells_))
                continue;

            const double center_x = terrain_origin_x_ +
                (static_cast<double>(grid_x) + 0.5) * resolution_;
            const double center_y = terrain_origin_y_ +
                (static_cast<double>(grid_y) + 0.5) * resolution_;
            const double dx = center_x - target_center_x;
            const double dy = center_y - target_center_y;
            if (dx * dx + dy * dy > radius_squared + 1e-12) {

                // 超出邻域半径的点跳过，这是因为双层 for 表示的是一个方形区域
                continue;
            }

            const std::size_t index = static_cast<std::size_t>(grid_y) *
                terrain_grid_width_cells_ + static_cast<std::size_t>(grid_x);
            const auto &cell = terrain_grid_[index];
            // 无效的支持点跳过
            if (!cell.support_valid || !std::isfinite(cell.support_z))
                continue;

            samples.emplace_back(center_x, center_y, cell.support_z);

            min_sample_z = std::min(
                min_sample_z, static_cast<double>(cell.support_z));
            max_sample_z = std::max(
                max_sample_z, static_cast<double>(cell.support_z));
        }
    }
    if (samples.size() < pca_min_cells_)
        return false;

    constexpr double kPi = 3.14159265358979323846;
    const double max_height_span =
        2.0 * pca_radius_ * std::tan(propagation_max_slope_deg_ * kPi / 180.0) +
        2.0 * propagation_height_tolerance_; // 最大高度跨度
    if (!std::isfinite(min_sample_z) || !std::isfinite(max_sample_z) ||
        max_sample_z - min_sample_z > max_height_span)
        return false;

    // 计算质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &sample : samples)
        centroid += sample;
    centroid /= static_cast<double>(samples.size());

    // 计算协方差矩阵
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &sample : samples) {

        const Eigen::Vector3d centered = sample - centroid;
        covariance.noalias() += centered * centered.transpose();
    }
    covariance /= static_cast<double>(samples.size());

    // 计算特征值和特征向量
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return false;

    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    if (!eigenvalues.allFinite() || eigenvalues[1] <= 1e-8)
        return false;

    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (!normal.allFinite())
        return false;
    if (normal.z() < 0.0)
        normal = -normal;
    if (normal.z() <= 1e-6)
        return false;

    // 计算残差
    const double residual = std::sqrt(std::max(0.0, eigenvalues[0]));
    if (!std::isfinite(residual) || residual > max_plane_residual_)
        return false;

    // 计算地面高度
    const double ground_z = centroid.z() -
        (normal.x() * (target_center_x - centroid.x()) +
         normal.y() * (target_center_y - centroid.y())) / normal.z();
    constexpr double kRadiansToDegrees = 180.0 / kPi;
    // 计算坡度
    const double slope_deg = std::atan2(
        std::hypot(normal.x(), normal.y()), normal.z()) * kRadiansToDegrees;
    if (!std::isfinite(ground_z) || !std::isfinite(slope_deg))
        return false;

    // 计算支持点数量得分
    const double count_score = std::min(
        1.0,
        static_cast<double>(samples.size()) /
            (2.0 * static_cast<double>(pca_min_cells_)));
    const double residual_score = std::max(
        0.0,
        1.0 - residual / max_plane_residual_);

    auto &cell = terrain_grid_[cell_index];
    cell.ground_valid = true;
    cell.ground_z = static_cast<float>(ground_z);
    // 存储地面法向量和坡度
    cell.ground_normal_x = static_cast<float>(normal.x());
    cell.ground_normal_y = static_cast<float>(normal.y());
    cell.ground_normal_z = static_cast<float>(normal.z());
    cell.slope_deg = static_cast<float>(slope_deg);
    cell.plane_residual = static_cast<float>(residual);
    cell.ground_confidence = static_cast<float>(count_score * residual_score);
    return true;
}

void TerrainMappingNode::publishDebugOutputs(
    const CloudMsg &source_cloud,
    const std::vector<TerrainPoint> &filtered_points,
    bool publish_maps
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

    if (!publish_maps)
        return;

    // 发布观测地图
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

    // 发布地面候选地图
    if (publish_ground_candidate_map_ && ground_candidate_map_publisher_) {

        nav_msgs::msg::OccupancyGrid ground_candidate_map;
        {
            std::lock_guard<std::mutex> lock(terrain_mutex_);
            if (!terrain_grid_initialized_)
                return;

            ground_candidate_map.header = source_cloud.header;
            ground_candidate_map.header.frame_id = odom_frame_;
            ground_candidate_map.info.resolution =
                static_cast<float>(resolution_);
            ground_candidate_map.info.width = static_cast<std::uint32_t>(
                terrain_grid_width_cells_);
            ground_candidate_map.info.height = static_cast<std::uint32_t>(
                terrain_grid_height_cells_);
            ground_candidate_map.info.origin.position.x = terrain_origin_x_;
            ground_candidate_map.info.origin.position.y = terrain_origin_y_;
            ground_candidate_map.info.origin.orientation.w = 1.0;
            ground_candidate_map.data.assign(
                terrain_grid_.size(),
                static_cast<std::int8_t>(-1));
            for (std::size_t index = 0U;
                 index < terrain_grid_.size();
                 ++ index) {

                if (terrain_grid_[index].support_seed)
                    ground_candidate_map.data[index] = 100;
                else if (terrain_grid_[index].support_valid)
                    ground_candidate_map.data[index] = 0;
            }
        }
        ground_candidate_map_publisher_->publish(ground_candidate_map);
    }

    // 发布地面点云
    if (publish_ground_cloud_ && ground_cloud_publisher_) {

        std::vector<TerrainPoint> ground_points;
        {
            std::lock_guard<std::mutex> lock(terrain_mutex_);
            if (!terrain_grid_initialized_)
                return;

            ground_points.reserve(terrain_grid_.size());
            for (std::size_t index = 0U; index < terrain_grid_.size(); ++ index) {

                const auto &cell = terrain_grid_[index];
                if (!cell.ground_valid)
                    continue;

                const std::size_t grid_x = index % terrain_grid_width_cells_;
                const std::size_t grid_y = index / terrain_grid_width_cells_;
                ground_points.push_back({
                    static_cast<float>(terrain_origin_x_ +
                        (static_cast<double>(grid_x) + 0.5) * resolution_),
                    static_cast<float>(terrain_origin_y_ +
                        (static_cast<double>(grid_y) + 0.5) * resolution_),
                    cell.ground_z});
            }
        }

        CloudMsg ground_cloud;
        ground_cloud.header = source_cloud.header;
        ground_cloud.header.frame_id = odom_frame_;
        sensor_msgs::PointCloud2Modifier modifier(ground_cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(ground_points.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(ground_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ground_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ground_cloud, "z");
        for (const auto &point : ground_points) {

            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            ++ iter_x;
            ++ iter_y;
            ++ iter_z;
        }
        ground_cloud.is_dense = true;
        ground_cloud_publisher_->publish(ground_cloud);
    }

    // 调试图直接保存 0~90 度的坡度，暂不映射为 Nav2 代价。
    if (publish_slope_map_ && slope_map_publisher_) {

        nav_msgs::msg::OccupancyGrid slope_map;
        {
            std::lock_guard<std::mutex> lock(terrain_mutex_);
            if (!terrain_grid_initialized_)
                return;

            slope_map.header = source_cloud.header;
            slope_map.header.frame_id = odom_frame_;
            slope_map.info.resolution = static_cast<float>(resolution_);
            slope_map.info.width = static_cast<std::uint32_t>(
                terrain_grid_width_cells_);
            slope_map.info.height = static_cast<std::uint32_t>(
                terrain_grid_height_cells_);
            slope_map.info.origin.position.x = terrain_origin_x_;
            slope_map.info.origin.position.y = terrain_origin_y_;
            slope_map.info.origin.orientation.w = 1.0;
            slope_map.data.assign(
                terrain_grid_.size(),
                static_cast<std::int8_t>(-1));
            for (std::size_t index = 0U; index < terrain_grid_.size(); ++ index) {

                const auto &cell = terrain_grid_[index];
                if (!cell.ground_valid)
                    continue;

                const long rounded_slope = std::lround(cell.slope_deg);
                slope_map.data[index] = static_cast<std::int8_t>(
                    std::clamp(rounded_slope, 0L, 90L));
            }
        }
        slope_map_publisher_->publish(slope_map);
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
