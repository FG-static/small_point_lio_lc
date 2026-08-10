#ifndef SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_
#define SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace small_point_lio_map_tools {

class TerrainMappingTestPeer;

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

constexpr std::size_t kTerrainMaxZLayers = 4U;

struct TerrainZLayer {
    float center_z{0.0F};
    std::uint32_t point_count{0U};
    float min_z{std::numeric_limits<float>::infinity()};
    float max_z{-std::numeric_limits<float>::infinity()};
};

// 保存原始高度统计和阶段三得到的连续支撑面候选。
struct TerrainCell {
    bool observed{false};
    bool support_valid{false}; // 可以被其他 cell 通过连续 z 传播过来
    bool support_seed{false}; // 作为种子传播
    std::uint32_t point_count{0U};
    float min_z{std::numeric_limits<float>::infinity()};
    float max_z{-std::numeric_limits<float>::infinity()};
    double z_sum{0.0};
    std::array<TerrainZLayer, kTerrainMaxZLayers> z_layers{};
    std::size_t z_layer_count{0U};
    float support_z{std::numeric_limits<float>::quiet_NaN()}; // 有效候选z
    std::int64_t support_update_stamp_ns{-1}; // 最近一次传播确认时间
    bool ground_valid{false};
    float ground_z{std::numeric_limits<float>::quiet_NaN()}; // 地面高度
    // 地面法向量
    float ground_normal_x{std::numeric_limits<float>::quiet_NaN()};
    float ground_normal_y{std::numeric_limits<float>::quiet_NaN()};
    float ground_normal_z{std::numeric_limits<float>::quiet_NaN()};
    // 地面坡度角度
    float slope_deg{std::numeric_limits<float>::quiet_NaN()};
    float plane_residual{std::numeric_limits<float>::quiet_NaN()};
    // 只是由邻域格数和拟合残差构成的质量分数，不是概率。
    float ground_confidence{0.0F};
};

struct TerrainPoint {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

class TerrainMappingNode final : public rclcpp::Node {
public:
    TerrainMappingNode();

private:
    friend class TerrainMappingTestPeer;
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    using OdomMsg = nav_msgs::msg::Odometry;

    void loadParams();

    // 输入回调和时间匹配接口。
    void onCloud(const CloudMsg::ConstSharedPtr msg);
    void onOdom(const OdomMsg::ConstSharedPtr msg);
    bool findNearestOdom(
        std::int64_t cloud_stamp_ns,
        OdomSample &matched_odom) const;
    void trimOdomBuffer(std::int64_t newest_stamp_ns);
    std::int64_t stampToNanoseconds(const rclcpp::Time &stamp) const;

    // 阶段二：局部滚动 XY 栅格和点云处理。
    bool processSynchronizedCloud(
        const CloudMsg &cloud,
        const OdomSample &matched_odom,
        std::vector<TerrainPoint> &filtered_points,
        bool *ground_support_updated = nullptr);
    void initializeTerrainGridLocked(double center_x, double center_y);
    void resetTerrainGrid();
    void recenterTerrainGridLocked(double robot_x, double robot_y);
    bool worldToGridLocked(
        double x,
        double y,
        std::size_t &cell_index) const;
    void insertTerrainPointLocked(const TerrainPoint &point);
    void updateZLayersLocked(TerrainCell &cell, float z);
    bool computeExpectedGroundPoint(
        const OdomSample &odom,
        TerrainPoint &expected_ground) const;
    void updateGroundSupportLocked(
        const OdomSample &odom,
        std::size_t &seed_count,
        std::size_t &support_count,
        TerrainPoint &expected_ground);
    std::size_t updateGroundFitsLocked();
    bool fitGroundPlaneAtCellLocked(std::size_t cell_index);
    bool selectLayerNearHeight(
        const TerrainCell &cell,
        float reference_z,
        float max_height_difference,
        float &selected_z) const;
    void publishDebugOutputs(
        const CloudMsg &source_cloud,
        const std::vector<TerrainPoint> &filtered_points,
        bool publish_maps);

    rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
    rclcpp::Publisher<CloudMsg>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        observed_map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        ground_candidate_map_publisher_;
    rclcpp::Publisher<CloudMsg>::SharedPtr ground_cloud_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        slope_map_publisher_;

    mutable std::mutex odom_mutex_;
    std::deque<OdomSample> odom_buffer_;

    mutable std::mutex terrain_mutex_;
    std::vector<TerrainCell> terrain_grid_;
    std::size_t terrain_grid_width_cells_{0U};
    std::size_t terrain_grid_height_cells_{0U};
    double terrain_origin_x_{0.0};
    double terrain_origin_y_{0.0};
    bool terrain_grid_initialized_{false};

    std::size_t max_odom_buffer_size_{200};
    std::int64_t max_sync_delay_ns_{50000000LL};

    double resolution_{0.10};
    double map_width_{20.0};
    double map_height_{20.0};
    double recenter_distance_{2.0};
    double self_filter_radius_{0.30};
    double min_point_range_{0.30};
    double z_layer_merge_threshold_{0.10};
    double base_to_ground_height_{0.0};
    double seed_radius_{0.80};
    double seed_height_tolerance_{0.15};
    std::size_t seed_min_cells_{3U};
    std::size_t ground_layer_min_points_{2U};
    double propagation_max_slope_deg_{25.0};
    double propagation_height_tolerance_{0.02};
    double max_ground_step_{0.10};
    double ground_update_rate_hz_{10.0};
    double ground_support_hold_time_sec_{0.50};
    double pca_radius_{0.30};
    std::size_t pca_min_cells_{8U};
    double max_plane_residual_{0.05};
    std::int64_t ground_update_period_ns_{100000000LL};
    std::int64_t ground_support_hold_time_ns_{500000000LL};
    std::int64_t last_ground_update_stamp_ns_{-1};
    bool publish_filtered_cloud_{true};
    bool publish_observed_map_{true};
    bool publish_ground_candidate_map_{true};
    bool publish_ground_cloud_{true};
    bool publish_slope_map_{true};

    std::string cloud_topic_{"/cloud_registered"};
    std::string odom_topic_{"/Odometry"};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
};
}  // namespace small_point_lio_map_tools

#endif  // SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_
