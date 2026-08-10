#include "small_point_lio_map_tools/terrain_mapping_node.hpp"

#include <gtest/gtest.h> // GoogleTest GTest

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace small_point_lio_map_tools {

class TerrainMappingTestPeer {

public:

    static bool processCloud(
        TerrainMappingNode & node,
        const sensor_msgs::msg::PointCloud2 & cloud,
        const OdomSample & odom,
        std::vector<TerrainPoint> & filtered_points,
        bool * ground_support_updated = nullptr
    ) {

        return node.processSynchronizedCloud(
            cloud,
            odom,
            filtered_points,
            ground_support_updated);
    }

    static std::optional<TerrainCell> cellAt(
        TerrainMappingNode & node,
        const double x,
        const double y
    ) {

        std::lock_guard<std::mutex> lock(node.terrain_mutex_);
        std::size_t cell_index = 0U;
        if (!node.worldToGridLocked(x, y, cell_index))
            return std::nullopt;
        return node.terrain_grid_.at(cell_index);
    }

    static std::size_t observedCellCount(TerrainMappingNode & node) {

        std::lock_guard<std::mutex> lock(node.terrain_mutex_);
        return static_cast<std::size_t>(std::count_if(
            node.terrain_grid_.cbegin(),
            node.terrain_grid_.cend(),
            [](const TerrainCell & cell) {
                return cell.observed;
            }));
    }

    static std::size_t supportCellCount(TerrainMappingNode & node) {

        std::lock_guard<std::mutex> lock(node.terrain_mutex_);
        return static_cast<std::size_t>(std::count_if(
            node.terrain_grid_.cbegin(),
            node.terrain_grid_.cend(),
            [](const TerrainCell & cell) {
                return cell.support_valid;
            }));
    }

    static void configureGroundSupport(TerrainMappingNode & node) {

        node.base_to_ground_height_ = 0.30;
        node.seed_radius_ = 0.80;
        node.seed_height_tolerance_ = 0.15;
        node.seed_min_cells_ = 3U;
        node.ground_layer_min_points_ = 1U;
        node.propagation_max_slope_deg_ = 25.0;
        node.propagation_height_tolerance_ = 0.02;
        node.max_ground_step_ = 0.10;
        node.ground_update_period_ns_ = 100000000LL;
        node.ground_support_hold_time_ns_ = 500000000LL;
    }

    static bool gridInitialized(const TerrainMappingNode & node) {

        std::lock_guard<std::mutex> lock(node.terrain_mutex_);
        return node.terrain_grid_initialized_;
    }

    static double originX(const TerrainMappingNode & node) {

        std::lock_guard<std::mutex> lock(node.terrain_mutex_);
        return node.terrain_origin_x_;
    }

    static void onOdom(
        TerrainMappingNode & node,
        const std::int32_t seconds
    ) {

        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        msg->header.frame_id = "odom";
        msg->header.stamp.sec = seconds;
        msg->child_frame_id = "base_link";
        msg->pose.pose.orientation.w = 1.0;
        node.onOdom(msg);
    }
};

namespace {

void setStamp(
    builtin_interfaces::msg::Time & stamp,
    const std::int64_t stamp_ns
) {

    stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
}

sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<TerrainPoint> & points,
    const std::int64_t stamp_ns = 1000000000LL
) {

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "odom";
    setStamp(cloud.header.stamp, stamp_ns);

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto &point : points) {

        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        ++ iter_x;
        ++ iter_y;
        ++ iter_z;
    }
    cloud.is_dense = false;
    return cloud;
}

OdomSample makeOdom(
    const double x,
    const double y,
    const std::int64_t stamp_ns = 1000000000LL
) {

    OdomSample odom;
    odom.stamp_ns = stamp_ns;
    odom.position_x = x;
    odom.position_y = y;
    odom.position_z = 0.0;
    odom.orientation_w = 1.0;
    return odom;
}

}  // namespace

class TerrainMappingNodeTest : public ::testing::Test {

protected:

    static void SetUpTestSuite() {

        if (!rclcpp::ok()) {

            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {

        if (rclcpp::ok()) {

            rclcpp::shutdown();
        }
    }
};

TEST_F(TerrainMappingNodeTest, FiltersPointsAndStoresZStatistics) {

    auto node = std::make_shared<TerrainMappingNode>();
    const std::vector<TerrainPoint> points{
        {1.00F, 0.00F, 0.20F},
        {1.04F, 0.00F, 0.25F},
        {0.10F, 0.10F, 0.00F},
        {30.00F, 0.00F, 0.00F},
        {std::numeric_limits<float>::quiet_NaN(), 0.00F, 0.00F},
    };

    std::vector<TerrainPoint> filtered_points;
    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud(points),
        makeOdom(0.0, 0.0),
        filtered_points)
    );

    ASSERT_EQ(filtered_points.size(), 2U);
    EXPECT_TRUE(TerrainMappingTestPeer::gridInitialized(*node));
    EXPECT_EQ(TerrainMappingTestPeer::observedCellCount(*node), 1U);

    const auto cell = TerrainMappingTestPeer::cellAt(*node, 1.0, 0.0);
    ASSERT_TRUE(cell.has_value());
    EXPECT_TRUE(cell->observed);
    EXPECT_EQ(cell->point_count, 2U);
    EXPECT_EQ(cell->z_layer_count, 1U);
    EXPECT_NEAR(cell->z_sum, 0.45, 1e-6);
    EXPECT_NEAR(cell->min_z, 0.20F, 1e-6F);
    EXPECT_NEAR(cell->max_z, 0.25F, 1e-6F);
}

TEST_F(TerrainMappingNodeTest, RollsMapAndPreservesOverlap) {

    auto node = std::make_shared<TerrainMappingNode>();
    std::vector<TerrainPoint> filtered_points;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{1.0F, 0.0F, 0.0F}}),
        makeOdom(0.0, 0.0),
        filtered_points));
    EXPECT_NEAR(TerrainMappingTestPeer::originX(*node), -10.0, 1e-6);

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{4.0F, 0.0F, 0.0F}}),
        makeOdom(3.0, 0.0),
        filtered_points));
    EXPECT_NEAR(TerrainMappingTestPeer::originX(*node), -7.0, 1e-6);
    const auto preserved_cell =
        TerrainMappingTestPeer::cellAt(*node, 1.0, 0.0);
    ASSERT_TRUE(preserved_cell.has_value());
    EXPECT_TRUE(preserved_cell->observed);
    EXPECT_EQ(TerrainMappingTestPeer::observedCellCount(*node), 2U);

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{31.0F, 0.0F, 0.0F}}),
        makeOdom(30.0, 0.0),
        filtered_points));
    EXPECT_NEAR(TerrainMappingTestPeer::originX(*node), 20.0, 1e-6);
    EXPECT_FALSE(TerrainMappingTestPeer::cellAt(*node, 1.0, 0.0).has_value());
    EXPECT_EQ(TerrainMappingTestPeer::observedCellCount(*node), 1U);
}

TEST_F(TerrainMappingNodeTest, TimeResetClearsTerrainMap) {

    auto node = std::make_shared<TerrainMappingNode>();
    std::vector<TerrainPoint> filtered_points;

    TerrainMappingTestPeer::onOdom(*node, 2);
    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{1.0F, 0.0F, 0.0F}}, 2000000000LL),
        makeOdom(0.0, 0.0, 2000000000LL),
        filtered_points));
    ASSERT_TRUE(TerrainMappingTestPeer::gridInitialized(*node));

    TerrainMappingTestPeer::onOdom(*node, 1);
    EXPECT_FALSE(TerrainMappingTestPeer::gridInitialized(*node));
    EXPECT_EQ(TerrainMappingTestPeer::observedCellCount(*node), 0U);
}

// 测试从机器人种子传播平坦地面
TEST_F(TerrainMappingNodeTest, PropagatesFlatGroundFromRobotSeeds) {

    auto node = std::make_shared<TerrainMappingNode>();
    TerrainMappingTestPeer::configureGroundSupport(*node);
    const std::vector<TerrainPoint> points{
        {0.45F, 0.05F, 0.0F},
        {0.55F, 0.05F, 0.0F},
        {0.65F, 0.05F, 0.0F},
        {0.75F, 0.05F, 0.0F},
        {0.85F, 0.05F, 0.0F},
        {0.95F, 0.05F, 0.0F},
        {1.05F, 0.05F, 0.0F},
        {1.15F, 0.05F, 0.0F},
    };
    auto odom = makeOdom(0.0, 0.0);
    odom.position_z = 0.30;
    std::vector<TerrainPoint> filtered_points;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node, makeCloud(points), odom, filtered_points));

    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), points.size());
    const auto last_cell = TerrainMappingTestPeer::cellAt(*node, 1.15, 0.05);
    ASSERT_TRUE(last_cell.has_value());
    EXPECT_TRUE(last_cell->support_valid);
    EXPECT_FALSE(last_cell->support_seed);
    EXPECT_NEAR(last_cell->support_z, 0.0F, 1e-6F);
}

// 测试传播 gentle slope
TEST_F(TerrainMappingNodeTest, PropagatesGentleSlope) {

    auto node = std::make_shared<TerrainMappingNode>();
    TerrainMappingTestPeer::configureGroundSupport(*node);
    constexpr float kSlopeTangent = 0.176327F;
    std::vector<TerrainPoint> points;
    for (int index = 0; index < 9; ++ index) {

        const float x = 0.45F + 0.10F * static_cast<float>(index);
        points.push_back({x, 0.05F, (x - 0.45F) * kSlopeTangent});
    }
    auto odom = makeOdom(0.0, 0.0);
    odom.position_z = 0.30;
    std::vector<TerrainPoint> filtered_points;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node, makeCloud(points), odom, filtered_points));

    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), points.size());
    const auto last_cell = TerrainMappingTestPeer::cellAt(*node, 1.25, 0.05);
    ASSERT_TRUE(last_cell.has_value());
    EXPECT_TRUE(last_cell->support_valid);
    EXPECT_NEAR(last_cell->support_z, 0.80F * kSlopeTangent, 1e-5F);
}

// 测试拒绝断开的盒子顶部
TEST_F(TerrainMappingNodeTest, RejectsDisconnectedBoxTop) {

    auto node = std::make_shared<TerrainMappingNode>();
    TerrainMappingTestPeer::configureGroundSupport(*node);
    const std::vector<TerrainPoint> points{
        {0.45F, 0.05F, 0.0F},
        {0.55F, 0.05F, 0.0F},
        {0.65F, 0.05F, 0.0F},
        {0.75F, 0.05F, 0.0F},
        {0.85F, 0.05F, 0.30F},
        {0.95F, 0.05F, 0.30F},
        {1.05F, 0.05F, 0.30F},
    };
    auto odom = makeOdom(0.0, 0.0);
    odom.position_z = 0.30;
    std::vector<TerrainPoint> filtered_points;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node, makeCloud(points), odom, filtered_points));

    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 4U);
    const auto box_cell = TerrainMappingTestPeer::cellAt(*node, 0.85, 0.05);
    ASSERT_TRUE(box_cell.has_value());
    EXPECT_TRUE(box_cell->observed);
    EXPECT_FALSE(box_cell->support_valid);
}

// 测试保持地面未知，没有机器人种子
TEST_F(TerrainMappingNodeTest, KeepsGroundUnknownWithoutRobotSeed) {

    auto node = std::make_shared<TerrainMappingNode>();
    TerrainMappingTestPeer::configureGroundSupport(*node);
    auto odom = makeOdom(0.0, 0.0);
    odom.position_z = 0.30;
    std::vector<TerrainPoint> filtered_points;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{2.05F, 0.05F, 0.0F}}),
        odom,
        filtered_points));

    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 0U);
    const auto cell = TerrainMappingTestPeer::cellAt(*node, 2.05, 0.05);
    ASSERT_TRUE(cell.has_value());
    EXPECT_TRUE(cell->observed);
    EXPECT_FALSE(cell->support_valid);
}

TEST_F(TerrainMappingNodeTest, LimitsGroundUpdatesToConfiguredRate) {

    auto node = std::make_shared<TerrainMappingNode>();
    TerrainMappingTestPeer::configureGroundSupport(*node);
    const std::vector<TerrainPoint> ground_points{
        {0.45F, 0.05F, 0.0F},
        {0.55F, 0.05F, 0.0F},
        {0.65F, 0.05F, 0.0F},
        {0.75F, 0.05F, 0.0F},
    };
    auto first_odom = makeOdom(0.0, 0.0, 1000000000LL);
    first_odom.position_z = 0.30;
    std::vector<TerrainPoint> filtered_points;
    bool ground_support_updated = false;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud(ground_points, first_odom.stamp_ns),
        first_odom,
        filtered_points,
        &ground_support_updated));
    EXPECT_TRUE(ground_support_updated);
    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 4U);

    auto early_odom = makeOdom(3.0, 0.0, 1050000000LL);
    early_odom.position_z = 0.30;
    ground_support_updated = true;
    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{4.0F, 0.0F, 1.0F}}, early_odom.stamp_ns),
        early_odom,
        filtered_points,
        &ground_support_updated));

    EXPECT_FALSE(ground_support_updated);
    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 4U);
}

TEST_F(TerrainMappingNodeTest, HoldsGroundAcrossBriefSeedFailure) {

    auto node = std::make_shared<TerrainMappingNode>();
    TerrainMappingTestPeer::configureGroundSupport(*node);
    const std::vector<TerrainPoint> ground_points{
        {0.45F, 0.05F, 0.0F},
        {0.55F, 0.05F, 0.0F},
        {0.65F, 0.05F, 0.0F},
        {0.75F, 0.05F, 0.0F},
    };
    auto first_odom = makeOdom(0.0, 0.0, 1000000000LL);
    first_odom.position_z = 0.30;
    std::vector<TerrainPoint> filtered_points;

    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud(ground_points, first_odom.stamp_ns),
        first_odom,
        filtered_points));
    ASSERT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 4U);

    auto failed_odom = makeOdom(3.0, 0.0, 1100000000LL);
    failed_odom.position_z = 0.30;
    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{4.0F, 0.0F, 1.0F}}, failed_odom.stamp_ns),
        failed_odom,
        filtered_points));
    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 4U);

    auto expired_odom = makeOdom(3.0, 0.0, 1700000000LL);
    expired_odom.position_z = 0.30;
    ASSERT_TRUE(TerrainMappingTestPeer::processCloud(
        *node,
        makeCloud({{4.0F, 0.0F, 1.0F}}, expired_odom.stamp_ns),
        expired_odom,
        filtered_points));
    EXPECT_EQ(TerrainMappingTestPeer::supportCellCount(*node), 0U);
}

}  // namespace small_point_lio_map_tools
