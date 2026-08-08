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
        std::vector<TerrainPoint> & filtered_points
    ) {

        return node.processSynchronizedCloud(cloud, odom, filtered_points);
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

}  // namespace small_point_lio_map_tools
