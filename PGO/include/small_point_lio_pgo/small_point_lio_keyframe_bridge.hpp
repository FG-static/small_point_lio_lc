#ifndef SMALL_POINT_LIO_PGO__SMALL_POINT_LIO_KEYFRAME_BRIDGE_HPP
#define SMALL_POINT_LIO_PGO__SMALL_POINT_LIO_KEYFRAME_BRIDGE_HPP

#include "small_point_lio_pgo/msg/key_frame.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace small_point_lio_pgo {

    class SmallPointLioKeyframeBridge : public rclcpp::Node {

    public:

        SmallPointLioKeyframeBridge();

    private:

        void loadParams();

        void callbackOdom(
            nav_msgs::msg::Odometry::ConstSharedPtr msg
        );
        void callbackCloud(
            sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
        );
        void processCloud(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg
        );

        bool findNearestOdom(
            const builtin_interfaces::msg::Time &stamp,
            nav_msgs::msg::Odometry &odom,
            double &delay_sec
        );
        void publishCandidate(
            const geometry_msgs::msg::Pose &pose,
            const sensor_msgs::msg::PointCloud2 &cloud,
            double odom_delay_sec
        );

        static std::int64_t stampToNanoseconds(
            const builtin_interfaces::msg::Time &stamp
        );
        static bool poseFinite(
            const geometry_msgs::msg::Pose &pose
        );
        static std::size_t pointCount(
            const sensor_msgs::msg::PointCloud2 &cloud
        );

        std::mutex odom_mutex_;
        // 以下两个队列只用于跨 topic 的时间同步，不做多帧点云累积。
        std::deque<nav_msgs::msg::Odometry> odom_buffer_;
        std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr>
            pending_clouds_;
        std::int64_t last_odom_stamp_ns_ = 0;
        bool has_last_odom_stamp_ = false;
        std::uint32_t next_candidate_id_ = 0;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
        rclcpp::Publisher<small_point_lio_pgo::msg::KeyFrame>::SharedPtr
            pub_keyframe_;

        std::string odom_topic_ = "/Odometry";
        std::string cloud_topic_ = "/cloud_registered";
        std::string keyframe_topic_ = "/keyframe_candidates";
        std::string odom_frame_ = "odom";
        std::string body_frame_ = "base_link";
        double max_cloud_delay_sec_ = 0.05;
        double odom_buffer_duration_sec_ = 1.0;
        int min_cloud_points_ = 10;
    };

}  // namespace small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__SMALL_POINT_LIO_KEYFRAME_BRIDGE_HPP
