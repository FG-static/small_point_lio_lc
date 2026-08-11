/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "lidar_adapter/base_lidar.h"
#include "small_point_lio/small_point_lio.h"
#include "small_point_lio_interfaces/msg/local_tracking_map.hpp"
#include "small_point_lio_interfaces/msg/scan_to_map_correction.hpp"
#include "util/pointcloud_mapping.h"
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pch.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::unique_ptr<LidarAdapterBase> lidar_adapter;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher;
        /// 发布前端 packet-level scan-to-map correction 证据。
        std::shared_ptr<rclcpp::Publisher<
                small_point_lio_interfaces::msg::ScanToMapCorrection>>
                scan_to_map_correction_publisher;
        /// 接收后端重建的局部 tracking map。
        std::shared_ptr<rclcpp::Subscription<
                small_point_lio_interfaces::msg::LocalTrackingMap>>
                local_tracking_map_subscription;
        nav_msgs::msg::Path path_msg;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
                static_tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        common::Odometry last_odometry;
        std::unique_ptr<util::PointcloudMapping> pointcloud_mapping;

        bool estimator_pose_to_base_pose(
                const common::Pose3d &estimator_pose,
                const builtin_interfaces::msg::Time &stamp,
                const std::string &body_frame,
                const std::string &lidar_frame,
                geometry_msgs::msg::Pose &base_pose);

        /// 校验共享安装外参并发布唯一的 body -> LiDAR 静态 TF。
        void publish_body_lidar_static_transform(
                const std::string &body_frame,
                const std::string &lidar_frame,
                const std::vector<double> &translation,
                const std::vector<double> &rpy);

        static builtin_interfaces::msg::Time timestamp_to_msg(double timestamp);

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);
    };

}// namespace small_point_lio
