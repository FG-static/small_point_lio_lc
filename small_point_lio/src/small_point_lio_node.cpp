/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        bool save_pcd = declare_parameter<bool>("save_pcd");
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        bool local_map_feedback_enable = false;
        get_parameter("local_map_feedback_enable", local_map_feedback_enable);
        const std::string scan_to_map_correction_topic =
                declare_parameter<std::string>(
                        "scan_to_map_correction_topic",
                        "/pointlio_scan_to_map_correction");
        const std::string local_tracking_map_topic =
                declare_parameter<std::string>(
                        "local_tracking_map_topic",
                        "/local_tracking_map");
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
        path_publisher = create_publisher<nav_msgs::msg::Path>("/path", 1000);
        path_msg.header.frame_id = "odom";
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        if (save_pcd) {
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
        }
        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd, lidar_frame](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        res->success = false;
                        res->message = "pcd save is disabled";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
                        return;
                    }
                    res->success = true;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");
                    auto pointcloud_to_save = std::make_shared<std::vector<Eigen::Vector3f>>();
                    *pointcloud_to_save = pointcloud_mapping->get_points();
                    std::thread([pointcloud_to_save, lidar_frame]() {
                        io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", *pointcloud_to_save);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
                    }).detach();
                });
        small_point_lio->set_odometry_callback([this, lidar_frame](const common::Odometry &odometry) {
            last_odometry = odometry;

            const builtin_interfaces::msg::Time time_msg =
                    timestamp_to_msg(odometry.timestamp);

            common::Pose3d estimator_pose;
            estimator_pose.position = odometry.position;
            estimator_pose.orientation = odometry.orientation;
            geometry_msgs::msg::Pose base_pose;
            if (!estimator_pose_to_base_pose(
                        estimator_pose, time_msg, lidar_frame, base_pose)) {
                return;
            }

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = time_msg;
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = "base_link";
            transform_stamped.transform.translation.x = base_pose.position.x;
            transform_stamped.transform.translation.y = base_pose.position.y;
            transform_stamped.transform.translation.z = base_pose.position.z;
            transform_stamped.transform.rotation = base_pose.orientation;

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";
            odometry_msg.pose.pose = base_pose;

            // TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link
            // odometry_msg.twist.twist.linear.x = odometry.velocity.x();
            // odometry_msg.twist.twist.linear.y = odometry.velocity.y();
            // odometry_msg.twist.twist.linear.z = odometry.velocity.z();
            // odometry_msg.twist.twist.angular.x = odometry.angular_velocity.x();
            // odometry_msg.twist.twist.angular.y = odometry.angular_velocity.y();
            // odometry_msg.twist.twist.angular.z = odometry.angular_velocity.z();

            tf_broadcaster->sendTransform(transform_stamped);
            odometry_publisher->publish(odometry_msg);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = odometry_msg.header;
            pose_stamped.pose = odometry_msg.pose.pose;
            path_msg.header.stamp = time_msg;
            path_msg.poses.push_back(pose_stamped);
            path_publisher->publish(path_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd, lidar_frame](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_publisher->get_subscription_count() > 0) {
                builtin_interfaces::msg::Time time_msg;
                time_msg.sec = std::floor(last_odometry.timestamp);
                time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

                geometry_msgs::msg::TransformStamped lidar_frame_to_base_link_transform;
                try {
                    lidar_frame_to_base_link_transform = tf_buffer->lookupTransform("base_link", lidar_frame, time_msg);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform from %s to base_link: %s", lidar_frame.c_str(), ex.what());
                    return;
                }
                Eigen::Vector3f lidar_frame_to_base_link_T;
                lidar_frame_to_base_link_T << static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.x),
                        static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.y),
                        static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.z);
                Eigen::Matrix3f lidar_frame_to_base_link_R =
                        Eigen::Quaternionf(
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.w),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.x),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.y),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.z))
                                .toRotationMatrix();
                sensor_msgs::msg::PointCloud2 msg;
                msg.header.stamp = time_msg;
                msg.header.frame_id = "odom";
                msg.width = pointcloud.size();
                msg.height = 1;
                msg.fields.reserve(4);
                sensor_msgs::msg::PointField field;
                field.name = "x";
                field.offset = 0;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "y";
                field.offset = 4;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "z";
                field.offset = 8;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "intensity";
                field.offset = 12;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                msg.is_bigendian = false;
                msg.point_step = 16;
                msg.row_step = msg.width * msg.point_step;
                msg.data.resize(msg.row_step * msg.height);
                Eigen::Vector3f transformed_point;
                auto pointer = reinterpret_cast<float *>(msg.data.data());
                for (const auto &point: pointcloud) {
                    transformed_point = lidar_frame_to_base_link_R * point + lidar_frame_to_base_link_T;
                    *pointer = transformed_point.x();
                    ++pointer;
                    *pointer = transformed_point.y();
                    ++pointer;
                    *pointer = transformed_point.z();
                    ++pointer;
                    *pointer = 0;
                    ++pointer;
                }
                msg.is_dense = false;
                pointcloud_publisher->publish(msg);
            }
            if (save_pcd) {
                for (const auto &point: pointcloud) {
                    pointcloud_mapping->add_point(point);
                }
            }
        });
        if (local_map_feedback_enable) {
            scan_to_map_correction_publisher = create_publisher<
                    small_point_lio_interfaces::msg::ScanToMapCorrection>(
                    scan_to_map_correction_topic,
                    rclcpp::QoS(100).reliable());
            small_point_lio->set_scan_to_map_correction_callback(
                    [this, lidar_frame](
                            const common::ScanToMapCorrection &correction) {
                        small_point_lio_interfaces::msg::ScanToMapCorrection msg;
                        msg.header.stamp = timestamp_to_msg(correction.timestamp);
                        msg.header.frame_id = "odom";
                        msg.sequence = correction.sequence;
                        msg.tracking_map_version = correction.tracking_map_version;
                        if (!estimator_pose_to_base_pose(
                                    correction.packet_predicted_pose,
                                    msg.header.stamp,
                                    lidar_frame,
                                    msg.packet_predicted_pose) ||
                            !estimator_pose_to_base_pose(
                                    correction.epoch_predicted_pose,
                                    msg.header.stamp,
                                    lidar_frame,
                                    msg.epoch_predicted_pose) ||
                            !estimator_pose_to_base_pose(
                                    correction.corrected_pose,
                                    msg.header.stamp,
                                    lidar_frame,
                                    msg.corrected_pose)) {
                            return;
                        }
                        msg.attempted_point_updates =
                                correction.attempted_point_updates;
                        msg.accepted_point_updates =
                                correction.accepted_point_updates;
                        msg.residual_rms = correction.residual_rms;
                        msg.residual_max_abs = correction.residual_max_abs;
                        msg.active_map_source_correction_sequence =
                                correction.active_map_source_correction_sequence;
                        scan_to_map_correction_publisher->publish(msg);
                    });

            local_tracking_map_subscription = create_subscription<
                    small_point_lio_interfaces::msg::LocalTrackingMap>(
                    local_tracking_map_topic,
                    rclcpp::QoS(1).reliable(),
                    [this, lidar_frame](const small_point_lio_interfaces::msg::
                                   LocalTrackingMap::ConstSharedPtr msg) {
                        if (msg->header.frame_id != "odom" ||
                            msg->cloud.header.frame_id != "odom") {
                            RCLCPP_WARN(
                                    get_logger(),
                                    "Ignoring local tracking map outside odom frame");
                            return;
                        }

                        common::LocalTrackingMapUpdate update;
                        update.cutoff_timestamp =
                                static_cast<double>(msg->header.stamp.sec) +
                                static_cast<double>(msg->header.stamp.nanosec) *
                                        1e-9;
                        update.source_correction_sequence =
                                msg->source_correction_sequence;
                        update.source_tracking_map_version =
                                msg->source_tracking_map_version;
                        update.target_tracking_map_version =
                                msg->target_tracking_map_version;
                        update.pgo_graph_version = msg->pgo_graph_version;
                        update.anchor_candidate_id = msg->anchor_candidate_id;
                        update.points_odom.reserve(
                                static_cast<size_t>(msg->cloud.width) *
                                static_cast<size_t>(msg->cloud.height));

                        // /Odometry and /cloud_registered use the existing
                        // base-link odom convention, while the estimator's
                        // iVox stores points in its internal LiDAR-odom basis.
                        // Undo the same static basis transform used by the
                        // registered-cloud publisher before rebuilding iVox.
                        geometry_msgs::msg::TransformStamped lidar_from_base_msg;
                        try {
                            lidar_from_base_msg = tf_buffer->lookupTransform(
                                    lidar_frame,
                                    "base_link",
                                    msg->header.stamp);
                        } catch (const tf2::TransformException &error) {
                            RCLCPP_WARN(
                                    get_logger(),
                                    "Ignoring local tracking map without base-to-lidar transform: %s",
                                    error.what());
                            return;
                        }
                        const Eigen::Quaternionf lidar_from_base_quaternion(
                                static_cast<float>(
                                        lidar_from_base_msg.transform.rotation.w),
                                static_cast<float>(
                                        lidar_from_base_msg.transform.rotation.x),
                                static_cast<float>(
                                        lidar_from_base_msg.transform.rotation.y),
                                static_cast<float>(
                                        lidar_from_base_msg.transform.rotation.z));
                        if (!lidar_from_base_quaternion.coeffs().allFinite() ||
                            lidar_from_base_quaternion.norm() <= 1e-6F) {
                            RCLCPP_WARN(
                                    get_logger(),
                                    "Ignoring local tracking map with invalid base-to-lidar transform");
                            return;
                        }
                        const Eigen::Matrix3f lidar_from_base_rotation =
                                lidar_from_base_quaternion.normalized()
                                        .toRotationMatrix();
                        const Eigen::Vector3f lidar_from_base_translation(
                                static_cast<float>(
                                        lidar_from_base_msg.transform.translation.x),
                                static_cast<float>(
                                        lidar_from_base_msg.transform.translation.y),
                                static_cast<float>(
                                        lidar_from_base_msg.transform.translation.z));
                        try {
                            sensor_msgs::PointCloud2ConstIterator<float> iter_x(
                                    msg->cloud, "x");
                            sensor_msgs::PointCloud2ConstIterator<float> iter_y(
                                    msg->cloud, "y");
                            sensor_msgs::PointCloud2ConstIterator<float> iter_z(
                                    msg->cloud, "z");
                            for (; iter_x != iter_x.end();
                                 ++iter_x, ++iter_y, ++iter_z) {
                                const Eigen::Vector3f point_external_odom(
                                        *iter_x, *iter_y, *iter_z);
                                if (point_external_odom.allFinite()) {
                                    update.points_odom.push_back(
                                            lidar_from_base_rotation *
                                                    point_external_odom +
                                            lidar_from_base_translation);
                                }
                            }
                        } catch (const std::runtime_error &error) {
                            RCLCPP_WARN(
                                    get_logger(),
                                    "Ignoring malformed local tracking cloud: %s",
                                    error.what());
                            return;
                        }
                        // 推送到前端节点进行地图校验与重建
                        small_point_lio->queue_local_tracking_map(
                                std::move(update));
                    });

            RCLCPP_INFO(
                    get_logger(),
                    "Local-map feedback enabled: correction=%s map=%s",
                    scan_to_map_correction_topic.c_str(),
                    local_tracking_map_topic.c_str());
        }
        if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "livox_custom_msg requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "livox_pointcloud2") {
            lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
        } else if (lidar_type == "custom_mid360_driver") {
            lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

    bool SmallPointLioNode::estimator_pose_to_base_pose(
            const common::Pose3d &estimator_pose,
            const builtin_interfaces::msg::Time &stamp,
            const std::string &lidar_frame,
            geometry_msgs::msg::Pose &base_pose) {
        geometry_msgs::msg::TransformStamped lidar_from_base_msg;
        try {
            lidar_from_base_msg =
                    tf_buffer->lookupTransform(lidar_frame, "base_link", stamp);
        } catch (const tf2::TransformException &error) {
            RCLCPP_ERROR(
                    get_logger(),
                    "Failed to lookup transform from base_link to %s: %s",
                    lidar_frame.c_str(),
                    error.what());
            return false;
        }

        Eigen::Quaterniond orientation = estimator_pose.orientation;
        if (!estimator_pose.position.allFinite() ||
            !orientation.coeffs().allFinite() || orientation.norm() <= 1e-9) {
            return false;
        }
        orientation.normalize();

        tf2::Transform estimator_transform;
        estimator_transform.setOrigin(tf2::Vector3(
                estimator_pose.position.x(),
                estimator_pose.position.y(),
                estimator_pose.position.z()));
        estimator_transform.setRotation(tf2::Quaternion(
                orientation.x(),
                orientation.y(),
                orientation.z(),
                orientation.w()));
        tf2::Transform lidar_from_base;
        tf2::fromMsg(lidar_from_base_msg.transform, lidar_from_base);

        // Keep exactly the same estimator-to-base convention used by the
        // published /Odometry path so correction and keyframe poses share a
        // coordinate contract.
        const tf2::Transform odom_from_base =
                lidar_from_base.inverse() * estimator_transform *
                lidar_from_base;
        const geometry_msgs::msg::Transform transform_msg =
                tf2::toMsg(odom_from_base);
        base_pose.position.x = transform_msg.translation.x;
        base_pose.position.y = transform_msg.translation.y;
        base_pose.position.z = transform_msg.translation.z;
        base_pose.orientation = transform_msg.rotation;
        return true;
    }

    builtin_interfaces::msg::Time SmallPointLioNode::timestamp_to_msg(
            const double timestamp) {
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<int32_t>(std::floor(timestamp));
        result.nanosec = static_cast<uint32_t>(
                std::max(0.0, timestamp - static_cast<double>(result.sec)) *
                1e9);
        return result;
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
