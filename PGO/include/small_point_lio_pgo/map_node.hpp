#ifndef SMALL_POINT_LIO_PGO__MAP_NODE_HPP
#define SMALL_POINT_LIO_PGO__MAP_NODE_HPP

#include <Eigen/Geometry>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

#include "small_point_lio_pgo/msg/key_frame.hpp"
#include "small_point_lio_pgo/msg/optimized_key_frames.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace small_point_lio_pgo {

    struct StoredKeyFrame {
        rclcpp::Time stamp;///< 关键帧时间。
        geometry_msgs::msg::Pose raw_pose;///< 原始 T_odom_body 位姿。
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;///< body 坐标系局部点云。
    };

    class MapNode : public rclcpp::Node {

    public:

        MapNode();
    private:

        void loadParams();

        /// 保存关键帧原始点云和 odom 位姿。
        void callbackKeyFrameMsg(
            small_point_lio_pgo::msg::KeyFrame::ConstSharedPtr msg
        );
        /// 接收全量 PGO 位姿快照并触发全局地图重建。
        void callbackOptimizedKeyFrames(
            small_point_lio_pgo::msg::OptimizedKeyFrames::ConstSharedPtr msg
        );
        void callbackSaveMap(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response
        );

        /// 根据 raw/optimized 位姿重新投影所有关键帧并发布地图。
        void rebuildMap(
            const rclcpp::Time &stamp
        );

        Eigen::Isometry3d poseToIsometry(
            const geometry_msgs::msg::Pose &pose
        ) const;

        pcl::PointCloud<pcl::PointXYZ>::Ptr normalizeCloudToBodyFrame(
            const small_point_lio_pgo::msg::KeyFrame &msg
        ) const;

        std::unordered_map<uint32_t, StoredKeyFrame> keyframes_;///< 已接收的关键帧数据库。
        std::unordered_map<uint32_t, geometry_msgs::msg::Pose> optimized_poses_;///< 最新 PGO 位姿快照。
        Eigen::Isometry3d pgo_correction_ = Eigen::Isometry3d::Identity();///< 尾部未优化关键帧的近似 map<-odom 修正。
        Eigen::Isometry3d T_body_lidar_ = Eigen::Isometry3d::Identity();
        bool has_pgo_correction_ = false;///< pgo_correction_ 是否有效。
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;///< 当前发布的全局地图视图。
        std::mutex map_mutex_;
        rclcpp::Subscription<small_point_lio_pgo::msg::KeyFrame>::SharedPtr sub_keyframe_;
        rclcpp::Subscription<small_point_lio_pgo::msg::OptimizedKeyFrames>::SharedPtr sub_optimized_keyframes_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_map_;

        double map_leaf_size_ = 0.1;
        std::string map_topic_ = "/global_map";
        std::string map_frame_ = "map";
        std::string odom_frame_ = "odom";
        std::string keyframe_topic_ = "/keyframe_msg";
        std::string optimized_keyframes_topic_ = "/optimized_keyframes";
        std::string save_map_service_name_ = "/save_map";
        std::string map_save_path_ = "/home/goose/small_point_lio_lc/global_map.pcd";
        std::string body_frame_ = "base_link";
        std::string lidar_frame_ = "livox_frame";
        bool rebuild_unoptimized_keyframes_with_approximation_ = true;
        bool apply_pgo_correction_to_unoptimized_keyframes_ = true;
        int optimized_keyframes_tail_exclusion_count_ = 0;
        int keyframe_rebuild_interval_ = 20;
        int keyframes_since_last_rebuild_ = 0;
    };
} // small_point_lio_pgo

#endif  // SMALL_POINT_LIO_PGO__MAP_NODE_HPP
