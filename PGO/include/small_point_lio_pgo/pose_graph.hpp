#ifndef SMALL_DLIO__POSE_GRAPH_HPP
#define SMALL_DLIO__POSE_GRAPH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace small_point_lio_pgo {

    enum class PoseGraphEdgeType {

        Odom,
        Loop
    };

    struct PoseGraphOptions {

        int max_iterations = 20;
        bool fix_first_node = true;
        // just used by fallback
        double odom_edge_weight = 100.0;
        double loop_edge_weight = 500.0;
    };

    struct PoseGraphNode {

        uint32_t id = 0;
        double stamp = 0.0;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        bool fixed = false;
    };

    struct PoseGraphEdge {

        uint32_t from_id = 0;
        uint32_t to_id = 0;
        PoseGraphEdgeType type = PoseGraphEdgeType::Odom;
        Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
        Eigen::Matrix<double, 6, 6> information =
            Eigen::Matrix<double, 6, 6>::Identity();
        double score = 0.0;
        bool robust_kernel_enabled = false;
        double robust_kernel_delta = 1.0;
    };

    struct PoseGraphOptimizationSummary {

        bool success = false;
        int iterations = 0; // LM 算法迭代次数
        int node_count = 0;
        int edge_count = 0;
        double initial_chi2 = 0.0; // 优化前后卡方
        double final_chi2 = 0.0;
        std::string message; // 优化状态
    };

    class PoseGraph {

    public:

        void configure(
            const PoseGraphOptions &options
        );

        void clear();

        bool empty() const;

        size_t nodeCount() const;

        size_t edgeCount() const;

        bool hasNode(
            uint32_t id
        ) const;

        bool addNode(
            const PoseGraphNode &node
        );

        bool addOdomEdge(
            uint32_t from_id,
            uint32_t to_id,
            const Eigen::Isometry3d &relative_pose,
            const Eigen::Matrix<double, 6, 6> &information
        );

        bool addLoopEdge(
            uint32_t from_id,
            uint32_t to_id,
            const Eigen::Isometry3d &relative_pose,
            const Eigen::Matrix<double, 6, 6> &information,
            double score,
            bool robust_kernel_enabled,
            double robust_kernel_delta
        );

        PoseGraphOptimizationSummary optimize();

        bool getPose(
            uint32_t id,
            Eigen::Isometry3d &pose
        ) const;

        const std::vector<PoseGraphNode> &nodes() const;

        const std::vector<PoseGraphEdge> &edges() const;

        const PoseGraphOptions &options() const;

    private:

        bool addEdge(
            PoseGraphEdge edge
        );

        PoseGraphOptions options_;
        std::vector<PoseGraphNode> nodes_;
        std::vector<PoseGraphEdge> edges_;
        std::unordered_map<uint32_t, size_t> node_index_;
        bool graph_dirty_ = false;
    };

} // namespace small_point_lio_pgo

#endif // SMALL_DLIO__POSE_GRAPH_HPP
