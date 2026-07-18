#include "small_point_lio_pgo/pose_graph.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <cmath>
#include <memory>
#include <utility>

namespace small_point_lio_pgo {

    namespace {

        bool isFiniteIsometry(
            const Eigen::Isometry3d &pose
        ) {

            return pose.matrix().allFinite();
        }

        bool isFiniteInformation(
            const Eigen::Matrix<double, 6, 6> &information
        ) {

            return information.allFinite();
        }

        // 确保信息矩阵对称
        Eigen::Matrix<double, 6, 6> sanitizedInformation(
            const Eigen::Matrix<double, 6, 6> &information
        ) {

            return 0.5 * (information + information.transpose());
        }
    }

    void PoseGraph::configure(
        const PoseGraphOptions &options
    ) {

        options_ = options;
        if (options_.max_iterations <= 0)
            options_.max_iterations = 20;
        if (!std::isfinite(options_.odom_edge_weight) ||
            options_.odom_edge_weight <= 0.0)
            options_.odom_edge_weight = 100.0;
        if (!std::isfinite(options_.loop_edge_weight) ||
            options_.loop_edge_weight <= 0.0)
            options_.loop_edge_weight = 500.0;
    }

    void PoseGraph::clear() {

        nodes_.clear();
        edges_.clear();
        node_index_.clear();
        graph_dirty_ = false;
    }

    bool PoseGraph::empty() const {

        return nodes_.empty();
    }

    size_t PoseGraph::nodeCount() const {

        return nodes_.size();
    }

    size_t PoseGraph::edgeCount() const {

        return edges_.size();
    }

    bool PoseGraph::hasNode(
        const uint32_t id
    ) const {

        return node_index_.find(id) != node_index_.end();
    }

    bool PoseGraph::addNode(
        const PoseGraphNode &node
    ) {

        if (hasNode(node.id) || !isFiniteIsometry(node.pose))
            return false;

        PoseGraphNode stored = node;
        if (options_.fix_first_node && nodes_.empty())
            stored.fixed = true;

        node_index_[stored.id] = nodes_.size();
        nodes_.push_back(stored);
        graph_dirty_ = true;
        return true;
    }

    /**
     * @brief 添加里程计边
     * @param from_id 前一帧关键帧的 id
     * @param to_id 后一帧关键帧的 id
     * @param relative_pose 相对位姿( from 到 to 坐标系的变换)
     * @param information 信息矩阵(协方差矩阵的逆，表示测量的置信度)
     * @return bool 是否成功
     *
     * edge.score 的 score 表示 GICP 配准的质量分数
     */
    bool PoseGraph::addOdomEdge(
        const uint32_t from_id,
        const uint32_t to_id,
        const Eigen::Isometry3d &relative_pose,
        const Eigen::Matrix<double, 6, 6> &information
    ) {

        PoseGraphEdge edge;
        edge.from_id = from_id;
        edge.to_id = to_id;
        edge.type = PoseGraphEdgeType::Odom;
        edge.relative_pose = relative_pose;
        edge.information = information;
        edge.score = 0.0;
        return addEdge(std::move(edge));
    }

    // 回环帧的边的添加
    bool PoseGraph::addLoopEdge(
        const uint32_t from_id,
        const uint32_t to_id,
        const Eigen::Isometry3d &relative_pose,
        const Eigen::Matrix<double, 6, 6> &information,
        const double score,
        const bool robust_kernel_enabled,
        const double robust_kernel_delta
    ) {

        PoseGraphEdge edge;
        edge.from_id = from_id;
        edge.to_id = to_id;
        edge.type = PoseGraphEdgeType::Loop;
        edge.relative_pose = relative_pose;
        edge.information = information;
        edge.score = score;
        edge.robust_kernel_enabled = robust_kernel_enabled;
        edge.robust_kernel_delta = robust_kernel_delta;
        return addEdge(std::move(edge));
    }

    PoseGraphOptimizationSummary PoseGraph::optimize() {

        PoseGraphOptimizationSummary summary;
        summary.node_count = static_cast<int>(nodes_.size());
        summary.edge_count = static_cast<int>(edges_.size());

        if (nodes_.empty()) {

            summary.message = "pose graph has no nodes";
            return summary;
        }

        if (edges_.empty()) {

            summary.message = "pose graph has no edges";
            return summary;
        }

        // 1. 构造 g2o 优化器。当前图只有 SE3 pose 节点和 SE3 相对位姿边，
        // 这里选择 Levenberg-Marquardt + Eigen 线性求解器完成稀疏图优化。
        // 将稀疏矩阵分块
        using BlockSolverType =
            g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
        // 线性方程求解
        using LinearSolverType =
            g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        auto linear_solver = std::make_unique<LinearSolverType>();
        linear_solver->setBlockOrdering(false);

        auto block_solver =
            std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto algorithm =
            new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
        optimizer.setAlgorithm(algorithm);

        // 2. 将内部 PoseGraphNode 转成 g2o VertexSE3。固定首节点的逻辑
        // 已经在 addNode() 里写入 node.fixed，这里只按节点状态照搬。
        // 位姿来源（后端工作坐标系）：node.pose 由关键帧 odom pose 初始化，
        // 不是通过 TF 读取的 map pose；优化完成后它才成为 T_map_body。
        for (const auto &node : nodes_) {

            auto *vertex = new g2o::VertexSE3();
            vertex->setId(static_cast<int>(node.id));
            vertex->setEstimate(node.pose);
            vertex->setFixed(node.fixed);

            if (!optimizer.addVertex(vertex)) {

                delete vertex;
                summary.message = "failed to add g2o vertex";
                return summary;
            }
        }

        // 3. 将 odom/loop 边统一转成 g2o EdgeSE3。区别只在信息矩阵、
        // 分数记录和 loop edge 可选的 robust kernel，测量模型保持一致。
        for (const auto &edge : edges_) {

            auto *from_vertex =
                optimizer.vertex(static_cast<int>(edge.from_id));
            auto *to_vertex =
                optimizer.vertex(static_cast<int>(edge.to_id));
            if (!from_vertex || !to_vertex) {

                summary.message = "edge references missing g2o vertex";
                return summary;
            }

            auto *g2o_edge = new g2o::EdgeSE3();
            g2o_edge->setVertex(0, from_vertex);
            g2o_edge->setVertex(1, to_vertex);
            g2o_edge->setMeasurement(edge.relative_pose);
            g2o_edge->setInformation(edge.information);
            if (edge.type == PoseGraphEdgeType::Loop &&
                edge.robust_kernel_enabled) {

                // 只给 loop edge 挂鲁棒核；odom edge 保持普通二乘，
                // 避免削弱连续里程计约束。
                // Huber Kernel 就是 rho(r) =
                // 小残差：0.5 * r^2 二次损失，和普通 least squares一样; |r| <= delta
                // 大残差：delta * (|r| - 0.5 * delta) 线性损失; |r| > delta
                auto *robust_kernel = new g2o::RobustKernelHuber();
                robust_kernel->setDelta(edge.robust_kernel_delta);
                g2o_edge->setRobustKernel(robust_kernel);
            }

            if (!optimizer.addEdge(g2o_edge)) {

                delete g2o_edge;
                summary.message = "failed to add g2o edge";
                return summary;
            }
        }

        // 4. 记录优化前后 chi2，便于日志判断 PGO 是否真的改变了图。
        if (!optimizer.initializeOptimization()) {

            summary.message = "failed to initialize g2o optimization";
            return summary;
        }

        optimizer.computeActiveErrors();
        summary.initial_chi2 = optimizer.activeChi2();

        const int iterations = optimizer.optimize(options_.max_iterations);
        summary.iterations = iterations;

        optimizer.computeActiveErrors();
        summary.final_chi2 = optimizer.activeChi2();

        if (iterations <= 0) {

            summary.message = "g2o optimization did not run";
            return summary;
        }

        // 5. 把 g2o 优化结果 T_map_body 直接写回内部节点。这里读取的是
        // g2o vertex estimate，不查询 TF，也不会再乘一次 map->odom。
        // 后续 /optimized_path、
        // /optimized_keyframes 和 map_node 重建地图都读取这里的节点姿态。
        for (auto &node : nodes_) {

            const auto *vertex = dynamic_cast<const g2o::VertexSE3 *>(
                optimizer.vertex(static_cast<int>(node.id)));
            if (!vertex) {

                summary.message = "failed to read optimized g2o vertex";
                return summary;
            }

            const Eigen::Isometry3d optimized_pose = vertex->estimate();
            if (!isFiniteIsometry(optimized_pose)) {

                summary.message = "optimized pose is not finite";
                return summary;
            }

            node.pose = optimized_pose;
        }

        graph_dirty_ = false;
        summary.success = true;
        summary.message = "ok";
        return summary;
    }

    bool PoseGraph::getPose(
        const uint32_t id,
        Eigen::Isometry3d &pose
    ) const {

        const auto it = node_index_.find(id);
        if (it == node_index_.end())
            return false;

        pose = nodes_[it->second].pose;
        return true;
    }

    const std::vector<PoseGraphNode> &PoseGraph::nodes() const {

        return nodes_;
    }

    const std::vector<PoseGraphEdge> &PoseGraph::edges() const {

        return edges_;
    }

    const PoseGraphOptions &PoseGraph::options() const {

        return options_;
    }

    bool PoseGraph::addEdge(
        PoseGraphEdge edge
    ) {

        if (edge.from_id == edge.to_id)
            return false;
        if (!hasNode(edge.from_id) || !hasNode(edge.to_id))
            return false;
        if (!isFiniteIsometry(edge.relative_pose) ||
            !isFiniteInformation(edge.information))
            return false;
        if (edge.type == PoseGraphEdgeType::Loop &&
            !std::isfinite(edge.score))
            return false;
        if (edge.type == PoseGraphEdgeType::Loop &&
            edge.robust_kernel_enabled &&
            (!std::isfinite(edge.robust_kernel_delta) ||
             edge.robust_kernel_delta <= 0.0))
            return false;

        edge.information = sanitizedInformation(edge.information);
        edges_.push_back(std::move(edge));
        graph_dirty_ = true;
        return true;
    }

} // namespace small_point_lio_pgo
