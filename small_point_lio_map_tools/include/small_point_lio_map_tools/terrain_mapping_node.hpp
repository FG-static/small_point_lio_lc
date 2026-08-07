#ifndef SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_
#define SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace small_point_lio_map_tools {

// 地形建图节点的最小运行骨架。
class TerrainMappingNode final : public rclcpp::Node {
public:
    TerrainMappingNode();

private:
    void loadParams();
};

}  // namespace small_point_lio_map_tools

#endif  // SMALL_POINT_LIO_MAP_TOOLS__TERRAIN_MAPPING_NODE_HPP_
