#include "small_point_lio_map_tools/terrain_mapping_node.hpp"

namespace small_point_lio_map_tools {

TerrainMappingNode::TerrainMappingNode()
    : rclcpp::Node("terrain_mapping_node")
{
    loadParams();
}

void TerrainMappingNode::loadParams()
{
    // 阶段 1 在此声明输入输出参数。
}

}  // namespace small_point_lio_map_tools
