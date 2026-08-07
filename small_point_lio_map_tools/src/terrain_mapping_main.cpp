#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "small_point_lio_map_tools/terrain_mapping_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<small_point_lio_map_tools::TerrainMappingNode>()
    );
    rclcpp::shutdown();
    return 0;
}
