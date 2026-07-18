#include "small_point_lio_pgo/loop_detector_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<small_point_lio_pgo::LoopDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
