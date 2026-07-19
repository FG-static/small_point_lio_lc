#include "small_point_lio_pgo/local_map_feedback_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
            std::make_shared<small_point_lio_pgo::LocalMapFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}
