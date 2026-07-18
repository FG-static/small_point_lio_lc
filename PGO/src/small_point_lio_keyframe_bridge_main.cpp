#include "small_point_lio_pgo/small_point_lio_keyframe_bridge.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<
            small_point_lio_pgo::SmallPointLioKeyframeBridge
        >()
    );
    rclcpp::shutdown();
    return 0;
}
