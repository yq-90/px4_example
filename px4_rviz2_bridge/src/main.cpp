#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "px4_rviz2_bridge/bridge.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto bridge = Bridge("px4_rviz2_bridge");

    rclcpp::spin(bridge.bridge_node);
    return 0;
}
