#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "px4_single_control/control.hpp"

using namespace std::chrono_literals;

static void initialize_params(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter<int>("instance_id", 0);
    node->declare_parameter<double>("initial_height", 10.0);
    node->declare_parameter("commanding_frame", "base_link");
    node->declare_parameter("vehicle_prefix", "px4_");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<rclcpp::Node>("px4_single_control");
    initialize_params(control_node);
    auto ctrl = Control(control_node, true);

    rclcpp::spin(control_node);
    return 0;
}
