#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "px4_commander/commander.hpp"

using namespace std::chrono_literals;

static void initialize_parameters(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter("frame", "/uranus/ground_zero");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto cmd_node = std::make_shared<rclcpp::Node>("commander");
    initialize_parameters(cmd_node);
    auto commander = Commander(cmd_node);

    rclcpp::spin(cmd_node);
    return 0;
}
