#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "px4_commander/commander.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto commander = Commander("px4_commander");

    rclcpp::spin(commander.commander_node);
    return 0;
}
