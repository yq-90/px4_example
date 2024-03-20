#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "px4_single_control/control.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto ctrl = Control("px4_single_control", 1);

    rclcpp::spin(ctrl.node);
    return 0;
}
