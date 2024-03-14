#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <functional>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;

class Control {
    public:
        using OffCtrlModeTy = px4_msgs::msg::OffboardControlMode;
        using VehCmdTy = px4_msgs::msg::VehicleCommand;
        using TrajSetptTy = px4_msgs::msg::TrajectorySetpoint;

        Control(const std::string &name) {
            node = std::make_shared<rclcpp::Node>(name);

            offboard_control_mode_profile_.position = true;
            offboard_control_mode_profile_.velocity = false;
            offboard_control_mode_profile_.acceleration = false;
            offboard_control_mode_profile_.attitude = false;
            offboard_control_mode_profile_.body_rate = false;

            offboard_control_mode_publisher_ =
                node->create_publisher<OffCtrlModeTy>("/fmu/in/offboard_control_mode",
                        rclcpp::SystemDefaultsQoS());
            vehicle_command_publisher_ =
                node->create_publisher<VehCmdTy>("/fmu/in/vehicle_command",
                        rclcpp::SystemDefaultsQoS());
            trajectory_setpoint_publisher_ =
                node->create_publisher<TrajSetptTy>("/fmu/in/trajectory_setpoint",
                        rclcpp::SystemDefaultsQoS());

            auto timer_cb = [this]() -> void {
                TrajSetptTy traj_msg{};
                traj_msg.position = {0.0, 0.0, -5.0};
                traj_msg.yaw = -3.14; // [-PI:PI]
                traj_msg.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;

                this->updateOffboardControlModeTimestamp();
                this->offboard_control_mode_publisher_->publish(this->offboard_control_mode_profile_);
                RCLCPP_DEBUG(this->node->get_logger(), "Offboard updated");

                trajectory_setpoint_publisher_->publish(traj_msg);
            };

            timer_ = node->create_wall_timer(100ms, timer_cb);
        }

        void updateOffboardControlModeTimestamp();
        void updateOffboardControlMode(const OffCtrlModeTy &rhs);

        void setOffboardMode();
        void arm();

        rclcpp::Node::SharedPtr node;
    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<OffCtrlModeTy>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<VehCmdTy>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<TrajSetptTy>::SharedPtr trajectory_setpoint_publisher_;
        OffCtrlModeTy offboard_control_mode_profile_;
};

#endif
