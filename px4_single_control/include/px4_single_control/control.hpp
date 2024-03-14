#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <functional>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "frame_transforms/frame_transforms.hpp"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "px4_topic/px4_topic.hpp"

using namespace std::chrono_literals;

class Control {
    public:
        using OffCtrlModeTy = px4_msgs::msg::OffboardControlMode;
        using VehCmdTy = px4_msgs::msg::VehicleCommand;
        using TrajSetptTy = px4_msgs::msg::TrajectorySetpoint;
        using VehOdomTy = px4_msgs::msg::VehicleOdometry;
        using VehAttSetptTy = px4_msgs::msg::VehicleAttitudeSetpoint;

        Control(const std::string &node_name, const std::string &vehicle_name = "") :
            vehicle_name_(vehicle_name) {

                node = std::make_shared<rclcpp::Node>(node_name);

                offboard_control_mode_profile_.position = true;
                offboard_control_mode_profile_.velocity = false;
                offboard_control_mode_profile_.acceleration = false;
                offboard_control_mode_profile_.attitude = false;
                offboard_control_mode_profile_.body_rate = false;

                offboard_control_mode_publisher_ =
                    node->create_publisher<OffCtrlModeTy>(
                            *px4_topic::get_offboard_control_mode_topic(vehicle_name),
                            rclcpp::SystemDefaultsQoS());
                vehicle_command_publisher_ =
                    node->create_publisher<VehCmdTy>(
                            *px4_topic::get_vehicle_command_topic(vehicle_name),
                            rclcpp::SystemDefaultsQoS());
                trajectory_setpoint_publisher_ =
                    node->create_publisher<TrajSetptTy>(
                            *px4_topic::get_trajectory_setpoint_topic(vehicle_name),
                            rclcpp::SystemDefaultsQoS());

                traj_target_.position = {0.0, 0.0, -5.0};

                auto timer_cb = [this]() -> void {
                    this->traj_target_.timestamp =
                        this->node->get_clock()->now().nanoseconds() / 1000;

                    this->updateOffboardControlModeTimestamp();

                    offboard_control_mode_publisher_->publish(this->offboard_control_mode_profile_);
                    trajectory_setpoint_publisher_->publish(this->traj_target_);

                    RCLCPP_DEBUG(this->node->get_logger(),
                            "Offboard updated: Pose {%f, %f, %f}",
                            this->traj_target_.position[0], this->traj_target_.position[1],
                            this->traj_target_.position[2]);
                };

                heartbeat_timer_ = node->create_wall_timer(100ms, timer_cb);

                auto takeoff = [this]() -> void {
                    this->setOffboardMode();
                    this->arm();
                    this->oneoff_timer_->cancel();
                };
                // Delay for 5 seconds to takeoff and turn off the timer
                oneoff_timer_ = node->create_wall_timer(5s, takeoff);

                auto update_odom = [this](const VehOdomTy &msg) -> void {
                    this->vehicle_odometry_ = msg;
                };
                update_vehicle_odometry_subscription_ =
                    this->node->create_subscription<VehOdomTy>(
                            *px4_topic::get_vehicle_odometry_topic(vehicle_name),
                            rclcpp::SystemDefaultsQoS(), update_odom);

                auto update_trajectory_target = [this] (const TrajSetptTy &msg) -> void {
                    this->traj_target_ = msg;
                };
                update_trajecotry_target_subscription_ =
                    this->node->create_subscription<TrajSetptTy>(
                            *px4_topic::get_update_traject_target_topic(vehicle_name_),
                            rclcpp::SystemDefaultsQoS(), update_trajectory_target);
            }

        void updateOffboardControlModeTimestamp();
        void updateOffboardControlMode(const OffCtrlModeTy &rhs);

        void setOffboardMode();
        void arm();

        void setTrajSetpoint(float x, float y, float z);

        rclcpp::Node::SharedPtr node;

    private:
        const std::string vehicle_name_;

        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::TimerBase::SharedPtr oneoff_timer_;

        rclcpp::Publisher<OffCtrlModeTy>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<VehCmdTy>::SharedPtr vehicle_command_publisher_;
        OffCtrlModeTy offboard_control_mode_profile_;

        rclcpp::Publisher<TrajSetptTy>::SharedPtr trajectory_setpoint_publisher_;
        TrajSetptTy traj_target_;

        rclcpp::Subscription<VehOdomTy>::SharedPtr update_vehicle_odometry_subscription_;
        VehOdomTy vehicle_odometry_;

        rclcpp::Subscription<TrajSetptTy>::SharedPtr update_trajecotry_target_subscription_;
};

#endif
