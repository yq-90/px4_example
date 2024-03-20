#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <functional>
#include <string>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "frame_transforms/frame_transforms.hpp"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "px4_single_control/topic.hpp"

using namespace std::chrono_literals;

#define TOPIC_INITIALIZER(TOPIC_NAME) \
    ((!instance_id && !need_prefix) ? \
     get_##TOPIC_NAME##_topic() : \
     (vehicle_name_ + get_##TOPIC_NAME##_topic()))

class Control {
    public:
        using OffCtrlModeTy = px4_msgs::msg::OffboardControlMode;
        using VehCmdTy = px4_msgs::msg::VehicleCommand;
        using TrajSetptTy = px4_msgs::msg::TrajectorySetpoint;
        using VehOdomTy = px4_msgs::msg::VehicleOdometry;
        using VehAttSetptTy = px4_msgs::msg::VehicleAttitudeSetpoint;
        using PoseStampedTy = geometry_msgs::msg::PoseStamped;
        using TFStampedTy = geometry_msgs::msg::TransformStamped;
        using GoalTfTy = TFStampedTy;

        Control(const std::string &node_name,
                const uint16_t instance_id = 0,
                bool need_prefix = false,
                const std::string &vehicle_prefix = "px4_",
                const std::string &commanding_frame = "base_link",
                float default_height = 10) :
            ros_instance_id_(instance_id), mavlink_system_id_(1 + instance_id),
            commanding_frame_(commanding_frame),
            vehicle_name_(vehicle_prefix + std::to_string(instance_id)),
            frame_(vehicle_name_ + "_link"),
            height_(default_height),
            offboard_control_mode_topic_(TOPIC_INITIALIZER(offboard_control_mode)),
            vehicle_command_topic_(TOPIC_INITIALIZER(vehicle_command)),
            trajectory_setpoint_topic_(TOPIC_INITIALIZER(trajectory_setpoint)),
            vehicle_odometry_topic_(TOPIC_INITIALIZER(vehicle_odometry)),
            update_trajectory_target_topic_(TOPIC_INITIALIZER(update_trajectory_target)) {

                node = std::make_shared<rclcpp::Node>(node_name);

                offboard_control_mode_profile_.position = true;
                offboard_control_mode_profile_.velocity = false;
                offboard_control_mode_profile_.acceleration = false;
                offboard_control_mode_profile_.attitude = false;
                offboard_control_mode_profile_.body_rate = false;

                offboard_control_mode_publisher_ =
                    node->create_publisher<OffCtrlModeTy>(
                            offboard_control_mode_topic_,
                            rclcpp::SystemDefaultsQoS());
                vehicle_command_publisher_ =
                    node->create_publisher<VehCmdTy>(
                            vehicle_command_topic_,
                            rclcpp::SystemDefaultsQoS());
                trajectory_setpoint_publisher_ =
                    node->create_publisher<TrajSetptTy>(
                            trajectory_setpoint_topic_,
                            rclcpp::SystemDefaultsQoS());

                traj_target_.position = {0.0, 0.0, -height_};

                auto heartbeat_cb = [this]() -> void {
                    this->updateOffboardControlModeTimestamp();
                    offboard_control_mode_publisher_->publish(
                            this->offboard_control_mode_profile_);

                    this->traj_target_.timestamp =
                        this->node->get_clock()->now().nanoseconds() / 1000;
                    trajectory_setpoint_publisher_->publish(this->traj_target_);

                    RCLCPP_DEBUG(this->node->get_logger(),
                            "Offboard updated: Pose {%f, %f, %f}",
                            this->traj_target_.position[0], this->traj_target_.position[1],
                            this->traj_target_.position[2]);
                };

                heartbeat_timer_ = node->create_wall_timer(200ms, heartbeat_cb);

                update_pose_publisher_ =
                    node->create_publisher<PoseStampedTy>(vehicle_name_ + "/uranus/local_position",
                            rclcpp::SensorDataQoS());
                this->ros_pose_.header.frame_id = "map";
                auto update_odom = [this](const VehOdomTy &msg) -> void {
                    using namespace frame_transforms;

                    Eigen::Vector3d pose_ned({msg.position[0], msg.position[1], msg.position[2]});
                    Eigen::Quaterniond q_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);

                    auto pose_enu = ned_to_enu_local_frame(pose_ned);
                    auto q_enu = px4_to_ros_orientation(q_ned);

                    this->ros_pose_.header.stamp = this->node->get_clock()->now();
                    this->ros_pose_.pose.position.x = pose_enu[0];
                    this->ros_pose_.pose.position.y = pose_enu[1];
                    this->ros_pose_.pose.position.z = pose_enu[2];

                    this->ros_pose_.pose.orientation.w = q_enu.w();
                    this->ros_pose_.pose.orientation.x = q_enu.x();
                    this->ros_pose_.pose.orientation.y = q_enu.y();
                    this->ros_pose_.pose.orientation.z = q_enu.z();

                    this->update_pose_publisher_->publish(this->ros_pose_);
                };
                update_vehicle_odometry_subscription_ =
                    this->node->create_subscription<VehOdomTy>(
                            vehicle_odometry_topic_,
                            rclcpp::SystemDefaultsQoS(), update_odom);

                auto update_trajectory_target = [this] (const TrajSetptTy &msg) -> void {
                    this->traj_target_ = msg;
                };
                update_trajecotry_target_subscription_ =
                    this->node->create_subscription<TrajSetptTy>(
                            update_trajectory_target_topic_,
                            rclcpp::SystemDefaultsQoS(), update_trajectory_target);

#ifdef URANUS_DEBUG
                odometry_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
                auto update_tf = [this](const VehOdomTy &msg) -> void {
                    using namespace Eigen;
                    using namespace frame_transforms;

                    geometry_msgs::msg::TransformStamped t;

                    t.header.stamp = this->node->get_clock()->now();
                    t.header.frame_id = "map";
                    t.child_frame_id = this->frame_;

                    Quaterniond enu_q = px4_to_ros_orientation(Quaterniond(
                                msg.q[0], msg.q[1], msg.q[2], msg.q[3]));

                    Vector3d enu_pose = ned_to_enu_local_frame(Vector3d(
                                {msg.position[0], msg.position[1], msg.position[2]}));

                    t.transform.translation.x = enu_pose[0];
                    t.transform.translation.y = enu_pose[1];
                    t.transform.translation.z = enu_pose[2];

                    t.transform.rotation.x = enu_q.x();
                    t.transform.rotation.y = enu_q.y();
                    t.transform.rotation.z = enu_q.z();
                    t.transform.rotation.w = enu_q.w();

                    this->odometry_tf_broadcaster_->sendTransform(t);
                };
                update_odometry_tf_ =
                    this->node->create_subscription<VehOdomTy>(
                            *px4_topic::get_vehicle_odometry_topic(vehicle_name_),
                            rclcpp::SystemDefaultsQoS(), update_tf);
#endif
                goal_pose_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
                goal_listener_ = std::make_shared<tf2_ros::TransformListener>(*goal_pose_buffer_);

                auto setting_goal = [this] () -> void {
                    using namespace frame_transforms;
                    using namespace Eigen;

                    GoalTfTy t;
                    try {
                        t = this->goal_pose_buffer_->lookupTransform(
                                "map", this->commanding_frame_, tf2::TimePointZero);
                    } catch (const tf2::TransformException &e) {
                        RCLCPP_ERROR(this->node->get_logger(),
                                "Failed to fetch transform %s -> %s",
                                "map",
                                this->commanding_frame_.c_str());
                        return ;
                    }

                    auto q_enu = Quaterniond(
                            t.transform.rotation.w, t.transform.rotation.x,
                            t.transform.rotation.y, t.transform.rotation.z);
                    auto pose_enu = Vector3d({t.transform.translation.x,
                        t.transform.translation.y, t.transform.translation.z});
                    Quaterniond q_ned = ros_to_px4_orientation(q_enu);

                    Vector3d pose_ned = enu_to_ned_local_frame(pose_enu);

                    RCLCPP_DEBUG(this->node->get_logger(), "Setting pose: {%f, %f, %f}",
                            pose_ned[0], pose_ned[1], pose_ned[2]);
                    setTrajSetpoint(pose_ned[0], pose_ned[1], -this->height_,
                            frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned));
                };

                auto takeoff = [this, setting_goal]() -> void {
                    this->setOffboardMode();
                    this->arm();
                    // Delay for 5 seconds to takeoff and turn off the timer
                    this->oneoff_timer_->cancel();
                    this->goal_listener_timer_ = this->node->create_wall_timer(500ms, setting_goal);
                };
                oneoff_timer_ = node->create_wall_timer(5s, takeoff);

            }

        void updateOffboardControlModeTimestamp();
        void updateOffboardControlMode(const OffCtrlModeTy &rhs);

        void setOffboardMode();
        void arm();

        void setTrajSetpoint(float x, float y, float z, float yaw);
        void setHeight(float h);

        rclcpp::Node::SharedPtr node;

    private:
        const uint16_t ros_instance_id_;
        const uint16_t mavlink_system_id_;

        const std::string commanding_frame_;
        const std::string vehicle_name_;
        const std::string frame_;

        float height_;

        const std::string offboard_control_mode_topic_;
        const std::string vehicle_command_topic_;
        const std::string trajectory_setpoint_topic_;
        const std::string vehicle_odometry_topic_;
        const std::string update_trajectory_target_topic_;

        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::TimerBase::SharedPtr oneoff_timer_;

        rclcpp::Publisher<OffCtrlModeTy>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<VehCmdTy>::SharedPtr vehicle_command_publisher_;
        OffCtrlModeTy offboard_control_mode_profile_;

        rclcpp::Publisher<TrajSetptTy>::SharedPtr trajectory_setpoint_publisher_;
        TrajSetptTy traj_target_;

        rclcpp::Publisher<PoseStampedTy>::SharedPtr update_pose_publisher_;
        rclcpp::Subscription<VehOdomTy>::SharedPtr update_vehicle_odometry_subscription_;
        PoseStampedTy ros_pose_;

        rclcpp::Subscription<TrajSetptTy>::SharedPtr update_trajecotry_target_subscription_;

#ifdef URANUS_DEBUG
        rclcpp::Subscription<VehOdomTy>::SharedPtr update_odometry_tf_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_tf_broadcaster_;
#endif

        rclcpp::TimerBase::SharedPtr goal_listener_timer_;
        std::unique_ptr<tf2_ros::Buffer> goal_pose_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> goal_listener_;
};

#endif
