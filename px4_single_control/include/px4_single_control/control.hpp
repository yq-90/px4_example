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
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "frame_transforms/frame_transforms.hpp"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "px4_single_control/topic.hpp"

using namespace std::chrono_literals;

#define TOPIC_INITIALIZER(TOPIC_NAME) \
    ((!ros_instance_id_ && !need_prefix) ? \
     get_##TOPIC_NAME##_topic() : \
     (vehicle_name_ + get_##TOPIC_NAME##_topic()))

const Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");

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

        Control(rclcpp::Node::SharedPtr node,
                const Eigen::Matrix4d &init_tf,
                const Eigen::Matrix4d &formation_tf,
                bool need_prefix = false) :
            control_node(node),
            ros_instance_id_(node->get_parameter("instance_id").as_int()),
            mavlink_system_id_(1 + ros_instance_id_),
            commanding_frame_(node->get_parameter("commanding_frame").as_string()),
            vehicle_name_(node->get_parameter("vehicle_prefix").as_string() +
                    std::to_string(ros_instance_id_)),
            frame_(vehicle_name_ + "/base_link"),
            odom_frame_(vehicle_name_ + "/odom"),
            height_(node->get_parameter("initial_height").as_double()),
            initial_transform_(init_tf),
            formation_transform_(formation_tf),
            ground_zero_transform_(Eigen::Matrix4d::Identity()),
            offboard_control_mode_topic_(TOPIC_INITIALIZER(offboard_control_mode)),
            vehicle_command_topic_(TOPIC_INITIALIZER(vehicle_command)),
            trajectory_setpoint_topic_(TOPIC_INITIALIZER(trajectory_setpoint)),
            vehicle_odometry_topic_(TOPIC_INITIALIZER(vehicle_odometry)),
            update_trajectory_target_topic_(TOPIC_INITIALIZER(update_trajectory_target)) {

                offboard_control_mode_profile_.position = true;
                offboard_control_mode_profile_.velocity = false;
                offboard_control_mode_profile_.acceleration = false;
                offboard_control_mode_profile_.attitude = false;
                offboard_control_mode_profile_.body_rate = false;

                offboard_control_mode_publisher_ =
                    control_node->create_publisher<OffCtrlModeTy>(
                            offboard_control_mode_topic_,
                            rclcpp::SystemDefaultsQoS());
                vehicle_command_publisher_ =
                    control_node->create_publisher<VehCmdTy>(
                            vehicle_command_topic_,
                            rclcpp::SystemDefaultsQoS());
                trajectory_setpoint_publisher_ =
                    control_node->create_publisher<TrajSetptTy>(
                            trajectory_setpoint_topic_,
                            rclcpp::SystemDefaultsQoS());

                traj_target_.position = {0.0, 0.0, -height_};

                initial_odometry_tf_broadcaster_ =
                    std::make_shared<tf2_ros::StaticTransformBroadcaster>(*control_node);
                geometry_msgs::msg::TransformStamped init_odom_tf;

                Eigen::Affine3d init_tf_ned(initial_transform_);
                Eigen::Vector3d init_pose_enu =
                    frame_transforms::ned_to_enu_local_frame(Eigen::Vector3d {
                        init_tf_ned.translation()[0],
                        init_tf_ned.translation()[1],
                        init_tf_ned.translation()[2]
                    });

                init_odom_tf.header.stamp = this->control_node->get_clock()->now();
                init_odom_tf.header.frame_id = "map";
                init_odom_tf.child_frame_id = this->odom_frame_;

                init_odom_tf.transform.translation.x = init_pose_enu[0];
                init_odom_tf.transform.translation.y = init_pose_enu[1];
                init_odom_tf.transform.translation.z = init_pose_enu[2];

                init_odom_tf.transform.rotation.x = 0;
                init_odom_tf.transform.rotation.y = 0;
                init_odom_tf.transform.rotation.z = 0;
                init_odom_tf.transform.rotation.w = 1;

                initial_odometry_tf_broadcaster_->sendTransform(init_odom_tf);

                auto heartbeat_cb = [this]() -> void {
                    this->updateOffboardControlModeTimestamp();
                    offboard_control_mode_publisher_->publish(
                            this->offboard_control_mode_profile_);

                    this->traj_target_.timestamp =
                        this->control_node->get_clock()->now().nanoseconds() / 1000;
                    trajectory_setpoint_publisher_->publish(this->traj_target_);

                    RCLCPP_DEBUG(this->control_node->get_logger(),
                            "Offboard updated: Pose {%f, %f, %f}",
                            this->traj_target_.position[0], this->traj_target_.position[1],
                            this->traj_target_.position[2]);
                };

                heartbeat_timer_ = control_node->create_wall_timer(200ms, heartbeat_cb);

                auto update_trajectory_target = [this] (const TrajSetptTy &msg) -> void {
                    this->traj_target_ = msg;
                };
                update_trajecotry_target_subscription_ =
                    this->control_node->create_subscription<TrajSetptTy>(
                            update_trajectory_target_topic_,
                            rclcpp::SystemDefaultsQoS(), update_trajectory_target);

                odometry_tf_broadcaster_ =
                    std::make_unique<tf2_ros::TransformBroadcaster>(*control_node);
                auto update_tf = [this](const VehOdomTy &msg) -> void {
                    using namespace Eigen;
                    using namespace frame_transforms;

                    geometry_msgs::msg::TransformStamped t;

                    t.header.stamp = this->control_node->get_clock()->now();
                    t.header.frame_id = this->odom_frame_;
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
                    this->control_node->create_subscription<VehOdomTy>(
                            this->vehicle_odometry_topic_,
                            rclcpp::SystemDefaultsQoS(), update_tf);

                goal_pose_buffer_ = std::make_unique<tf2_ros::Buffer>(control_node->get_clock());
                goal_listener_ = std::make_shared<tf2_ros::TransformListener>(*goal_pose_buffer_);

                auto setting_goal = [this] () -> void {
                    using namespace frame_transforms;
                    using namespace Eigen;

                    GoalTfTy t;
                    try {
                        t = this->goal_pose_buffer_->lookupTransform(
                                "map", this->commanding_frame_, tf2::TimePointZero);
                    } catch (const tf2::TransformException &e) {
                        RCLCPP_ERROR(this->control_node->get_logger(),
                                "Failed to fetch transform %s -> %s",
                                "map",
                                this->commanding_frame_.c_str());
                        return ;
                    }

                    // transform matrix from the ground zero vectors (translation & rotation)
                    this->ground_zero_transform_.translation() = enu_to_ned_local_frame(
                            Vector3d({t.transform.translation.x,
                                t.transform.translation.y, t.transform.translation.z}));
                    this->ground_zero_transform_.translation().z() = -this->height_;
                    this->ground_zero_transform_.linear() = ros_to_px4_orientation(Quaterniond(
                                t.transform.rotation.w, t.transform.rotation.x,
                                t.transform.rotation.y, t.transform.rotation.z)).toRotationMatrix();

                    // If we want drone's position to be absolute, use
                    // this->formation_transform_ * this->ground_zero_transform
                    Affine3d steering_trans =
                        this->ground_zero_transform_ * this->formation_transform_;
                    steering_trans = this->initial_transform_.inverse() * steering_trans;
                    const Affine3d::TranslationPart steering_pose = steering_trans.translation();

                    std::stringstream ss;
                    ss << steering_trans.matrix().format(clean_fmt);

                    RCLCPP_DEBUG(this->control_node->get_logger(),
                            "UAV%d Steering transform: \n%s",
                            this->ros_instance_id_, ss.str().c_str());
                    RCLCPP_DEBUG(this->control_node->get_logger(),
                            "Steering pose (in ned): {%f, %f, %f}",
                            steering_pose[0], steering_pose[1], steering_pose[2]);

                    setTrajSetpoint(steering_pose[0], steering_pose[1], steering_pose[2],
                            frame_transforms::utils::quaternion::quaternion_get_yaw(
                                Quaterniond(steering_trans.linear())));
                };

                auto takeoff = [this, setting_goal]() -> void {
                    this->setOffboardMode();
                    this->arm();
                    // Delay for 5 seconds to takeoff and turn off the timer
                    this->oneoff_timer_->cancel();
                    this->goal_listener_timer_ =
                        this->control_node->create_wall_timer(500ms, setting_goal);
                };
                oneoff_timer_ = control_node->create_wall_timer(5s, takeoff);

            }

        void updateOffboardControlModeTimestamp();
        void updateOffboardControlMode(const OffCtrlModeTy &rhs);

        void setOffboardMode();
        void arm();

        void setTrajSetpoint(float x, float y, float z, float yaw);
        void setHeight(float h);

        rclcpp::Node::SharedPtr control_node;

    private:
        const uint16_t ros_instance_id_;
        const uint16_t mavlink_system_id_;

        const std::string commanding_frame_;
        const std::string vehicle_name_;
        const std::string frame_;
        const std::string odom_frame_;

        float height_;

        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::TimerBase::SharedPtr oneoff_timer_;

        rclcpp::Publisher<OffCtrlModeTy>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<VehCmdTy>::SharedPtr vehicle_command_publisher_;
        OffCtrlModeTy offboard_control_mode_profile_;

        rclcpp::Publisher<TrajSetptTy>::SharedPtr trajectory_setpoint_publisher_;
        TrajSetptTy traj_target_;

        rclcpp::Subscription<TrajSetptTy>::SharedPtr update_trajecotry_target_subscription_;

        rclcpp::Subscription<VehOdomTy>::SharedPtr update_odometry_tf_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> initial_odometry_tf_broadcaster_;

        rclcpp::TimerBase::SharedPtr goal_listener_timer_;
        std::unique_ptr<tf2_ros::Buffer> goal_pose_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> goal_listener_;

        const Eigen::Affine3d initial_transform_;
        // The spatial offset of the drone wrt the ground zero point
        // Non-const for future extension upon dynamic formation change
        Eigen::Affine3d formation_transform_;
        Eigen::Affine3d ground_zero_transform_;

        // Topic constants
        const std::string offboard_control_mode_topic_;
        const std::string vehicle_command_topic_;
        const std::string trajectory_setpoint_topic_;
        const std::string vehicle_odometry_topic_;
        const std::string update_trajectory_target_topic_;
};

#endif
