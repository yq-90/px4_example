#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "frame_transforms/frame_transforms.hpp"
#include "px4_topic/px4_topic.hpp"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include <array>
#include <memory>

using namespace std::chrono_literals;

class Commander
{
    public:
        using PoseStampedTy = geometry_msgs::msg::PoseStamped;
        using OdomTy = nav_msgs::msg::Odometry;
        using GoalPoseStampedTy = geometry_msgs::msg::PoseStamped;

        Commander(const std::string &name,
                const std::string &frame = "base_link") :
            commander_node(std::make_shared<rclcpp::Node>(name)), frame_(frame),
            base_link_transform_(geometry_msgs::msg::TransformStamped{}) {

                command_frame_broadcaster_ =
                    std::make_unique<tf2_ros::TransformBroadcaster>(*commander_node);

                // Broadcasting initial base_link position, which is identical to link
                base_link_transform_.header.stamp = this->commander_node->get_clock()->now();
                base_link_transform_.header.frame_id = "map";
                base_link_transform_.child_frame_id = this->frame_;

                base_link_transform_.transform.translation.x = 0;
                base_link_transform_.transform.translation.y = 0;
                base_link_transform_.transform.translation.z = 0;

                base_link_transform_.transform.rotation.x = 0;
                base_link_transform_.transform.rotation.y = 0;
                base_link_transform_.transform.rotation.z = 0;
                base_link_transform_.transform.rotation.w = 1;

                command_frame_broadcaster_->sendTransform(base_link_transform_);

                auto broadcasting_tf = [this]() -> void {
                    this->base_link_transform_.header.stamp =
                        this->commander_node->get_clock()->now();
                    command_frame_broadcaster_->sendTransform(base_link_transform_);
                };
                tf_timer_ = commander_node->create_wall_timer(3s, broadcasting_tf);

                auto update_goal = [this](const GoalPoseStampedTy &msg) -> void {
                    this->base_link_transform_.header.stamp =
                        this->commander_node->get_clock()->now();

                    this->base_link_transform_.transform.translation.x = msg.pose.position.x;
                    this->base_link_transform_.transform.translation.y = msg.pose.position.y;
                    this->base_link_transform_.transform.translation.z = msg.pose.position.z;

                    this->base_link_transform_.transform.rotation.x = msg.pose.orientation.x;
                    this->base_link_transform_.transform.rotation.y = msg.pose.orientation.y;
                    this->base_link_transform_.transform.rotation.z = msg.pose.orientation.z;
                    this->base_link_transform_.transform.rotation.w = msg.pose.orientation.w;

                    command_frame_broadcaster_->sendTransform(base_link_transform_);
                };
                goal_pose_subscription_ =
                    this->commander_node->create_subscription<GoalPoseStampedTy>("/goal_pose",
                            rclcpp::SystemDefaultsQoS(), update_goal);
            }
        rclcpp::Node::SharedPtr commander_node;
    private:
        const std::string frame_;

        rclcpp::Subscription<GoalPoseStampedTy>::SharedPtr goal_pose_subscription_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> command_frame_broadcaster_;

        geometry_msgs::msg::TransformStamped base_link_transform_;
        rclcpp::TimerBase::SharedPtr tf_timer_;
};

#endif
