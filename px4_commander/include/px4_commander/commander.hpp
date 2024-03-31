#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "frame_transforms/frame_transforms.hpp"

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
        using GoalPoseStampedTy = PoseStampedTy;

        Commander(rclcpp::Node::SharedPtr node) :
            commander_node(node), frame_(node->get_parameter("frame").as_string()) {

                command_frame_broadcaster_ =
                    std::make_unique<tf2_ros::TransformBroadcaster>(*commander_node);

                // Broadcasting initial ground zero position
                ground_zero_transform_.header.stamp = this->commander_node->get_clock()->now();
                ground_zero_transform_.header.frame_id = "map";
                ground_zero_transform_.child_frame_id = this->frame_;

                ground_zero_transform_.transform.translation.x = 0;
                ground_zero_transform_.transform.translation.y = 0;
                ground_zero_transform_.transform.translation.z = 0;

                ground_zero_transform_.transform.rotation.x = 0;
                ground_zero_transform_.transform.rotation.y = 0;
                ground_zero_transform_.transform.rotation.z = 0;
                ground_zero_transform_.transform.rotation.w = 1;

                command_frame_broadcaster_->sendTransform(ground_zero_transform_);

                goal_pose_buffer_ = std::make_unique<tf2_ros::Buffer>(commander_node->get_clock());
                goal_listener_ = std::make_shared<tf2_ros::TransformListener>(*goal_pose_buffer_);

                auto broadcasting_tf = [this]() -> void {
                    this->ground_zero_transform_.header.stamp =
                        this->commander_node->get_clock()->now();
                    command_frame_broadcaster_->sendTransform(ground_zero_transform_);
                };
                tf_timer_ = commander_node->create_wall_timer(3s, broadcasting_tf);

                auto update_goal = [this](const GoalPoseStampedTy &msg) -> void {
                    this->ground_zero_transform_.header.stamp =
                        this->commander_node->get_clock()->now();

                    PoseStampedTy map_goal_pose;
                    try {
                        // Don't know why, but is seems we need a timeout for transform
                        // Otherwise this would yield error in message timestamp while looking
                        // for the corresponding frame
                        this->goal_pose_buffer_->transform(msg, map_goal_pose, "map",
                                tf2::durationFromSec(10));
                    } catch (const tf2::TransformException &e) {
                        RCLCPP_ERROR(this->commander_node->get_logger(),
                                "Failed to transform %s -> %s",
                                msg.header.frame_id.c_str(),
                                "map");
                        return ;
                    }

                    this->ground_zero_transform_.transform.translation.x = map_goal_pose.pose.position.x;
                    this->ground_zero_transform_.transform.translation.y = map_goal_pose.pose.position.y;
                    this->ground_zero_transform_.transform.translation.z = map_goal_pose.pose.position.z;

                    this->ground_zero_transform_.transform.rotation.x = map_goal_pose.pose.orientation.x;
                    this->ground_zero_transform_.transform.rotation.y = map_goal_pose.pose.orientation.y;
                    this->ground_zero_transform_.transform.rotation.z = map_goal_pose.pose.orientation.z;
                    this->ground_zero_transform_.transform.rotation.w = map_goal_pose.pose.orientation.w;

                    command_frame_broadcaster_->sendTransform(ground_zero_transform_);
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

        geometry_msgs::msg::TransformStamped ground_zero_transform_;
        rclcpp::TimerBase::SharedPtr tf_timer_;

        std::unique_ptr<tf2_ros::Buffer> goal_pose_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> goal_listener_;
};

#endif
