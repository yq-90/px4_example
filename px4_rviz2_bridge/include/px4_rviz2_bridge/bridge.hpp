#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include "frame_transforms/frame_transforms.hpp"
#include "px4_topic/px4_topic.hpp"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include <array>
#include <memory>

class Bridge
{
    public:
        using VehOdomTy = px4_msgs::msg::VehicleOdometry;
        using PoseStampedTy = geometry_msgs::msg::PoseStamped;
        using OdomTy = nav_msgs::msg::Odometry;
        using TrajSetptTy = px4_msgs::msg::TrajectorySetpoint;
        using GoalPoseStampedTy = geometry_msgs::msg::PoseStamped;

        Bridge(const std::string &name, const std::string &vehicle_name = "") :
            bridge_node(std::make_shared<rclcpp::Node>(name)), vehicle_name_(vehicle_name) {
                local_pose_publisher_ =
                    this->bridge_node->create_publisher<PoseStampedTy>("/px4_rviz2_bridge/pose",
                            rclcpp::SystemDefaultsQoS());
                vehicle_odometry_subscription_ =
                    this->bridge_node->create_subscription<VehOdomTy>(
                            *px4_topic::get_vehicle_odometry_topic(this->vehicle_name_),
                            rclcpp::SystemDefaultsQoS(),
                            [this](const VehOdomTy &msg) -> void {
                                using namespace frame_transforms;
                                using namespace Eigen;
                                PoseStampedTy pose = PoseStampedTy();
                                OdomTy odom = OdomTy();

                                pose.header.frame_id = "link";
                                pose.header.stamp.sec =
                                    this->bridge_node->get_clock()->now().seconds();
                                pose.header.stamp.nanosec =
                                    this->bridge_node->get_clock()->now().nanoseconds();

                                this->enu_q = px4_to_ros_orientation(Quaterniond(
                                            msg.q[0], msg.q[1], msg.q[2], msg.q[3]));

                                enu_pose = ned_to_enu_local_frame(Vector3d(
                                            {msg.position[0], msg.position[1], msg.position[2]}));

                                pose.pose.position = tf2::toMsg(enu_pose);
                                pose.pose.orientation = tf2::toMsg(enu_q);

                                local_pose_publisher_->publish(pose);

                                this->vehicle_odometry_ = msg;
                            });

                update_trajecotry_target_publisher_ = this->bridge_node->create_publisher<TrajSetptTy>(
                        *px4_topic::get_update_traject_target_topic(vehicle_name_),
                        rclcpp::SystemDefaultsQoS());
                auto update_goal = [this](const GoalPoseStampedTy &msg) -> void {
                    using namespace Eigen;
                    using namespace frame_transforms;
                    using namespace frame_transforms::utils::quaternion;

                    Quaterniond goal_ned_q = enu_to_ned_orientation(Quaterniond(
                                msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z));

                    Vector3d goal_ned_pose = enu_to_ned_local_frame(Vector3d(
                                {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z}));

                    TrajSetptTy traj_target{};

                    traj_target.position[0] = goal_ned_pose[0];
                    traj_target.position[1] = goal_ned_pose[1];
                    // Keeping the altitude
                    traj_target.position[2] = this->vehicle_odometry_.position[2];

                    RCLCPP_DEBUG(this->bridge_node->get_logger(), "Goal (NED): {%f, %f, %f}",
                            goal_ned_pose[0], goal_ned_pose[1], goal_ned_pose[2]);
                    RCLCPP_DEBUG(this->bridge_node->get_logger(), "Goal (ENU): {%f, %f, %f}",
                            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
                    traj_target.yaw = quaternion_get_yaw(goal_ned_q);
                    this->update_trajecotry_target_publisher_->publish(traj_target);
                };
                goal_pose_subscription_ =
                    this->bridge_node->create_subscription<GoalPoseStampedTy>("/goal_pose",
                            rclcpp::SystemDefaultsQoS(), update_goal);
            }
        rclcpp::Node::SharedPtr bridge_node;
    private:
        const std::string vehicle_name_;

        rclcpp::Subscription<VehOdomTy>::SharedPtr vehicle_odometry_subscription_;
        rclcpp::Publisher<PoseStampedTy>::SharedPtr local_pose_publisher_;
        VehOdomTy vehicle_odometry_;

        rclcpp::Publisher<TrajSetptTy>::SharedPtr update_trajecotry_target_publisher_;
        rclcpp::Subscription<GoalPoseStampedTy>::SharedPtr goal_pose_subscription_;

        Eigen::Quaterniond enu_q;
        Eigen::Vector3d enu_pose;

};

#endif
