#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "frame_transforms/frame_transforms.hpp"
#include "px4_topic/px4_topic.hpp"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include <array>
#include <memory>

class Bridge
{
    public:
        using VehicleOdomTy = px4_msgs::msg::VehicleOdometry;
        using PoseStampedTy = geometry_msgs::msg::PoseStamped;
        using OdomTy = nav_msgs::msg::Odometry;

        Bridge(const std::string &name) :
            bridge_node(std::make_shared<rclcpp::Node>(name)) {
                local_pose_publisher_ =
                    this->bridge_node->create_publisher<PoseStampedTy>("/px4_rviz2_bridge/pose",
                        rclcpp::SystemDefaultsQoS());
                vehicle_odometry_subscription_ =
                    this->bridge_node->create_subscription<VehicleOdomTy>("/fmu/out/vehicle_odometry",
                            rclcpp::SystemDefaultsQoS(),
                            [this](const VehicleOdomTy &msg) -> void {
                                using namespace frame_transforms;
                                using namespace Eigen;
                                PoseStampedTy pose = PoseStampedTy();
                                OdomTy odom = OdomTy();

                                pose.header.frame_id = "link";
                                pose.header.stamp.sec =
                                    this->bridge_node->get_clock()->now().seconds();
                                pose.header.stamp.nanosec =
                                    this->bridge_node->get_clock()->now().nanoseconds();

                                this->enu_q = ned_to_enu_orientation(Quaterniond(
                                            msg.q[0], msg.q[1], msg.q[2], msg.q[3]));

                                enu_pose = ned_to_enu_local_frame(Vector3d(
                                            {msg.position[0], msg.position[1], msg.position[2]}));

                                pose.pose.position = tf2::toMsg(enu_pose);
                                pose.pose.orientation = tf2::toMsg(enu_q);

                                local_pose_publisher_->publish(pose);

                                this->vehicle_odometry_ = msg;
                            });
            }
        rclcpp::Node::SharedPtr bridge_node;
    private:
        rclcpp::Subscription<VehicleOdomTy>::SharedPtr vehicle_odometry_subscription_;
        rclcpp::Publisher<PoseStampedTy>::SharedPtr local_pose_publisher_;
        VehicleOdomTy vehicle_odometry_;
        Eigen::Quaterniond enu_q;
        Eigen::Vector3d enu_pose;
};

#endif
