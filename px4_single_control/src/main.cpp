#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <sstream>

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "px4_single_control/control.hpp"

using RawFormationTFTy = std::vector<std::vector<double>>;

using namespace std::chrono_literals;

static void initialize_params(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter<int>("instance_id", 0);
    node->declare_parameter<double>("initial_height", 10.0);
    node->declare_parameter("commanding_frame", "base_link");
    node->declare_parameter("vehicle_prefix", "px4_");
    // The transform matrix of the drone's initial position
    node->declare_parameter("initial_transform", std::vector<double>{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    });
    // The transform matrix of the drone's position within a swarm formation
    node->declare_parameter("formation_transform", std::vector<double>{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    });

}

void initialize_transform(Eigen::Matrix4d &tf, const rclcpp::Node::SharedPtr node,
        const std::string &param)
{
    auto tf_param = node->get_parameter(param);
    size_t tf_param_size = tf_param.as_double_array().size();
    if (tf_param_size != 16) {
        RCLCPP_ERROR(node->get_logger(),
                "%s takes a 4x4 matrix, %ld elements are provided. Falling back to identity matrix",
                param.c_str(), tf_param_size);
    } else {
        tf = Eigen::Map<const Eigen::Matrix4d>(tf_param.as_double_array().data());
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<rclcpp::Node>("px4_single_control");

    initialize_params(control_node);
    Eigen::Matrix4d init_tf{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d form_tf{Eigen::Matrix4d::Identity()};
    initialize_transform(init_tf, control_node, "initial_transform");
    initialize_transform(form_tf, control_node, "formation_transform");

    std::stringstream ss_init, ss_form;

    // FIXME Making clean_fmt a accessible global object
    Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
    ss_init << init_tf.format(clean_fmt);
    ss_form << form_tf.format(clean_fmt);
    RCLCPP_DEBUG(control_node->get_logger(), "Initial transform matrix:\n%s",
            ss_init.str().c_str());
    RCLCPP_DEBUG(control_node->get_logger(), "Ground zero point transform matrix:\n%s",
            ss_form.str().c_str());

    auto ctrl = Control(control_node, init_tf, form_tf, true);

    rclcpp::spin(control_node);
    return 0;
}
