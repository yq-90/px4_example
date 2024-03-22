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
    node->declare_parameter("formation_transform", std::vector<double>{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    });
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<rclcpp::Node>("px4_single_control");
    initialize_params(control_node);
    auto matrix_param = control_node->get_parameter("formation_transform");

    size_t param_matrix_size = matrix_param.as_double_array().size();

    Eigen::Matrix4d form_tf{Eigen::Matrix4d::Identity()};
    if (param_matrix_size != 16) {
        RCLCPP_ERROR(control_node->get_logger(),
                "Formation transform takes a 4x4 matrix, %ld elements are provided. Switching to identity matrix",
                param_matrix_size);
    } else {
        form_tf = Eigen::Map<const Eigen::Matrix4d>(matrix_param.as_double_array().data());
    }

    std::stringstream ss;

    // FIXME Making clean_fmt a accessible global object
    Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
    ss << form_tf.format(clean_fmt);
    RCLCPP_INFO(control_node->get_logger(), "%s", ss.str().c_str());

    auto ctrl = Control(control_node, form_tf, true);

    rclcpp::spin(control_node);
    return 0;
}
