#include "px4_single_control/control.hpp"

void Control::updateOffboardControlModeTimestamp()
{
    // FIXME This function is in a timer callback
    // There would be a division every hundreds of microseconds
    this->offboard_control_mode_profile_.timestamp =
        this->node->get_clock()->now().nanoseconds() / 1000;
}

void Control::updateOffboardControlMode(const OffCtrlModeTy &rhs)
{
    this->offboard_control_mode_profile_ = rhs;
    this->offboard_control_mode_profile_.timestamp =
        this->node->get_clock()->now().nanoseconds() / 1000;
}

void Control::setOffboardMode()
{
    VehCmdTy cmd{};
    // TODO What the hell on earth are these two params...
    // As long as it goes to offboard mode
    cmd.param1 = 1;
    cmd.param2 = 6;
    cmd.command = VehCmdTy::VEHICLE_CMD_DO_SET_MODE;
    cmd.target_system = mavlink_system_id_;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;
    this->vehicle_command_publisher_->publish(cmd);
    RCLCPP_DEBUG(this->node->get_logger(), "Setting Offboard Mode");
}

void Control::arm()
{
    VehCmdTy cmd{};
    cmd.param1 = VehCmdTy::ARMING_ACTION_ARM;
    cmd.command = VehCmdTy::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = mavlink_system_id_;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;
    this->vehicle_command_publisher_->publish(cmd);
    RCLCPP_DEBUG(this->node->get_logger(), "Arming");
}

void Control::setTrajSetpoint(float x, float y, float z, float yaw)
{
    this->traj_target_.position = {x, y, z};
    this->traj_target_.yaw = yaw;
    this->traj_target_.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;
}

void Control::setHeight(float h)
{
    this->height_ = h;
}
