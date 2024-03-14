#ifndef TOPIC_HPP
#define TOPIC_HPP

#include <string>
#include <memory>
#include <sstream>

std::shared_ptr<const std::string> get_vehicle_odometry_topic(const std::string &vehicle_name);
std::shared_ptr<const std::string> get_vehicle_command_topic(const std::string &vehicle_name);
std::shared_ptr<const std::string> get_trajectory_setpoint_topic(const std::string &vehicle_name);
std::shared_ptr<const std::string> get_offboard_control_mode_topic(const std::string &vehicle_name);

#endif
