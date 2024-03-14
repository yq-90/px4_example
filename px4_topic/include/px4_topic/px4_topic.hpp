#ifndef PX4_TOPIC__PX4_TOPIC_HPP_
#define PX4_TOPIC__PX4_TOPIC_HPP_

#include "px4_topic/visibility_control.h"

#include <string>
#include <memory>
#include <sstream>

namespace px4_topic
{

    std::shared_ptr<const std::string> get_vehicle_odometry_topic(const std::string &vehicle_name);
    std::shared_ptr<const std::string> get_vehicle_command_topic(const std::string &vehicle_name);
    std::shared_ptr<const std::string> get_trajectory_setpoint_topic(const std::string &vehicle_name);
    std::shared_ptr<const std::string> get_offboard_control_mode_topic(const std::string &vehicle_name);
    std::shared_ptr<const std::string> get_update_traject_target_topic(const std::string &vehicle_name);

}  // namespace px4_topic

#endif  // PX4_TOPIC__PX4_TOPIC_HPP_
