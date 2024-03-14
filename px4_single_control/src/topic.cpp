#include "px4_single_control/topic.hpp"

const std::string &px4_in_prefix()
{
    static const std::string in_prefix("/fmu/in/");
    return in_prefix;
}

const std::string &px4_out_prefix()
{
    static const std::string out_prefix("/fmu/out/");
    return out_prefix;
}

std::string build_topic(const std::string &vehicle_name,
        const std::string &io,
        const std::string &topic)
{
    std::stringstream ss;
    ss << vehicle_name << io << topic;
    return ss.str();
}

// --------------------- Getters --------------------

std::shared_ptr<const std::string> get_vehicle_odometry_topic(const std::string &vehicle_name)
{
    static std::shared_ptr<const std::string> ret =
        std::make_shared<const std::string>(build_topic(vehicle_name,
                    px4_out_prefix(), "vehicle_odometry"));
    return ret;
}

std::shared_ptr<const std::string> get_vehicle_command_topic(const std::string &vehicle_name)
{
    static std::shared_ptr<const std::string> ret =
        std::make_shared<const std::string>(build_topic(vehicle_name,
                    px4_in_prefix(), "vehicle_command"));
    return ret;
}

std::shared_ptr<const std::string> get_trajectory_setpoint_topic(const std::string &vehicle_name)
{
    static std::shared_ptr<const std::string> ret =
        std::make_shared<const std::string>(build_topic(vehicle_name,
                    px4_in_prefix(), "trajectory_setpoint"));
    return ret;
}

std::shared_ptr<const std::string> get_offboard_control_mode_topic(const std::string &vehicle_name)
{
    static std::shared_ptr<const std::string> ret =
        std::make_shared<const std::string>(build_topic(vehicle_name,
                    px4_in_prefix(), "offboard_control_mode"));
    return ret;
}

std::shared_ptr<const std::string> get_update_traject_target_topic(const std::string &vehicle_name)
{
    static std::shared_ptr<const std::string> ret =
        std::make_shared<const std::string>(build_topic(vehicle_name,
                    px4_in_prefix(), "update_trajectory_target"));
    return ret;
}
