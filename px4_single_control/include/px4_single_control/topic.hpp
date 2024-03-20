#ifndef TOPIC_HPP_
#define TOPIC_HPP_

#include <string>

#define GET_TOPIC_FUNC_DECL(TOPIC) \
    const std::string &get_##TOPIC##_topic()

#define GET_TOPIC_FUNC_DEF(IO, TOPIC) \
    const std::string &get_##TOPIC##_topic() { \
        static const std::string ret("/fmu/"#IO"/"#TOPIC); \
        return ret; \
    }

GET_TOPIC_FUNC_DECL(offboard_control_mode);
GET_TOPIC_FUNC_DECL(vehicle_command);
GET_TOPIC_FUNC_DECL(trajectory_setpoint);
GET_TOPIC_FUNC_DECL(vehicle_odometry);
GET_TOPIC_FUNC_DECL(update_trajectory_target);

#endif  // TOPIC_HPP_
