'''
Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
'''
#include "unitree_legged_real/unitree_services_handler.hpp"

/**
 * @brief Construct a new Client Services Handler:: Client Services Handler object
 * 
 * @param n the ros node handle of the ros node using this class
 */
ClientServicesHandler::ClientServicesHandler(ros::NodeHandle n): n_{n}
{
    power_off_srv_ = n_.serviceClient<std_srvs::Trigger>("power_off");
    emergency_button_srv_ = n_.serviceClient<std_srvs::Trigger>("emergency_stop");
    set_mode_srv_ = n_.serviceClient<unitree_legged_msgs::SetInt>("set_robot_mode");
    get_mode_srv_ = n_.serviceClient<unitree_legged_msgs::GetInt>("get_robot_mode");
    set_gate_srv_ = n_.serviceClient<unitree_legged_msgs::SetInt>("set_gate_type");
    get_gate_srv_ = n_.serviceClient<unitree_legged_msgs::GetInt>("get_gate_type");
    set_speed_srv_ = n_.serviceClient<unitree_legged_msgs::SetInt>("set_speed_level");
    battery_state_srv_ = n_.serviceClient<unitree_legged_msgs::GetBatteryState>("get_battery_state");
    set_foot_height_srv_ = n_.serviceClient<unitree_legged_msgs::SetFloat>("set_foot_height");
    get_foot_height_srv_ = n_.serviceClient<unitree_legged_msgs::GetFloat>("get_foot_height");
    set_body_height_srv_ = n_.serviceClient<unitree_legged_msgs::SetFloat>("set_body_height");
    get_body_height_srv_ = n_.serviceClient<unitree_legged_msgs::GetFloat>("get_body_height");
    set_body_orientation_srv_ = n_.serviceClient<unitree_legged_msgs::SetBodyOrientation>("set_body_orientation");

}

/**
 * @brief Power off service interface 
 * 
 * @return true if the robot power off
 * @return false otherwise
 */
bool ClientServicesHandler::powerOff()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    power_off_srv_.call(req, res);

    if (!res.success)
    {
        ROS_ERROR(res.message.c_str());
    }

    return res.success;
}

/**
 * @brief Emergency stop service interface
 * 
 * @return true always because the robot power off in any case
 * @return false never
 */
bool ClientServicesHandler::emergencyStop()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    emergency_button_srv_.call(req, res);

    ROS_INFO(res.message.c_str());
    return res.success;
}

/**
 * @brief Set robot mode service interface
 * 
 * @param mode the RobotMode modality you want to set
 * @return true if the modality switching has been performed
 * @return false otherwise
 */
bool ClientServicesHandler::setRobotMode(RobotMode mode)
{
    unitree_legged_msgs::SetIntRequest req;
    unitree_legged_msgs::SetIntResponse res;
    req.value = static_cast<uint8_t>(mode);
    set_mode_srv_.call(req, res);

    if (res.success)
        ROS_INFO(res.message.c_str());
    else
        ROS_ERROR(res.message.c_str());

    return res.success;
}

/**
 * @brief Get robot mode service interface
 * 
 * @return RobotMode the current modality of the robot
 */
ClientServicesHandler::RobotMode ClientServicesHandler::getRobotMode()
{
    unitree_legged_msgs::GetIntRequest req;
    unitree_legged_msgs::GetIntResponse res;
    get_mode_srv_.call(req, res);

    ROS_INFO(res.message.c_str());
    return static_cast<RobotMode>(res.value);
}

/**
 * @brief set gate type service interface 
 * 
 * @param type the GateType requested for the robot
 * @return true if the gate type is correctly set
 * @return false otherwise
 */
bool ClientServicesHandler::setGateType(GateType type)
{
    unitree_legged_msgs::SetIntRequest req;
    unitree_legged_msgs::SetIntResponse res;
    req.value = static_cast<uint8_t>(type);
    set_gate_srv_.call(req, res);

    if (res.success)
        ROS_INFO(res.message.c_str());
    else
        ROS_ERROR(res.message.c_str());

    return res.success;
}

/**
 * @brief get gate type service interface 
 * 
 * @return ClientServicesHandler::GateType the current gate type
 */
ClientServicesHandler::GateType ClientServicesHandler::getGateType()
{
    unitree_legged_msgs::GetIntRequest req;
    unitree_legged_msgs::GetIntResponse res;
    get_gate_srv_.call(req, res);

    ROS_INFO(res.message.c_str());
    return static_cast<GateType>(res.value);
}

/**
 * @brief set speed level service interface
 * 
 * @param speed the required speed
 * @return true if speed has been correctly set
 * @return false othrewise
 */
bool ClientServicesHandler::setSpeedLevel(SpeedLevel speed)
{
    unitree_legged_msgs::SetIntRequest req;
    unitree_legged_msgs::SetIntResponse res;
    req.value = static_cast<uint8_t>(speed);
    set_speed_srv_.call(req, res);

    if (res.success)
        ROS_INFO(res.message.c_str());
    else
        ROS_ERROR(res.message.c_str());

    return res.success;
}

/**
 * @brief get battery state interface for retrieving state of charge percentage
 * 
 * @return float the state of charge of the battery in percentage
 */
float ClientServicesHandler::getBatterySoc()
{
    unitree_legged_msgs::GetBatteryStateRequest req;
    unitree_legged_msgs::GetBatteryStateResponse res;
    battery_state_srv_.call(req, res);

    return res.battery_state.percentage;
}

/**
 * @brief set foot height service interface
 * 
 * @param height the heigth delta value to add to the current foot height
 * @return true if the delta height is added
 * @return false otherwise
 */
bool ClientServicesHandler::setFootHeight(float height)
{
    unitree_legged_msgs::SetFloatRequest req;
    unitree_legged_msgs::SetFloatResponse res;
    set_foot_height_srv_.call(req, res);

    if (res.success)
        ROS_INFO(res.message.c_str());
    else
        ROS_ERROR(res.message.c_str());

    return res.success;
}

/**
 * @brief the get foot height service interface
 * 
 * @return float the current foot height of the robot steps
 */
float ClientServicesHandler::getFootHeight()
{
    unitree_legged_msgs::GetFloatRequest req;
    unitree_legged_msgs::GetFloatResponse res;
    get_foot_height_srv_.call(req, res);

    return res.value;
}

/**
 * @brief the set body height service interface
 * 
 * @param height the delta body height to be added to the current one
 * @return true if the delta has been added 
 * @return false if it is not
 */
bool ClientServicesHandler::setBodyHeight(float height)
{
    unitree_legged_msgs::SetFloatRequest req;
    unitree_legged_msgs::SetFloatResponse res;
    set_body_height_srv_.call(req, res);

    if (res.success)
        ROS_INFO(res.message.c_str());
    else
        ROS_ERROR(res.message.c_str());

    return res.success;
}

/**
 * @brief the get body height sensor interface
 * 
 * @return float the current body height
 */
float ClientServicesHandler::getBodyHeight()
{
    unitree_legged_msgs::GetFloatRequest req;
    unitree_legged_msgs::GetFloatResponse res;
    get_body_height_srv_.call(req, res);

    return res.value;
}

/**
 * @brief the set body orientation service interface
 * 
 * @param roll the requested roll angle
 * @param pitch  the requested pitch angle
 * @param yaw  the requested yaw angle
 * @return true if the orientation is correctly set
 * @return false otherwise
 */
bool ClientServicesHandler::setBodyOrientation(float roll, float pitch, float yaw)
{
    unitree_legged_msgs::SetBodyOrientationRequest req;
    unitree_legged_msgs::SetBodyOrientationResponse res;
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    req.rpy = rpy;
    set_body_orientation_srv_.call(req, res);

    if (res.success)
        ROS_INFO(res.message.c_str());
    else
        ROS_ERROR(res.message.c_str());

    return res.success;
}