/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#include "unitree_legged_real/unitree_ros_client.hpp"

UnitreeRosClient::UnitreeRosClient(ros::NodeHandle n): n_{n}{}

void UnitreeRosClient::initRosInterface()
{
    // Services
    power_off_srv_ = n_.serviceClient<std_srvs::Trigger>("power_off");
    emergency_button_srv_ = n_.serviceClient<std_srvs::Trigger>("emergency_stop");
    get_mode_srv_ = n_.serviceClient<unitree_legged_msgs::GetInt>("get_robot_mode");
    stand_up_srv_ = n_.serviceClient<std_srvs::Trigger>("stand_up");
    stand_down_srv_ = n_.serviceClient<std_srvs::Trigger>("stand_down");
    battery_state_srv_ = n_.serviceClient<unitree_legged_msgs::GetBatteryState>("get_battery_state");
    set_foot_height_srv_ = n_.serviceClient<unitree_legged_msgs::SetFloat>("set_foot_height");
    get_foot_height_srv_ = n_.serviceClient<unitree_legged_msgs::GetFloat>("get_foot_height");
    set_body_height_srv_ = n_.serviceClient<unitree_legged_msgs::SetFloat>("set_body_height");
    get_body_height_srv_ = n_.serviceClient<unitree_legged_msgs::GetFloat>("get_body_height");

    // publisher
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    cmd_orient_pub_ = n_.advertise<geometry_msgs::Vector3>("/cmd_orient", 20);
}

bool UnitreeRosClient::powerOff()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    power_off_srv_.call(req, res);

    if (!res.success)
    {
        ROS_ERROR("%s", res.message.c_str());
    }

    return res.success;
}

bool UnitreeRosClient::emergencyStop()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    emergency_button_srv_.call(req, res);

    ROS_INFO("%s", res.message.c_str());
    return res.success;
}

bool UnitreeRosClient::standUp()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    stand_up_srv_.call(req, res);

    if (res.success)
        ROS_INFO("%s", res.message.c_str());
    else
        ROS_ERROR("%s", res.message.c_str());

    return res.success;
}

bool UnitreeRosClient::standDown()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    stand_down_srv_.call(req, res);

    if (res.success)
        ROS_INFO("%s", res.message.c_str());
    else
        ROS_ERROR("%s", res.message.c_str());

    return res.success;
}

uint8_t UnitreeRosClient::getRobotMode()
{
    unitree_legged_msgs::GetIntRequest req;
    unitree_legged_msgs::GetIntResponse res;
    get_mode_srv_.call(req, res);

    ROS_INFO("%s", res.message.c_str());
    return res.value;
}

float UnitreeRosClient::getBatterySoc()
{
    unitree_legged_msgs::GetBatteryStateRequest req;
    unitree_legged_msgs::GetBatteryStateResponse res;
    battery_state_srv_.call(req, res);

    return res.battery_state.percentage;
}

bool UnitreeRosClient::setFootHeight(float height)
{
    unitree_legged_msgs::SetFloatRequest req;
    unitree_legged_msgs::SetFloatResponse res;
    set_foot_height_srv_.call(req, res);

    if (res.success)
        ROS_INFO("%s", res.message.c_str());
    else
        ROS_ERROR("%s", res.message.c_str());

    return res.success;
}

float UnitreeRosClient::getFootHeight()
{
    unitree_legged_msgs::GetFloatRequest req;
    unitree_legged_msgs::GetFloatResponse res;
    get_foot_height_srv_.call(req, res);

    return res.value;
}

bool UnitreeRosClient::setBodyHeight(float height)
{
    unitree_legged_msgs::SetFloatRequest req;
    unitree_legged_msgs::SetFloatResponse res;
    set_body_height_srv_.call(req, res);

    if (res.success)
        ROS_INFO("%s", res.message.c_str());
    else
        ROS_ERROR("%s", res.message.c_str());

    return res.success;
}

float UnitreeRosClient::getBodyHeight()
{
    unitree_legged_msgs::GetFloatRequest req;
    unitree_legged_msgs::GetFloatResponse res;
    get_body_height_srv_.call(req, res);

    return res.value;
}

void UnitreeRosClient::pubCmdVel(float vx, float vy, float w)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = w;

    cmd_vel_pub_.publish(cmd_vel);
}

void UnitreeRosClient::pubCmdOrient(float roll, float pitch, float yaw)
{
    geometry_msgs::Vector3 cmd_orient;
    cmd_orient.x = roll;
    cmd_orient.y = pitch;
    cmd_orient.z = yaw;

    cmd_orient_pub_.publish(cmd_orient);
}