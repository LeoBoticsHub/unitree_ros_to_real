/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#ifndef UNITREE_ROS_CLIENT_HPP
#define UNITREE_ROS_CLIENT_HPP

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include "unitree_legged_msgs/GetBatteryState.h"
#include "unitree_legged_msgs/GetFloat.h" 
#include "unitree_legged_msgs/GetInt.h" 
#include "unitree_legged_msgs/SetBodyOrientation.h"
#include "unitree_legged_msgs/SetFloat.h"
#include "unitree_legged_msgs/SetInt.h"

/**
 * @brief Class used to handle the unitree services for controlling the robot in a smoot way
 */
class UnitreeRosClient
{
private:
    ros::ServiceClient power_off_srv_;
    ros::ServiceClient emergency_button_srv_;
    ros::ServiceClient get_mode_srv_;
    ros::ServiceClient stand_up_srv_;
    ros::ServiceClient stand_down_srv_;
    ros::ServiceClient battery_state_srv_;
    ros::ServiceClient set_foot_height_srv_;
    ros::ServiceClient set_body_height_srv_;
    ros::ServiceClient get_foot_height_srv_;
    ros::ServiceClient get_body_height_srv_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_orient_pub_;

    ros::NodeHandle n_;

public:

    /**
     * @brief Construct a new Client Services Handler:: Client Services Handler object
     * 
     * @param n the ros node handle of the ros node using this class
     */
    UnitreeRosClient(ros::NodeHandle n);

    /**
     * @brief Initialize ros service clients and publishers
     */
    void initRosInterface();
    
    /**
     * @brief Power off service interface 
     * 
     * @return true if the robot power off
     * @return false otherwise
     */
    bool powerOff();

    /**
     * @brief Emergency stop service interface
     * 
     * @return true always because the robot power off in any case
     * @return false never
     */
    bool emergencyStop();

    /**
     * @brief Stand up servie interface
     * 
     * @return true if the robot stands up
     * @return false otherwise
     */
    bool standUp();

    /**
     * @brief Stand Down servie interface
     * 
     * @return true if the robot stands down
     * @return false otherwise
     */
    bool standDown();

    /**
     * @brief Get robot mode service interface
     * 
     * @return RobotMode the current modality of the robot
     */
    uint8_t getRobotMode();

    /**
     * @brief get battery state interface for retrieving state of charge percentage
     * 
     * @return float the state of charge of the battery in percentage
     */
    float getBatterySoc();

    /**
     * @brief set foot height service interface
     * 
     * @param height the heigth delta value to add to the current foot height
     * @return true if the delta height is added
     * @return false otherwise
     */
    bool setFootHeight(float height);

    /**
     * @brief the get foot height service interface
     * 
     * @return float the current foot height of the robot steps
     */
    float getFootHeight();

    /**
     * @brief the set body height service interface
     * 
     * @param height the delta body height to be added to the current one
     * @return true if the delta has been added 
     * @return false if it is not
     */
    bool setBodyHeight(float height);

    /**
     * @brief the get body height sensor interface
     * 
     * @return float the current body height
     */
    float getBodyHeight();

    /**
     * @brief To set the robot cmd vel
     * 
     * @param vx linear velocity along x axis (forward)
     * @param vy linear velocity along y axis (lateral)
     * @param w angular velocity around z axis (yaw)
     */
    void pubCmdVel(float vx, float vy, float w);

    /**
     * @brief to set the body orientation
     * 
     * @param roll the requested roll angle
     * @param pitch  the requested pitch angle
     * @param yaw  the requested yaw angle
     */
    void pubCmdOrient(float roll, float pitch, float yaw);
    
};

#endif