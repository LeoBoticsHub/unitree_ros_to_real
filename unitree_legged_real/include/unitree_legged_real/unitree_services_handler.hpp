#ifndef UNITREE_SERVICE_HANDLER_HPP
#define UNITREE_SERVICE_HANDLER_HPP

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/BatteryState.h>
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
class ClientServicesHandler
{
private:
    ros::ServiceClient power_off_srv_;
    ros::ServiceClient emergency_button_srv_;
    ros::ServiceClient set_mode_srv_;
    ros::ServiceClient get_mode_srv_;
    ros::ServiceClient set_gate_srv_;
    ros::ServiceClient get_gate_srv_;
    ros::ServiceClient set_speed_srv_;
    ros::ServiceClient battery_state_srv_;
    ros::ServiceClient set_foot_height_srv_;
    ros::ServiceClient set_body_height_srv_;
    ros::ServiceClient get_foot_height_srv_;
    ros::ServiceClient get_body_height_srv_;
    ros::ServiceClient set_body_orientation_srv_;

    ros::NodeHandle n_;

public:
    ClientServicesHandler(ros::NodeHandle n);

    enum class RobotMode { 
        IDLE_STAND=0, FORCED_STAND=1, VELOCITY_WALKING=2, SPEED=3, 
        STAND_DOWN=5, STAND_UP=6, DAMPING=7, RECOVERY_STAND=9
    };

    enum class GateType { IDLE=0, TROT, TROT_RUNNING, CLIMB_STAIRS, TROT_OBSTACLES };

    enum class SpeedLevel { LOW=0, MEDIUM, FAST };
    
    // intermediate functions to call services without interfacing directly with ros services
    bool powerOff();
    bool emergencyStop();
    bool setRobotMode(RobotMode mode);
    RobotMode getRobotMode();
    bool setGateType(GateType type);
    GateType getGateType();
    bool setSpeedLevel(SpeedLevel speed);
    float getBatterySoc();
    bool setFootHeight(float height);
    float getFootHeight();
    bool setBodyHeight(float height);
    float getBodyHeight();
    bool setBodyOrientation(float roll, float pitch, float yaw);
    
};

#endif