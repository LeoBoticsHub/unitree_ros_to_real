/*
** Copyright (C) 2024 Federico Rollo, Leonardo Innovation Labs - All Rights Reserved
**/

#include <ros/ros.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <chrono>
#include <pthread.h>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

#include "unitree_legged_msgs/GetBatteryState.h"
#include "unitree_legged_msgs/GetFloat.h" 
#include "unitree_legged_msgs/GetInt.h" 
#include "unitree_legged_msgs/SetBodyOrientation.h"
#include "unitree_legged_msgs/SetFloat.h"
#include "unitree_legged_msgs/SetInt.h"

using namespace UNITREE_LEGGED_SDK;

/**
 * @brief Custom unitree class for connecting via UDP to the robot controller
 */
class Custom
{
public:
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    xRockerBtnDataStruct keyData;

public:
    Custom(): 
        high_udp(8090, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
    }

    void highUdpSend()
    {
        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void highUdpRecv()
    {
        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

/**
 * @brief Average function to compute the mean of a vector
 */
template< class T >
double avg(T* vect, unsigned int size)
{
    T res = 0.0;
    for(unsigned int i = 0; i < size; i++)
        res = res + vect[i];
    return res/size;
}

Custom custom;

// Subscribers
ros::Subscriber cmd_vel_sub;

// Publishers
ros::Publisher imu_pub;
ros::Publisher joint_states_pub;
ros::Publisher odom_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;

// Services
ros::ServiceServer power_off_srv;
ros::ServiceServer emergency_button_srv;
ros::ServiceServer set_mode_srv;
ros::ServiceServer get_mode_srv;
ros::ServiceServer set_gate_srv;
ros::ServiceServer get_gate_srv;
ros::ServiceServer set_speed_srv;
ros::ServiceServer battery_state_srv;
ros::ServiceServer set_foot_height_srv;
ros::ServiceServer set_body_height_srv;
ros::ServiceServer get_foot_height_srv;
ros::ServiceServer get_body_height_srv;
ros::ServiceServer set_body_orientation_srv;

// ROS messages
sensor_msgs::Imu imu_msg;
sensor_msgs::JointState joint_state_msg;
nav_msgs::Odometry odom_msg;
sensor_msgs::BatteryState battery_msg;
geometry_msgs::TransformStamped odom_H_trunk;

// Constants
const int N_MOTORS{12};
const std::string BASE_LINK_NAME{"trunk"};
const std::string IMU_NAME{"imu"};
const std::string ODOM_NAME{"odom"};

// Joint variables
static std::array<unsigned int, 12> b1_motor_idxs
{{
    UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2, // LF
    UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2, // LH
    UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2, // RF
    UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2, // RH
}};
static std::array<std::string, 12> b1_motor_names
{{
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
}};

// robot state map
const std::map<int,std::vector<int>> robot_state_mapping =  {
    {0, std::vector<int>{1,2,3,5}}, // DEFUALT STAND
    {1, std::vector<int>{0,2,3,5}}, // FORCED STAND
    {2, std::vector<int>{0,1,3,5}}, // VELOCITY WALKING
    {3, std::vector<int>{0,1,2,5}}, // RUNNING MODE?
    // {4, std::vector<int>()}, // PATH FOLLOWING?
    {5, std::vector<int>{6,7}}, // STAND DOWN
    {6, std::vector<int>{0,1,2,3,5}}, // STAND UP
    {7, std::vector<int>(6)}, // DAMPING MODE
    // {9, std::vector<int>()}, // RECOVERY STAND
};

// Other variables
ros::Time t, t_prev, t_timer, t_mode_timer;
bool timer_on = false;

// ROS State Publisher
/**
 * @brief This function reads the Robot high state values from an UDP message (joint states, imu, odom, wirless remote) and publish them on ros messages
 */
void highStatePublisher()
{
    t = ros::Time::now();

    if (t != t_prev)
    {   
        // joint states
        joint_state_msg.header.seq++;
        joint_state_msg.header.stamp = t;
        joint_state_msg.header.frame_id = BASE_LINK_NAME;

        for (unsigned int motor_id = 0; motor_id < N_MOTORS; ++motor_id)
	    {
            joint_state_msg.name[motor_id]     = b1_motor_names[motor_id];
            joint_state_msg.position[motor_id] = static_cast<double>(custom.high_state.motorState[b1_motor_idxs[motor_id]].q);
            joint_state_msg.velocity[motor_id] = static_cast<double>(custom.high_state.motorState[b1_motor_idxs[motor_id]].dq); // NOTE: this order is different than google
            joint_state_msg.effort[motor_id]   = static_cast<double>(custom.high_state.motorState[b1_motor_idxs[motor_id]].tauEst);
	    }

        // imu
        imu_msg.header.seq++;
	    imu_msg.header.stamp = t;
	    imu_msg.header.frame_id = IMU_NAME;
	    imu_msg.orientation.w = static_cast<double>(custom.high_state.imu.quaternion[0]);
	    imu_msg.orientation.x = static_cast<double>(custom.high_state.imu.quaternion[1]);
	    imu_msg.orientation.y = static_cast<double>(custom.high_state.imu.quaternion[2]);
	    imu_msg.orientation.z = static_cast<double>(custom.high_state.imu.quaternion[3]);
	    imu_msg.angular_velocity.x = static_cast<double>(custom.high_state.imu.gyroscope[0]);
	    imu_msg.angular_velocity.y = static_cast<double>(custom.high_state.imu.gyroscope[1]);
	    imu_msg.angular_velocity.z = static_cast<double>(custom.high_state.imu.gyroscope[2]);
	    imu_msg.linear_acceleration.x = static_cast<double>(custom.high_state.imu.accelerometer[0]);
	    imu_msg.linear_acceleration.y = static_cast<double>(custom.high_state.imu.accelerometer[1]);
	    imu_msg.linear_acceleration.z = static_cast<double>(custom.high_state.imu.accelerometer[2]);

        // odom

        odom_msg.header.seq++;
	    odom_msg.header.stamp = t;
	    odom_msg.header.frame_id            = ODOM_NAME;
	    odom_msg.child_frame_id             = BASE_LINK_NAME;
	    odom_msg.pose.pose.position.x       = static_cast<double>(custom.high_state.position[0]);
	    odom_msg.pose.pose.position.y       = static_cast<double>(custom.high_state.position[1]);
	    odom_msg.pose.pose.position.z       = static_cast<double>(custom.high_state.position[2]);
	    odom_msg.pose.pose.orientation      = imu_msg.orientation;
	    odom_msg.twist.twist.linear.x       = static_cast<double>(custom.high_state.velocity[0]);
	    odom_msg.twist.twist.linear.y       = static_cast<double>(custom.high_state.velocity[1]);
	    odom_msg.twist.twist.linear.z       = static_cast<double>(custom.high_state.velocity[2]);
	    odom_msg.twist.twist.angular        = imu_msg.angular_velocity;

        // odom -> base_link
        odom_H_trunk.header.seq++;
        odom_H_trunk.header.stamp = t;
        odom_H_trunk.transform.translation.x       = static_cast<double>(custom.high_state.position[0]);
	    odom_H_trunk.transform.translation.y       = static_cast<double>(custom.high_state.position[1]);
	    odom_H_trunk.transform.translation.z       = static_cast<double>(custom.high_state.position[2]);
	    odom_H_trunk.transform.rotation            = imu_msg.orientation;

        // publish state messages
        joint_states_pub.publish(joint_state_msg);
        imu_pub.publish(imu_msg);
        odom_pub.publish(odom_msg);
        tf_pub->sendTransform(odom_H_trunk);

        // check wirless remote received commands (used to bypass ros cmd_vel if needed)
        memcpy(&custom.keyData, &custom.high_state.wirelessRemote[0], 40); 
    }

    t_prev = t;
}

// Callbacks
/**
 *
 * @brief cmd_vel callback that receive a twist command message and send it to the robot
 * 
 * @param msg The twist velocity message to send to the robot
 * 
 */
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // Check if wirless remote command are given
    if ( std::abs(custom.keyData.rx) < 0.1 && 
         std::abs(custom.keyData.lx) < 0.1 && 
         std::abs(custom.keyData.ry) < 0.1 && 
         std::abs(custom.keyData.ly) < 0.1 )
    {
        // check timer if previous wirless remote commands have been receives
        if(timer_on && (t - t_timer).sec >= 5 || !timer_on)
        {
            custom.high_cmd.velocity[0] = msg->linear.x;
            custom.high_cmd.velocity[1] = msg->linear.y;
            custom.high_cmd.yawSpeed = msg->angular.z;
  
            timer_on = false;
        }
    }
    else
    {
        // initialize timer if wirless remote commands are received
        if(!timer_on)
            timer_on = true;
        t_timer = ros::Time::now();
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed = 0;
    }

}

/**
 * @brief callback used to power off the robot when it is in crouched position
 */
bool powerOffCallback(
    std_srvs::Trigger::Request& req, 
    std_srvs::Trigger::Response& res
)
{
    // only if in stand down position
    if(custom.high_state.mode == 7)
    {
        BmsCmd battery_cmd;
        battery_cmd.off = 0xA5;
        sleep(3);
        custom.high_cmd.bms = battery_cmd;
    }
    else
    {
        res.success = false;
        res.message = "The robot is not in stand down position. Cannot power off in this situation.";
    }

    return true;
}

/**
 * @brief callback used to power off the robot always as an emergency button. BE CAREFUL IN USING IT.
 */
bool emergencyStopCallback(
    std_srvs::Trigger::Request& req, 
    std_srvs::Trigger::Response& res
)
{
    BmsCmd battery_cmd;
    battery_cmd.off = 0xA5;
    custom.high_cmd.bms = battery_cmd;

    res.success = true;
    res.message = "Powering off the robot.";

    return true;
}

/**
 * @brief battery state callback to check current robot battery state
 */
bool batteryStateCallback(
    unitree_legged_msgs::GetBatteryState::Request& /* req */, 
    unitree_legged_msgs::GetBatteryState::Response& res
)
{
    battery_msg.header.seq++;
    battery_msg.header.stamp = t;
    battery_msg.header.frame_id = BASE_LINK_NAME;
    battery_msg.current = static_cast<float>(1000.0 * custom.high_state.bms.current); // mA -> A
    battery_msg.voltage = static_cast<float>(1000.0 * avg<uint16_t>(&custom.high_state.bms.cell_vol[0], 10)); // mV -> V
    battery_msg.percentage = custom.high_state.bms.SOC;

    res.battery_state = battery_msg;
    
    return true;
}

/**
 * @brief Callback used for setting robot mode, e.g. stand up, stand down, walk, etc
 */
bool setRobotModeCallback(
    unitree_legged_msgs::SetInt::Request& req, 
    unitree_legged_msgs::SetInt::Response& res
)
{
    // iterator on robot state mapping that point to the current mode value
    auto map_iterator = robot_state_mapping.find(custom.high_state.mode);

    if ((t - t_mode_timer).sec >= 3 && // check previous call timer
        map_iterator != robot_state_mapping.end() && // check if map value exists (it should be useless because states should all exists)
        std::find(map_iterator->second.begin(), map_iterator->second.end(), req.value) != 
        map_iterator->second.end() // check if requested value is available in vector of possible states
    )
    {
        custom.high_cmd.euler[0] = 0;
        custom.high_cmd.euler[1] = 0;
        custom.high_cmd.euler[2] = 0;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed    = 0;
        custom.high_cmd.mode = req.value;
        res.success = true;
        res.message = "Robot state set to: " + std::to_string(req.value);

        t_mode_timer = t;
    }
    else
    {
        res.success = false;
        res.message = "Error! Invalid Robot state! \
        Please be sure to wait some time before switching consecutive states. \
        check also if Robot state switch from current mode " + std::to_string(custom.high_state.mode) + \
        " to requested state mode " + std::to_string(req.value) + " is permitted.";
    }

    return true;
}

/**
 * @brief Callback used for getting robot mode, e.g. stand up, stand down, walk, etc
 */
bool getRobotModeCallback(
    unitree_legged_msgs::GetInt::Request& /* req */, 
    unitree_legged_msgs::GetInt::Response& res
)
{
    res.value = custom.high_state.mode;

    return true;
}

/**
 * @brief Callback used for setting robot gate type, e.g. trot, trot running, staris climb, etc
 */
bool setGateTypeCallback(
    unitree_legged_msgs::SetInt::Request& req, 
    unitree_legged_msgs::SetInt::Response& res
)
{   
    if (req.value >= 0 && req.value <= 4)
    {
        custom.high_cmd.gaitType = req.value;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed    = 0;
        res.success = true;
        res.message = "Gate type set to: " + std::to_string(req.value);
    }
    else
    {
        res.success = false;
        res.message = "Error! Invalid gate type! Please set a gate type between 0 and 4";
    }

    return true;
}

/**
 * @brief Callback used for getting robot gate type, e.g. trot, trot running, staris climb, etc
 */
bool getGateTypeCallback(
    unitree_legged_msgs::GetInt::Request& /* req */, 
    unitree_legged_msgs::GetInt::Response& res
)
{
    res.value = custom.high_state.gaitType;

    return true;
}

/**
 * @brief Callback used for setting robot speed level, e.g. normal, fast, etc
 */
bool setSpeedLevelCallback(
    unitree_legged_msgs::SetInt::Request& req, 
    unitree_legged_msgs::SetInt::Response& res
)
{
    // apparently it works just in mode 3 // TODO TO BE CHECKED
    if (custom.high_state.mode == 3 && req.value >= 0 && req.value <= 2)
    {
        custom.high_cmd.speedLevel = req.value;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed    = 0;
        res.success = true;
        res.message = "Speed level set to: " + std::to_string(req.value);
    }
    else
    {
        res.success = false;
        res.message = "Error! Invalid speed level! \
        Please check if robot state mode is set to 3 and set a speed level between 0 and 2";
    }

    return true;
}

/**
 * @brief Callback used for setting robot foot raise height
 * 
 * @param unitree_legged_msgs/SetFloat delta value to add to the current foot height. Range: [-0.1, 0.15]
 */
bool setFootHeightCallback(
    unitree_legged_msgs::SetFloat::Request& req, 
    unitree_legged_msgs::SetFloat::Response& res
)
{
    // check if the request is in the allowed delta bounds
    if (req.value >= -0.1 && req.value <= 0.15)
    {
        custom.high_cmd.footRaiseHeight = req.value;
        res.success = true;
        res.message = "Foot hight delta set to: " + std::to_string(req.value);
    }
    else
    {
        res.success = false;
        res.message = "Error! Invalid foot height delta! \
        Please provide a delta in the range [-0.1, 0.15].";
    }

    return true;
}

/**
 * @brief Callback used for getting current robot foot raise height
 */
bool getFootHeightCallback(
    unitree_legged_msgs::GetFloat::Request& /* req */, 
    unitree_legged_msgs::GetFloat::Response& res
)
{
    res.value = custom.high_state.footRaiseHeight;

    return true;
}

/**
 * @brief Callback used for setting robot body height
 * 
 * @param unitree_legged_msgs/SetFloat delta value to add to the current body height. Range: [-0.16, 0.16]
 */
bool setBodyHeightCallback(
    unitree_legged_msgs::SetFloat::Request& req, 
    unitree_legged_msgs::SetFloat::Response& res
)
{
    // check if the request is in the allowed delta bounds
    if (req.value >= -0.16 && req.value <= 0.16)
    {
        custom.high_cmd.bodyHeight = req.value;
        res.success = true;
        res.message = "Body height delta set to: " + std::to_string(req.value);
    }
    else
    {
        res.success = false;
        res.message = "Error! Invalid body height delta! \
        Please provide a delta in the range [-0.1, 0.15].";
    }

    return true;
}

/**
 * @brief Callback used for getting current robot body height
 */
bool getBodyHeightCallback(
    unitree_legged_msgs::GetFloat::Request& /* req */, 
    unitree_legged_msgs::GetFloat::Response& res
)
{
    res.value = custom.high_state.bodyHeight;

    return true;
}

/**
 * @brief Callback used for setting robot speed level, e.g. normal, fast, etc
 * 
 * @param unitree_legged_msgs/SetBodyOrientation roll pitch yaw angle references. 
 * roll range: [-0.3, 0.3], pitch range: [-0.3, 0.3], yaw range: [-0.6, 0.6]
 */
bool setBodyOrientationCallback(
    unitree_legged_msgs::SetBodyOrientation::Request& req, 
    unitree_legged_msgs::SetBodyOrientation::Response& res
)
{
    // it works just in mode 1 andhas bounds for the three angles
    if (custom.high_state.mode == 1 && 
        req.rpy.x >= -0.3 && req.rpy.x <= 0.3 &&
        req.rpy.y >= -0.3 && req.rpy.y <= 0.3 &&
        req.rpy.z >= -0.6 && req.rpy.z <= 0.6
    )
    {
        custom.high_cmd.euler[0] = req.rpy.x;
        custom.high_cmd.euler[0] = req.rpy.y;
        custom.high_cmd.euler[0] = req.rpy.z;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed    = 0;
        res.success = true;
        res.message = "Robot orientation set to: roll = " + std::to_string(req.rpy.x) + ", pitch = " + \
        std::to_string(req.rpy.y) + ", yaw = " + std::to_string(req.rpy.z);
    }
    else
    {
        res.success = false;
        res.message = "Error! Invalid Robot orientation! \
        Please check if robot state mode is set to 1 and set a robot orientation between \
        roll range: [-0.3, 0.3], pitch range: [-0.3, 0.3], yaw range: [-0.6, 0.6]";
    }

    res.success = true;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_high_ros_control");

    ros::NodeHandle nh;

    // set joint state size
    joint_state_msg.name.resize(N_MOTORS);
    joint_state_msg.position.resize(N_MOTORS);
    joint_state_msg.velocity.resize(N_MOTORS);
    joint_state_msg.effort.resize(N_MOTORS);

    // set odom frame ids
    odom_H_trunk.header.frame_id = ODOM_NAME;
    odom_H_trunk.child_frame_id = BASE_LINK_NAME;

    // initialize time
    t = t_prev = t_timer = t_mode_timer = ros::Time::now();

    // Subscribers
    cmd_vel_sub = nh.subscribe("cmd_vel", 20, cmdVelCallback);

    // Publishers
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 20);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 20);
    odom_pub = nh.advertise<nav_msgs::Odometry>("robot_odom", 20);
    tf_pub.reset(new tf2_ros::TransformBroadcaster);

    // Services
    power_off_srv = nh.advertiseService("power_off", powerOffCallback);
    emergency_button_srv = nh.advertiseService("emergency_stop", emergencyStopCallback);
    set_mode_srv = nh.advertiseService("set_robot_mode", setRobotModeCallback);
    get_mode_srv = nh.advertiseService("get_robot_mode", getRobotModeCallback);
    set_gate_srv = nh.advertiseService("set_gate_type", setGateTypeCallback);
    get_gate_srv = nh.advertiseService("get_gate_type", getGateTypeCallback);
    set_speed_srv = nh.advertiseService("set_speed_level", setSpeedLevelCallback);
    battery_state_srv = nh.advertiseService("get_battery_state", batteryStateCallback);
    set_foot_height_srv = nh.advertiseService("set_foot_height", setFootHeightCallback);
    get_foot_height_srv = nh.advertiseService("get_foot_height", getFootHeightCallback);
    set_body_height_srv = nh.advertiseService("set_body_height", setBodyHeightCallback);
    get_body_height_srv = nh.advertiseService("get_body_height", getBodyHeightCallback);
    set_body_orientation_srv = nh.advertiseService("set_body_orientation", setBodyOrientationCallback);

    // high state udp loop function
    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    // State publisher loop function
    LoopFunc loop_StatePub("high_state_pub", 0.0025, 3, &highStatePublisher);

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_StatePub.start();

    printf("HIGHLEVEL is initialized\n");

    ros::spin();

    return 0;
}
