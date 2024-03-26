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
#include <mutex>

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
ros::Subscriber body_orient_sub;

// Publishers
ros::Publisher imu_pub;
ros::Publisher joint_states_pub;
ros::Publisher odom_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;

// Services
ros::ServiceServer power_off_srv;
ros::ServiceServer emergency_button_srv;
ros::ServiceServer stand_up_srv;
ros::ServiceServer stand_down_srv;
ros::ServiceServer get_mode_srv;
ros::ServiceServer battery_state_srv;
ros::ServiceServer set_foot_height_srv;
ros::ServiceServer set_body_height_srv;
ros::ServiceServer get_foot_height_srv;
ros::ServiceServer get_body_height_srv;

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

// Other variables
ros::Time t, t_prev, t_timer, t_cmd_vel, t_cmd_orient;
bool timer_on, cmd_vel_active, cmd_orient_active = false;
std::mutex stand_mtx;

// ROS State Publisher
/**
 * @brief This function reads the Robot high state values from an UDP message (joint states, imu, odom, wirless remote) and publish them on ros messages
 */
void highStatePublisher()
{
    t = ros::Time::now();

    // zero cmd vel if no command are received 
    if ((t - t_cmd_vel).sec > 1 && custom.high_state.mode != 7 && cmd_vel_active)
    {
        custom.high_cmd.mode = 2;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        custom.high_cmd.mode = 0;
        cmd_vel_active = false;
    }

    // zero cmd vel if no command are received 
    if ((t - t_cmd_orient).sec > 1 && custom.high_state.mode != 7 && cmd_orient_active)
    {
        custom.high_cmd.mode = 1;
        custom.high_cmd.euler[0] = 0;
        custom.high_cmd.euler[1] = 0;
        custom.high_cmd.euler[2] = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        custom.high_cmd.mode = 2;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        custom.high_cmd.mode = 0;
        cmd_orient_active = false;
    }

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
 * @param msg The twist velocity message to send to the robot, 
 * forwardSpeed range:[-0.8, 1.2], sideSpeed range: [-0.25, 0.25], range:[-0.75, 0.75]
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
            custom.high_cmd.mode = 2;
            custom.high_cmd.euler[0] = 0;
            custom.high_cmd.euler[1] = 0;
            custom.high_cmd.euler[2] = 0;
            custom.high_cmd.velocity[0] = msg->linear.x;
            custom.high_cmd.velocity[1] = msg->linear.y;
            custom.high_cmd.yawSpeed = msg->angular.z;
  
            timer_on = false;
            cmd_vel_active = true;
            t_cmd_vel = ros::Time::now();
        }
    }
    else
    {
        // initialize timer if wirless remote commands are received
        if(!timer_on)
            timer_on = true;
        t_timer = ros::Time::now();
        custom.high_cmd.mode = 0;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed = 0;
    }
}


/**
 * @brief Callback used for setting robot orientation
 * 
 * @param msg [geometry_msgs/Vector3] roll pitch yaw angle references.
 * roll range: [-0.3, 0.3], pitch range: [-0.3, 0.3], yaw range: [-0.6, 0.6]
 */
void cmdBodyOrientationCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
        // Check if wirless remote command are given
    if ( std::abs(custom.keyData.rx) < 0.1 && 
         std::abs(custom.keyData.lx) < 0.1 && 
         std::abs(custom.keyData.ry) < 0.1 && 
         std::abs(custom.keyData.ly) < 0.1 )
    {
        if (!cmd_vel_active &&
            (timer_on && (t - t_timer).sec >= 5 || !timer_on) &&
            msg->x >= -0.3 && msg->x <= 0.3 &&
            msg->y >= -0.3 && msg->y <= 0.3 &&
            msg->z >= -0.6 && msg->z <= 0.6
        )
        {
            custom.high_cmd.mode = 1;
            custom.high_cmd.euler[0] = msg->x;
            custom.high_cmd.euler[1] = msg->y;
            custom.high_cmd.euler[2] = msg->z;
            custom.high_cmd.velocity[0] = 0;
            custom.high_cmd.velocity[1] = 0;
            custom.high_cmd.yawSpeed    = 0;

            cmd_orient_active = true;
            t_cmd_orient = ros::Time::now();
        }
    }
    else
    {
        // initialize timer if wirless remote commands are received
        if(!timer_on)
            timer_on = true;
        t_timer = ros::Time::now();
        custom.high_cmd.mode = 0;
        custom.high_cmd.euler[0] = 0;
        custom.high_cmd.euler[1] = 0;
        custom.high_cmd.euler[2] = 0;
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
    custom.high_cmd.euler[0] = 0;
    custom.high_cmd.euler[1] = 0;
    custom.high_cmd.euler[2] = 0;
    custom.high_cmd.velocity[0] = 0;
    custom.high_cmd.velocity[1] = 0;
    custom.high_cmd.yawSpeed    = 0;
    custom.high_cmd.mode = 7;

    res.success = true;
    res.message = "Emergency Stop! Robot set in dumping mode.";

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
 * @brief Callback used for standing up the robot
 */
bool standUpCallback(
    std_srvs::Trigger::Request& req, 
    std_srvs::Trigger::Response& res
)
{
    const std::lock_guard<std::mutex> lock(stand_mtx);
    if (custom.high_state.mode == 7) // If in dumping mode
    {
        custom.high_cmd.euler[0] = 0;
        custom.high_cmd.euler[1] = 0;
        custom.high_cmd.euler[2] = 0;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed    = 0;
        sleep(1);
        custom.high_cmd.mode = 6;
        sleep(3);
        custom.high_cmd.mode = 1;

        res.success = true;
        res.message = "Robot stand up completed.";
    }
    else
    {
        res.success = false;
        res.message = "Error! The robot is not in dumping position. Cannot stand up.";
    }

    return true;
}

/**
 * @brief Callback used for standing down the robot
 */
bool standDownCallback(
    std_srvs::Trigger::Request& req, 
    std_srvs::Trigger::Response& res
)
{
    const std::lock_guard<std::mutex> lock(stand_mtx);
    if (custom.high_state.mode == 1) // If in standing mode
    {
        custom.high_cmd.euler[0] = 0;
        custom.high_cmd.euler[1] = 0;
        custom.high_cmd.euler[2] = 0;
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed    = 0;
        custom.high_cmd.mode = 6;
        sleep(3);
        custom.high_cmd.mode = 5;

        res.success = true;
        res.message = "Robot stand down completed.";
    }
    else
    {
        res.success = false;
        res.message = "Error! The robot is not in fixed standing position. Cannot stand down.";
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
    if (req.value >= -0.1 && req.value <= 0.15001)
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
        res.message = "Error! Invalid body height delta! Please provide a delta in the range [-0.1, 0.15].";
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
    t = t_prev = t_timer = t_cmd_vel = t_cmd_orient = ros::Time::now();

    // Subscribers
    cmd_vel_sub = nh.subscribe("cmd_vel", 20, cmdVelCallback);
    body_orient_sub = nh.subscribe("cmd_orient", 20, cmdBodyOrientationCallback);
    

    // Publishers
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 20);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 20);
    odom_pub = nh.advertise<nav_msgs::Odometry>("robot_odom", 20);
    tf_pub.reset(new tf2_ros::TransformBroadcaster);

    // Services
    power_off_srv = nh.advertiseService("power_off", powerOffCallback);
    emergency_button_srv = nh.advertiseService("emergency_stop", emergencyStopCallback);
    stand_up_srv = nh.advertiseService("stand_up", standUpCallback);
    stand_down_srv = nh.advertiseService("stand_down", standDownCallback);
    get_mode_srv = nh.advertiseService("get_robot_mode", getRobotModeCallback);
    battery_state_srv = nh.advertiseService("get_battery_state", batteryStateCallback);
    set_foot_height_srv = nh.advertiseService("set_foot_height", setFootHeightCallback);
    get_foot_height_srv = nh.advertiseService("get_foot_height", getFootHeightCallback);
    set_body_height_srv = nh.advertiseService("set_body_height", setBodyHeightCallback);
    get_body_height_srv = nh.advertiseService("get_body_height", getBodyHeightCallback);

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
