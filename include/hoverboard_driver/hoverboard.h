#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include "hoverboard_driver/HoverboardConfig.h"
#include "hoverboard_driver/pid.h"

class HoverboardAPI;

class Hoverboard : public hardware_interface::RobotHW {
public:
    static Hoverboard& getInstance();
    ~Hoverboard();
    
    void read();
    void write(const ros::Time& time, const ros::Duration& period);
    void tick();

    void hallCallback();
    void electricalCallback();
 private:
    Hoverboard();
 
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    double max_velocity = 0.0;

    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint {
        std_msgs::Float64 pos;
        std_msgs::Float64 vel;
        std_msgs::Float64 eff;
        std_msgs::Float64 cmd;
    } joints[2];

    double wheel_radius;
    ros::Time last_read;
    HoverboardAPI *api;

    PID pids[2];

    // For debug purposes only
    ros::NodeHandle nh;
    ros::Publisher left_pos_pub, right_pos_pub;
    ros::Publisher left_vel_pub, right_vel_pub;
    ros::Publisher left_eff_pub, right_eff_pub;
    ros::Publisher left_cmd_pub, right_cmd_pub;
    ros::Publisher left_cur_pub, right_cur_pub;
    ros::Publisher voltage_pub;
};
