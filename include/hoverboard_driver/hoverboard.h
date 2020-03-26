#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include "hoverboard_driver/HoverboardConfig.h"

class HoverboardAPI;

class Hoverboard : public hardware_interface::RobotHW {
public:
    Hoverboard();
    ~Hoverboard();

    void read();
    void write();
    void tick();

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

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

    // For debug purposes only
    ros::NodeHandle nh;
    ros::Publisher left_pos_pub, right_pos_pub;
    ros::Publisher left_vel_pub, right_vel_pub;
    ros::Publisher left_eff_pub, right_eff_pub;
    ros::Publisher left_cmd_pub, right_cmd_pub;

    // Supporting dynamic reconfigure for PID control
    dynamic_reconfigure::Server<hoverboard_driver::HoverboardConfig> *dsrv;
    void reconfigure_callback(hoverboard_driver::HoverboardConfig& config, uint32_t level);
    hoverboard_driver::HoverboardConfig config;
    bool have_config = false;
};
