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
#include "protocol.h"

class HoverboardAPI;

class Hoverboard : public hardware_interface::RobotHW {
public:
    Hoverboard();
    ~Hoverboard();
    
    void read();
    void write(const ros::Time& time, const ros::Duration& period);
    void tick();
 private:
    void protocol_recv (char c);
 
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint {
        std_msgs::Float64 pos;
        std_msgs::Float64 vel;
        std_msgs::Float64 eff;
        std_msgs::Float64 cmd;
    } joints[2];

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher vel_pub[2];
    ros::Publisher cmd_pub[2];
    ros::Publisher voltage_pub;
    ros::Publisher temp_pub;

    double wheel_radius;
    double max_velocity = 0.0;
    ros::Time last_read;

    // Hoverboard protocol
    int port_fd;
    int msg_len = 0;
    char prev_byte = 0;
    uint16_t start_frame = 0;
    char* p;
    SerialFeedback msg, prev_msg;

    PID pids[2];
};
