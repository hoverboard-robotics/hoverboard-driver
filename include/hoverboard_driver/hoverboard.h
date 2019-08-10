#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

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
        double pos;
        double vel;
        double eff;
        double cmd;
    } joints[2];

    double max_linear_speed;
    double wheel_radius;

    ros::Time last_read;

    HoverboardAPI *api;
};
