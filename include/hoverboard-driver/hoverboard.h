#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class Hoverboard : public hardware_interface::RobotHW {
public:
    Hoverboard();
    ~Hoverboard();

    void read();
    void write();

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    struct Joint {
        double pos;
        double vel;
        double eff;
        double cmd;
    } joints[2];
};
