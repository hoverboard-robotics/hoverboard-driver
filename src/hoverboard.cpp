#include "hoverboard-driver/hoverboard.h"


Hoverboard::Hoverboard() {
    hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel", &joints[0].pos, &joints[0].vel, &joints[0].eff);
    hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel", &joints[1].pos, &joints[1].vel, &joints[1].eff);

    joint_state_interface.registerHandle (left_wheel_state_handle);
    joint_state_interface.registerHandle (right_wheel_state_handle);

    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle left_wheel_vel_handle(joint_state_interface.getHandle("left_wheel"), &joints[0].cmd);
    hardware_interface::JointHandle right_wheel_vel_handle(joint_state_interface.getHandle("right_wheel"), &joints[1].cmd);

    velocity_joint_interface.registerHandle (left_wheel_vel_handle);
    velocity_joint_interface.registerHandle (right_wheel_vel_handle);

    registerInterface(&velocity_joint_interface);
}

Hoverboard::~Hoverboard() {


}

void Hoverboard::read() {


}

void Hoverboard::write() {


}
