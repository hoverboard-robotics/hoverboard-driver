#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "hoverboard-driver/hoverboard.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hoverboard-driver");

    Hoverboard hoverboard;
    controller_manager::ControllerManager cm(&hoverboard);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0);

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        hoverboard.read();
        cm.update(time, period);
        hoverboard.write();

        rate.sleep();
    }

    return 0;
}