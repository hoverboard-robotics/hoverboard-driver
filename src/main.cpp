#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include "hoverboard.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hoverboard_driver");

    Hoverboard hoverboard;
    controller_manager::ControllerManager cm(&hoverboard);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(100.0);

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        hoverboard.read();
        cm.update(time, period);
        hoverboard.write(time, period);

        rate.sleep();
    }

    return 0;
}
