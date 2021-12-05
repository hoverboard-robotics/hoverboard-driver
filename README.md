# hoverboard-driver
![main workflow](https://github.com/alex-makarov/hoverboard-driver/actions/workflows/main.yml/badge.svg)

ROS hardware driver for UART-controlled hoverboard. Can be used with [diff_drive_controller](http://wiki.ros.org/diff_drive_controller). Hoverboard is using modified [firmware](https://github.com/alex-makarov/hoverboard-firmware-hack-FOC) by [Emanuel Feru](https://github.com/EmanuelFeru), changed to report wheel odometry via serial protocol.

If you're looking for the version of Hoverboard driver for 
 [bipropellant firmware](https://github.com/bipropellant/bipropellant-hoverboard-firmware), check the respective branch.

This driver is built for [Robaka](https://github.com/alex-makarov/robaka-ros), a prototyping robotic platform based on hoverboard, Jetson Nano and a bunch of sensors. There is a [Slack community](https://join.slack.com/t/robaka/shared_invite/zt-q52yfvnl-IP0h~JDOmgh3VmJ7Hh69Jw) for Robaka.

## Usage

1. Hoverboard port can be set as a "port" parameter in the node namespace. Check `hoverboard.launch` and `4x4.launch` for examples of 2WD and 4WD configurations. 4WD config is purely for illustration on how to use several ports, but was **not** tested with two hoverboards.
2. `roslaunch hoverboard_driver hoverboard.launch`
3. Use any tool (keyboard_teleop, rqt) to send speed commands to `hoverboard_velocity_controller/cmd_vel`.

In 4x4 configuration, if you want to control both axis with the same `cmd_vel`, you can apply remapping as following:
```xml
<launch>
    <group ns="front">
        <param name="port" type="str" value="/dev/ttyTHS1"/>
        <remap from="/front/hoverboard_velocity_controller/cmd_vel" to="/cmd_vel"/>
        <include file="$(find hoverboard_driver)/launch/hoverboard.launch" />
    </group>
    <group ns="rear">
        <param name="port" type="str" value="/dev/ttyTHS2"/>
        <remap from="/rear/hoverboard_velocity_controller/cmd_vel" to="/cmd_vel"/>
        <include file="$(find hoverboard_driver)/launch/hoverboard.launch" />
    </group>
</launch>
```

## DISCLAIMER
I bear **no responsibility** for any damage, direct or indirect, caused by using this project. Hoverboards are powerful and can be dangerous! Make sure you take all safety precautions!
