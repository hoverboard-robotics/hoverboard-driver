# hoverboard-driver
![main workflow](https://github.com/alex-makarov/hoverboard-driver/actions/workflows/main.yml/badge.svg)

ROS hardware driver for UART-controlled hoverboard. Can be used with [diff_drive_controller](http://wiki.ros.org/diff_drive_controller). Hoverboard is using modified [firmware](https://github.com/alex-makarov/hoverboard-firmware-hack-FOC) by [Emanuel Feru](https://github.com/EmanuelFeru), changed to report wheel odometry via serial protocol.

If you're looking for the version of Hoverboard driver for 
 [bipropellant firmware](https://github.com/bipropellant/bipropellant-hoverboard-firmware), check the respective branch.

This driver is built for [Robaka](https://github.com/alex-makarov/robaka-ros), a prototyping robotic platform based on hoverboard, Jetson Nano and a bunch of sensors. There is a [Slack community](https://join.slack.com/t/robaka/shared_invite/zt-q52yfvnl-IP0h~JDOmgh3VmJ7Hh69Jw) for Robaka.

## DISCLAIMER
I bear **no responsibility** for any damage, direct or indirect, caused by using this project. Hoverboards are powerful and can be dangerous! Make sure you take all safety precautions!
