# ROS2_CONTROL for Hoverboard Driver based on DiffBot
This is a ros2_control hardware interface implementation for Hoverboard Driver based on ros2_control DiffBot example.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_2/doc/userdoc.html).

# Hints
Connect Hoverboard PCB (USART3, GND, TX, RX, no VCC!!) to USB-TTL converter (or UART interface of your SBC).
Set the serial port according to your setup in hoverboard_driver.ros2_control.xacro file

# Launch
```
ros2 launch hoverboard_driver diffbot.launch.py
```

# Classes
The entire package consists of three main classes:
1. hoverboard_driver_node
2. hoverboard_driver
3. pid

whereas the first two are implemented in the same file named hoverboard_driver.cpp (blame on me)

## hoverboard_driver
This is the main class and implements the hardare interface for diff_drove_controller itself. This class is responsible for read/write to hardware.

## hoverboard_driver_node
This class is a member/attribute of hoverboard_driver class and acts as a helper class. As hoverboard_driver itselfs derives from hardware_interface::SystemInterface class, hoverboard_driver class is not able to define publishers or settable parameters. To overcome this, hoverboard_driver_node has been introduced. This class derives from rclcpp::Node and can therefore publish topics and also can define reconfigurable parameters. 

This class publishes state of Hoverboard PCB like velocity, pose, command, voltage, temperature, battery level,current consumption of each motor and the state of serial interface(connected, not connected).

Also this class provides dynamic parameters for PID controller

## pid
The PID class is defined as /attribute of overboard_Driver class (array, of PIDs, one per wheel).
It works in general but it's not active now as I idn't find proper PID settings so far.
To activate, change this code section in hoverboard_driver.cpp

```    // Convert PID outputs in RAD/S to RPM
    //double set_speed[2] = {
     //   pid_outputs[0] / 0.10472,
      //  pid_outputs[1] / 0.10472};

     double set_speed[2] = {
           hw_commands_[left_wheel] / 0.10472,
           hw_commands_[right_wheel] / 0.10472
     };
     ```

# TODO
- add serial port as argument to launch file
- add working PID controller
- mapping /cmd_vel to hoverboard_driver_base/cmd_vel_unstamped not working now
- split hoverboard_driver.cpp classes into separate files
- clean up name mixup between hoverboard and diffbot to be more clear
