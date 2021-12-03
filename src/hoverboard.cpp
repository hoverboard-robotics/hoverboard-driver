#include "config.h"
#include "hoverboard.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dynamic_reconfigure/server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

Hoverboard::Hoverboard() {
    hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel",
								 &joints[0].pos.data,
								 &joints[0].vel.data,
								 &joints[0].eff.data);
    hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel",
								  &joints[1].pos.data,
								  &joints[1].vel.data,
								  &joints[1].eff.data);
    joint_state_interface.registerHandle (left_wheel_state_handle);
    joint_state_interface.registerHandle (right_wheel_state_handle);
    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle left_wheel_vel_handle(
        joint_state_interface.getHandle("left_wheel"),
        &joints[0].cmd.data);
    hardware_interface::JointHandle right_wheel_vel_handle(
        joint_state_interface.getHandle("right_wheel"),
        &joints[1].cmd.data);
    velocity_joint_interface.registerHandle (left_wheel_vel_handle);
    velocity_joint_interface.registerHandle (right_wheel_vel_handle);
    registerInterface(&velocity_joint_interface);

    // These publishers are only for debugging purposes
    vel_pub[0]    = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/velocity", 3);
    vel_pub[1]    = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/velocity", 3);
    pos_pub[0]    = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/position", 3);
    pos_pub[1]    = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/position", 3);    
    cmd_pub[0]    = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/cmd", 3);
    cmd_pub[1]    = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/cmd", 3);
    voltage_pub   = nh.advertise<std_msgs::Float64>("hoverboard/battery_voltage", 3);
    temp_pub      = nh.advertise<std_msgs::Float64>("hoverboard/temperature", 3);
    connected_pub = nh.advertise<std_msgs::Bool>("hoverboard/connected", 3);

    std::size_t error = 0;
    error += !rosparam_shortcuts::get("hoverboard_driver", nh, "hoverboard_velocity_controller/wheel_radius", wheel_radius);
    error += !rosparam_shortcuts::get("hoverboard_driver", nh, "hoverboard_velocity_controller/linear/x/max_velocity", max_velocity);
    error += !rosparam_shortcuts::get("hoverboard_driver", nh, "robaka/direction", direction_correction);
    rosparam_shortcuts::shutdownIfError("hoverboard_driver", error);

    if (!rosparam_shortcuts::get("hoverboard_driver", nh, "port", port)) {
        port = DEFAULT_PORT;
        ROS_WARN("Port is not set in config, using default %s", port.c_str());
    } else {
        ROS_INFO("Using port %s", port.c_str());
    }

    // Convert m/s to rad/s
    max_velocity /= wheel_radius;

    low_wrap = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    high_wrap = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    last_wheelcountR = last_wheelcountL = 0;
    multR = multL = 0;

    ros::NodeHandle nh_left(nh, "pid/left");
    ros::NodeHandle nh_right(nh, "pid/right");
    // Init PID controller
    pids[0].init(nh_left, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[0].setOutputLimits(-max_velocity, max_velocity);
    pids[1].init(nh_right, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[1].setOutputLimits(-max_velocity, max_velocity);

    if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        ROS_FATAL("Cannot open serial port to hoverboard");
        exit(-1);
    }
    
    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);
}

Hoverboard::~Hoverboard() {
    if (port_fd != -1) 
        close(port_fd);
}

void Hoverboard::read() {
    if (port_fd != -1) {
        unsigned char c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
            protocol_recv(c);

        if (i > 0)
	        last_read = ros::Time::now();

        if (r < 0 && errno != EAGAIN)
            ROS_ERROR("Reading from serial %s failed: %d", port.c_str(), r);
    }

    if ((ros::Time::now() - last_read).toSec() > 1) {
        ROS_FATAL("Timeout reading from serial %s failed", port.c_str());

        //publish false when not receiving serial data
        std_msgs::Bool b;
        b.data = false;
        connected_pub.publish(b);
    } else {
		//we must be connected - publish true
        std_msgs::Bool b;
        b.data = true;
        connected_pub.publish(b);
	}
}

void Hoverboard::protocol_recv (char byte) {
    start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

    // Read the start frame
    if (start_frame == START_FRAME) {
        p = (char*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.cmd1 ^
            msg.cmd2 ^
            msg.speedR_meas ^
            msg.speedL_meas ^
	        msg.wheelR_cnt ^
	        msg.wheelL_cnt ^
            msg.batVoltage ^
            msg.boardTemp ^
            msg.cmdLed);

        if (msg.start == START_FRAME && msg.checksum == checksum) {
            std_msgs::Float64 f;

            f.data = (double)msg.batVoltage/100.0;
            voltage_pub.publish(f);

            f.data = (double)msg.boardTemp/10.0;
            temp_pub.publish(f);

            // Convert RPM to RAD/S
            joints[0].vel.data = direction_correction * (abs(msg.speedL_meas) * 0.10472);
            joints[1].vel.data = direction_correction * (abs(msg.speedR_meas) * 0.10472);
            vel_pub[0].publish(joints[0].vel);
            vel_pub[1].publish(joints[1].vel);

            // Process encoder values and update odometry
            on_encoder_update (msg.wheelR_cnt, msg.wheelL_cnt);
        } else {
            ROS_WARN("Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;
}

void Hoverboard::write(const ros::Time& time, const ros::Duration& period) {
    if (port_fd == -1) {
        ROS_ERROR("Attempt to write on closed serial");
        return;
    }
    // Inform interested parties about the commands we've got
    cmd_pub[0].publish(joints[0].cmd);
    cmd_pub[1].publish(joints[1].cmd);

    double pid_outputs[2];
    pid_outputs[0] = pids[0](joints[0].vel.data, joints[0].cmd.data, period);
    pid_outputs[1] = pids[1](joints[1].vel.data, joints[1].cmd.data, period);

    // Convert PID outputs in RAD/S to RPM
    double set_speed[2] = {
        pid_outputs[0] / 0.10472,
        pid_outputs[1] / 0.10472
    };

    // Calculate steering from difference of left and right
    const double speed = (set_speed[0] + set_speed[1])/2.0;
    const double steer = (set_speed[0] - speed)*2.0;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(port_fd, (const void*)&command, sizeof(command));
    if (rc < 0) {
        ROS_ERROR("Error writing to hoverboard serial port");
    }
}

void Hoverboard::on_encoder_update (int16_t right, int16_t left) {
    double posL = 0.0, posR = 0.0;

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
        multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
        multR--;
    posR = right + multR*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
        multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
        multL--;
    posL = left + multL*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountL = left;

    // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
    // This section accumulates ticks even if board shuts down and is restarted   
    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;
    
    //IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
    //(the board seems to often report 1-3 ticks on startup instead of zero)
    //reset the last read ticks to the startup values
    if((ros::Time::now() - last_read).toSec() > 0.2
		&& abs(posL) < 5 && abs(posR) < 5){
            lastPosL = posL;
            lastPosR = posR;
	}
    double posLDiff = 0;
    double posRDiff = 0;

    //if node is just starting keep odom at zeros
	if(nodeStartFlag){
		nodeStartFlag = false;
	}else{
            posLDiff = posL - lastPosL;
            posRDiff = posR - lastPosR;
	}

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;
    
    // Convert position in accumulated ticks to position in radians
    joints[0].pos.data = 2.0*M_PI * lastPubPosL/(double)TICKS_PER_ROTATION;
    joints[1].pos.data = 2.0*M_PI * lastPubPosR/(double)TICKS_PER_ROTATION;

    pos_pub[0].publish(joints[0].pos);
    pos_pub[1].publish(joints[1].pos);
}
