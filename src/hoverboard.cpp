#include "config.h"
#include "hoverboard.h"
#include "HoverboardAPI.h"
#include "protocolFunctions.h" // temp

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dynamic_reconfigure/server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int port_fd = -1;
int serialWrite(unsigned char *data, int len) {
    return (int)write(port_fd, data, len);
}

Hoverboard& Hoverboard::getInstance() {
  static Hoverboard hoverboard;
  return hoverboard;
}

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

    hardware_interface::JointHandle left_wheel_vel_handle(joint_state_interface.getHandle("left_wheel"),
							  &joints[0].cmd.data);
    hardware_interface::JointHandle right_wheel_vel_handle(joint_state_interface.getHandle("right_wheel"),
							   &joints[1].cmd.data);
    velocity_joint_interface.registerHandle (left_wheel_vel_handle);
    velocity_joint_interface.registerHandle (right_wheel_vel_handle);
    registerInterface(&velocity_joint_interface);

    // These publishers are only for debugging purposes
    left_pos_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/position", 3);
    right_pos_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/position", 3);
    left_vel_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/velocity", 3);
    right_vel_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/velocity", 3);
    left_eff_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/eff", 3);
    right_eff_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/eff", 3);
    left_cmd_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/cmd", 3);
    right_cmd_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/cmd", 3);
    left_cur_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/current", 3);
    right_cur_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/current", 3);
    voltage_pub   = nh.advertise<std_msgs::Float64>("hoverboard/battery_voltage", 3);
    
    std::size_t error = 0;
    error += !rosparam_shortcuts::get("hoverboard_driver", nh, "hoverboard_velocity_controller/wheel_radius", wheel_radius);
    error += !rosparam_shortcuts::get("hoverboard_driver", nh, "hoverboard_velocity_controller/linear/x/max_velocity", max_velocity);
    rosparam_shortcuts::shutdownIfError("hoverboard_driver", error);

    // Convert m/s to rad/s
    max_velocity /= wheel_radius;

    ros::NodeHandle nh_left(nh, "pid/left");
    ros::NodeHandle nh_right(nh, "pid/right");
    
    // Init PID controller
    pids[0].init(nh_left, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[0].setOutputLimits(-max_velocity, max_velocity);
    pids[1].init(nh_right, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[1].setOutputLimits(-max_velocity, max_velocity);

    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
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

    api = new HoverboardAPI(serialWrite);
}

Hoverboard::~Hoverboard() {
    if (port_fd != -1) 
        close(port_fd);
    delete api;
}

void Hoverboard::read() {
    int i = 0, j = 0;
    
    if (port_fd != -1) {
        const int max_length = 1024; // HoverboardAPI limit
        unsigned char c;
        int r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < max_length)
            api->protocolPush(c);

	if (i > 0)
	  last_read = ros::Time::now();

	if (r < 0 && errno != EAGAIN)
	  ROS_ERROR("Reading from serial %s failed: %d", PORT, r);
    }

    // FIXME -- scheduled reads are broken in the latest version of hoverboard protocol
    // therefore triggering callbacks manually
    Hoverboard::getInstance().hallCallback();
    Hoverboard::getInstance().electricalCallback();
	    
    if ((ros::Time::now() - last_read).toSec() > 1) {
      ROS_FATAL("Timeout reading from serial %s failed", PORT);
    }
}

void Hoverboard::hallCallback() {

    // int32_t HallPosn; // 90 per revolution
    // int32_t HallSpeed; // speed part calibrated to speed demand value

    // float HallPosnMultiplier; // m per hall segment

    // int32_t HallPosn_lastread; // posn offset set via protocol in raw value
    // int32_t HallPosn_mm; // posn in mm
    // int32_t HallPosn_mm_lastread; // posn offset set via protocol in mm
    // int32_t HallSpeed_mm_per_s; // speed in m/s

    // uint32_t HallTimeDiff;
    // uint32_t HallSkipped;

    for (int i = 0; i < 2; i++)
	printf ("P: %d S: %d LR: %d MM: %d MMLR: %d MMPS: %d TD: %d SKIP: %d\n",
	    HallData[i].HallPosn,
	    HallData[i].HallSpeed,
	    HallData[i].HallPosn_lastread,
	    HallData[i].HallPosn_mm,
	    HallData[i].HallPosn_mm_lastread,
	    HallData[i].HallSpeed_mm_per_s,
	    HallData[i].HallTimeDiff,
	    HallData[i].HallSkipped);	    
    printf("\n-\n");
    
    // Convert m/s to rad/s
    double sens_speed0 = api->getSpeed0_mms();
    double sens_speed1 = api->getSpeed1_mms();

    if (fabs(sens_speed0-last_left_speed) > 1000.0 || fabs(sens_speed1-last_right_speed) > 1000.0) {
	ROS_ERROR("Absurd values: %.2f %.2f", sens_speed0, sens_speed1);
	return;
    }

    joints[0].vel.data = DIRECTION_CORRECTION * (sens_speed0 / 1000.0) / wheel_radius;
    joints[1].vel.data = DIRECTION_CORRECTION * (sens_speed1 / 1000.0) / wheel_radius;
    joints[0].pos.data = DIRECTION_CORRECTION * (api->getPosition0_mm() / 1000.0);
    joints[1].pos.data = DIRECTION_CORRECTION * (api->getPosition1_mm() / 1000.0);
    left_vel_pub.publish(joints[0].vel);
    right_vel_pub.publish(joints[1].vel);
    left_pos_pub.publish(joints[0].pos);
    right_pos_pub.publish(joints[1].pos);
    
//    printf("[%.3f, %.3f] -> [%.3f, %.3f]\n", sens_speed0, sens_speed1, joints[0].vel.data, joints[1].vel.data);
    last_left_speed = sens_speed0;
    last_right_speed = sens_speed1;
}

void Hoverboard::electricalCallback() {
    std_msgs::Float64 f;

    f.data = api->getMotorAmpsAvg(0);
    left_cur_pub.publish(f);

    f.data = api->getMotorAmpsAvg(1);
    right_cur_pub.publish(f);

    f.data = api->getBatteryVoltage();
    voltage_pub.publish(f);
}

void Hoverboard::write(const ros::Time& time, const ros::Duration& period) {
    if (port_fd == -1) {
        ROS_ERROR("Attempt to write on closed serial");
        return;
    }

    // joints[i].cmd -- speed for wheel rotation in rad/s
    left_cmd_pub.publish(joints[0].cmd);
    right_cmd_pub.publish(joints[1].cmd);

    double pid_outputs[2];
    double motor_cmds[2] ;

    // use PID controller to determine the right _speed_ in rad/s,
    // given commanded speed (.cmd) and last known actual speed (.vel)
    
    pid_outputs[0] = pids[0](joints[0].vel.data, joints[0].cmd.data, period);
    pid_outputs[1] = pids[1](joints[1].vel.data, joints[1].cmd.data, period);

    // Get motor CMD as a ratio of full max_velocity in rad/s, range 0..1 
    motor_cmds[0] = (pid_outputs[0] / max_velocity);
    motor_cmds[1] = (pid_outputs[1] / max_velocity);

    // Use motor cmd range with max_pwm
    int max_pwm   = 200; // Empirically chosen value. Control the rest with PID.
    int left_pwm  = motor_cmds[0] * max_pwm;
    int right_pwm = motor_cmds[1] * max_pwm;

//    printf("PWM CMD [%.2f, %.2f] -> [%.2f, %.2f] -> [%d, %d]\n",
//    	   joints[0].cmd.data, joints[1].cmd.data,
//    	   pid_outputs[0], pid_outputs[1],
//     	   left_pwm, right_pwm);

    api->sendDifferentialPWM (left_pwm, right_pwm);
    api->requestRead(HoverboardAPI::Codes::sensHall);
//    api->requestRead(HoverboardAPI::Codes::sensElectrical);    
//    api->sendSpeedData(pid_outputs[0]*wheel_radius, pid_outputs[1]*wheel_radius, 600, 40);
}

void Hoverboard::tick() {
    api->protocolTick();
}
