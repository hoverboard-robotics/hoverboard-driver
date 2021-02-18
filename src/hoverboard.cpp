#include "config.h"
#include "hoverboard.h"
#include "HoverboardAPI.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dynamic_reconfigure/server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int port_fd = -1;

int serialWrite(unsigned char *data, int len) {
    return (int)write(port_fd, data, len);
}

void readCallback(PROTOCOL_STAT* s, PARAMSTAT* param, uint8_t fn_type, unsigned char* content, int len) {
  if (fn_type == FN_TYPE_POST_READRESPONSE) {
    if (param->code == HoverboardAPI::Codes::sensHall) {
      Hoverboard::getInstance().hallCallback();
    } else if (param->code == HoverboardAPI::Codes::sensElectrical) {
      Hoverboard::getInstance().electricalCallback();
    }
  }
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

    const double max_vel = 1.0;
    pids[0].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_vel, -max_vel);
    pids[0].setOutputLimits(-max_vel, max_vel);
    pids[1].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_vel, -max_vel);
    pids[1].setOutputLimits(-max_vel, max_vel);

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
    api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, 50, PROTOCOL_SOM_NOACK);
    api->scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 50, PROTOCOL_SOM_NOACK);
    api->updateParamHandler(HoverboardAPI::Codes::sensHall, readCallback);
    api->updateParamHandler(HoverboardAPI::Codes::sensElectrical, readCallback);
    api->disablePoweroffTimer();
}

Hoverboard::~Hoverboard() {
    if (port_fd != -1) 
        close(port_fd);
    delete api;
}

void Hoverboard::read() {
    if (port_fd != -1) {
        const int max_length = 1024; // HoverboardAPI limit
        unsigned char c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
            api->protocolPush(c);

	if (i > 0)
	  last_read = ros::Time::now();

	if (r < 0 && errno != EAGAIN)
	  ROS_ERROR("Reading from serial %s failed: %d", PORT, r);
    }

    if ((ros::Time::now() - last_read).toSec() > 1) {
      ROS_FATAL("Timeout reading from serial %s failed", PORT);
    }
}

void Hoverboard::hallCallback() {
    // Convert m/s to rad/s
    double sens_speed0 = api->getSpeed0_mms();
    double sens_speed1 = api->getSpeed1_mms();

    joints[0].vel.data = DIRECTION_CORRECTION * (sens_speed0 / 1000.0);
    joints[1].vel.data = DIRECTION_CORRECTION * (sens_speed1 / 1000.0);
    joints[0].pos.data = DIRECTION_CORRECTION * (api->getPosition0_mm() / 1000.0);
    joints[1].pos.data = DIRECTION_CORRECTION * (api->getPosition1_mm() / 1000.0);
    left_vel_pub.publish(joints[0].vel);
    right_vel_pub.publish(joints[1].vel);
    left_pos_pub.publish(joints[0].pos);
    right_pos_pub.publish(joints[1].pos);
    // printf("[%.3f, %.3f] -> [%.3f, %.3f]\n", sens_speed0, sens_speed1, joints[0].vel.data, joints[1].vel.data);
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
    // Inform interested parties about the commands we've got
    left_cmd_pub.publish(joints[0].cmd);
    right_cmd_pub.publish(joints[1].cmd);

    double pid_outputs[2];
    double motor_cmds[2] ;
    pid_outputs[0] = pids[0](joints[0].vel.data, joints[0].cmd.data, period);
    pid_outputs[1] = pids[1](joints[1].vel.data, joints[1].cmd.data, period);

    // TODO - figure out the right PWM setting
    motor_cmds[0] = pid_outputs[0] / max_velocity * 100.0;
    motor_cmds[1] = pid_outputs[1] / max_velocity * 100.0;
    int left_pwm  = motor_cmds[0];
    int right_pwm = motor_cmds[1];

    // int left_pwm  = joints[0].cmd.data * 30;
    // int right_pwm = joints[1].cmd.data * 30;
    // api->sendDifferentialPWM (left_pwm, right_pwm);
}

void Hoverboard::tick() {
    api->protocolTick();
}
