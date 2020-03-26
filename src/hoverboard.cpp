#include "config.h"
#include "hoverboard.h"
#include "HoverboardAPI.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dynamic_reconfigure/server.h>

int port_fd = -1;

int serialWrite(unsigned char *data, int len) {
    return (int)write(port_fd, data, len);
}

Hoverboard::Hoverboard() {
    hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel", &joints[0].pos.data, &joints[0].vel.data, &joints[0].eff.data);
    hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel", &joints[1].pos.data, &joints[1].vel.data, &joints[1].eff.data);
    joint_state_interface.registerHandle (left_wheel_state_handle);
    joint_state_interface.registerHandle (right_wheel_state_handle);
    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle left_wheel_vel_handle(joint_state_interface.getHandle("left_wheel"), &joints[0].cmd.data);
    hardware_interface::JointHandle right_wheel_vel_handle(joint_state_interface.getHandle("right_wheel"), &joints[1].cmd.data);
    velocity_joint_interface.registerHandle (left_wheel_vel_handle);
    velocity_joint_interface.registerHandle (right_wheel_vel_handle);
    registerInterface(&velocity_joint_interface);

    // These publishers are only for debugging purposes
    left_pos_pub = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/position", 3);
    right_pos_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/position", 3);
    left_vel_pub = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/velocity", 3);
    right_vel_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/velocity", 3);
    left_eff_pub = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/eff", 3);
    right_eff_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/eff", 3);
    left_cmd_pub = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/cmd", 3);
    right_cmd_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/cmd", 3);
    
    // FIXME! Read parameters from ROS
    wheel_radius = WHEEL_RADIUS;

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
    api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, 20, PROTOCOL_SOM_NOACK);
    api->scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 20, PROTOCOL_SOM_NOACK);

    // Support dynamic reconfigure
    dsrv = new dynamic_reconfigure::Server<hoverboard_driver::HoverboardConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<hoverboard_driver::HoverboardConfig>::CallbackType cb = boost::bind(&Hoverboard::reconfigure_callback, this, _1, _2);
    dsrv->setCallback(cb);

}

Hoverboard::~Hoverboard() {
    if (port_fd != -1) 
        close(port_fd);

    delete api;
}

void Hoverboard::reconfigure_callback(hoverboard_driver::HoverboardConfig& _config, uint32_t level) {
  config = _config;
  have_config = true;

  printf("Reconfigured PID to [%d, %d, %d, %d]\n", config.Kp, config.Ki, config.Kd, config.Incr);
  api->sendPIDControl(config.Kp, config.Ki, config.Kd, config.Incr);
}

void Hoverboard::read() {
    if (port_fd != -1) {
        const int max_length = 1024; // HoverboardAPI limit
        unsigned char c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024) {
            api->protocolPush(c);
        }

	if (i > 0) {
	  last_read = ros::Time::now();
	}

	if (r < 0 && errno != EAGAIN) {
	  ROS_ERROR("Reading from serial %s failed: %d", PORT, r);
	}
    }

    if ((ros::Time::now() - last_read).toSec() > 1) {
      ROS_FATAL("Timeout reading from serial %s failed", PORT);
    }

    // Convert m/s to rad/s
    double sens_speed0 = api->getSpeed0_mms();
    double sens_speed1 = api->getSpeed1_mms();

    //    printf("[%.3f] ", ros::Time::now().toSec());

    // Basic sanity check, speed should be less than 10 m/s
    // Sometimes, it seems during EMI peaks, we're getting ridiculous values here
    // Don't know what to do with it, just ignoring for now
    if (fabs(sens_speed0) < 10000 && fabs(sens_speed1) < 10000) { 
      joints[0].vel.data = DIRECTION_CORRECTION * (sens_speed0 / 1000.0) / wheel_radius;
      joints[1].vel.data = DIRECTION_CORRECTION * (sens_speed1 / 1000.0) / wheel_radius;
      joints[0].pos.data = DIRECTION_CORRECTION * (api->getPosition0_mm() / 1000.0) / wheel_radius;
      joints[1].pos.data = DIRECTION_CORRECTION * (api->getPosition1_mm() / 1000.0) / wheel_radius;

      left_vel_pub.publish(joints[0].vel);
      right_vel_pub.publish(joints[1].vel);
      left_pos_pub.publish(joints[0].pos);
      right_pos_pub.publish(joints[1].pos);
    } else {
      //      printf(" Stupid speeds of [%.2f, %.2f] ignored\n", sens_speed0, sens_speed1);
    }
}

void Hoverboard::write() {
    if (port_fd == -1) {
        ROS_ERROR("Attempt to write on closed serial");
        return;
    }

    // Inform interested parties about the commands we've got
    left_cmd_pub.publish(joints[0].cmd);
    right_cmd_pub.publish(joints[1].cmd);

    // Convert rad/s to m/s
    double left_speed = DIRECTION_CORRECTION * joints[0].cmd.data * wheel_radius;
    double right_speed = DIRECTION_CORRECTION * joints[1].cmd.data * wheel_radius;

    // Cap according to dynamic_reconfigure
    const int max_power = have_config ? config.MaxPwr : 100;
    const int min_speed = have_config ? config.MinSpd : 40;
    api->sendSpeedData (left_speed, right_speed, max_power, min_speed);
}

void Hoverboard::tick() {
    api->protocolTick();
    //    api->printStats();
}
