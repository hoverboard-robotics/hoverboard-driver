#include "config.h"
#include "hoverboard.h"
#include "HoverboardAPI.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


int port_fd = -1;

int serialWrite(unsigned char *data, int len) {
    return (int)write(port_fd, data, len);
}

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

    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        ROS_FATAL("Cannot open serial port to hoverboard");
        exit(-1);
    }
    
    //CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	struct termios options;
	tcgetattr(port_fd, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(port_fd, TCIFLUSH);
	tcsetattr(port_fd, TCSANOW, &options);

    api = new HoverboardAPI(serialWrite);
    api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);
}

Hoverboard::~Hoverboard() {
    if (port_fd != -1) 
        close(port_fd);

    delete api;
}

void Hoverboard::read() {
   api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);

   if (port_fd != -1) {
       const int max_length = 1024; // HoverboardAPI limit
       unsigned char c;
       int i = 0, r = 0;

       while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024) {
           api->protocolPush(c);
       }

       if (r < 0) {
           ROS_ERROR("Reading from serial %s failed: %d", PORT, r);
       }
   }

   joints[0].vel = api->getSpeed0_mms();
   joints[1].vel = api->getSpeed1_mms();
}

void Hoverboard::write() {
    if (port_fd == -1) {
        ROS_ERROR("Attempt to write on closed serial");
        return;
    }

    api->sendPWM(joints[0].cmd, joints[1].cmd, PROTOCOL_SOM_NOACK);
}

void Hoverboard::tick() {
    api->protocolTick();
}
