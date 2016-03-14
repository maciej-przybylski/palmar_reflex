#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>

/* My Arduino is on /dev/ttyACM0 */
// const char *portname = std::string("/dev/ttyACM0").c_str();
// const char *portname = "/dev/ttyACM1";
char *portname;
char buf[100];

int main(int argc, char **argv)
{
	if(argc!=2)
	{
		printf("Usage: break_beam_publisher portname\n");
		printf("e.g. break_beam_publisher /dev/ttyACM0\n");
		exit(1);
	}
	
	ros::init(argc, argv, "break_beam_publisher");
	ros::NodeHandle node;

	ros::Publisher break_beam_pub = node.advertise<std_msgs::String>("break_beam", 1000);
	ros::Rate loop_rate(10);
	std_msgs::String msg;

	int fd;
	portname = argv[1];
	/* Open the file descriptor in non-blocking mode */
	fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)  {
		printf("init_serialport %d:  Unable to open port %s\n", fd, portname);
		return -1;
	}
	
	/* Set up the control structure */
	struct termios toptions;
	
	/* Get currently set options for the tty */
	if (tcgetattr(fd, &toptions) < 0) {
		printf("init_serialport: Couldn't get term attributes\n");
		return -1;
	}
	
	/* Set custom options */
	
	/* 9600 baud */
	cfsetispeed(&toptions, B9600);
	cfsetospeed(&toptions, B9600);
	/* 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	/* no hardware flow control */
	toptions.c_cflag &= ~CRTSCTS;
	/* enable receiver, ignore status lines */
	toptions.c_cflag |= CREAD | CLOCAL;
	/* disable input/output flow control, disable restart chars */
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
	/* disable canonical input, disable echo,
	 d isable visually erase ch*ars,
	 disable terminal-generated signals */
	toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/* disable output processing */
	toptions.c_oflag &= ~OPOST;
	
	/* wait for 24 characters to come in before read returns */
	toptions.c_cc[VMIN] = 0;
	/* no minimum time to wait before read returns */
	toptions.c_cc[VTIME] = 0;
	
	/* commit the options */
	if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
		printf("init_serialport: Couldn't set term attributes");
		return -1;
	}
	
	/* Wait for the Arduino to reset */
	usleep(1000*1000);
	/* Flush anything already in the serial buffer */
	tcflush(fd, TCIFLUSH);
	/* read up to 128 bytes from the fd */
	memset(buf, 0, sizeof(buf));
	char b[1];
	b[0]='.';
// 	int n = read(fd, b, 1);
	int n;
	int i=0;
	while (1)
	{
		i = 0;
		do{
			n = read(fd, b, 1);	
			while (n <= 0) { n = read(fd, b, 1);	}
			if(b[0]!='\n')
				buf[i] = b[0];
			i++;
		}while(b[0] != '\n');
		
		if(buf[0]=='U')
		{
			std::stringstream ss;
			ss << "Unbroken";
			msg.data = ss.str();
			ROS_INFO("publish %s", msg.data.c_str());
			break_beam_pub.publish(msg);
		}else if(buf[0]=='B')
		{
			std::stringstream ss;
			ss << "Broken";
			msg.data = ss.str();
			ROS_INFO("publish %s", msg.data.c_str());
			break_beam_pub.publish(msg);
		}
	}
	
	return 0;
}
