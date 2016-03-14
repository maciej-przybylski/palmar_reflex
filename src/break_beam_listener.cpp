/**
 * \file break_beam_listener.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief listen to topic /break_beam and close left gripper when the beams are broken
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include <cstring>

ros::Publisher gripper_publiser;
char break_beam[]="Broken";

void breakbeamCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("listen %s", msg->data.c_str());
	if(strcmp(msg->data.c_str(), break_beam)==0)
	{
		baxter_core_msgs::EndEffectorCommand gripper_command;
		gripper_command.id = 65664;
	// 	gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_CONFIGURE;
		//{"holding_force": 1e9*dmp, "velocity": 50.0, "dead_zone": 5.0, "moving_force": 20.0}
		std::stringstream gripper_command_args;
		gripper_command_args << "{\"holding_force\": " << 80 << ", \"velocity\": 50.0, \"dead_zone\": 5.0, \"moving_force\": 20.0}";
		gripper_command.args = gripper_command_args.str();
		gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
		gripper_publiser.publish(gripper_command);
	}
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "break_beam_listener");
	
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;
	
	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber sub = n.subscribe("break_beam", 1000, breakbeamCallback);
	gripper_publiser = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
	
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();
	
	return 0;
}