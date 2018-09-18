/*
Node to control a TalonSRX device on position servo mode.
Internally, it subscribes to the /received_messages and /sent_messages topics of the type can_msgs::Frame from the socketcan_bridge package to talk to the Talon through CAN protocol. Keep in mind that to work properly it is required for your system to have a properly working CAN interface. A list of USB-CAN drivers/devices that are known to work with the socketcan_bridge package may ben fond at wiki.ros.org/socketcan_interface.
This node also publishes the /talonsrx/current_position
*/

#include <talon_bridge/talon.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talon_srx");
	ros::NodeHandle n;

	talon::TalonSRX talon(&n);
	talon.setup(15, modeServoPosition); 

	ros::spin();
	return 0;
}
