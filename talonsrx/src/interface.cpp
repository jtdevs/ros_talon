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
