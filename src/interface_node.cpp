#include <ros/ros.h>
#include "turtle_controller/Interface.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "interface");
	ros::NodeHandle nodeHandle("~");

	interface_control::Interface interface(
			nodeHandle);

	ros::spin();
	return 0;
}
