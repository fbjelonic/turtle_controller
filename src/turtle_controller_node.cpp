#include <ros/ros.h>
#include "turtle_controller/TurtleController.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_controller");
	ros::NodeHandle nodeHandle("~");

	turtle_controller::TurtleController turtle_controller(
			nodeHandle);

	ros::spin();
	return 0;
}
