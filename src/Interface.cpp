#include "turtle_controller/Interface.hpp"

namespace interface_control {
/*!
 * Main class for the node to handle the ROS interfacing.
 */
Interface::Interface(ros::NodeHandle& nodeHandle)
	: nodeHandle_(nodeHandle), actionHandle_("turtle_controller", true)
{
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	ROS_INFO("Waiting for %s action server to start.", actionTopic_.c_str());

	bool waiting = actionHandle_.waitForServer(ros::Duration(5.0));

	if (waiting) {
		ROS_INFO("Action server started, sending a goal is ready.");
	} else {
		ROS_ERROR("Could not connect to action server!");
	}
	serviceServer_ = nodeHandle_.advertiseService("set_goal",
			&Interface::serviceCallback, this);
}

/*!
 * Destructor.
 */
Interface::~Interface()
{
}

bool Interface::readParameters() {
	if (!nodeHandle_.getParam("action_topic", actionTopic_)) return false;
	return true;
}

bool Interface::serviceCallback(turtle_controller::goal::Request& request,
                     turtle_controller::goal::Response& response)
{
	 goal_.x= request.x;
	 goal_.y = request.y;
	 ROS_INFO("Successfully received your goal.");
	 actionHandle_.sendGoal(goal_);
	 return true;
}

} // namespace interface_control
