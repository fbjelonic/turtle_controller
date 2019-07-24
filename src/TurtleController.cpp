#include "turtle_controller/TurtleController.hpp"

namespace turtle_controller {

TurtleController::TurtleController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), actionHandle_(nodeHandle, "", false)
{
	// Check if Parameters from Parameter file is available
	if (!readParameters()) {
	    ROS_ERROR("Could not read parameters.");
	    ros::requestShutdown();
	}
	// check if a feasible controller is selected
	if (controllerParam_!=1 && controllerParam_!=2){
		ROS_ERROR("You entered a wrong Parameter for the launch file! Argument \" controller \" should be either 1 or 2.");
		ros::requestShutdown();
	}
	try {
		actionHandle_.registerGoalCallback(boost::bind(&TurtleController::actionGoalCallBack, this));
	} catch (int i){
		ROS_ERROR("Could not create Ros Actionserver");
	}

	actionHandle_.registerPreemptCallback(boost::bind(&TurtleController::actionPreemptCallBack, this));

	ROS_INFO("Start subscribing to topic %s", subscriberTopic_.c_str());

	subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
											&TurtleController::topicCallback, this);

	publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(publisherTopic_, queueSize_);

	ROS_INFO("Trying to start action server");
	actionHandle_.start();
	ROS_INFO("Successfully launched node.");
}

TurtleController::~TurtleController()
{
}

bool TurtleController::readParameters()
{
	if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
	if (!nodeHandle_.getParam("publisher_topic", publisherTopic_)) return false;
	if (!nodeHandle_.getParam("queue_size", queueSize_)) return false;
	if (!nodeHandle_.getParam("controller_alg", controllerParam_)) return false;
	return true;
}

void TurtleController::topicCallback(const turtlesim::Pose& pose) {
	//ROS_INFO("I see where you are. You are at: %f and %f " , pose.x , pose.y);
	if (actionHandle_.isActive()) {
		if (pow(pose.x - goal_->x, 2) + pow(pose.y - goal_->y, 2) > 0.01) {
			if (controllerParam_==1){
				vel = algorithm_.goToGoal(pose, goal_);
			}
			else {
				vel = algorithm_.goToGoalPID(pose, goal_);
			}
			feedback_.distance_to_goal = algorithm_.getDistance();
			ROS_INFO_THROTTLE(0.5,
					"Goal is %2f cm away, Wall safety Variable is: %2f", feedback_.distance_to_goal, algorithm_.getbumpSafety());
			actionHandle_.publishFeedback(feedback_);
		} else {
			result_.result = true;
			actionHandle_.setSucceeded(result_);
			if (result_.result) {
				ROS_INFO("The turtle reached the goal!");
			}
			else {
				ROS_INFO("The turtle had problems reaching the goal");
			}
			vel.angular.z = 0;
			vel.linear.x = 0;
		}
		publisher_.publish(vel);
	}
}

void TurtleController::actionGoalCallBack()
{
	ROS_INFO("I got your call!");
	try {
		goal_ = actionHandle_.acceptNewGoal();
		ROS_INFO("Goal is accepted!");
	} catch (int i) {
		ROS_ERROR("The goal is not accepted!");
	}
	ROS_INFO("You entered the goal x=%f and y=%F", goal_->x, goal_->y);
}

void TurtleController::actionPreemptCallBack()
{
	// set the action state to preempted
	actionHandle_.setPreempted();
	ROS_INFO("Action Server Preempted");
}

}
