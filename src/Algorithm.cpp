#include "turtle_controller/Algorithm.hpp"

namespace turtle_controller {

Algorithm::Algorithm()
	: distance{0,0}, old_deviation{0,0}, control_angle{1,1,1}, control_speed{1,1,0.1}, pi(atan(1)*4), bumpSafety(1)
{
}

Algorithm::~Algorithm()
{
}

geometry_msgs::Twist Algorithm::goToGoal(const turtlesim::Pose &pose, const boost::shared_ptr<const turtle_controller::goToGoalGoal> &goal)
{
	geometry_msgs::Twist vel;
	distance.x = goal->x - pose.x;
	distance.y = goal->y - pose.y;
//	ROS_INFO("Difference is: %f and %f", distance.x,distance.y);
	float angle = Algorithm::getDiffAngle(pose.theta);
	vel.angular.z = angle;
	if ((angle<pi/2 && angle > 0) || (-angle<pi/2 && angle < 0)){
		vel.linear.x=1;
	}
	else {
		vel.linear.x=-1;
	}
	return vel;
}

geometry_msgs::Twist Algorithm::goToGoalPID(const turtlesim::Pose &pose, const boost::shared_ptr<const turtle_controller::goToGoalGoal> &goal)
{
	geometry_msgs::Twist vel;
	distance.x = goal->x - pose.x;
	distance.y = goal->y - pose.y;
	float angle = Algorithm::getDiffAngle(pose.theta);
	float distance = getDistance();
	bumpSafety = wallbumpSafety(pose);
	vel.angular.z = (angle*control_angle.kp
			+ (angle - old_deviation.angle) * control_angle.kd
			+ (angle + old_deviation.angle) * control_angle.ki)
			* bumpSafety;
	vel.linear.x= (distance * control_speed.kp
			+ (distance - old_deviation.distance) * control_speed.kd
			+ (distance + old_deviation.distance) * control_speed.ki)
			/ bumpSafety;
//	ROS_INFO("Bumpsafety: %f2 and angular speed: %f2",bumpSafety, vel.angular.z);

	old_deviation.angle = angle;
	old_deviation.distance = distance;
	return vel;
}

float Algorithm::getDistance()
{
	return sqrt(pow(distance.x,2)+pow(distance.y,2));
}

float Algorithm::getbumpSafety()
{
	return bumpSafety;
}

float Algorithm::wallbumpSafety(const turtlesim::Pose &pose){
	/* This function keeps the turtle away from the boundaries. The closer it
	 * comes to the edges of the map, the higher this bump coefficient increases.
	 * It will directly increase the angular and decrease the linear velocity. */
	float bump;
	bump=(pow(exp((1-pose.x)),4)+pow(exp((1-pose.y)),4)+pow(exp((pose.x-10)),4)+pow(exp((pose.y-10)),4))+1;
//	bump = 1;
	if (bump>=20) {
		bump=20;
	}
	return bump;
}

float Algorithm::getDiffAngle(float turtle)
{
	if (turtle < 0){
		turtle = 2*pi+turtle;
	}
//	ROS_INFO("Turtle angle is: %f", turtle);
	float goalAngle;

	// Get the angle between turtle and goal in reference to x=1, y=0 vector
	// 0 < goalAngle < 2 * pi
	if (distance.x>=0 && distance.y>=0){
		goalAngle=atan(distance.y/distance.x);
	}
	else if (distance.x<0 && distance.y>=0) {
		goalAngle=pi-atan(distance.y/(-distance.x));
	}
	else if (distance.x<0 && distance.y<0) {
			goalAngle=pi+atan(distance.y/distance.x);
		}
	else if (distance.x>=0 && distance.y<0) {
			goalAngle=2*pi-atan((-distance.y)/distance.x);
		}
//	ROS_INFO("Goal angle is: %f", goalAngle);

	// The turtle must be twisted by this return angle, to get the !!shortest!! rotation towards the goal
	// -pi < return <= pi
	float angle = goalAngle-turtle;
	if (angle >= pi) {
		return (angle - 2 * pi);
	}
	else if (angle < - pi) {
		return (2 * pi + angle);
	}
	else {
		return angle;
	}
}

}

