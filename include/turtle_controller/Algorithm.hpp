#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtle_controller/goToGoalAction.h>
#include <math.h>

namespace turtle_controller {

struct dist {
	float x;
	float y;
};

struct system_var {
	float angle;
	float distance;
};

struct control {
	float kp;
	float kd;
	float ki;
};

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  geometry_msgs::Twist goToGoal(const turtlesim::Pose &pose, const boost::shared_ptr<const turtle_controller::goToGoalGoal> &goal);

  geometry_msgs::Twist goToGoalPID(const turtlesim::Pose &pose, const boost::shared_ptr<const turtle_controller::goToGoalGoal> &goal);

  float getDistance();

  float getbumpSafety();

 private:

  float getDiffAngle(float);

  float wallbumpSafety(const turtlesim::Pose &pose);

  float bumpSafety;

  struct dist distance;

  struct system_var old_deviation;

  struct control control_angle;
  struct control control_speed;

  float pi;



};

} /* namespace */
