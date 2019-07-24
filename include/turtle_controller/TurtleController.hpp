#pragma once

#include "turtle_controller/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <turtle_controller/goToGoalAction.h>
#include <cmath>
#include "turtlesim/Pose.h"

namespace turtle_controller {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class TurtleController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TurtleController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TurtleController();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const turtlesim::Pose &pose);

  void actionGoalCallBack();

  void actionPreemptCallBack();

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  actionlib::SimpleActionServer<turtle_controller::goToGoalAction> actionHandle_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  ros::Publisher publisher_;

  std::string publisherTopic_;

  int controllerParam_;

  int queueSize_;

  geometry_msgs::Twist vel;

  boost::shared_ptr<const turtle_controller::goToGoalGoal> goal_;
  turtle_controller::goToGoalFeedback feedback_;
  turtle_controller::goToGoalResult result_;


  Algorithm algorithm_;

};

} /* namespace */
