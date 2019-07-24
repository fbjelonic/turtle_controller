#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_controller/goToGoalAction.h>
#include <std_srvs/Trigger.h>
#include <turtle_controller/goal.h>
#include <string>

namespace interface_control {
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class Interface
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Interface(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Interface();

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
  void goalCallback();

  bool serviceCallback(turtle_controller::goal::Request& request,
                       turtle_controller::goal::Response& response);

  ros::ServiceServer serviceServer_;
  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  actionlib::SimpleActionClient<turtle_controller::goToGoalAction> actionHandle_;

  turtle_controller::goToGoalGoal goal_;

  std::string actionTopic_;
};

} /* namespace */
