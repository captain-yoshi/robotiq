#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/Robotiq2fGripperCommandAction.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gripper_2f_gripper_action_client");

  ros::NodeHandle pnh("~");

  std::string action_server_name;
  double gap_size;
  double speed;
  double effort_scaling;
  double timeout;

  pnh.getParam("action_server_name", action_server_name);
  pnh.getParam("gap_size", gap_size);
  pnh.getParam("speed", speed);
  pnh.getParam("effort_scaling", effort_scaling);
  pnh.getParam("timeout", timeout);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<control_msgs::Robotiq2fGripperCommandAction> ac(
      action_server_name, true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  bool server_exists =
      ac.waitForServer(ros::Duration(1.0)); // will wait for infinite time

  if (!server_exists) {
    ROS_ERROR_STREAM("Cannot reach the Action Server: " << action_server_name);
    return 0;
  }

  // send a goal to the action
  control_msgs::Robotiq2fGripperCommandGoal goal;
  goal.command.position = gap_size;
  goal.command.speed = speed;
  goal.command.effort_scaling = effort_scaling;

  ROS_INFO_STREAM("Action server started, sending goal:\n" << goal);

  ac.sendGoal(goal);

  if (ac.waitForResult(ros::Duration(timeout))) {
    actionlib::SimpleClientGoalState state = ac.getState();

    if (state.state_ == state.SUCCEEDED) {
      ROS_INFO("Action Succeeded");

      auto result = ac.getResult();

      std::cout << "==== Results ====" << std::endl;
      std::cout << "current gap size = " << result->position << std::endl;
      std::cout << "current effort   = " << result->effort << std::endl;
      std::cout << "reached goal     = " << +result->reached_goal << std::endl;
      std::cout << "stalled          = " << +result->stalled << std::endl;

    } else {
      ROS_ERROR("Action finished: %s", state.toString().c_str());
    }
  } else {
    ROS_ERROR("Action did not finish before the time out.");
    ac.cancelGoal();
  }

  // exit
  return 0;
}
