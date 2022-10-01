#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_robotiq_2f_gripper_action_server");

  ros::NodeHandle pnh("~");

  std::string gripper_name;
  double position;
  double max_effort;
  int max_retry;
  double timeout;
  pnh.param<std::string>("gripper_name", gripper_name, "gripper");
  pnh.param<double>("position", position, 0.0);
  pnh.param<double>("max_effort", max_effort, 0.0);
  pnh.param<int>("max_retry", max_retry, 10);
  pnh.param<double>("timeout", timeout, 1.0);

  // create the action client
  // true causes the client to spin its own thread
  ROS_INFO_STREAM(gripper_name);
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac(
      gripper_name, true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  control_msgs::GripperCommandGoal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  bool success = false;
  for (std::size_t i = 0; i < max_retry; ++i) {
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout));

    if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
      success = true;
      break;
    }
  }

  if (!success)
    ROS_WARN(
        "Action did not complete. Tried %d times with a timeout of %f seconds.",
        max_retry, timeout);

  // exit
  return 0;
}
