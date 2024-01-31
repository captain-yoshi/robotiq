#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"

int main(int argc, char **argv) {
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "robotiq_2f_gripper_action_server");

  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string action_server_name;
  robotiq_2f_gripper_action_server::Robotiq2FGripperParams params;

  private_nh.getParam("action_server_name", action_server_name);
  private_nh.getParam("min_gap_size", params.min_gap_size);
  private_nh.getParam("max_gap_size", params.max_gap_size);
  private_nh.getParam("min_speed", params.min_speed);
  private_nh.getParam("max_speed", params.max_speed);

  std::cout << "action_server_name" << action_server_name << std::endl;
  std::cout << "min_gap_size" << params.min_gap_size << std::endl;
  std::cout << "max_gap_size" << params.max_gap_size << std::endl;
  std::cout << "min_speed" << params.min_speed << std::endl;
  std::cout << "max_speed" << params.max_speed << std::endl;

  // Fill out 2F-Gripper Params

  // Min because fingers can push forward before the mechanical stops are
  // reached
  // private_nh.param<double>("min_effort", cparams.min_effort_,
  //                          cparams.min_effort_);
  // private_nh.param<double>("max_effort", cparams.max_effort_,
  //                          cparams.max_effort_);

  ROS_INFO_STREAM("Initializing Robotiq action server for gripper "
                  << action_server_name);

  // The name of the gripper -> this server communicates over name/inputs and
  // name/outputs
  robotiq_2f_gripper_action_server::Robotiq2FGripperActionServer gripper(
      action_server_name, params);

  ROS_INFO("Robotiq action-server spinning for gripper: %s",
           action_server_name.c_str());
  ros::spin();
}
