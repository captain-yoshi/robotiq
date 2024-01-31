/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq 2F gripper
 */

#ifndef ROBOTIQ_2F_GRIPPER_ACTION_SERVER_H
#define ROBOTIQ_2F_GRIPPER_ACTION_SERVER_H

// STL
#include <string>
// ROS standard
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/Robotiq2fGripperCommandAction.h>
#include <ros/ros.h>

// Repo specific includes
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

namespace robotiq_2f_gripper_action_server {

typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_input GripperInput;
typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_output GripperOutput;

typedef control_msgs::Robotiq2fGripperCommandGoal GripperCommandGoal;
typedef control_msgs::Robotiq2fGripperCommandFeedback GripperCommandFeedback;
typedef control_msgs::Robotiq2fGripperCommandResult GripperCommandResult;

/**
 * @brief Structure containing the parameters necessary to translate
 *        GripperCommand actions to register-based commands to a
 *        particular gripper (and vice versa).
 *
 *        The min gap can be less than zero. This represents the case where the
 *        gripper fingers close and then push forward.
 */
struct Robotiq2FGripperParams {
  double min_gap_size; // meters
  double max_gap_size;
  double effort_scaling; // 0-1
  double min_speed;      // m/s
  double max_speed;

  int32_t pos_offset;    // Offset to make sure that the object is holding
  int32_t pos_nl_offset; // Position offset when in non linear area
  int32_t
      pos_tol_open; // Tolerance for validating position when an object is hold
  int32_t pos_tol_closed;
  double goal_cmd_pos;
};

/**
 * @brief The Robotiq2FGripperActionServer class. Takes as arguments the name of
 * the gripper it is to command, and a set of parameters that define the
 * physical characteristics of the particular gripper.
 *
 *        Listens for messages on input and publishes on output. Remap these.
 */
class Robotiq2FGripperActionServer {
public:
  Robotiq2FGripperActionServer(const std::string &name,
                               const Robotiq2FGripperParams &params);

  // These functions are meant to be called by simple action server
  void goalCB();
  void preemptCB();
  void analysisCB(const GripperInput::ConstPtr &msg);

private:
  void issueActivation();

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::Robotiq2fGripperCommandAction>
      as_;

  ros::Subscriber state_sub_; // Subs to grippers "input" topic
  ros::Publisher goal_pub_;   // Pubs to grippers "output" topic

  GripperOutput goal_reg_state_;   // Goal information in gripper-register form
  GripperInput current_reg_state_; // State info in gripper-register form

  /* Used to translate GripperCommands in engineering units
   * to/from register states understood by gripper itself. Different
   * for different models/generations of Robotiq grippers */
  Robotiq2FGripperParams gripper_params_;

  std::string action_name_;
};

} // namespace robotiq_2f_gripper_action_server
#endif
