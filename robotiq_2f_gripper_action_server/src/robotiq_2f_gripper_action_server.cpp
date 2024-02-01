/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq 2F gripper
 */

#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"
#include <math.h>

#include <algorithm>

// To keep the fully qualified names managable

// Anonymous namespaces are file local -> sort of like global static objects
namespace {
using namespace robotiq_2f_gripper_action_server;

/*  This struct is declared for the sole purpose of being used as an exception
   internally to keep the code clean (i.e. no output params). It is caught by
   the action_server and should not propogate outwards. If you use these
   functions yourself, beware.
*/
struct BadArgumentsError {};

uint8_t gap_size_to_count(double gap_size) {
  uint8_t count;

  // Gripper has quasi-linear position (function determined by testing
  // different position)
  if (gap_size <= 0.1) { // 78 to 255
    count = static_cast<uint8_t>(-1479.31 * gap_size + 226.84558);
  } else if (gap_size >= 0.134) { // 0 to 15 Non-linear portition

    count = static_cast<uint8_t>(-1810.890525 * gap_size + 257.4);
  } else { // 15 to 77
    count = static_cast<uint8_t>(-1661.8310875 * gap_size + 245.66409);
  }
  return count;
}

double count_to_gap_size(uint8_t count) {
  double gap_size;

  if (count <= 15) {
    gap_size = (static_cast<double>(count) - 257.4) / -1810.890525;
  } else if (count <= 77) {
    gap_size = (static_cast<double>(count) - 245.66409) / -1661.8310875;
  } else {
    gap_size = (static_cast<double>(count) - 226.84558) / -1479.31;
  }
  return gap_size;
}

GripperOutput goalToRegisterState(const GripperCommandGoal &goal,
                                  Robotiq2FGripperParams &params,
                                  const GripperInput &current_state) {

  GripperOutput result;
  result.rACT = 0x1; // active gripper
  result.rGTO = 0x1; // go to position
  result.rATR = 0x0; // No emergency release

  // Check position gap range
  if (goal.command.gap_size > params.max_gap_size ||
      goal.command.gap_size < params.min_gap_size) {
    ROS_WARN("Goal gripper gap size is out of range(%f to %f): %f m",
             params.min_gap_size, params.max_gap_size, goal.command.gap_size);
    throw BadArgumentsError();
  }

  // Check velocity range
  if (goal.command.speed > params.max_speed ||
      goal.command.speed < params.min_speed) {
    ROS_WARN("Goal gripper speed is out of range(%f to %f): %f m/s",
             params.min_speed, params.max_speed, goal.command.speed);
    throw BadArgumentsError();
  }

  // Check effort
  if (goal.command.effort_scaling < 0 || goal.command.effort_scaling > 1.0) {
    ROS_WARN("Goal gripper effort scaling out of range (%f to %f): %f", 0.0,
             1.0, goal.command.effort_scaling);
    throw BadArgumentsError();
  }

  // Convert gap position to rPR
  result.rPR = gap_size_to_count(goal.command.gap_size);

  // Apply position offset
  if (goal.command.gap_size_offset != 0.0) {

    uint8_t gap_count_offsetted;

    // Closing gripper -> 255 counts
    if (current_state.gPO < result.rPR) {

      double gap_size_offsetted =
          std::clamp(goal.command.gap_size - goal.command.gap_size_offset,
                     params.min_gap_size, params.max_gap_size);

      gap_count_offsetted = gap_size_to_count(gap_size_offsetted);

    }
    // Opening gripper -> 0 counts
    else if (current_state.gPO > result.rPR) {
      double gap_size_offsetted =
          std::clamp(goal.command.gap_size + goal.command.gap_size_offset,
                     params.min_gap_size, params.max_gap_size);

      gap_count_offsetted = gap_size_to_count(gap_size_offsetted);
    }
    // Correct location
    else {
      ROS_ERROR(
          "Requested position with offset, but already at good position. "
          "Cannot determine which way to go, e.g. internal vs external grasp.");
      throw BadArgumentsError();
    }

    result.rPR = gap_count_offsetted;
  }

  // Convert speed to rSP
  result.rSP =
      static_cast<uint8_t>((goal.command.speed - params.min_speed) /
                           ((params.max_speed - params.min_speed) / 255.0));

  // convert effort to rFR
  result.rFR = static_cast<uint8_t>(goal.command.effort_scaling * 255.0);

  return result;
} // namespace

/*  This function is templatized because both GripperCommandResult and
   GripperCommandFeedback consist of the same fields yet have different
   types. Templates here act as a "duck typing" mechanism to avoid code
   duplication.
*/
template <typename T>
T registerStateToResultT(const GripperInput &input,
                         const Robotiq2FGripperParams &params,
                         const GripperOutput &goal) {

  T result;

  // current gap position
  result.gap_size = count_to_gap_size(input.gPO);

  // approximate effort (~10 to 125 N)
  if (input.gOBJ == 0x0 || input.gOBJ == 0x3) {
    // 0x0 = Fingers are in motion towards requested position. No object
    // detected.
    // 0x3 = Fingers are at requested position. No object detected or
    // object has been loss / dropped.
    result.effort = 0.0;
  } else {
    std::cout << +target_force_count << std::endl;
    result.effort = static_cast<double>(goal.rFR) * (115.0 / 255.0) + 10.0;
  }

  // has the gripper stalled ?
  result.stalled = input.gOBJ == 0x1 || input.gOBJ == 0x2;

  // has gripper reached the goal ^
  double gap_error_counts = std::abs(static_cast<double>(target_gap_count) -
                                     static_cast<double>(input.gPO));

  // each count represents about 0.6 mm of resolution
  if (gap_error_counts < 9) {
    result.reached_goal = true;
  } else {
    result.reached_goal = false;
  }

  // result.reached_goal = false;
  return result;
}

// Inline api-transformers to avoid confusion when reading the action_server
// source
inline GripperCommandResult
registerStateToResult(const GripperInput &input,
                      const Robotiq2FGripperParams &params,
                      const GripperOutput &goal) {
  return registerStateToResultT<GripperCommandResult>(input, params, goal);
}

inline GripperCommandFeedback
registerStateToFeedback(const GripperInput &input,
                        const Robotiq2FGripperParams &params,
                        const GripperOutput &goal) {
  return registerStateToResultT<GripperCommandFeedback>(input, params, goal);
}

} // namespace

namespace robotiq_2f_gripper_action_server {

Robotiq2FGripperActionServer::Robotiq2FGripperActionServer(
    const std::string &name, const Robotiq2FGripperParams &params)
    : nh_(), as_(nh_, name, false), action_name_(name),
      gripper_params_(params) {
  as_.registerGoalCallback(
      boost::bind(&Robotiq2FGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(
      boost::bind(&Robotiq2FGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe("input", 1,
                             &Robotiq2FGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>("output", 1);

  as_.start();
}

void Robotiq2FGripperActionServer::goalCB() {
  // Check to see if the gripper is in an active state where it can take goals
  if (current_reg_state_.gSTA != 0x3) {
    ROS_WARN("%s could not accept goal because the gripper is not yet active",
             action_name_.c_str());
    return;
  }

  GripperCommandGoal current_goal(*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested()) {
    as_.setPreempted();
  }

  try {
    goal_reg_state_ =
        goalToRegisterState(current_goal, gripper_params_, current_reg_state_);
    goal_pub_.publish(goal_reg_state_);
  } catch (BadArgumentsError &e) {

    as_.setAborted(registerStateToResult(current_reg_state_, gripper_params_,
                                         goal_reg_state_));
  }
}

void Robotiq2FGripperActionServer::preemptCB() {
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void Robotiq2FGripperActionServer::analysisCB(
    const GripperInput::ConstPtr &msg) {
  current_reg_state_ = *msg;
  if (!as_.isActive())
    return;

  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gSTA != 0x3) {
    // Check to see if the gripper is active or if it has been asked to be
    // active
    if (current_reg_state_.gSTA == 0x0 && goal_reg_state_.rACT != 0x1) {
      // If it hasn't been asked, active it
      issueActivation();
    }

    // Otherwise wait for the gripper to activate
    // TODO: If message delivery isn't guaranteed, then we may want to resend
    // activate
    return;
  }

  // Check for errors
  if (current_reg_state_.gFLT) {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(),
             current_reg_state_.gFLT);
    as_.setAborted(registerStateToResult(current_reg_state_, gripper_params_,
                                         goal_reg_state_));
  } else if (current_reg_state_.gGTO && current_reg_state_.gOBJ &&
             current_reg_state_.gPR == goal_reg_state_.rPR) {

    as_.setSucceeded(registerStateToResult(current_reg_state_, gripper_params_,
                                           goal_reg_state_));

  }

  else {
    // Publish feedback
    as_.publishFeedback(registerStateToFeedback(
        current_reg_state_, gripper_params_, goal_reg_state_));
  }
}

void Robotiq2FGripperActionServer::issueActivation() {
  ROS_INFO("Activating gripper for gripper action server: %s",
           action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  // out.rGTO = 0x1;
  // other params should be zero
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}
} // namespace robotiq_2f_gripper_action_server
