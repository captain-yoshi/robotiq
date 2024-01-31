/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq 2F gripper
 */

#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"
#include <math.h>
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

GripperOutput goalToRegisterState(const GripperCommandGoal &goal,
                                  Robotiq2FGripperParams &params,
                                  const uint8_t curr_reg_state_gPO) {

  GripperOutput result;
  result.rACT = 0x1; // active gripper
  result.rGTO = 0x1; // go to position
  result.rATR = 0x0; // No emergency release

  // Check position gap range
  if (goal.command.position > params.max_gap_size ||
      goal.command.position < params.min_gap_size) {
    ROS_WARN("Goal gripper gap size is out of range(%f to %f): %f m",
             params.min_gap_size, params.max_gap_size, goal.command.position);
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
  // Gripper has quasi-linear position (function determined by testing different
  // position)
  if (goal.command.position <= 0.1) { // 78 to 255
    result.rPR =
        static_cast<uint8_t>(-1479.31 * goal.command.position + 226.84558);
  } else if (goal.command.position >= 0.134) { // 0 to 15 Non-linear portition

    result.rPR =
        static_cast<uint8_t>(-1810.890525 * goal.command.position + 257.4);
  } else { // 15 to 77
    result.rPR =
        static_cast<uint8_t>(-1661.8310875 * goal.command.position + 245.66409);
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
   GripperCommandFeedback consist of the same fields yet have different types.
   Templates here act as a "duck typing" mechanism to avoid code duplication.
*/
template <typename T>
T registerStateToResultT(const GripperInput &input,
                         const Robotiq2FGripperParams &params,
                         uint8_t target_gap_count, uint8_t target_force_count) {

  T result;

  // current gap position
  if (input.gPO <= 15) {
    result.position = (static_cast<double>(input.gPO) - 257.4) / -1810.890525;
  } else if (input.gPO <= 77) {
    result.position =
        (static_cast<double>(input.gPO) - 245.66409) / -1661.8310875;
  } else {
    result.position = (static_cast<double>(input.gPO) - 226.84558) / -1479.31;
  }

  // approximate effort (~10 to 125 N)
  if (input.gOBJ == 0x0 || input.gOBJ == 0x3) {
    // 0x0 = Fingers are in motion towards requested position. No object
    // detected.
    // 0x3 = Fingers are at requested position. No object detected or
    // object has been loss / dropped.
    result.effort = 0.0;
  } else {
    std::cout << +target_force_count << std::endl;
    result.effort =
        static_cast<double>(target_force_count) * (115.0 / 255.0) + 10.0;
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
                      uint8_t target_gap_count, uint8_t target_force_count) {
  return registerStateToResultT<GripperCommandResult>(
      input, params, target_gap_count, target_force_count);
}

inline GripperCommandFeedback
registerStateToFeedback(const GripperInput &input,
                        const Robotiq2FGripperParams &params,
                        uint8_t target_gap_count, uint8_t target_force_count) {
  return registerStateToResultT<GripperCommandFeedback>(
      input, params, target_gap_count, target_force_count);
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
    goal_reg_state_ = goalToRegisterState(current_goal, gripper_params_,
                                          current_reg_state_.gPO);
    goal_pub_.publish(goal_reg_state_);
  } catch (BadArgumentsError &e) {

    as_.setAborted(registerStateToResult(current_reg_state_, gripper_params_,
                                         goal_reg_state_.rPR,
                                         goal_reg_state_.rFR));

    ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
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
                                         goal_reg_state_.rPR,
                                         goal_reg_state_.rFR));
  } else if (current_reg_state_.gGTO && current_reg_state_.gOBJ &&
             current_reg_state_.gPR == goal_reg_state_.rPR) {

    as_.setSucceeded(registerStateToResult(current_reg_state_, gripper_params_,
                                           goal_reg_state_.rPR,
                                           goal_reg_state_.rFR));

  }

  else {
    // Publish feedback
    as_.publishFeedback(
        registerStateToFeedback(current_reg_state_, gripper_params_,
                                goal_reg_state_.rPR, goal_reg_state_.rFR));
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
