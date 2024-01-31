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
                         double target_rPR) {

  T result;

  if (input.gPO <= 15) {
    result.position = (static_cast<double>(input.gPO) - 257.4) / -1810.890525;
  } else if (input.gPO <= 77) {
    result.position =
        (static_cast<double>(input.gPO) - 245.66409) / -1661.8310875;
  } else {
    result.position = (static_cast<double>(input.gPO) - 226.84558) / -1479.31;
  }

  std::cout << "rPR = " << target_rPR << std::endl;
  std::cout << "gPO = " << +input.gPO << std::endl;

  // result.effort = input.gCU * eff_per_tick + params.min_effort_;
  result.stalled = input.gOBJ == 0x1 || input.gOBJ == 0x2;

  double gap_error_counts = std::abs(target_rPR - input.gPO);

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
                      const Robotiq2FGripperParams &params, double target_gap) {
  return registerStateToResultT<GripperCommandResult>(input, params,
                                                      target_gap);
}

inline GripperCommandFeedback
registerStateToFeedback(const GripperInput &input,
                        const Robotiq2FGripperParams &params,
                        double target_gap) {
  return registerStateToResultT<GripperCommandFeedback>(input, params,
                                                        target_gap);
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
                                         goal_reg_state_.rPR));

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
                                         goal_reg_state_.rPR));
  } else if (current_reg_state_.gGTO && current_reg_state_.gOBJ &&
             current_reg_state_.gPR == goal_reg_state_.rPR) {

    as_.setSucceeded(registerStateToResult(current_reg_state_, gripper_params_,
                                           goal_reg_state_.rPR));

    // // If commanded to move and if at a goal state and if the position
    // request
    // // matches the echo'd PR, we're done with a move

    // // TODO change because we have 3 functions depending on the position
    // double max_tol =
    //     ((goal_reg_state_.rPR - gripper_params_.pos_offset +
    //       gripper_params_.pos_nl_offset - gripper_params_.pos_tol_closed) -
    //      228.260379) /
    //     -1526.308005;
    // double min_tol =
    //     ((goal_reg_state_.rPR - gripper_params_.pos_offset +
    //       gripper_params_.pos_nl_offset + gripper_params_.pos_tol_open) -
    //      228.260379) /
    //     -1526.308005;
    // double curr_pos = (current_reg_state_.gPO - 228.260379) / -1526.308005;

    // // Enable thin detection object not detect by robotiq (No need to check
    // // against gOBJ = 0x03)
    // if (static_cast<int16_t>(current_reg_state_.gPO) >=
    //         (static_cast<int16_t>(goal_reg_state_.rPR) -
    //          gripper_params_.pos_offset + gripper_params_.pos_nl_offset -
    //          gripper_params_.pos_tol_closed) &&
    //     static_cast<int16_t>(current_reg_state_.gPO) <=
    //         (static_cast<int16_t>(goal_reg_state_.rPR) -
    //          gripper_params_.pos_offset + gripper_params_.pos_nl_offset +
    //          gripper_params_.pos_tol_open)) {

    //   as_.setSucceeded(registerStateToResult(
    //       current_reg_state_, gripper_params_, goal_reg_state_.rPR));
    // }
    // // Throw error because
    // else {
    //   ROS_WARN("gPO = %d, rPR = %d, Compare1 = %d, Compare2 = %d",
    //            current_reg_state_.gPO, goal_reg_state_.rPR,
    //            (goal_reg_state_.rPR - gripper_params_.pos_offset +
    //             gripper_params_.pos_nl_offset -
    //             gripper_params_.pos_tol_closed),
    //            (goal_reg_state_.rPR - gripper_params_.pos_offset +
    //             gripper_params_.pos_nl_offset +
    //             gripper_params_.pos_tol_open));
    //   ROS_WARN("Error: Gripper position MUST be between %f and %f meters. "
    //            "Current position is %f meters.",
    //            min_tol, max_tol, curr_pos);
    //   as_.setSucceeded(registerStateToResult(
    //       current_reg_state_, gripper_params_, goal_reg_state_.rPR));
    //   // as_.setAborted(registerStateToResult(current_reg_state_,
    //   //                                      gripper_params_,
    //   //                                      goal_reg_state_.rPR));
    // }
  }

  else {
    // Publish feedback
    as_.publishFeedback(registerStateToFeedback(
        current_reg_state_, gripper_params_, goal_reg_state_.rPR));
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
