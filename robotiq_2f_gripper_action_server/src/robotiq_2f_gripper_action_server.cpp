/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq 2F gripper
 */

#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"
#include <math.h>  
// To keep the fully qualified names managable

//Anonymous namespaces are file local -> sort of like global static objects
namespace
{
  using namespace robotiq_2f_gripper_action_server;

  /*  This struct is declared for the sole purpose of being used as an exception internally
      to keep the code clean (i.e. no output params). It is caught by the action_server and 
      should not propogate outwards. If you use these functions yourself, beware.
  */
  struct BadArgumentsError {};


  GripperOutput goalToRegisterState(const GripperCommandGoal& goal, Robotiq2FGripperParams& params, const uint8_t curr_reg_state_gPO)
  {
    GripperOutput result;
    result.rACT = 0x1; // active gripper
    result.rGTO = 0x1; // go to position
    result.rATR = 0x0; // No emergency release
    result.rSP  = 128; // Middle ground speed

    if (goal.command.position > params.max_gap_ || goal.command.position < params.min_gap_)
    {
      ROS_WARN("Goal gripper gap size is out of range(%f to %f): %f m",
               params.min_gap_, params.max_gap_, goal.command.position);
      throw BadArgumentsError();
    }
    if (goal.command.max_effort < params.min_effort_ || goal.command.max_effort > params.max_effort_)
    {
      ROS_WARN("Goal gripper effort out of range (%f to %f N): %f N",
               params.min_effort_, params.max_effort_, goal.command.max_effort);
      throw BadArgumentsError();
    }
    // For futur use
    params.goal_cmd_pos = goal.command.position;

    // Change joint position (rad) to gripper gap size (m)
    //double goal_gap_size = 0.14 - (goal.command.position *  0.14 / params.max_gap_);

    // Change joint position (rad) to gripper gap size (m) (Based on CAD)
    double theta_offset = acos(0.06691/0.1);
    double goal_gap_size = 2 * (0.1 * cos(goal.command.position + theta_offset) - 0.00735 +0.0127);

    // Gripper has quasi-linear position (function determined by testing different position)
    if (goal_gap_size <= 0.1) { //78 to 226
      result.rPR = static_cast<uint8_t>(-1479.31 * goal_gap_size + 226.84558);
      params.pos_nl_offset = 0;
    }
    else if (goal_gap_size >= 0.137) {  // Non-linear portition
      result.rPR = 3;
      params.pos_nl_offset = 10;
    }
    else { // 15 to 77
      result.rPR = static_cast<uint8_t>(-1661.8310875 * goal_gap_size + 245.66409);
      params.pos_nl_offset = 0;
    }

    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    // TODO remove when fixed in MTC
    //result.rFR = static_cast<uint8_t>((50 - params.min_effort_) / eff_per_tick);
    // TODO uncomment when fixed in MTC
    result.rFR = static_cast<uint8_t>((goal.command.max_effort - params.min_effort_) / eff_per_tick);


    // In force control (fc) when goal.command.max_effort > 0
    // In position control (pc) when goal.command.max_effort == 0

    // TODO remove when fixed in MTC
    //int32_t goal_cmd_pos_offset = 0;  // Needed so that it never reaches the original goal
    // TODO uncomment when fixed in MTC
    int32_t goal_cmd_pos_offset = 255;  // Needed so that it never reaches the original goal

    // Tolerance for position so that a GripperCommand succeeds (Used in fc & pc)
    // TODO remove when fixed in MTC
    //uint16_t pos_tol_high = 25;	
    //uint16_t pos_tol_low  = 25;  
    //TODO uncomment when fixed in MTC
    uint16_t pos_tol_high = 5;	// default:5
    uint16_t pos_tol_low  = 2;  // default:2

    // Closing gripper
    if (curr_reg_state_gPO <= result.rPR) {
      if (result.rPR + goal_cmd_pos_offset > 255) {
        params.pos_offset = 255 - result.rPR;
      }
      else {
        params.pos_offset = goal_cmd_pos_offset;
      }
      params.pos_tol_open   = pos_tol_high;
      params.pos_tol_closed = pos_tol_low;
    }
    // Opening gripper
    else if (curr_reg_state_gPO > result.rPR) {
      if (result.rPR - goal_cmd_pos_offset < 0) {
        params.pos_offset = 0 - result.rPR;
      }
      else {
        params.pos_offset = -goal_cmd_pos_offset;
      }
      params.pos_tol_open   = pos_tol_low;
      params.pos_tol_closed = pos_tol_high;
    }
    else {
        params.pos_offset = 0;
    }

    // No offsets in fc
    if (result.rFR == 0) {
      params.pos_offset = 0;
      params.pos_nl_offset = 0;
    }

    result.rPR = static_cast<uint8_t>(result.rPR + params.pos_offset);
/*
    ROS_INFO("params.max_gap = %f", params.max_gap_);
    ROS_INFO("params.min_gap = %f", params.min_gap_);
    ROS_INFO("params.min_effort_ = %f", params.min_effort_);
    ROS_INFO("params.max_effort_ = %f", params.max_effort_);
    ROS_INFO("eff_per_tick = %f", eff_per_tick);
    ROS_INFO("goal.command.position = %f", goal.command.position);
    ROS_INFO("goal.command.max_effort = %f", goal.command.max_effort);
    ROS_INFO("cmd_pos_offset = %d", params.pos_offset);

    ROS_INFO("Setting goal position register to %hhu", result.rPR);
*/
    return result;
  }

  /*  This function is templatized because both GripperCommandResult and GripperCommandFeedback consist
      of the same fields yet have different types. Templates here act as a "duck typing" mechanism to avoid
      code duplication.
  */
  template<typename T>
  T registerStateToResultT(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    T result;
    // TODO change this like section above
    double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;
    
    result.position = input.gPO * dist_per_tick + params.min_gap_;
    result.effort = input.gCU * eff_per_tick + params.min_effort_;
    result.stalled = input.gOBJ == 0x1 || input.gOBJ == 0x2;
    result.reached_goal = input.gPO == goal_pos;
    //result.reached_goal = false;
    return result;
  }

  // Inline api-transformers to avoid confusion when reading the action_server source
  inline
  GripperCommandResult registerStateToResult(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandResult>(input, params, goal_pos);
  }

  inline
  GripperCommandFeedback registerStateToFeedback(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandFeedback>(input, params, goal_pos);
  }

} // end of anon namespace

namespace robotiq_2f_gripper_action_server
{

Robotiq2FGripperActionServer::Robotiq2FGripperActionServer(const std::string& name, const Robotiq2FGripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
{
  as_.registerGoalCallback(boost::bind(&Robotiq2FGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&Robotiq2FGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe("input", 1, &Robotiq2FGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>("output", 1);

  as_.start();
}

void Robotiq2FGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
  if (current_reg_state_.gSTA != 0x3)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
    return;
  }

  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    goal_reg_state_ = goalToRegisterState(current_goal, gripper_params_, current_reg_state_.gPO);
    goal_pub_.publish(goal_reg_state_);
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
  }
}

void Robotiq2FGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void Robotiq2FGripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
{
  current_reg_state_ = *msg;
  if (!as_.isActive()) return;

  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gSTA != 0x3)
  {
    // Check to see if the gripper is active or if it has been asked to be active
    if (current_reg_state_.gSTA == 0x0 && goal_reg_state_.rACT != 0x1)
    {
      // If it hasn't been asked, active it
      issueActivation();
    }

    // Otherwise wait for the gripper to activate
    // TODO: If message delivery isn't guaranteed, then we may want to resend activate
    return;
  }

  // Check for errors
  if (current_reg_state_.gFLT)
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
    as_.setAborted(registerStateToResult(current_reg_state_,
                                         gripper_params_,
                                         goal_reg_state_.rPR));
  }
  else if (current_reg_state_.gGTO && current_reg_state_.gOBJ && current_reg_state_.gPR == goal_reg_state_.rPR)
  {
    // If commanded to move and if at a goal state and if the position request matches the echo'd PR, we're
    // done with a move

    // TODO change because we have 3 functions depending on the position
    double max_tol = ((goal_reg_state_.rPR - gripper_params_.pos_offset + gripper_params_.pos_nl_offset - gripper_params_.pos_tol_closed)-228.260379) /-1526.308005;
    double min_tol = ((goal_reg_state_.rPR - gripper_params_.pos_offset + gripper_params_.pos_nl_offset + gripper_params_.pos_tol_open)-228.260379) /-1526.308005;
    double curr_pos = (current_reg_state_.gPO - 228.260379) /-1526.308005;

    // Enable thin detection object not detect by robotiq (No need to check against gOBJ = 0x03)
    if (static_cast<int16_t>(current_reg_state_.gPO) >= (static_cast<int16_t>(goal_reg_state_.rPR) - gripper_params_.pos_offset + gripper_params_.pos_nl_offset - gripper_params_.pos_tol_closed) && 
        static_cast<int16_t>(current_reg_state_.gPO) <= (static_cast<int16_t>(goal_reg_state_.rPR) - gripper_params_.pos_offset + gripper_params_.pos_nl_offset + gripper_params_.pos_tol_open))  
    {

      as_.setSucceeded(registerStateToResult(current_reg_state_,
                                             gripper_params_,
                                             goal_reg_state_.rPR));
    }
    // Throw error because
    else {
      ROS_WARN("gPO = %d, rPR = %d, Compare1 = %d, Compare2 = %d", current_reg_state_.gPO, goal_reg_state_.rPR, (goal_reg_state_.rPR - gripper_params_.pos_offset + gripper_params_.pos_nl_offset - gripper_params_.pos_tol_closed), (goal_reg_state_.rPR - gripper_params_.pos_offset + gripper_params_.pos_nl_offset + gripper_params_.pos_tol_open));
      ROS_WARN("Error: Gripper position MUST be between %f and %f meters. Current position is %f meters.", min_tol, max_tol, curr_pos);
      as_.setAborted(registerStateToResult(current_reg_state_,
                                           gripper_params_,
                                           goal_reg_state_.rPR));
    }
  }

  else
  {
    // Publish feedback
    as_.publishFeedback(registerStateToFeedback(current_reg_state_,
                                                gripper_params_,
                                                goal_reg_state_.rPR));
  }

  
}

void Robotiq2FGripperActionServer::issueActivation()
{
  ROS_INFO("Activating gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  //out.rGTO = 0x1;
  // other params should be zero
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}
} // end robotiq_2f_gripper_action_server namespace
