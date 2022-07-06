#ifndef ROBOTIQ_2F_GRIPPER_SERIAL_CLIENT_H
#define ROBOTIQ_2F_GRIPPER_SERIAL_CLIENT_H

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>

// Forward declaration of SerialManager
namespace robotiq_2f_hardware 
{
  class ROBOTIQ2FUSB;
}

namespace robotiq_2f_gripper_control
{

/**
 * \brief This class provides a client for the Serial manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class Robotiq2FGripperSerialClient
{
public:
  typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_output GripperOutput;
  typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_input GripperInput;

  /**
   * \brief Constructs a control interface to a 2F Robotiq gripper on
   *        the given Serial network and the given slave_no.
   *
   * @param[in] manager The interface to an Serial network that the gripper
   *                    is connected to.
   *
   * @param[in] slave_no The slave number of the gripper on the Serial network
   *                     (>= 1)
   */
  Robotiq2FGripperSerialClient(robotiq_2f_hardware::ROBOTIQ2FUSB& manager, int slave_no);

  /**
   * \brief Write the given set of control flags to the memory of the gripper
   * 
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const GripperOutput& output);
  
  /**
   * \brief Reads set of input-register values from the gripper.
   * \return The gripper input registers as read from the controller IOMap
   */
  GripperInput readInputs() const;
  
  /**
   * \brief Reads set of output-register values from the gripper.
   * \return The gripper output registers as read from the controller IOMap
   */
  GripperOutput readOutputs() const;

private:
  robotiq_2f_hardware::ROBOTIQ2FUSB& manager_;
  const int slave_no_;
  
  mutable uint8_t map_input_[6];
  uint8_t map_output_[6];

};

}

#endif
