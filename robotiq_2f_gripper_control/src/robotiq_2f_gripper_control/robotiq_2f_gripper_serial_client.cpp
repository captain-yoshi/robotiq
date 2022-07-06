#include "robotiq_2f_gripper_control/robotiq_2f_gripper_serial_client.h"
#include "robotiq_2f_gripper_control/robotiq_2f_hw_usb.hpp"

// See Robotiq's documentation for the register mapping
namespace robotiq_2f_gripper_control
{

Robotiq2FGripperSerialClient::Robotiq2FGripperSerialClient(robotiq_2f_hardware::ROBOTIQ2FUSB& manager,
                                                    int slave_no)
  : manager_(manager)
  , slave_no_(slave_no)
{}

/*
  See support.robotiq.com -> manual for the register output meanings
*/
void Robotiq2FGripperSerialClient::writeOutputs(const GripperOutput& output)
{
  //uint8_t map[6] = {0}; // array containing all 6 output registers

  // Pack the Action Request register byte
  map_output_[0] = (output.rACT & 0x1) | ((output.rGTO << 0x3) & 0x8) | ((output.rATR << 0x4) & 0x10);
  // registers 1 & 2 reserved by Robotiq (Set to 0)
  map_output_[1] = 0;
  map_output_[2] = 0;
  map_output_[3] = output.rPR;
  map_output_[4] = output.rSP;
  map_output_[5] = output.rFR;

  //uint16_t buf_[9];
  //manager_.buf_msr_[0] = ((uint16_t)map[0] << 8) | map[1];
  //manager_.buf_msr_[1] = ((uint16_t)map[2] << 8) | map[3];
  //manager_.buf_msr_[2] = ((uint16_t)map[4] << 8) | map[5];

  manager_.write(map_output_);

}

Robotiq2FGripperSerialClient::GripperInput Robotiq2FGripperSerialClient::readInputs() const
{
  //uint8_t map[6];


  //uint16_t buf_[9];
  manager_.readInput(map_input_);

  /*
  map[0] = (uint8_t)((buf_[0] & 0xFF00) >> 8);
  map[1] = (uint8_t)(buf_[0] & 0x00FF);
  map[2] = (uint8_t)((buf_[1] & 0xFF00) >> 8);
  map[3] = (uint8_t)(buf_[1] & 0x00FF);
  map[4] = (uint8_t)((buf_[2] & 0xFF00) >> 8);
  map[5] = (uint8_t)(buf_[2] & 0x00FF);
*/

  // Decode Input Registers
  GripperInput input;
  input.gACT = map_input_[0] & 0x1;
  input.gGTO = (map_input_[0] >> 0x3) & 0x1;
  input.gSTA = (map_input_[0] >> 0x4) & 0x3;
  input.gOBJ = (map_input_[0] >> 0x6) & 0x3;
  // map[1] is reserved by the protocol
  input.gFLT = map_input_[2] & 0xF;
  input.gPR = map_input_[3];
  input.gPO = map_input_[4];
  input.gCU = map_input_[5];

  return input;
}


Robotiq2FGripperSerialClient::GripperOutput Robotiq2FGripperSerialClient::readOutputs() const
{
  //uint8_t map[6];

  //uint16_t buf_[9];

  manager_.readOutput(map_input_);

 /** 
  map_input_[0] = (uint8_t)((buf_[0] & 0xFF00) >> 8);
  map[1] = (uint8_t)(buf_[0] & 0x00FF);
  map[2] = (uint8_t)((buf_[1] & 0xFF00) >> 8);
  map[3] = (uint8_t)(buf_[1] & 0x00FF);
  map[4] = (uint8_t)((buf_[2] & 0xFF00) >> 8);
  map[5] = (uint8_t)(buf_[2] & 0x00FF);
*/
  GripperOutput output;
  output.rACT = map_input_[0] & 1;
  output.rGTO = (map_input_[0] >> 0x3) & 0x1;
  output.rATR = (map_input_[0] >> 0x4) & 0x1;
  output.rPR = map_input_[3];
  output.rSP = map_input_[4];
  output.rFR = map_input_[5];

  return output;
}


} // end of robotiq_2f_gripper_control namespace
