#ifndef ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_HW_USB_H
#define ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_HW_USB_H

#include <modbus.h>

#include "ros/ros.h"
#include <chrono>
#include <thread>


#ifndef DEBUG
#define DEBUG 0
#endif


/*
  Endieness (from Robotiq manual 2019):

    Modbus RTU is a communication protocol based on a Big Endian byte order. Therefore, the 16-bit register addresses are
    transmitted with the most significant byte first. However, the data port is in the case of Robotiq products based on the Little
    Endian byte order. As such, the data parts of Modbus RTU messages are sent with the less significant byte first.
*/

namespace robotiq_2f_hardware
{

// this is fixed for the usb mode
const int CMD_ADDR = 0x03E8;
const int MSR_ADDR = 0x07D0;

// the activation cmd also contains the default values for the GoTo command
// speed in device at 100%, the decimation is done by the joint_trajectory controller
// effort in device at 0
const uint16_t ACTIVATION_CMD[3] = {(1 + (1 << 3) + (0 << 4)) << 8, 0x0000, (0xFF << 8) + 0xFF};
// the reset command clears all values, clears all faults
// and stops any current motion
const uint16_t RESET_CMD[3] = {0x0, 0x0, 0x0};
// the emergency stop command makes all other commands to stop, and perform a slow
// openning of the gripper. A reset, and activation commands are required to re-enable
// the gripper
const uint16_t ESTOP_CMD[3] = {(1 + (0 << 3) + (1 << 4)) << 8, 0x0, 0x0};



class ROBOTIQ2FUSB
{
public:

	/**
	 * @brief init
	 * @return
	 */
	bool init()
	{
		// configure and connect to device
		ctx_ptr_ = NULL;
		ctx_ptr_ = modbus_new_rtu(port_.c_str(), 115200, 'N', 8, 1);
		// Max 15 char (Fix when libmodbus releases version 3.2.0)
#if DEBUG
		modbus_set_debug(ctx_ptr_, TRUE);
#endif
		modbus_rtu_set_serial_mode(ctx_ptr_, MODBUS_RTU_RS485);
		modbus_set_slave(ctx_ptr_, server_id_);
		modbus_connect(ctx_ptr_);

		// init command structure with default values
		buf_cmd_[0] = ACTIVATION_CMD[0];
		buf_cmd_[1] = ACTIVATION_CMD[1];
		buf_cmd_[2] = ACTIVATION_CMD[2];

		// finally activate the device (handshake) and check
		int ra = modbus_write_registers(ctx_ptr_, CMD_ADDR, 3, ACTIVATION_CMD);
		if(ra < 0)
		{
			std::cout << "Couldn't perform an activation on the USB device" << std::endl;
			return false;
		}
		// the sleep is needed because the ready flag is turned true before
		// completing the initial movement after activation
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		bool ready = false;
		int timeout = 0;
		uint8_t u_act;

		while(!ready)
		{
			modbus_read_registers(ctx_ptr_, MSR_ADDR, 3, buf_msr_);
			u_act = (buf_msr_[0] & 0xFF00) >> 8;
			ready = (u_act >> 0) & 0x01;

			timeout++;
			if (timeout > 50)
			{
				std::cout << "After 5seg, the USB device dind't get activated, quit." << std::endl;
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		return true;
	}

	/**
	 * @brief readInput
	 */
	void readInput(uint8_t* map_in_)
	{


		int rc = modbus_read_registers(ctx_ptr_, MSR_ADDR, 3, buf_msr_);
		if(rc < 0)
		{
			ROS_FATAL("Couldn't read the last state on the USB device");
			exit(-1);
			return;
		}

		map_in_[0] = (uint8_t)((buf_msr_[0] & 0xFF00) >> 8);
  		map_in_[1] = (uint8_t)(buf_msr_[0] & 0x00FF);
  		map_in_[2] = (uint8_t)((buf_msr_[1] & 0xFF00) >> 8);
  		map_in_[3] = (uint8_t)(buf_msr_[1] & 0x00FF);
  		map_in_[4] = (uint8_t)((buf_msr_[2] & 0xFF00) >> 8);
  		map_in_[5] = (uint8_t)(buf_msr_[2] & 0x00FF);
	}

	void readOutput(uint8_t* map_in_)
	{
		int rc = modbus_read_registers(ctx_ptr_, CMD_ADDR, 3, buf_msr_);
		if(rc < 0)
		{
			ROS_FATAL("Couldn't read the last state on the USB device");
			exit(-1);
			return;
		}
		map_in_[0] = (uint8_t)((buf_msr_[0] & 0xFF00) >> 8);
  		map_in_[1] = (uint8_t)(buf_msr_[0] & 0x00FF);
  		map_in_[2] = (uint8_t)((buf_msr_[1] & 0xFF00) >> 8);
  		map_in_[3] = (uint8_t)(buf_msr_[1] & 0x00FF);
  		map_in_[4] = (uint8_t)((buf_msr_[2] & 0xFF00) >> 8);
  		map_in_[5] = (uint8_t)(buf_msr_[2] & 0x00FF);
	}


	/**
	 * @brief write
	 */
	void write(uint8_t* map_out_)
	{
		buf_cmd_[0] = ((uint16_t)map_out_[0] << 8) | map_out_[1];
  		buf_cmd_[1] = ((uint16_t)map_out_[2] << 8) | map_out_[3];
  		buf_cmd_[2] = ((uint16_t)map_out_[4] << 8) | map_out_[5];

		int rc = modbus_write_registers(ctx_ptr_, CMD_ADDR, 3, buf_cmd_);
		if(rc < 0)
		{
			ROS_FATAL("Couldn't write the last command on the USB device");
			exit(-1);
		}
	}

	/**
	 * @brief close This function resets the gripper, and then release the USB port.
	 */
	void close()
	{
		modbus_write_registers(ctx_ptr_, CMD_ADDR, 3, RESET_CMD);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		modbus_close(ctx_ptr_);
	}

	/**
	 * @brief reactivate This function resets to clear errors, and reactivate the gripper
	 */
	void reactivate()
	{
		int rs = modbus_write_registers(ctx_ptr_, CMD_ADDR, 3, RESET_CMD);
		if(rs < 0)
		{
			std::cout << "Couldn't perform a reset on the USB device" << std::endl;
			exit(-2);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		int ra = modbus_write_registers(ctx_ptr_, CMD_ADDR, 3, ACTIVATION_CMD);
		if(ra < 0)
		{
			std::cout << "Couldn't perform an activation on the USB device" << std::endl;
			exit(-2);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}

	/**
	 * @brief estop This function resets the gripper to clear all commands and errors
	 */
	void estop()
	{
		modbus_write_registers(ctx_ptr_, CMD_ADDR, 3, ESTOP_CMD);
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	}

	void setPort(std::string port){port_ = port;};

	void setServerID(int server_id){server_id_ = server_id;};

private:
	modbus_t *ctx_ptr_;

	std::string port_;
	int server_id_;

	// Protocol uses 6 bytes for each register
	uint16_t buf_cmd_[3];	// Register: ACTION REQUEST
	uint16_t buf_msr_[3];	// Register: GRIPPER STATUS
};

} // robotiq_2f_hardware

#endif // ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_HW_USB_H
