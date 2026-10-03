// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
// This file is part of ASClinic-System.
//    
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// I2C driver for the PCA9685 16-channel 12-bit PWM driver
//
// ----------------------------------------------------------------------------





#include "pca9685/pca9685.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
PCA9685::PCA9685(I2C_Driver * i2c_driver)
{
	this->m_i2c_address = PCA9685_I2C_ADDRESS_DEFAULT;
	this->m_i2c_driver = i2c_driver;
}

PCA9685::PCA9685(I2C_Driver * i2c_driver, uint8_t address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("Address supplied is greater than 127. Instead setting the address to the default of 13.");
		// Default the address to PCA9685_I2C_ADDRESS_DEFAULT
		address = PCA9685_I2C_ADDRESS_DEFAULT;
	}

	this->m_i2c_address = address;
	this->m_i2c_driver = i2c_driver;
}





// ------------------------------------------------------
//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
//  G      E        T        &        S      E        T
//  G  GG  EEE      T        && &      SSS   EEE      T
//  G   G  E        T       &  &          S  E        T
//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
// ------------------------------------------------------

uint8_t PCA9685::get_i2c_address()
{
	return this->m_i2c_address;
}

bool PCA9685::set_i2c_address(uint8_t new_address)
{
	if (new_address>127)
	{
		return false;
	}
	this->m_i2c_address = new_address;
	return true;
}





// ------------------------------------------------------------
//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
// ------------------------------------------------------------

// PRIVATE FUNCTION:
bool PCA9685::read_register(uint8_t register_address, uint8_t * value)
{
	// Put the "register address" into a uint8 array
	uint8_t write_array[] = { register_address };
	// Specify the number of bytes in the array
	int num_write_bytes = sizeof(write_array);

	// Initialise a uint8 array for the returned
	// value of the requested register
	int num_value_bytes = 1;
	uint8_t value_array[num_value_bytes];

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes, write_array, num_value_bytes, value_array);
	// Check the status
	if (wasSuccessful)
	{
		// Put the unit8 value into the return variable
		// > Note that the calling function is
		//   responsible to convert this to int8
		//   if the variable takes signed values.
		*value = value_array[0];
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PRIVATE FUNCTION:
bool PCA9685::write_register(uint8_t register_address, uint8_t value, uint8_t num_attempts)
{
	// Put the "register address" and the "value"
	// into a uint8 array
	uint8_t write_array[] = { register_address , value };
	// Specify the number of bytes in the array
	int num_write_bytes = sizeof(write_array);

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes, write_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		if (num_attempts <=  1)
		{
			// Return flag that the operation was unsuccessful
			return false;
		}
		else
		{
			// Sleep for a little bit
			usleep(PCA9685_I2C_ATTEMPT_WAIT_IN_MICRO_SECONDS);
			// Call recursively with one less attempt
			return this->write_register(register_address, value, num_attempts-1);
		}
	}
}

// PRIVATE FUNCTION
bool PCA9685::write_pwm_pulse(uint8_t register_address, uint16_t on_count, uint16_t off_count, uint8_t num_attempts)
{
	// Convert the on and off counts to their most and least
	// significant bytes
	uint8_t on_lsb  = (on_count  & 0x00FF);
	uint8_t on_msb  = (on_count  & 0x0F00) >> 8;
	uint8_t off_lsb = (off_count & 0x00FF);
	uint8_t off_msb = (off_count & 0x0F00) >> 8;

	// Write depending on whether auto-increment is enabled
	bool wasSuccessful = false;
	if (this->m_auto_increment_enabled)
	{
		// Put the "register address" and the "values"
		// into a uint8 array
		uint8_t write_array[] = { register_address , on_lsb , on_msb , off_lsb , off_msb };
		// Specify the number of bytes in the array
		int num_write_bytes = sizeof(write_array);

		// Call the i2c_driver function
		wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes, write_array);
	}
	else
	{
		// > Write byte 1
		uint8_t write_array1[] = { static_cast<uint8_t>(register_address + 0u) , on_lsb };
		int num_write_bytes1 = sizeof(write_array1);
		bool wasSuccessful1 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes1, write_array1);
		// > Write byte 2
		uint8_t write_array2[] = { static_cast<uint8_t>(register_address + 1u) , on_msb };
		int num_write_bytes2 = sizeof(write_array2);
		bool wasSuccessful2 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes2, write_array2);
		// > Write byte 3
		uint8_t write_array3[] = { static_cast<uint8_t>(register_address + 2u) , off_lsb };
		int num_write_bytes3 = sizeof(write_array3);
		bool wasSuccessful3 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes3, write_array3);
		// > Write byte 4
		uint8_t write_array4[] = { static_cast<uint8_t>(register_address + 3u) , off_msb };
		int num_write_bytes4 = sizeof(write_array4);
		bool wasSuccessful4 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes4, write_array4);
		// Combine the success flags
		wasSuccessful = wasSuccessful1 && wasSuccessful2 && wasSuccessful3 && wasSuccessful4;
	}

	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		if (num_attempts <=  1)
		{
			// Return flag that the operation was unsuccessful
			return false;
		}
		else
		{
			// Sleep for a little bit
			usleep(PCA9685_I2C_ATTEMPT_WAIT_IN_MICRO_SECONDS);
			// Call recursively with one less attempt
			return this->write_pwm_pulse(register_address, on_count, off_count, num_attempts-1);
		}
	}
}

// PUBLIC FUNCTION
bool PCA9685::write_pwm_full_on_or_full_off(uint8_t register_address, bool flag_on_off, uint8_t num_attempts)
{
	// Convert the on/off flag to the bytes to write
	uint8_t on_lsb  = 0x00;
	uint8_t on_msb  = 0x00;
	uint8_t off_lsb = 0x00;
	uint8_t off_msb = 0x00;
	// > For full on
	if (flag_on_off)
	{
		on_msb = PCA9685_CHANNEL_FULL_ON;
	}
	// > For full off
	else
	{
		off_msb = PCA9685_CHANNEL_FULL_OFF;
	}

	// Write depending on whether auto-increment is enabled
	bool wasSuccessful = false;
	if (this->m_auto_increment_enabled)
	{
		// Put the "register address" and the "values"
		// into a uint8 array
		uint8_t write_array[] = { register_address , on_lsb , on_msb , off_lsb , off_msb };
		// Specify the number of bytes in the array
		int num_write_bytes = sizeof(write_array);

		// Call the i2c_driver function
		wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes, write_array);
	}
	else
	{
		// > Write byte 1
		uint8_t write_array1[] = { static_cast<uint8_t>(register_address + 0u) , on_lsb };
		int num_write_bytes1 = sizeof(write_array1);
		bool wasSuccessful1 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes1, write_array1);
		// > Write byte 2
		uint8_t write_array2[] = { static_cast<uint8_t>(register_address + 1u) , on_msb };
		int num_write_bytes2 = sizeof(write_array2);
		bool wasSuccessful2 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes2, write_array2);
		// > Write byte 3
		uint8_t write_array3[] = { static_cast<uint8_t>(register_address + 2u) , off_lsb };
		int num_write_bytes3 = sizeof(write_array3);
		bool wasSuccessful3 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes3, write_array3);
		// > Write byte 4
		uint8_t write_array4[] = { static_cast<uint8_t>(register_address + 3u) , off_msb };
		int num_write_bytes4 = sizeof(write_array4);
		bool wasSuccessful4 = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes4, write_array4);
		// > Combine the success flags
		wasSuccessful = wasSuccessful1 && wasSuccessful2 && wasSuccessful3 && wasSuccessful4;
	}

	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		if (num_attempts <=  1)
		{
			// Return flag that the operation was unsuccessful
			return false;
		}
		else
		{
			// Sleep for a little bit
			usleep(PCA9685_I2C_ATTEMPT_WAIT_IN_MICRO_SECONDS);
			// Call recursively with one less attempt
			return this->write_pwm_full_on_or_full_off(register_address, flag_on_off, num_attempts-1);
		}
	}
}

// PRIVATE FUNCTION:
bool PCA9685::read_pwm_pulse_bytes(uint8_t register_address, uint16_t * on_bytes, uint16_t * off_bytes)
{
	// Put the "register address" into a uint8 array
	uint8_t write_array[] = { register_address };
	// Specify the number of bytes in the array
	int num_write_bytes = sizeof(write_array);

	// Initialise a uint8 array for the returned
	// value of the requested register
	// > Note that all pwm pulse details are
	//   stored in 4 consecutive bytes
	int num_value_bytes = 4;
	uint8_t value_array[num_value_bytes];

	// Read depending on whether auto-increment is enabled
	bool wasSuccessful = false;
	if (this->m_auto_increment_enabled)
	{
		// Call the i2c_driver function
		wasSuccessful = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes, write_array, num_value_bytes, value_array);
	}
	else
	{
		// > Read byte 1
		uint8_t write_array1[] = { static_cast<uint8_t>(register_address + 0u) };
		int num_write_bytes1 = sizeof(write_array1);
		int num_value_bytes1 = 1;
		uint8_t value_array1[num_value_bytes1];
		bool wasSuccessful1 = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes1, write_array1, num_value_bytes1, value_array1);
		// > Read byte 2
		uint8_t write_array2[] = { static_cast<uint8_t>(register_address + 1u) };
		int num_write_bytes2 = sizeof(write_array2);
		int num_value_bytes2 = 1;
		uint8_t value_array2[num_value_bytes2];
		bool wasSuccessful2 = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes2, write_array2, num_value_bytes2, value_array2);
		// > Read byte 3
		uint8_t write_array3[] = { static_cast<uint8_t>(register_address + 2u) };
		int num_write_bytes3 = sizeof(write_array3);
		int num_value_bytes3 = 1;
		uint8_t value_array3[num_value_bytes3];
		bool wasSuccessful3 = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes3, write_array3, num_value_bytes3, value_array3);
		// > Read byte 4
		uint8_t write_array4[] = { static_cast<uint8_t>(register_address + 3u) };
		int num_write_bytes4 = sizeof(write_array4);
		int num_value_bytes4 = 1;
		uint8_t value_array4[num_value_bytes4];
		bool wasSuccessful4 = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes4, write_array4, num_value_bytes4, value_array4);
		// > Put the bytes together
		value_array[0] = value_array1[0];
		value_array[1] = value_array2[0];
		value_array[2] = value_array3[0];
		value_array[3] = value_array4[0];
		// > Combine the success flags
		wasSuccessful = wasSuccessful1 && wasSuccessful2 && wasSuccessful3 && wasSuccessful4;
	}
	// Check the status
	if (wasSuccessful)
	{
		// Put the four unit8 values into the two uint16
		// return variable
		*on_bytes  = value_array[0] + 256 * value_array[1];
		*off_bytes = value_array[2] + 256 * value_array[3];
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Set the return variables to zero
		*on_bytes  = 0x0000;
		*off_bytes = 0x0000;
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::reset()
{
	// STEP 1: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Check that bit 7 (RESTART) is a logic 1.
	bool wasSuccessful2 = true;
	if ((current_mode1 & PCA9685_MODE1_RESTART) == PCA9685_MODE1_RESTART)
	{
		// If it is, clear bit 4 (SLEEP).
		uint8_t current_mode1_without_sleep = current_mode1 & (~PCA9685_MODE1_SLEEP);
		wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, PCA9685_MODE1_SLEEP, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);
		// And allow time for oscillator to stabilize (500 micro seconds).
		usleep(500);
	}

	// STEP 3: Write logic 1 to bit 7 (RESTART) of MODE1 register.
	uint8_t current_mode1_without_sleep_with_restart = (current_mode1 & (~PCA9685_MODE1_SLEEP)) | PCA9685_MODE1_RESTART;
	bool wasSuccessful3 = this->write_register(PCA9685_REGISTER_MODE1, PCA9685_MODE1_RESTART, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// All PWM channels will restart and the RESTART bit will clear.

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2 && wasSuccessful3)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::sleep()
{
	// STEP 1: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Write back the MODE1 with the SLEEP bit set
	// to logic 1
	uint8_t current_mode1_with_sleep = current_mode1 | PCA9685_MODE1_SLEEP;
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, current_mode1_with_sleep, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::wakeup()
{
	// STEP 1: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Write back the MODE1 with the SLEEP bit set
	// to logic 0
	uint8_t current_mode1_with_sleep = current_mode1 & (~PCA9685_MODE1_SLEEP);
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, current_mode1_with_sleep, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::get_mode1(uint8_t * mode1)
{
	// Read the MODE 1 register
	bool wasSuccessful = this->read_register(PCA9685_REGISTER_MODE1, mode1);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful)
		return true;
	else
		return false;
}

// PUBLIC FUNCTION
bool PCA9685::get_mode2(uint8_t * mode2)
{
	// Read the MODE 2 register
	bool wasSuccessful = this->read_register(PCA9685_REGISTER_MODE2, mode2);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful)
		return true;
	else
		return false;
}

// PUBLIC FUNCTION
bool PCA9685::set_respond_to_i2c_bit(bool should_respond_sub_address_1, bool should_respond_sub_address_2, bool should_respond_sub_address_3, bool should_respond_all_call)
{
	// STEP 1: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Adjust the respond to I2C bits
	uint8_t new_mode1 = current_mode1;
	// > For responding to the I2C sub-address 1
	if (should_respond_sub_address_1)
		new_mode1 = new_mode1 | PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_1;
	else
		new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_1);
	// > For responding to the I2C sub-address 2
	if (should_respond_sub_address_2)
		new_mode1 = new_mode1 | PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_2;
	else
		new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_2);
	// > For responding to the I2C sub-address 3
	if (should_respond_sub_address_3)
		new_mode1 = new_mode1 | PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_3;
	else
		new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_3);
	// > For responding to the I2C sub-address all call
	if (should_respond_all_call)
		new_mode1 = new_mode1 | PCA9685_MODE1_RESPOND_TO_ALL_CALL_ADDRESS;
	else
		new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_ALL_CALL_ADDRESS);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, new_mode1, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_auto_increment_bit(bool should_auto_increment)
{
	// STEP 1: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Adjust the respond to I2C bits
	uint8_t new_mode1 = current_mode1;
	// > For performing auto-increment
	if (should_auto_increment)
		new_mode1 = new_mode1 | PCA9685_MODE1_AUTO_INCREMENT;
	else
		new_mode1 = new_mode1 & (~PCA9685_MODE1_AUTO_INCREMENT);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, new_mode1, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Update the class variable for tracking the
		// auto-increment mode
		this->m_auto_increment_enabled = should_auto_increment;
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_mode1_defaults()
{
	// STEP 1: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Adjust the bits
	uint8_t new_mode1 = current_mode1;
	// > Set perform auto-increment to logic 1
	new_mode1 = new_mode1 | PCA9685_MODE1_AUTO_INCREMENT;
	// > Set responding to the I2C sub-address 1 to logic 0
	new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_1);
	// > For responding to the I2C sub-address 2 to logic 0
	new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_2);
	// > For responding to the I2C sub-address 3 to logic 0
	new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_3);
	// > For responding to the I2C sub-address all call
	new_mode1 = new_mode1 & (~PCA9685_MODE1_RESPOND_TO_ALL_CALL_ADDRESS);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, new_mode1, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Update the class variable for tracking the
		// auto-increment mode
		this->m_auto_increment_enabled = true;
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_output_driver_mode(bool should_use_totem_pole_structure)
{
	// STEP 1: Read the MODE2 register.
	uint8_t current_mode2;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE2, &current_mode2);

	// STEP 2: Adjust the output driver (OUTDRV) bit
	uint8_t new_mode2 = current_mode2;
	if (should_use_totem_pole_structure)
		new_mode2 = new_mode2 | PCA9685_MODE2_OUTDRV;
	else
		new_mode2 = new_mode2 & (~PCA9685_MODE2_OUTDRV);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE2, new_mode2, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_output_logic_invert_mode(bool should_use_inverted)
{
	// STEP 1: Read the MODE2 register.
	uint8_t current_mode2;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE2, &current_mode2);

	// STEP 2: Adjust the output logic state (INVRT) bit
	uint8_t new_mode2 = current_mode2;
	if (should_use_inverted)
		new_mode2 = new_mode2 | PCA9685_MODE2_INVRT;
	else
		new_mode2 = new_mode2 & (~PCA9685_MODE2_INVRT);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE2, new_mode2, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_output_change_on_mode(bool should_change_on_ack)
{
	// STEP 1: Read the MODE2 register.
	uint8_t current_mode2;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE2, &current_mode2);

	// STEP 2: Adjust the output change on (OCH) bit
	uint8_t new_mode2 = current_mode2;
	if (should_change_on_ack)
		new_mode2 = new_mode2 | PCA9685_MODE2_OCH;
	else
		new_mode2 = new_mode2 & (~PCA9685_MODE2_OCH);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE2, new_mode2, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_mode2_defaults_for_driving_servos()
{
	// STEP 1: Read the MODE2 register.
	uint8_t current_mode2;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE2, &current_mode2);

	// STEP 2: Adjust the bits
	uint8_t new_mode2 = current_mode2;
	// > Set output logic state to be NOT inverted (logic 0)
	new_mode2 = new_mode2 & (~PCA9685_MODE2_INVRT);
	// > Set outputs to change on STOP command (logic 0)
	new_mode2 = new_mode2 & (~PCA9685_MODE2_OCH);
	// > Set output driver bit to totem pole structure (logic 1)
	new_mode2 = new_mode2 | PCA9685_MODE2_OUTDRV;
	// > Set the "active LOW output enable" to logic 00
	new_mode2 = new_mode2 & (~PCA9685_MODE2_OUTNE_0);
	new_mode2 = new_mode2 & (~PCA9685_MODE2_OUTNE_1);

	// STEP 3: Write the new mode
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE2, new_mode2, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_pwm_frequency_in_hz(float freq_in_hz)
{
	// STEP 1: Compute the pre-scaler for the given frequency
	// > Apply the equation from the data sheet
	int pre_scale_value = static_cast<int>( (static_cast<float>(this->m_oscillator_frequency) / (freq_in_hz * 4096.0)) + 0.5f - 1.0f );

	// > Clip the pre-scaler to the min and max allowed
	if (pre_scale_value < PCA9685_PRE_SCALE_MIN)
		pre_scale_value = PCA9685_PRE_SCALE_MIN;
	if (pre_scale_value > PCA9685_PRE_SCALE_MAX)
		pre_scale_value = PCA9685_PRE_SCALE_MAX;

	// > Cast the pre-scaler to a uint8
	uint8_t pre_scale_value_as_uint8 = static_cast<uint8_t>(pre_scale_value);

	// STEP 2: Read the MODE1 register.
	uint8_t current_mode1;
	bool wasSuccessful1 = this->read_register(PCA9685_REGISTER_MODE1, &current_mode1);

	// STEP 2: Write back the MODE1 with the SLEEP bit set
	// to logic 1 and the RESTART bit set to logic 0
	uint8_t mode1_for_sleep = (current_mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
	bool wasSuccessful2 = this->write_register(PCA9685_REGISTER_MODE1, mode1_for_sleep, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// STEP 3: Write the PRE_SCALE value
	bool wasSuccessful3 = this->write_register(PCA9685_PRE_SCALE, pre_scale_value_as_uint8, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// STEP 4: write back the original mode
	bool wasSuccessful4 = this->write_register(PCA9685_REGISTER_MODE1, current_mode1, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// And allow time for wakeup
	usleep(500);

	// STEP 5: Turn on auto increment.
	//this->write_register(PCA9685_MODE1, current_mode1 | PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTO_INCREMENT, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// Update the class variable for the PWM frequency
	if (wasSuccessful3)
	{
		this->m_pwm_frequency = static_cast<float>(this->m_oscillator_frequency) / ( (static_cast<float>(pre_scale_value_as_uint8)+1.0) * 4096.0 );
	}

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful1 && wasSuccessful2 && wasSuccessful3 && wasSuccessful4)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::get_pwm_frequency_in_hz_and_prescale(float * freq_in_hz, uint8_t * pre_scale)
{
	// STEP 1: Read the PRE_SCALE value
	bool wasSuccessful = this->read_register(PCA9685_PRE_SCALE, pre_scale);

	// STEP 2: Compute the frequency
	*freq_in_hz = static_cast<float>(this->m_oscillator_frequency) / ( (static_cast<float>(*pre_scale)+1.0) * 4096.0 );

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_pwm_pulse(uint8_t channel_number, uint16_t on_count, uint16_t off_count)
{
	// STEP 1: Check that the channel number is valid
	if ( (channel_number < PCA9685_CHANNEL_MIN) || (PCA9685_CHANNEL_MAX < channel_number) )
	{
		// Return flag that the operation was unsuccessful
		return false;
	}

	// STEP 2: Convert the channel number to a starting register
	uint8_t channel_start_register = PCA9685_REGISTER_CHANNEL_0_ON_L + 4 * channel_number;

	// STEP 3: Clip the pulse values to the min and max allowed
	// > For the on count:
	if (on_count < PCA9685_PULSE_ON_OFF_MIN)
		on_count = PCA9685_PULSE_ON_OFF_MIN;
	if (on_count > PCA9685_PULSE_ON_OFF_MAX)
		on_count = PCA9685_PULSE_ON_OFF_MAX;
	// > For the off count:
	if (off_count < PCA9685_PULSE_ON_OFF_MIN)
		off_count = PCA9685_PULSE_ON_OFF_MIN;
	if (off_count > PCA9685_PULSE_ON_OFF_MAX)
		off_count = PCA9685_PULSE_ON_OFF_MAX;

	// STEP 4: Check whether the full on or full off cases are triggered
	bool flag_full_on_or_full_off_requested = false;
	bool flag_on_off = false;
	if ( (on_count==PCA9685_PULSE_ON_OFF_MIN) && (off_count==PCA9685_PULSE_ON_OFF_MAX) )
	{
		flag_full_on_or_full_off_requested = true;
		flag_on_off = true;
	}
	else if ( (on_count==PCA9685_PULSE_ON_OFF_MAX) && (off_count==PCA9685_PULSE_ON_OFF_MIN) )
	{
		flag_full_on_or_full_off_requested = true;
		flag_on_off = false;
	}

	// STEP 5: Write the on and off values
	bool wasSuccessful = false;
	if (flag_full_on_or_full_off_requested)
	{
		wasSuccessful = this->write_pwm_full_on_or_full_off(channel_start_register, flag_on_off, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);
	}
	else
	{
		wasSuccessful = this->write_pwm_pulse(channel_start_register, on_count, off_count, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);
	}

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_pwm_pulse_in_microseconds(uint8_t channel_number, uint16_t pulse_with_in_microseconds)
{
	// STEP 1: Check that the channel number is valid
	if ( (channel_number < PCA9685_CHANNEL_MIN) || (PCA9685_CHANNEL_MAX < channel_number) )
	{
		// Return flag that the operation was unsuccessful
		return false;
	}

	// STEP 2: Convert the pulse width in micro-seconds
	// to a width in counts
	float micro_seconds_per_count = (1000000.0 / 4096.0) / this->m_pwm_frequency;
	uint16_t off_count = static_cast<uint16_t>( static_cast<float>(pulse_with_in_microseconds) / micro_seconds_per_count );

	// STEP 3: Call the function to set the pulse in counts
	return this->set_pwm_pulse(channel_number, 0, off_count);
}

// PUBLIC FUNCTION
bool PCA9685::set_pwm_full_on_or_full_off(uint8_t channel_number, bool flag_on_off)
{
	// STEP 1: Check that the channel number is valid
	if ( (channel_number < PCA9685_CHANNEL_MIN) || (PCA9685_CHANNEL_MAX < channel_number) )
	{
		// Return flag that the operation was unsuccessful
		return false;
	}

	// STEP 2: Convert the channel number to a starting register
	uint8_t channel_start_register = PCA9685_REGISTER_CHANNEL_0_ON_L + 4 * channel_number;

	// STEP 3: Write the full on or full off value
	bool wasSuccessful = this->write_pwm_full_on_or_full_off(channel_start_register, flag_on_off, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::get_pwm_pulse(uint8_t channel_number, uint16_t * on_bytes, uint16_t * off_bytes)
{
	// STEP 1: Check that the channel number is valid
	if ( (channel_number < PCA9685_CHANNEL_MIN) || (PCA9685_CHANNEL_MAX < channel_number) )
	{
		// Return flag that the operation was unsuccessful
		return false;
	}

	// STEP 2: Convert the channel number to a starting register
	uint8_t channel_start_register = PCA9685_REGISTER_CHANNEL_0_ON_L + 4 * channel_number;

	// STEP 3: Read the pulse details
	bool wasSuccessful = this->read_pwm_pulse_bytes(channel_start_register, on_bytes, off_bytes);

	// RETURN THE SUCCESS FLAG
	if (wasSuccessful)
	{		
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Set the return variables to zero
		*on_bytes  = 0x0000;
		*off_bytes = 0x0000;
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool PCA9685::set_all_channels_full_off()
{
	// STEP 1: Specify the start register
	uint8_t channel_start_register = PCA9685_ALL_CHANNEL_ON_L;

	// STEP 2: Write the full off value
	bool flag_on_off = false;
	return this->write_pwm_full_on_or_full_off(channel_start_register, flag_on_off, PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT);
}



// CONVENIENCE FUNCTIONS
bool PCA9685::initialise_with_frequency_in_hz(float new_freq_in_hz, bool verbose)
{

	// Initialise a boolean variable for the result
	// of calls to functions
	bool result;

	// Initialise a boolean variable for the overall
	// result to return
	bool return_result = true;

	// > Set the mode 1 defaults
	result = this->set_mode1_defaults();
	if (verbose)
	{
		if (result)
			printf("PCA9685 - set mode 1 defaults successful, for I2C address %d\n", this->get_i2c_address() );
		else
			printf("FAILED - PCA9685 - set mode 1 defaults NOT successful for I2C address %d\n", this->get_i2c_address() );
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// > Set the mode 2 defaults
	result = this->set_mode2_defaults_for_driving_servos();
	if (verbose)
	{
		if (result)
			printf("PCA9685 - set mode 2 defaults successful, for I2C address %d\n", this->get_i2c_address() );
		else
			printf("FAILED - PCA9685 - set mode 2 defaults NOT successful for I2C address %d\n", this->get_i2c_address() );
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// > Call the wakeup function
	result = this->wakeup();
	if (verbose)
	{
		if (result)
			printf("PCA9685 - wakeup successful, for I2C address %d\n", this->get_i2c_address() );
		else
			printf("FAILED - PCA9685 - wakeup NOT successful for I2C address %d\n", this->get_i2c_address() );
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// > Get the current mode 1 configuration
	uint8_t current_mode1;
	result = this->get_mode1(&current_mode1);
	if (verbose)
	{
		if (result)
		{
			std::cout << "PCA9685 - get mode 1 returned: " << std::bitset<8>(current_mode1) << ", for I2C address " << static_cast<int>(this->get_i2c_address()) << "\n";
			std::cout << "                     expected: " << "00100000" << "\n";
		}
		else
		{
			printf("FAILED - PCA9685 - get mode 1 NOT successful for I2C address %d\n", this->get_i2c_address() );
		}
	}
	// Update the "cumulative" result
	return_result = (return_result && result);


	// > Get the current mode 2 configuration
	uint8_t current_mode2;
	result = this->get_mode2(&current_mode2);
	if (verbose)
	{
		if (result)
		{
			std::cout << "PCA9685 - get mode 2 returned: " << std::bitset<8>(current_mode2) << ", for I2C address " << static_cast<int>(this->get_i2c_address()) << "\n";
			std::cout << "                     expected: " << "00000100" << "\n";
		}
		else
		{
			printf("FAILED - PCA9685 - get mode 2 NOT successful for I2C address %d\n", this->get_i2c_address() );
		}
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// > Set the frequency
	result = this->set_pwm_frequency_in_hz(new_freq_in_hz);
	if (verbose)
	{
		if (result)
			printf("PCA9685 - set the PWM frequency successfully, for I2C address %d\n", this->get_i2c_address() );
		else
			printf("FAILED - PCA9685 - set the PWM frequency NOT successful for I2C address %d\n", this->get_i2c_address() );
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// > Get the frequency to double check
	float pwm_freq_retrieved;
	uint8_t pre_scale_retrieved;
	result = this->get_pwm_frequency_in_hz_and_prescale(&pwm_freq_retrieved, &pre_scale_retrieved);
	if (verbose)
	{
		if (result)
			printf("PCA9685 - get PWM frequency returned:\n> freq = %f\n> pre scale = %d\nfor I2C address %d\n", pwm_freq_retrieved, pre_scale_retrieved, this->get_i2c_address() );
		else
			printf("FAILED - PCA9685 - get PWM frequency NOT successful for I2C address %d\n", this->get_i2c_address() );
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Check that the frequency retrieved agrees with
	// what was requested, to within 1Hz
	float freq_diff = pwm_freq_retrieved - new_freq_in_hz;
	if ( (freq_diff < -1.0) || (1.0 < freq_diff) )
	{
		if (verbose)
		{
			std::cout << "PCA9685 DRIVER: ERROR requested and set frequency differ by more than 1Hz. freq requested = " << new_freq_in_hz << ", freq set = " << pwm_freq_retrieved;
		}
		// Update the "cumulative" result
		return_result = false;
	}

	// Short sleep
	usleep(1000);

	// > Set all channels to full off
	result = this->set_all_channels_full_off();
	if (verbose)
	{
		if (result)
			printf("PCA9685 - set all channels to full off successfully, for I2C address %d\n", this->get_i2c_address() );
		else
			printf("FAILED - PCA9685 - set all channels to full off NOT successful for I2C address %d\n", this->get_i2c_address() );
	}

	if (verbose)
	{
		std::cout << "PCA9685 DRIVER: Finished setting up the PCA9685 servo driver with I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Return the "cumulative" result
	return return_result;
}
