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
// I2C Driver for Linux
//
// ----------------------------------------------------------------------------





#include "i2c_driver/i2c_driver.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
I2C_Driver::I2C_Driver(const char * device_name)
{
	// Check that the name provided is not too long
	if ((int)strlen(device_name) < MAX_DEVICE_NAME_LENGTH )
	{
		// Set the given name to the member variable
		strcpy( this->m_device_name , device_name );
	}
	else
	{
		// Inform the user
		printf("FAILED to initialise driver becuse provided device_name is too long.\n" );
		printf("> device_name = %s\n", device_name);
		printf("> strlen(device_name) = %d, MAX_DEVICE_NAME_LENGTH = %d\n", (int)strlen(device_name), MAX_DEVICE_NAME_LENGTH );
		printf("> Defaulting instead to /dev/i2c-1\n");
		// Set the device name to a default
		strcpy( this->m_device_name , "/dev/i2c-1" );
	}
	
	// Initialise the "I2C_State" as closed
	this->m_state = I2C_Driver::I2C_State::closed;
	// Initialise the file descriptor to the invalid value
	this->m_file_descriptor = -1;
}





// ------------------------------------------------------
//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
//  G      E        T        &        S      E        T
//  G  GG  EEE      T        && &      SSS   EEE      T
//  G   G  E        T       &  &          S  E        T
//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
// ------------------------------------------------------

const char * I2C_Driver::get_device_name()
{
	return this->m_device_name;
}

int I2C_Driver::get_state()
{
	return (int)this->m_state;
}

int I2C_Driver::get_file_descriptor()
{
	return this->m_file_descriptor;
}





// ------------------------------------------------------------
//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
// ------------------------------------------------------------

bool I2C_Driver::open_i2c_device()
{
	// Call the function to open the I2C device
	int fd = open(this->m_device_name, O_RDWR);

	// Determine success based on the returned integer
	if (fd == -1)
	{
		// Inform the user
		perror(this->m_device_name);
		// Set the file descriptor to the invalid value
		this->m_file_descriptor = -1;
		// Set the I2C State variable
		this->m_state = I2C_Driver::I2C_State::closed;
		// Return flag that the opening was unsuccessful
		return false;
	}

	// Set the file descriptor integer to the member variable
	this->m_file_descriptor = fd;
	// Set the I2C State variable
	this->m_state = I2C_Driver::I2C_State::open;
	
	// Return flag that the opening was successful
	return true;
}

bool I2C_Driver::close_i2c_device()
{
	// Check that the file descriptor is valid
	if (this->m_file_descriptor > -1)
	{
		// Call the function to close the I2C device
		close(this->m_file_descriptor);
		// Set the file descriptor to the invalid value
		this->m_file_descriptor = -1;
		// Set the I2C State variable
		this->m_state = I2C_Driver::I2C_State::closed;
		// Return flag that I2C close was successful
		return true;
	}
	// Return flag that I2C close was unsuccessful
	return false;
}


bool I2C_Driver::write_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array)
{
	// Create an array of "i2c_msg structs" with:
	// > One message for the data to write
	struct i2c_msg message = { address, 0, num_write_btyes, write_data_array };

	// Create the struct for using the ioctl interface
	struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

	// Call the ioctl interface
	int result = ioctl(this->m_file_descriptor, I2C_RDWR, &ioctl_data);

	// Check the result of the ioctl call
	if (result != 1)
	{
		// Inform the user
		//perror("FAILED result from call to ioctl.");
		// Return flag that ioctl was unsuccessful
		return false;
	}
	// Return flag that ioctl was successful
	return true;
}


bool I2C_Driver::write_data_then_read_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array, uint16_t num_read_btyes, uint8_t * read_data_array)
{
	// Create an array of "i2c_msg structs" with:
	// > First message for the data to write
	// > Second message for reading data
	struct i2c_msg messages[] = {
		{ address, 0       , num_write_btyes, write_data_array },
		{ address, I2C_M_RD, num_read_btyes , read_data_array  },
	};

	// Create the struct for using the ioctl interface
	struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

	// Call the ioctl interface
	int result = ioctl(this->m_file_descriptor, I2C_RDWR, &ioctl_data);

	// Check the result of the ioctl call
	if (result != 2)
	{
		// Inform the user
		//perror("FAILED result from call to ioctl.");
		// Return flag that ioctl was unsuccessful
		return false;
	}
	// Return flag that ioctl was successful
	return true;
}


bool I2C_Driver::check_for_device_at_address(uint8_t address)
{
	// As per this manual page:
	//   https://manpages.ubuntu.com/manpages/bionic/man8/i2cdetect.8.html
	// There is no standard method for detecting
	// a whether or not a device is connected at
	// a particular address.
	// The method used here is to write "0" to
	// the address and read one byte. If a byte
	// is received, then there is a device connected
	// at that address, otherwise there is not
	// considered to be a device at that address.

	// Specify writing zero and reading one byte
	uint16_t num_write_btyes = 1;
	uint8_t write_data_array[1] = {0};
	uint16_t num_read_btyes = 1;
	uint8_t read_data_array[1];

	// Call the "write_data_then_read_data" function
	return this->write_data_then_read_data(address, num_write_btyes, write_data_array, num_read_btyes, read_data_array);
}
