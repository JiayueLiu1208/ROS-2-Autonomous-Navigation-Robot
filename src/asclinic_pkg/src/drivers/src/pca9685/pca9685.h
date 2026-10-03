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

// NOTES TAKEN FROM THE NXP DOCUMENTATION
// > This documentation can be downloaded found here:
//   https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/ic-led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685
// > The Adafruit guide can be found here:
//   https://learn.adafruit.com/16-channel-pwm-servo-driver
// > The Adafruit Arduino library can be found here:
//   https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library





#ifndef PCA9685_H
#define PCA9685_H





#include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <bitset>

#include "pca9685/pca9685_constants.h"

#include "i2c_driver/i2c_driver.h"





// ----------------------------------------------------------
//   CCCC  L        A     SSSS   SSSS     DDDD   EEEEE  FFFFF
//  C      L       A A   S      S         D   D  E      F
//  C      L      A   A   SSS    SSS      D   D  EEE    FFF
//  C      L      AAAAA      S      S     D   D  E      F
//   CCCC  LLLLL  A   A  SSSS   SSSS      DDDD   EEEEE  F
// ----------------------------------------------------------

class PCA9685
{

private:
	uint8_t m_i2c_address;
	I2C_Driver * m_i2c_driver;

	// Keep track of the oscillator frequency
	unsigned int m_oscillator_frequency = PCA9685_OSCILLATOR_FREQUENCY_IN_HERTZ;

	// Keep track of the PWM frequency
	float m_pwm_frequency = PCA9685_PWM_DEFAULT_FREQUENCY_IN_HERTZ;

	// Keep track of the auto-increment mode
	bool m_auto_increment_enabled = false;





	// ----------------------------------------------------------------------------
	//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
	//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
	//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
	//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
	//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
	// ----------------------------------------------------------------------------
public:
	PCA9685(I2C_Driver * i2c_driver);
	PCA9685(I2C_Driver * i2c_driver, uint8_t address);





	// ------------------------------------------------------
	//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
	//  G      E        T        &        S      E        T
	//  G  GG  EEE      T        && &      SSS   EEE      T
	//  G   G  E        T       &  &          S  E        T
	//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
	// ------------------------------------------------------
public:
	uint8_t get_i2c_address();
	bool set_i2c_address(uint8_t new_address);





	// ------------------------------------------------------------
	//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
	//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
	//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
	//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
	//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
	// ------------------------------------------------------------

private:
	bool read_register(uint8_t register_address, uint8_t * value);

	bool write_register(uint8_t register_address, uint8_t value, uint8_t num_attempts);

	bool write_pwm_pulse(uint8_t register_address, uint16_t on_count, uint16_t off_count, uint8_t num_attempts);

	bool write_pwm_full_on_or_full_off(uint8_t register_address, bool flag_on_off, uint8_t num_attempts);

	bool read_pwm_pulse_bytes(uint8_t register_address, uint16_t * on_bytes, uint16_t * off_bytes);

public:

	// RESET, SLEEP, WAKEUP
	bool reset();
	bool sleep();
	bool wakeup();

	// GET THE MODE 1 AND 2 BYTE
	bool get_mode1(uint8_t * mode1);
	bool get_mode2(uint8_t * mode2);

	// SET THE RESPONDS TO I2C SUB-ADDRESSES AND ALL CALL
	bool set_respond_to_i2c_bit(bool should_respond_sub_address_1, bool should_respond_sub_address_2, bool should_respond_sub_address_3, bool should_respond_all_call);

	// SET THE AUTO-INCREMENT BIT
	bool set_auto_increment_bit(bool should_auto_increment);

	// SET THE DEFAULTS FOR THE MODE 1 REGISTER
	bool set_mode1_defaults();

	// SET THE OUTPUT DRIVER MODE
	bool set_output_driver_mode(bool should_use_totem_pole_structure);

	// SET THE OUTPUT LOGIC INVERT MODE
	bool set_output_logic_invert_mode(bool should_use_inverted);

	// SET THE OUTPUTS CHANGE ON MODE
	bool set_output_change_on_mode(bool should_change_on_ack);

	// SET THE DEFAULTS FOR THE MODE 2 REGISTER
	// WHEN DRIVING SERVOS
	bool set_mode2_defaults_for_driving_servos();

	// SET AND GET THE PWM FREQUENCY
	bool set_pwm_frequency_in_hz(float freq_in_hz);
	bool get_pwm_frequency_in_hz_and_prescale(float * freq_in_hz, uint8_t * pre_scale);

	// SET THE PULSE OF A CHANNEL
	bool set_pwm_pulse(uint8_t channel_number, uint16_t on_count, uint16_t off_count);

	// SET THE PULSE OF A CHANNEL IN MICRO SECONDS
	bool set_pwm_pulse_in_microseconds(uint8_t channel_number, uint16_t pulse_with_in_microseconds);

	// SET A CHANNEL TO BE FULL ON OR FULL OFF
	bool set_pwm_full_on_or_full_off(uint8_t channel_number, bool flag_on_off);

	// GET THE PULSE DETAILS FOR A CHANNEL
	bool get_pwm_pulse(uint8_t channel_number, uint16_t * on_bytes, uint16_t * off_bytes);

	// TURN OFF ALL CHANNELS
	bool set_all_channels_full_off();



// CONVENIENCE FUNCTIONS
public:
	bool initialise_with_frequency_in_hz(float new_freq_in_hz, bool verbose);

}; // END OF CLASS DEFINITION





#endif // PCA9685_H
