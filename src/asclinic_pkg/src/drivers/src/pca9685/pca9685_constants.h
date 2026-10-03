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





#ifndef PCA9685_CONSTANTS_H
#define PCA9685_CONSTANTS_H



// I2C ADDRESS
#define PCA9685_I2C_ADDRESS_DEFAULT     0x40 /**< Default PCA9685 I2C Slave Address */



// I2C WRITE ATTEMPTS
#define PCA9685_I2C_WRITE_ATTEMPTS_DEFAULT             5 /**< Number of times to attempt an I2C write */
#define PCA9685_I2C_ATTEMPT_WAIT_IN_MICRO_SECONDS     50 /**< Time to wait between failed I2C writes */



// FREQUENCY OF THE IN-BUILT CLOCK
#define PCA9685_OSCILLATOR_FREQUENCY_IN_HERTZ     25000000 /**< Frequency of the PCA9685 internal oscillator frequency, i.e., 25MHz as per the datasheet */



// DEFAULT FREQUENCY OF PWM SIGNAL
#define PCA9685_PWM_DEFAULT_FREQUENCY_IN_HERTZ     200 /**< Frequency of the PCA9685 internal oscillator frequency, i.e., 25MHz as per the datasheet */



// REGISTER NUMBERS
#define PCA9685_REGISTER_MODE1                0x00 /**< Register number for: Mode 1 */
#define PCA9685_REGISTER_MODE2                0x01 /**< Register number for: Mode 2 */
#define PCA9685_REGISTER_SUB_ADDRESS_1        0x02 /**< Register number for: I2C-bus subaddress 1 */
#define PCA9685_REGISTER_SUB_ADDRESS_2        0x03 /**< Register number for: I2C-bus subaddress 2 */
#define PCA9685_REGISTER_SUB_ADDRESS_3        0x04 /**< Register number for: I2C-bus subaddress 3 */
#define PCA9685_REGISTER_ALL_CALL_ADDRESS     0x05 /**< Register number for: LED All Call I2C-bus address */
#define PCA9685_REGISTER_CHANNEL_0_ON_L       0x06 /**< Register number for: Channel 0 on tick, low byte*/
#define PCA9685_REGISTER_CHANNEL_0_ON_H       0x07 /**< Register number for: Channel 0 on tick, high byte*/
#define PCA9685_REGISTER_CHANNEL_0_OFF_L      0x08 /**< Register number for: Channel 0 off tick, low byte */
#define PCA9685_REGISTER_CHANNEL_0_OFF_H      0x09 /**< Register number for: Channel 0 off tick, high byte */
//
// The register numbers for channels 1-15 are
// simply obtained by increment by 4, i.e:
// _LED0_OFF_H     0x45
//
#define PCA9685_ALL_CHANNEL_ON_L      0xFA /**< Register number for: load all the CHANNEL_n_ON registers, low byte */
#define PCA9685_ALL_CHANNEL_ON_H      0xFB /**< Register number for: load all the CHANNEL_n_ON registers, high byte */
#define PCA9685_ALL_CHANNEL_OFF_L     0xFC /**< Register number for: load all the CHANNEL_n_ON registers, low byte */
#define PCA9685_ALL_CHANNEL_OFF_H     0xFD /**< Register number for: load all the CHANNEL_n_ON registers,high byte */
#define PCA9685_PRE_SCALE             0xFE /**< Register number for: Pre-scaler for PWM output frequency. Note that the PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1. */
#define PCA9685_TESTMODE              0xFF /**< Register number for: defines the test mode to be entered */



// MODE1 BITS
#define PCA9685_MODE1_RESPOND_TO_ALL_CALL_ADDRESS     0x01 /**< If 1, then PCA9685 responds to LED All Call I2C-bus address */
#define PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_3        0x02 /**< If 1, then PCA9685 responds to I2C-bus subaddress 3 */
#define PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_2        0x04 /**< If 1, then PCA9685 responds to I2C-bus subaddress 2 */
#define PCA9685_MODE1_RESPOND_TO_SUB_ADDRESS_1        0x08 /**< If 1, then PCA9685 responds to I2C-bus subaddress 1 */
#define PCA9685_MODE1_SLEEP                           0x10 /**< If 1, then Low Power mode is active and the oscillator is off */
#define PCA9685_MODE1_AUTO_INCREMENT                  0x20 /**< If 1, then Auto-Increment is enabled */
#define PCA9685_MODE1_EXTERNAL_CLKOCK                 0x40 /**< If 1, then PCA9685 uses the EXTCLK pin as the clock */
#define PCA9685_MODE1_RESTART                         0x80 /**< If 1, then Restart is enabled */



// MODE2 BITS
#define PCA9685_MODE2_OUTNE_0     0x01 /**< Active LOW output enable input */
#define PCA9685_MODE2_OUTNE_1     0x02 /**< Active LOW output enable input - high impedience */
#define PCA9685_MODE2_OUTDRV      0x04 /**< If 1, then the 16-channels outputs are configured in a totem pole structure, otherwise configured in an open-drain structure */
#define PCA9685_MODE2_OCH         0x08 /**< If 1, then outputs change on ACK, otherwise outputs change on STOP */
#define PCA9685_MODE2_INVRT       0x10 /**< If 1, output logic state inverted, which is the value to use when no external driver is used, otherwise set to 0 */



// CHANNEL FULL ON AND FULL OFF BITS
#define PCA9685_CHANNEL_FULL_ON      0x10 /**< If 1, then the channel output is always ON. If both ON and OFF are set, then OFF takes precedence */
#define PCA9685_CHANNEL_FULL_OFF     0x10 /**< If 1, then the channel output is always OFF. If both ON and OFF are set, then OFF takes precedence */



// PRECALE LIMITS
#define PCA9685_PRE_SCALE_MIN     0x03 /**< Minimum pre-scale value, corresponds to maximum PWM frequency of 1526Hz when using the 25MHz clock */
#define PCA9685_PRE_SCALE_MAX     0xFF /**< Maximum pre-scale value, corresponds to minimum PWM frequency of 24Hz when using the 25MHz clock */



// CHANNEL LIMITS
#define PCA9685_CHANNEL_MIN      0 /**< Minimum channel number */
#define PCA9685_CHANNEL_MAX     15 /**< Maximum channel number */



// PWM PULSE WIDTH LIMITS
#define PCA9685_PULSE_ON_OFF_MIN        0 /**< Minimum count for when the pulse of a channel turns on */
#define PCA9685_PULSE_ON_OFF_MAX     4095 /**< Minimum count for when the pulse of a channel turns off, i.e., (2^12-1) as per the data sheet */



#endif // PCA9685_CONSTANTS_H
