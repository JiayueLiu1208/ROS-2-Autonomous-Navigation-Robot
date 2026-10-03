// Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
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
// Node for I2C bus with only a servo driver device connect
//
// ----------------------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/float32.hpp"

#include "i2c_driver/i2c_driver.h"
#include "pca9685/pca9685.h"

#include "asclinic_pkg/msg/servo_pulse_width.hpp"


// MEMBER VARIABLES THAT ARE PARAMETERS FOR THIS NODE:
// > For the verbosity level of displaying info
//   Note: the levels of increasing verbosity are defined as:
//   0 : Info is not displayed. Warnings and errors are still displayed
//   1 : Startup info is displayed
//   2 : Info about messages received is displayed
int m_servo_driver_verbosity = 1;

// Settings for the servo driver
float m_pwm_frequency_in_hz = 50.0;
uint16_t m_min_pulse_width_in_us = 500;
uint16_t m_max_pulse_width_in_us = 2500;



// ALL OTHER MEMBER VARIABLES FOR THIS NODE:
// Jetson Orin Nano: Pins 27 and 28 are on I2C bus 1
// For detection, run: sudo i2cdetect -y -r 1
const char * m_i2c_device_name = "/dev/i2c-1";
I2C_Driver m_i2c_driver (m_i2c_device_name);

// > For the PCA9685 PWM Servo Driver driver
const uint8_t m_pca9685_address = 0x42;
PCA9685 m_pca9685_servo_driver (&m_i2c_driver, m_pca9685_address);


using namespace asclinic_pkg::msg;

class I2C_FOR_SERVO : public rclcpp::Node
{
public:
  I2C_FOR_SERVO() : Node("i2c_for_servos")
  {
    // Getting Namespace
    std::string ns_for_group = this->get_namespace();
    
    // Display that this node is launching
    RCLCPP_INFO_STREAM(this->get_logger(), "[I2C FOR SERVOS] Now launching this node in namespace: " << ns_for_group);

    // Get the parameter values:
    // > For the verbosity
    this->declare_parameter<int>("servo_driver_verbosity", m_servo_driver_verbosity);
    m_servo_driver_verbosity_ = this->get_parameter("servo_driver_verbosity").as_int();


    // > For the PWM frequency parameter
    this->declare_parameter<float>("servo_driver_pwm_frequency_in_hertz", m_pwm_frequency_in_hz);
    m_pwm_frequency_in_hz_ = this->get_parameter("servo_driver_pwm_frequency_in_hertz").as_double();

    // > For the min pulse width parameter
    int temp_min_pulse_width;
    this->declare_parameter<int>("servo_driver_min_pulse_width_in_microseconds", static_cast<int>(m_min_pulse_width_in_us_));
    if(!this->get_parameter("servo_driver_min_pulse_width_in_microseconds", temp_min_pulse_width)){
      RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED to get \"min_pulse_width_in_microseconds\" parameter. Using default value instead.");
    }
    else
    {
      if(0 <= temp_min_pulse_width && temp_min_pulse_width <= 65535){  // Fixed: && instead of &
        m_min_pulse_width_in_us_ = static_cast<uint16_t>(temp_min_pulse_width);
      }
      else
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] \"min_pulse_width_in_microseconds\" of " << temp_min_pulse_width << " is not in the valid range of [0,65535]. Using default value instead.");
      }
    }
    
    // > For the max pulse width parameter
    int temp_max_pulse_width;
    this->declare_parameter<int>("servo_driver_max_pulse_width_in_microseconds", static_cast<int>(m_max_pulse_width_in_us_));
    if(!this->get_parameter("servo_driver_max_pulse_width_in_microseconds", temp_max_pulse_width)){
      RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED to get \"max_pulse_width_in_microseconds\" parameter. Using default value instead.");
    }
    else
    {
      if (0 <= temp_max_pulse_width && temp_max_pulse_width <= 65535){
        m_max_pulse_width_in_us_ = static_cast<uint16_t>(temp_max_pulse_width);
      }
      else
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] \"max_pulse_width_in_microseconds\" of " << temp_max_pulse_width << " is not in the valid range of [0,65535]. Using default value instead.");
      }
    }

    // Initialise a subscriber for the servo driver
    set_servo_pulse_width_subscriber_ = this->create_subscription<ServoPulseWidth>("set_servo_pulse_width", 1, 
      std::bind(&I2C_FOR_SERVO::servoSubscriberCallback, this, std::placeholders::_1));
    
    // Display command line command for publishing a servo pulse width request
    if(m_servo_driver_verbosity_ >= 1)
      RCLCPP_INFO_STREAM(this->get_logger(),
        "[I2C FOR SERVOS] publish servo requests from command line with: "
        "ros2 topic pub --once " << ns_for_group << "/set_servo_pulse_width "
        "asclinic_pkg/msg/ServoPulseWidth \"{channel: 14, pulse_width_in_microseconds: 1100}\"");

    // Open the I2C device
    bool open_success = m_i2c_driver.open_i2c_device();

    // Display the status
    if(!open_success){
      RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED to open I2C device named " 
        << m_i2c_driver.get_device_name());
    }
    else
    {
      if(m_servo_driver_verbosity_ >= 1)
        RCLCPP_INFO_STREAM(this->get_logger(), "[I2C FOR SERVOS] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
    }

    // SET THE CONFIGURATION OF THE SERVO DRIVER
    
    // Specify the frequency of the servo driver
    float new_frequency_in_hz = m_pwm_frequency_in_hz_;

    // Check if a device exists at the address
    bool servo_driver_is_connected = m_i2c_driver.check_for_device_at_address(
      m_pca9685_servo_driver.get_i2c_address());
    
    if (servo_driver_is_connected)
    {
      // Call the Servo Driver initialisation function
      bool verbose_display_for_servo_driver_init = false;
      bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);

      // Display if an error occurred
      if(!result_servo_init)
        RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()));
    }
    else
    {
      // Display that the device is not connected
      RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED - Servo driver device NOT detected at I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()));
    }

    if(m_servo_driver_verbosity_ >= 1)
      RCLCPP_INFO(this->get_logger(), "[I2C FOR SERVOS] Node initialisation complete");
  }
  
  ~I2C_FOR_SERVO()
  {
    // Close the I2C device
    bool close_success = m_i2c_driver.close_i2c_device();

    // Display the status
    if(!close_success)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED to close I2C device named " 
        << m_i2c_driver.get_device_name());
    }
    else
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "[I2C FOR SERVOS] Successfully closed device named " 
        << m_i2c_driver.get_device_name());
    }
  }

private:
  // MEMBER VARIABLES THAT ARE PARAMETERS FOR THIS NODE:
  // > For the verbosity level of displaying info
  //   Note: the levels of increasing verbosity are defined as:
  //   0 : Info is not displayed. Warnings and errors are still displayed
  //   1 : Startup info is displayed
  //   2 : Info about messages received is displayed
  int m_servo_driver_verbosity_ = 1;

  // Settings for the servo driver
  float m_pwm_frequency_in_hz_ = m_pwm_frequency_in_hz;
  uint16_t m_min_pulse_width_in_us_ = m_min_pulse_width_in_us;
  uint16_t m_max_pulse_width_in_us_ = m_max_pulse_width_in_us;



  rclcpp::Subscription<ServoPulseWidth>::SharedPtr set_servo_pulse_width_subscriber_;

  void servoSubscriberCallback(const ServoPulseWidth &msg);
};

// Respond to subscriber receiving a message
// > To test this out without creating an additional
//   ROS node
//   1) First use the command:
//        ros2 topic list
//      To identify the full name of this subscription topic.
//   2) Then use the following command to send message on
//      that topic
//        ros2 topic pub --once <namespace>/set_servo_pulse_width asclinic_pkg/msg/ServoPulseWidth "{channel: 14, pulse_width_in_microseconds: 1100}"
//      where "<namespace>/set_servo_pulse_width" is the full
//      name identified in step 1.
//      You can check the correct channel looking at the connections to the servo driver PCA9685
//
void I2C_FOR_SERVO::servoSubscriberCallback(const ServoPulseWidth& msg)
{
  // Extract the channel and pulse width from the message
  uint8_t channel = msg.channel;
  uint16_t pulse_width_in_us = msg.pulse_width_in_microseconds;

  // Display the values from the message received
  if (m_servo_driver_verbosity_ >= 2)
    RCLCPP_INFO_STREAM(this->get_logger(), "[I2C FOR SERVOS] Message received for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us));

  // Limit the pulse width to be either:
  // > zero, or
  // > in the range [m_min_pulse_width_in_us_,m_max_pulse_width_in_us_]
  if(pulse_width_in_us > 0)
  {
    if (pulse_width_in_us < m_min_pulse_width_in_us_)
      pulse_width_in_us = m_min_pulse_width_in_us_;
    if (pulse_width_in_us > m_max_pulse_width_in_us_)
      pulse_width_in_us = m_max_pulse_width_in_us_;
  }

  // Call the function to set the desired pulse width
  bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

  // Display if an error occurred
  if(!result)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[I2C FOR SERVOS] FAILED to set pulse width for servo at channel " << static_cast<int>(channel));
  }
}

int main(int argc, char* argv[])
{
  // Initialise the node
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<I2C_FOR_SERVO>();
  
  // Spin as a single-threaded node
  rclcpp::spin(node);

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
