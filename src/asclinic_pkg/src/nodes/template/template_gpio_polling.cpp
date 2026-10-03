// Copyright (C) 2025, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
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
// /_/   \_\____/ \____|_|_|_| |_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// Template node for polling the value of a GPIO pin at a fixed frequency
//
// ----------------------------------------------------------------------------



#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <gpiod.h>


// Using declarations for convenience
using std::placeholders::_1;        // For std::bind placeholder in callback binding

class TemplateGpioPolling : public rclcpp::Node
{
public:
	TemplateGpioPolling() : Node("template_gpio_polling"),
	line_number_(0)
	{	
		// Initialise a publisher
		gpio_event_publisher_ = this->create_publisher<std_msgs::msg::Int32>("gpio_event", 10);

		// Initialise a subscriber
		// > Note that the subscriber is included only for the purpose
		//   of demonstrating this template node running stand-alone
		gpio_event_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("gpio_event", 1, 
			std::bind(&TemplateGpioPolling::templateSubscriberCallback, this, _1));
		
		// Initialise a timer callback with 1000ms rate
		gpio_event_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TemplateGpioPolling::templateGPIOTimerCallback,this));
		
		// Specify the chip name of the GPIO interface
		// > Note: for the 40-pin header of the Jetson SBCs, this
		//   is "/dev/gpiochip1"
		gpio_chip_name_ = "/dev/gpiochip1";

		// Get the GPIO line number to monitor
    // Notes:
    // > If you look at the "template_gpio.launch" file located in
    //   the "launch" folder, you see the following lines of code:
    //       <param
    //           name   = "line_number"
    //           value  = 148
    //       />
    // > These lines of code add a parameter named "line_number"
    //   to the this node.
    // > Thus, to access this "line_number" parameter, we first
    //   get a handle to this node within the namespace that it
    //   was launched.
    this->declare_parameter<int>("line_number"); 
		if(!this->get_parameter("line_number",line_number_)){
			RCLCPP_INFO_STREAM(this->get_logger(),"[TEMPLATE GPIO POLLING] FAILED to get \"line_number\" parameter. Using default value instead.");
			line_number_ = 157;
		}

		
		// > Display the line number being monitored
		RCLCPP_INFO_STREAM(this->get_logger(),"[TEMPLATE GPIO POLLING] Will monitor \"line_number\" = " << line_number_);

		
		// Get and print the value of the GPIO line
		// > Note: the third argument to "gpiod_ctxless_get_value"
		//   is an "active_low" boolean input argument.
		//   If true, this indicate to the function that active state
		//   of this line is low.
		value_ = gpiod_ctxless_get_value(gpio_chip_name_, line_number_, false, "foobar");
		RCLCPP_INFO_STREAM(this->get_logger(), "[TEMPLATE GPIO POLLING] On startup of node, chip " << gpio_chip_name_ << " line " << line_number_ << " returned value = " << value_);

		// Open the GPIO chip
		chip_ = gpiod_chip_open(gpio_chip_name_);
		// Retrieve the GPIO line
		line_ = gpiod_chip_get_line(chip_,line_number_);
		// Display the status
		RCLCPP_INFO_STREAM(this->get_logger(), "[TEMPLATE GPIO POLLING] Chip " << gpio_chip_name_ << " opened and line " << line_number_ << " retrieved.");

	}

	~TemplateGpioPolling()
    {
        // Close the GPIO chip
        if (chip_) {
            gpiod_chip_close(chip_);
        }
    }


private:

	//Variable Declaration for Publisher, Subscriber and Timer.
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gpio_event_publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gpio_event_subscriber_;
	rclcpp::TimerBase::SharedPtr gpio_event_timer_;


	const char * gpio_chip_name_;
	int line_number_;
	int value_;
	struct gpiod_chip *chip_;
	struct gpiod_line *line_;


	void templateSubscriberCallback(const std_msgs::msg::Int32 &msg)
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "[TEMPLATE GPIO POLLING] Message received with data = " << msg.data);
	}

	void templateGPIOTimerCallback()
	{
		int current_value;
		current_value = gpiod_ctxless_get_value(gpio_chip_name_,line_number_,false, "foobar");

		// Respond only if the value is 0 or 1
		if(current_value == 0 || current_value==1)
		{
			// Display the value
			//RCLCPP_INFO_STREAM(this->get_logger(),"[TEMPLATE GPIO POLLING] gpiod_ctxless_get_value returned \"current_value\" = " << current_value);
			// Publish a message
			auto msg = std_msgs::msg::Int32();
			msg.data = current_value;
			gpio_event_publisher_->publish(msg);
		}
		else
		{
			// Display the status
			RCLCPP_INFO_STREAM(this->get_logger(), "[TEMPLATE GPIO POLLING] gpiod_ctxless_get_value returned unexpected value, \"current_value\" = " << current_value);
		}
	}
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemplateGpioPolling>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
