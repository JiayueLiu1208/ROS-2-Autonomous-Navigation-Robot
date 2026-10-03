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
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// Template node for mointoring edge events on a GPIO pin
//
// ----------------------------------------------------------------------------





#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <gpiod.h>

// Global node pointer for accessing node functions from global scope
rclcpp::Node::SharedPtr node_ptr;

// Global publisher and subscriber variables
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gpio_event_publisher;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gpio_event_subscriber;

// Respond to subscriber receiving a message
void templateSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
	RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] Message received with data = " << msg->data);
}

int main(int argc, char* argv[])
{
	// Initialise the node
	rclcpp::init(argc, argv);
	node_ptr = rclcpp::Node::make_shared("template_gpio_event_triggered");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = node_ptr->get_namespace();

	// Initialise a publisher
	gpio_event_publisher = node_ptr->create_publisher<std_msgs::msg::Int32>("gpio_event", 10);
	// Initialise a subscriber
	// > Note that the subscriber is included only for the purpose
	//   of demonstrating this template node running stand-alone
	gpio_event_subscriber = node_ptr->create_subscription<std_msgs::msg::Int32>("gpio_event", 1, templateSubscriberCallback);

	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this
	//   is "/dev/gpiochip1"
	const char * gpio_chip_name = "/dev/gpiochip1";

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
	//std::string namespace = node_ptr->get_namespace();
	int line_number = 0;
	node_ptr->declare_parameter("line_number", 148);
	line_number = node_ptr->get_parameter("line_number").as_int();
	
	// > Display the line number being monitored
	RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] Will monitor \"line_number\" = " << line_number);

	// Initialise a GPIO chip, line, and event object
	struct gpiod_chip *chip;
	struct gpiod_line *line;
	struct gpiod_line_event event;

	// Specify the timeout specifications
	// > The first entry is seconds
	// > The second entry is nano-seconds
	struct timespec ts = { 0, 100000000 };
	
	// Intialise a variable for the flags returned
	// by GPIO calls
	int returned_wait_flag;
	int returned_read_flag;

	// Initialise variables used for computing the time
	// between events on the GPIO line
	time_t prev_tv_sec    = -1;
	long int prev_tv_nsec = -1;
	bool prev_time_isValid = false;

	// Get and print the value of the GPIO line
	// > Note: the third argument to "gpiod_ctxless_get_value"
	//   is an "active_low" boolean input argument.
	//   If true, this indicate to the function that active state
	//   of this line is low.
	int value;
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number, false, "foobar");
	RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] On startup of node, chip " << gpio_chip_name << " line " << line_number << " returned value = " << value);

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO line
	line = gpiod_chip_get_line(chip,line_number);
	// Display the status
	RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] Chip " << gpio_chip_name << " opened and line " << line_number << " retrieved");

	// Request the line events to be mointored
	// > Note: only one of these should be uncommented
	//gpiod_line_request_rising_edge_events(line, "foobar");
	//gpiod_line_request_falling_edge_events(line, "foobar");
	gpiod_line_request_both_edges_events(line, "foobar");

	// Display the line event values for rising and falling
	RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] The constants defined for distinguishing line events are:, GPIOD_LINE_EVENT_RISING_EDGE = " << GPIOD_LINE_EVENT_RISING_EDGE << ", and GPIOD_LINE_EVENT_FALLING_EDGE = " << GPIOD_LINE_EVENT_FALLING_EDGE);

	// Enter a loop that continues while ROS is still running
	while (rclcpp::ok())
	{

		// Monitor for the requested events on the GPIO line
		// > Note: the function "gpiod_line_event_wait" returns:
		//    0  if wait timed out
		//   -1  if an error occurred
		//    1  if an event occurred.
		returned_wait_flag = gpiod_line_event_wait(line,&ts);

		// Respond based on the the return flag
		switch (returned_wait_flag)
		{
			// Event occurred:
			case 1:
			{
				// Read the pending event on the GPIO line
				// > Note: the function "gpiod_line_event_read" returns:
				//    0  if the event was read correctly
				//   -1  if an error occurred
				returned_read_flag = gpiod_line_event_read(line,&event);

				switch (returned_read_flag)
				{
					// Event read correctly
					case 0:
					{
						if (prev_time_isValid)
						{
							// Compute the time difference in seconds
							float this_diff_sec = float(event.ts.tv_nsec - prev_tv_nsec) / 1000000000;
							if (event.ts.tv_sec > prev_tv_sec)
							{
								this_diff_sec = this_diff_sec + (event.ts.tv_sec - prev_tv_sec);
							}

							// Display the event
							//RCLCPP_INFO_STREAM(node_ptr->get_logger(), "event type = " << event.event_type << ", time delta (sec) = " << this_diff_sec);

							// Publish a message
							std_msgs::msg::Int32 msg;
							msg.data = event.event_type;
							gpio_event_publisher->publish(msg);

							// Spin once so that this node can service the any
							// callbacks that this node has queued.
							// > This is required so that message is actually
							//   published.
							rclcpp::spin_some(node_ptr);
						}

						// Update the previous time
						prev_tv_sec  = event.ts.tv_sec;
						prev_tv_nsec = event.ts.tv_nsec;
						prev_time_isValid = true;
						break;
					}

					// Error occurred
					case -1:
					{
						// Display the status
						RCLCPP_INFO(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] gpiod_line_event_wait returned the status that an error occurred");
						break;
					}

					default:
					{
						// Display the status
						RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] gpiod_line_event_read returned an unrecognised status, return_flag =  " << returned_read_flag );
						break;
					}
				} // END OF: "switch (returned_read_flag)"
				break;
			}

			// Time out occurred
			case 0:
			{
				// Spin once so that this node can service the publishing
				// of this message, and any other callbacks that this
				// node has queued
				rclcpp::spin_some(node_ptr);
				break;
			}

			// Error occurred
			case -1:
			{
				// Display the status
				RCLCPP_INFO(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] gpiod_line_event_wait returned the status that an error occurred");
				break;
			}

			default:
			{
				// Display the status
				RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE GPIO EVENT TRIG.] gpiod_line_event_wait returned an unrecognised status, return_flag =  " << returned_wait_flag );
				break;
			}
		} // END OF: "switch (returned_wait_flag)"
	} // END OF: "while (rclcpp::ok())"

	// Close the GPIO chip
	gpiod_chip_close(chip);

	// Shutdown ROS2
	rclcpp::shutdown();

	return 0;
}
