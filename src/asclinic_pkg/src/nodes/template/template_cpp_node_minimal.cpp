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
// Template for C++ code in ROS 2
// Script style version (procedural, no classes): designed to be as simple as possible
//
// ----------------------------------------------------------------------------




// <memory>    provides std::shared_ptr (used by all ROS 2 shared pointers)
// <chrono>    provides std::chrono::milliseconds (used for the timer period)
#include <memory>
#include <chrono>

// rclcpp      is the core ROS 2 C++ library
// std_msgs    provides standard message types; UInt32 lives in std_msgs/msg/u_int32.hpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

// _1 is a placeholder used with std::bind to forward the first argument of a
// callback (i.e., the incoming message) to the handler function.
using std::placeholders::_1;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
//
// SharedPtr is ROS 2's alias for std::shared_ptr — it keeps objects alive as
// long as at least one SharedPtr to them exists.
//
// Publisher type:  rclcpp::Publisher<MessageType>::SharedPtr
// Timer type:      rclcpp::TimerBase::SharedPtr
// Node type:       rclcpp::Node::SharedPtr
rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_template_publisher;
rclcpp::TimerBase::SharedPtr m_timer_for_publishing;
rclcpp::Node::SharedPtr node_ptr;

// ----------------------------------------------------------------------------
// CALLBACK FUNCTIONS

// Timer callback — called automatically at the rate set by create_wall_timer()
void timerCallbackForPublishing()
{
    static uint counter = 0;
    counter++;

    // Create a message object of the desired type and populate its fields.
    std_msgs::msg::UInt32 msg;
    msg.data = counter;

    // Syntax: publisher->publish(msg)
    //   publisher   -- a SharedPtr to a Publisher created with create_publisher()
    //   msg         -- a message object whose type matches the publisher's type
    m_template_publisher->publish(msg);
}

// Subscriber callback — called automatically each time a message arrives.
// The argument is always a SharedPtr to the message type; access fields with ->
void templateSubscriberCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    // RCLCPP_INFO_STREAM logs a message at INFO level (visible in the terminal).
    // Syntax: RCLCPP_INFO_STREAM(logger, stream_expression)
    //   logger            -- obtain with node->get_logger()
    //   stream_expression -- use << to concatenate strings and values, like std::cout
    // Other log levels: RCLCPP_WARN_STREAM, RCLCPP_ERROR_STREAM, RCLCPP_DEBUG_STREAM
    RCLCPP_INFO_STREAM(node_ptr->get_logger(), "[TEMPLATE CPP NODE MINIMAL] Message receieved with data = " << msg->data);
}

// ----------------------------------------------------------------------------
// MAIN

int main(int argc, char* argv[])
{
    // Initialise ROS 2 communication (must be called before anything else)
    rclcpp::init(argc, argv);

    // Create the node and give it a name
    node_ptr = rclcpp::Node::make_shared("template_cpp_node_minimal");

    // Create a publisher.
    // Syntax: node->create_publisher<MessageType>("topic_name", queue_size)
    //   MessageType  -- the message class (must match the subscriber)
    //   "topic_name" -- string name of the topic to publish on
    //   queue_size   -- how many messages to buffer if the subscriber is slow
    m_template_publisher = node_ptr->create_publisher<std_msgs::msg::UInt32>("great_topic", 10);

    // Create a timer.
    // Syntax: node->create_wall_timer(period, callback)
    //   period    -- a std::chrono duration, e.g. std::chrono::milliseconds(1000) = 1 s
    //   callback  -- function called on each tick (no arguments)
    m_timer_for_publishing = node_ptr->create_wall_timer(std::chrono::milliseconds(1000), timerCallbackForPublishing);

    // Create a subscriber.
    // Syntax: node->create_subscription<MessageType>("topic_name", queue_size, callback)
    //   MessageType  -- the message class to receive (must match the publisher)
    //   "topic_name" -- string name of the topic to listen on
    //   queue_size   -- how many incoming messages to buffer
    //   callback     -- function called for each message; signature:
    //                   void callback(const MessageType::SharedPtr msg)
    auto template_subscriber = node_ptr->create_subscription<std_msgs::msg::UInt32>("great_topic", 1, templateSubscriberCallback);

    // Spin the node — blocks here and processes callbacks until Ctrl+C.
    rclcpp::spin(node_ptr);

    // Shutdown
    rclcpp::shutdown();
    return 0;
}
