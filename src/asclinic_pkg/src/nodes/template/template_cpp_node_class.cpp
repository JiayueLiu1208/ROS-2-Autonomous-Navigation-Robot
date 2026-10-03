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
// OOP version (with classes): more professional, makes the code more organised and reusable
// ----------------------------------------------------------------------------





// <memory>    provides std::shared_ptr (used by all ROS 2 shared pointers)
// <chrono>    provides std::chrono::milliseconds (used for the timer period)
#include <memory>
#include <chrono>

// rclcpp      is the core ROS 2 C++ library
// std_msgs    provides standard message types; UInt32 lives in std_msgs/msg/u_int32.hpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

// _1 is a placeholder used with std::bind to forward the incoming message
// argument to a subscriber callback method inside the class.
using std::placeholders::_1;


// ----------------------------------------------------------------------------
// NODE CLASS
//
// Inheriting from rclcpp::Node gives this class all the ROS 2 node
// functionality (publishers, subscribers, timers, logging, etc.).
// The string passed to the Node constructor sets the node's name as it
// appears in "ros2 node list".

// In ROS2, member variables are typically declared as private members
// within the class. They are available to all methods within the class.
// The naming convention used is that each variable name begins with "m_".
// Some notes about member variables:
// > They are available to all methods within this class.
// > The value stored persists across calls to methods.
// > ROS2 nodes are single-threaded by default (using single-threaded executor),
//   so you do NOT need to worry about simultaneous access to variables
//   unless you explicitly use multi-threaded executors.
class TemplateCppNodeMinimal : public rclcpp::Node
{
public:
    TemplateCppNodeMinimal() : Node("template_cpp_node_minimal")
    {
        // CREATE A PUBLISHER
        // Syntax: this->create_publisher<MessageType>("topic_name", queue_size)
        //   MessageType  -- the message class to publish
        //   "topic_name" -- string name of the topic to publish on
        //   queue_size   -- how many messages to buffer if subscribers are slow
        //
        // The result is stored as a member variable (m_template_publisher) so
        // that it remains alive for the lifetime of the node and can be used
        // inside the timer callback.
        m_template_publisher = this->create_publisher<std_msgs::msg::UInt32>("great_topic", 10);

        // CREATE A TIMER
        // Syntax: this->create_wall_timer(period, callback)
        //   period    -- a std::chrono duration (e.g. milliseconds(1000) = 1 s)
        //   callback  -- use std::bind to attach a class method as the callback.
        //                std::bind(&ClassName::methodName, this) wraps the method
        //                together with "this" (the current object) so ROS 2 can
        //                call it without needing an object reference later.
        m_timer_for_publishing = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TemplateCppNodeMinimal::timerCallbackForPublishing, this)
        );

        // CREATE A SUBSCRIBER
        // Syntax: this->create_subscription<MessageType>("topic_name", queue_size, callback)
        //   MessageType  -- the message class to receive (must match the publisher)
        //   "topic_name" -- string name of the topic to listen on
        //   queue_size   -- how many incoming messages to buffer
        //   callback     -- std::bind with _1 forwards the incoming message to the
        //                   method. _1 is a placeholder for the first argument
        //                   (the message pointer) that ROS 2 will supply at runtime.
        template_subscriber = this->create_subscription<std_msgs::msg::UInt32>(
            "great_topic", 1,
            std::bind(&TemplateCppNodeMinimal::templateSubscriberCallback, this, _1)
        );
    }

private:
    // Member variables hold the publisher, timer, and subscriber for the
    // lifetime of the node. SharedPtr (a std::shared_ptr) ensures that ROS 2
    // keeps the underlying objects alive as long as this class exists.
    //
    // Publisher type:      rclcpp::Publisher<MessageType>::SharedPtr
    // Timer type:          rclcpp::TimerBase::SharedPtr
    // Subscription type:   rclcpp::Subscription<MessageType>::SharedPtr
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_template_publisher;
    rclcpp::TimerBase::SharedPtr m_timer_for_publishing;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr template_subscriber;

    // Timer callback — called automatically at the rate set by create_wall_timer()
    void timerCallbackForPublishing()
    {
        static uint counter = 0;
        counter++;

        // Create a message object of the desired type and populate its fields.
        // For std_msgs::msg::UInt32, the only field is "data".
        std_msgs::msg::UInt32 msg;
        msg.data = counter;

        // Syntax: publisher->publish(msg)
        //   publisher  -- the SharedPtr member variable created in the constructor
        //   msg        -- a message object whose type matches the publisher's type
        m_template_publisher->publish(msg);
    }

    // Subscriber callback — called automatically each time a message arrives.
    // The argument is always a SharedPtr to the message type; access fields with ->
    void templateSubscriberCallback(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        // RCLCPP_INFO_STREAM logs a message at INFO level (visible in the terminal).
        // Syntax: RCLCPP_INFO_STREAM(logger, stream_expression)
        //   logger            -- obtain with this->get_logger() inside a class
        //   stream_expression -- use << to concatenate strings and values, like std::cout
        //
        // Other log levels: RCLCPP_WARN_STREAM, RCLCPP_ERROR_STREAM, RCLCPP_DEBUG_STREAM
        RCLCPP_INFO_STREAM(this->get_logger(), "[TEMPLATE CPP NODE MINIMAL] Message received with data = " << msg->data);
    }
};


// ----------------------------------------------------------------------------
// MAIN

int main(int argc, char* argv[])
{
    // Initialise the ROS 2 runtime (must be called before anything else).
    /**
     * The rclcpp::init() function needs to see argc and argv so that it can perform
     * any ROS2 arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use rclcpp::NodeOptions, but for most 
     * command-line programs, passing argc and argv is the easiest way to do it.
     */
    rclcpp::init(argc, argv);

    // Instantiate the node.
    // std::make_shared<T>() creates the object and returns a shared_ptr to it.
    // Using a shared_ptr ensures the node is properly cleaned up when it goes
    // out of scope, and allows it to be safely passed to rclcpp::spin().
    auto node = std::make_shared<TemplateCppNodeMinimal>();

    // Spin the node — blocks here, processing all callbacks until Ctrl+C.
    rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();
    return 0;
}
