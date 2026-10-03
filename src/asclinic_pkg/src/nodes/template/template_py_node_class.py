#!/usr/bin/env python3

# Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Template Python node with a publisher and subscriber - ROS2 version
# OOP version (with classes): more professional, makes the code more organised and reusable
#
# ----------------------------------------------------------------------------

# Import the ROS2 Python package
import rclpy
from rclpy.node import Node

# Import the standard and asclinic message types
from std_msgs.msg import UInt32
from asclinic_pkg.msg import TemplateMessage


class TemplatePyNodeMinimal(Node):

    def __init__(self):
        # Initialize the node with a name
        super().__init__('template_py_node')
        
        # INITIALISE THE PUBLISHER
        #   self.create_publisher(message_type, topic_name, qos_profile)
        # where:
        #   message_type: The message type (e.g., TemplateMessage)
        #   topic_name: String that defines the topic name
        #   qos_profile: Quality of Service profile ("queue_size")
        #
        self.template_publisher = self.create_publisher(
            TemplateMessage, 
            'template_topic', 
            10  
        )
        
        # INITIALISE THE TIMER FOR RECURRENT PUBLISHING
        #   self.create_timer(timer_period_sec, callback)
        # where:
        #   timer_period_sec: Period in seconds between timer callbacks
        #   callback: The callback function to be called
        #
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

        # INITIALISE THE SUBSCRIBER
        #   self.create_subscription(message_type, topic_name, callback, qos_profile)
        # where:
        #   message_type: The message type to subscribe to
        #   topic_name: String that defines the topic name to subscribe to
        #   callback: The callback function to be called when messages are received
        #   qos_profile: Quality of Service profile ("queue size")
        #
        self.subscription = self.create_subscription(
            TemplateMessage,
            'template_topic',
            self.template_subscriber_callback,
            10  
        )

    # Respond to timer callback
    def timer_callback(self):
        # Increment the counter
        self.counter += 1
        
        # PUBLISH A MESSAGE
        # Create a TemplateMessage instance
        template_message = TemplateMessage()
        
        # Set the values
        template_message.temp_bool = True
        template_message.temp_uint32 = self.counter
        template_message.temp_int32 = -1
        template_message.temp_float32 = 1.23
        template_message.temp_float64 = -1.23
        template_message.temp_string = "test"
        
        # Append elements into the array
        template_message.temp_float64_array.append(1.1)
        template_message.temp_float64_array.append(2.2)
        template_message.temp_float64_array.append(3.3)
        
        # Publish the message
        self.template_publisher.publish(template_message)
        
        # Log the publishing (using ROS2 logging)
        self.get_logger().info(f'Published message with counter: {self.counter}')

    # Respond to subscriber receiving a message
    def template_subscriber_callback(self, msg):
        # Display that a message was received
        self.get_logger().info(f'[TEMPLATE PY NODE] Message received with data = {msg.temp_uint32}')


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
   
    # Create the node
    template_py_node = TemplatePyNodeMinimal()
   
    # Spin as a single-threaded node
    rclpy.spin(template_py_node)
   
    # Cleanup
    template_py_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
