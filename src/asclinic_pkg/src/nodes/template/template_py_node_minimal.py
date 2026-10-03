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
# Script style version (procedural, no classes): designed to be as simple as possible
#
# ----------------------------------------------------------------------------


# Import the ROS2 Python package
import rclpy

# Import the standard message type
from std_msgs.msg import UInt32

# ----------------------------------------------------------------------------
# CALLBACK FUNCTIONS

def timerCallbackForPublishing():
    global counter
    counter += 1
    msg = UInt32()
    msg.data = counter
    # Syntax: publisher.publish(msg)
    #   publisher  -- the publisher object created with create_publisher()
    #   msg        -- a message object of the type specified when creating the publisher
    publisher.publish(msg)

def templateSubscriberCallback(msg):
    # Syntax: node.get_logger().info(string)
    #   node.get_logger()  -- returns the logger attached to this node
    #   .info(string)      -- logs at INFO level; other levels: .warn(), .error(), .debug()
    #   msg.data           -- accesses the 'data' field of the received message
    node.get_logger().info(f"[TEMPLATE PY NODE MINIMAL] Message received with data = {msg.data}")

# ----------------------------------------------------------------------------
# MAIN SCRIPT

# Initialize ROS2
rclpy.init()

# Create the node
node = rclpy.create_node('template_py_node_minimal')

# Counter variable
counter = 0

# Create a publisher
# Syntax: node.create_publisher(msg_type, topic_name, queue_size)
#   msg_type    -- the message class to publish (must match the subscriber)
#   topic_name  -- string name of the topic to publish on
#   queue_size  -- how many messages to buffer if the subscriber is slow
publisher = node.create_publisher(UInt32, 'template_topic', 10)

# Create a timer
# Syntax: node.create_timer(period_seconds, callback)
#   period_seconds  -- how often (in seconds) to call the callback
#   callback        -- function to call on each tick (no arguments)
timer = node.create_timer(1.0, timerCallbackForPublishing)

# Create a subscriber
# Syntax: node.create_subscription(msg_type, topic_name, callback, queue_size)
#   msg_type    -- the message class to receive (must match the publisher)
#   topic_name  -- string name of the topic to listen on
#   callback    -- function called each time a message arrives; takes msg as argument
#   queue_size  -- how many incoming messages to buffer
subscriber = node.create_subscription(UInt32, 'template_topic', templateSubscriberCallback, 10)

# Spin the node (keeps it running and processing callbacks)
rclpy.spin(node)

# Cleanup
node.destroy_node()
rclpy.shutdown()
