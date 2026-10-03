#!/usr/bin/env python3

# Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#
# See the root of the repository for license details.
#
# Launch file for the roboclaw_for_motors Python node.
# This node handles both motor control and encoder count publishing.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace    = "asc"
    asclinic_pkg = "asclinic_pkg"
    node_exec    = "roboclaw_for_motors.py"

    # --- Parameters (LaunchConfigurations with defaults) ---
    motor_driver_verbosity = LaunchConfiguration(
        'motor_driver_verbosity',
        default='1'
    )

    roboclaw_usb_port = LaunchConfiguration(
        'roboclaw_usb_port',
        default='/dev/ttyACM0'
    )

    roboclaw_baud_rate = LaunchConfiguration(
        'roboclaw_baud_rate',
        default='38400'
    )

    # Default RoboClaw packet-serial address: 0x80 = 128
    roboclaw_address = LaunchConfiguration(
        'roboclaw_address',
        default='128'
    )

    motor_driver_max_duty_cycle_limit_in_percent = LaunchConfiguration(
        'motor_driver_max_duty_cycle_limit_in_percent',
        default='100.0'
    )

    motor_driver_left_side_multiplier = LaunchConfiguration(
        'motor_driver_left_side_multiplier',
        default='1.0'
    )

    motor_driver_right_side_multiplier = LaunchConfiguration(
        'motor_driver_right_side_multiplier',
        default='-1.0'
    )

    encoder_left_side_multiplier = LaunchConfiguration(
        'encoder_left_side_multiplier',
        default='1.0'
    )

    encoder_right_side_multiplier = LaunchConfiguration(
        'encoder_right_side_multiplier',
        default='-1.0'
    )

    delta_t_for_publishing_encoder_counts = LaunchConfiguration(
        'delta_t_for_publishing_encoder_counts',
        default='0.1'
    )

    # --- Declare launch arguments ---
    declare_args = [
        DeclareLaunchArgument(
            'motor_driver_verbosity',
            default_value=motor_driver_verbosity,
            description='Verbosity for motor driver (0=quiet, 1=startup, 2=per-message)'
        ),
        DeclareLaunchArgument(
            'roboclaw_usb_port',
            default_value=roboclaw_usb_port,
            description='USB serial port for the RoboClaw (e.g., /dev/ttyACM0)'
        ),
        DeclareLaunchArgument(
            'roboclaw_baud_rate',
            default_value=roboclaw_baud_rate,
            description='Baud rate for the RoboClaw serial connection'
        ),
        DeclareLaunchArgument(
            'roboclaw_address',
            default_value=roboclaw_address,
            description='RoboClaw packet-serial address (default 128 = 0x80)'
        ),
        DeclareLaunchArgument(
            'motor_driver_max_duty_cycle_limit_in_percent',
            default_value=motor_driver_max_duty_cycle_limit_in_percent,
            description='Maximum allowable motor duty cycle [0, 100] %'
        ),
        DeclareLaunchArgument(
            'motor_driver_left_side_multiplier',
            default_value=motor_driver_left_side_multiplier,
            description='Left wheel direction multiplier (+1.0 or -1.0)'
        ),
        DeclareLaunchArgument(
            'motor_driver_right_side_multiplier',
            default_value=motor_driver_right_side_multiplier,
            description='Right wheel direction multiplier (+1.0 or -1.0)'
        ),
        DeclareLaunchArgument(
            'encoder_left_side_multiplier',
            default_value=encoder_left_side_multiplier,
            description='Left encoder direction multiplier (+1.0 or -1.0)'
        ),
        DeclareLaunchArgument(
            'encoder_right_side_multiplier',
            default_value=encoder_right_side_multiplier,
            description='Right encoder direction multiplier (+1.0 or -1.0)'
        ),
        DeclareLaunchArgument(
            'delta_t_for_publishing_encoder_counts',
            default_value=delta_t_for_publishing_encoder_counts,
            description='Publishing period for encoder counts in seconds (default 0.1 = 10 Hz)'
        ),
    ]

    # --- Node definition ---
    motor_node = Node(
        package    = asclinic_pkg,
        executable = node_exec,
        namespace  = namespace,
        name       = "roboclaw_for_motors",
        output     = "screen",
        parameters = [{
            'motor_driver_verbosity':                    motor_driver_verbosity,
            'roboclaw_usb_port':                         roboclaw_usb_port,
            'roboclaw_baud_rate':                        roboclaw_baud_rate,
            'roboclaw_address':                          roboclaw_address,
            'motor_driver_max_duty_cycle_limit_in_percent': motor_driver_max_duty_cycle_limit_in_percent,
            'motor_driver_left_side_multiplier':         motor_driver_left_side_multiplier,
            'motor_driver_right_side_multiplier':        motor_driver_right_side_multiplier,
            'encoder_left_side_multiplier':              encoder_left_side_multiplier,
            'encoder_right_side_multiplier':             encoder_right_side_multiplier,
            'delta_t_for_publishing_encoder_counts':     delta_t_for_publishing_encoder_counts,
        }],
    )

    # Build launch description
    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(motor_node)

    return ld
