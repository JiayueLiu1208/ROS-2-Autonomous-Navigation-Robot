#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace    = "asc"
    asclinic_pkg = "asclinic_pkg"
    node_exec    = "i2c_for_servos"

    # --- Parameters with ROS1 defaults ---
    servo_driver_verbosity = LaunchConfiguration(
        'servo_driver_verbosity',
        default='1'
    )

    servo_driver_pwm_frequency_in_hertz = LaunchConfiguration(
        'servo_driver_pwm_frequency_in_hertz',
        default='50.0'
    )

    servo_driver_min_pulse_width_in_microseconds = LaunchConfiguration(
        'servo_driver_min_pulse_width_in_microseconds',
        default='500'
    )

    servo_driver_max_pulse_width_in_microseconds = LaunchConfiguration(
        'servo_driver_max_pulse_width_in_microseconds',
        default='2500'
    )

    # --- Declare launch arguments ---
    declare_args = [

        DeclareLaunchArgument(
            'servo_driver_verbosity',
            default_value=servo_driver_verbosity,
            description='Verbosity level for servo driver'
        ),

        DeclareLaunchArgument(
            'servo_driver_pwm_frequency_in_hertz',
            default_value=servo_driver_pwm_frequency_in_hertz,
            description='PWM frequency for servo controller'
        ),

        DeclareLaunchArgument(
            'servo_driver_min_pulse_width_in_microseconds',
            default_value=servo_driver_min_pulse_width_in_microseconds,
            description='Minimum pulse width'
        ),

        DeclareLaunchArgument(
            'servo_driver_max_pulse_width_in_microseconds',
            default_value=servo_driver_max_pulse_width_in_microseconds,
            description='Maximum pulse width'
        ),
    ]

    # --- Node definition ---
    servo_node = Node(
        package    = asclinic_pkg,
        executable = node_exec,
        namespace  = namespace,
        name       = "i2c_for_servos",
        output     = "screen",
        parameters = [{
            'servo_driver_verbosity':                            servo_driver_verbosity,
            'servo_driver_pwm_frequency_in_hertz':               servo_driver_pwm_frequency_in_hertz,
            'servo_driver_min_pulse_width_in_microseconds':      servo_driver_min_pulse_width_in_microseconds,
            'servo_driver_max_pulse_width_in_microseconds':      servo_driver_max_pulse_width_in_microseconds,
        }]
    )

    # Build launch description
    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(servo_node)

    return ld
