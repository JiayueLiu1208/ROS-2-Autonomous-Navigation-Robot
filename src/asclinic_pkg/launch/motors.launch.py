#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    launch_roboclaw = LaunchConfiguration('launch_roboclaw')
    launch_servos = LaunchConfiguration('launch_servos')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('launch_roboclaw', default_value='true'),
        DeclareLaunchArgument('launch_servos', default_value='false'),
        DeclareLaunchArgument('roboclaw_usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('roboclaw_baud_rate', default_value='38400'),
        DeclareLaunchArgument('roboclaw_address', default_value='128'),
        DeclareLaunchArgument('motor_driver_verbosity', default_value='1'),
        DeclareLaunchArgument('motor_driver_max_duty_cycle_limit_in_percent', default_value='100.0'),
        DeclareLaunchArgument('motor_driver_left_side_multiplier', default_value='1.0'),
        DeclareLaunchArgument('motor_driver_right_side_multiplier', default_value='-1.0'),
        DeclareLaunchArgument('encoder_left_side_multiplier', default_value='1.0'),
        DeclareLaunchArgument('encoder_right_side_multiplier', default_value='-1.0'),
        DeclareLaunchArgument('delta_t_for_publishing_encoder_counts', default_value='0.1'),
        DeclareLaunchArgument('servo_driver_verbosity', default_value='1'),

        Node(
            package='asclinic_pkg',
            executable='roboclaw_for_motors.py',
            namespace=namespace,
            name='roboclaw_for_motors',
            output='screen',
            condition=IfCondition(launch_roboclaw),
            parameters=[{
                'motor_driver_verbosity': LaunchConfiguration('motor_driver_verbosity'),
                'roboclaw_usb_port': LaunchConfiguration('roboclaw_usb_port'),
                'roboclaw_baud_rate': LaunchConfiguration('roboclaw_baud_rate'),
                'roboclaw_address': LaunchConfiguration('roboclaw_address'),
                'motor_driver_max_duty_cycle_limit_in_percent': LaunchConfiguration(
                    'motor_driver_max_duty_cycle_limit_in_percent'
                ),
                'motor_driver_left_side_multiplier': LaunchConfiguration(
                    'motor_driver_left_side_multiplier'
                ),
                'motor_driver_right_side_multiplier': LaunchConfiguration(
                    'motor_driver_right_side_multiplier'
                ),
                'encoder_left_side_multiplier': LaunchConfiguration(
                    'encoder_left_side_multiplier'
                ),
                'encoder_right_side_multiplier': LaunchConfiguration(
                    'encoder_right_side_multiplier'
                ),
                'delta_t_for_publishing_encoder_counts': LaunchConfiguration(
                    'delta_t_for_publishing_encoder_counts'
                ),
            }],
        ),

        Node(
            package='asclinic_pkg',
            executable='i2c_for_servos',
            namespace=namespace,
            name='i2c_for_servos',
            output='screen',
            condition=IfCondition(launch_servos),
            parameters=[{
                'servo_driver_verbosity': LaunchConfiguration('servo_driver_verbosity'),
            }],
        ),
    ])
