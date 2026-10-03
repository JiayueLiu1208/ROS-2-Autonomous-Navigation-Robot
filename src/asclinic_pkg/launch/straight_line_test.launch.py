#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_file(name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('asclinic_pkg'), 'launch', name])
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('roboclaw_usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='3.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='wheel_odometry'),
        DeclareLaunchArgument('nominal_speed', default_value='0.12'),
        DeclareLaunchArgument('duty_cycle_limit', default_value='28.0'),
        DeclareLaunchArgument('min_moving_duty', default_value='18.0'),
        DeclareLaunchArgument('max_wheel_speed_at_full_duty', default_value='1.50'),
        DeclareLaunchArgument('duty_slew_rate_percent_per_sec', default_value='55.0'),
        DeclareLaunchArgument('command_filter_alpha', default_value='0.0'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.45'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.18'),
        DeclareLaunchArgument('slowdown_distance', default_value='0.70'),
        DeclareLaunchArgument('curvature_gain', default_value='1.0'),
        DeclareLaunchArgument('backward_target_turn_direction', default_value='1.0'),
        DeclareLaunchArgument('left_trim', default_value='1.0'),
        DeclareLaunchArgument('right_trim', default_value='0.95'),
        DeclareLaunchArgument('control_verbose', default_value='true'),
        DeclareLaunchArgument('launch_diagnostics', default_value='true'),
        DeclareLaunchArgument(
            'save_directory',
            default_value='~/asclinic-ros2/ros2_ws/results/straight_line_test',
        ),
        DeclareLaunchArgument('show_live_plot', default_value='true'),

        IncludeLaunchDescription(
            _launch_file('motors.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'roboclaw_usb_port': LaunchConfiguration('roboclaw_usb_port'),
                'launch_roboclaw': 'true',
                'launch_servos': 'false',
                'motor_driver_left_side_multiplier': '1.0',
                'motor_driver_right_side_multiplier': '-1.0',
                'encoder_left_side_multiplier': '1.0',
                'encoder_right_side_multiplier': '-1.0',
            }.items(),
        ),

        IncludeLaunchDescription(
            _launch_file('localisation.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'launch_encoder_odometry': 'true',
                'launch_fused_estimator': 'false',
                'launch_mock_odometry': 'false',
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
            }.items(),
        ),

        Node(
            package='asclinic_pkg',
            executable='straight_path_publisher.py',
            namespace=namespace,
            name='straight_path_publisher',
            output='screen',
            parameters=[{
                'start_x': LaunchConfiguration('initial_x'),
                'start_y': LaunchConfiguration('initial_y'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'path_topic': 'reference_path',
            }],
        ),

        IncludeLaunchDescription(
            _launch_file('control.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'odom_topic': LaunchConfiguration('odom_topic'),
                'reference_path_topic': 'reference_path',
                'nominal_speed': LaunchConfiguration('nominal_speed'),
                'duty_cycle_limit': LaunchConfiguration('duty_cycle_limit'),
                'min_moving_duty': LaunchConfiguration('min_moving_duty'),
                'max_wheel_speed_at_full_duty': LaunchConfiguration(
                    'max_wheel_speed_at_full_duty'
                ),
                'duty_slew_rate_percent_per_sec': LaunchConfiguration(
                    'duty_slew_rate_percent_per_sec'
                ),
                'command_filter_alpha': LaunchConfiguration('command_filter_alpha'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'slowdown_distance': LaunchConfiguration('slowdown_distance'),
                'curvature_gain': LaunchConfiguration('curvature_gain'),
                'backward_target_turn_direction': LaunchConfiguration(
                    'backward_target_turn_direction'
                ),
                'left_trim': LaunchConfiguration('left_trim'),
                'right_trim': LaunchConfiguration('right_trim'),
                'verbose': LaunchConfiguration('control_verbose'),
            }.items(),
        ),

        Node(
            package='asclinic_pkg',
            executable='path_tracking_diagnostics.py',
            namespace=namespace,
            name='path_tracking_diagnostics',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_diagnostics')),
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'path_topic': 'reference_path',
                'motor_cmd_topic': 'set_motor_duty_cycle',
                'save_directory': LaunchConfiguration('save_directory'),
                'show_live': LaunchConfiguration('show_live_plot'),
            }],
        ),
    ])
