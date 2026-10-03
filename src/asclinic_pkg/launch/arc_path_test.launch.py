#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


FINAL_DEMO_MARKER_WORLD_MAP = (
    '1:0.00,1.00,0.30,0;'
    '2:0.00,3.60,0.30,0;'
    '3:0.00,6.60,0.30,0;'
    '4:0.00,9.60,0.30,0;'
    '5:15.00,1.00,0.30,180;'
    '6:15.00,3.60,0.30,180;'
    '7:15.00,6.60,0.30,180;'
    '8:15.00,9.60,0.30,180;'
    '9:5.00,0.00,0.30,90;'
    '10:8.00,0.00,0.30,90;'
    '11:11.00,0.00,0.30,90;'
    '12:14.00,0.00,0.30,90;'
    '13:5.00,10.80,0.30,-90;'
    '14:8.00,10.80,0.30,-90;'
    '15:11.00,10.80,0.30,-90;'
    '16:14.00,10.80,0.30,-90;'
    '18:8.20,5.20,0.30,0;'
    '19:7.80,5.20,0.30,180;'
    '20:8.00,5.40,0.30,90;'
    '22:8.00,5.00,0.30,-90;'
    '27:1.50,3.60,0.30,90;'
    '29:8.00,1.30,0.30,180'
)


def _launch_file(name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('asclinic_pkg'), 'launch', name])
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('roboclaw_usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('launch_cv_localization', default_value='false'),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_fps', default_value='5'),
        DeclareLaunchArgument('publish_camera_images', default_value='true'),
        DeclareLaunchArgument('show_camera_images', default_value='false'),
        DeclareLaunchArgument('marker_world_map', default_value=FINAL_DEMO_MARKER_WORLD_MAP),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('center_x', default_value='0.0'),
        DeclareLaunchArgument('center_y', default_value='2.0'),
        DeclareLaunchArgument('radius', default_value='2.0'),
        DeclareLaunchArgument('start_angle', default_value='-1.57079632679'),
        DeclareLaunchArgument('end_angle', default_value='1.57079632679'),
        DeclareLaunchArgument('clockwise', default_value='false'),
        DeclareLaunchArgument('point_spacing', default_value='0.10'),
        DeclareLaunchArgument('odom_topic', default_value='wheel_odometry'),
        DeclareLaunchArgument('launch_tracking_controller', default_value='true'),
        DeclareLaunchArgument('launch_lqg_controller', default_value='false'),
        DeclareLaunchArgument('nominal_speed', default_value='0.10'),
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
            default_value='~/asclinic-ros2/ros2_ws/results/arc_path_test',
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
            _launch_file('perception.launch.py'),
            condition=IfCondition(LaunchConfiguration('launch_cv_localization')),
            launch_arguments={
                'namespace': namespace,
                'launch_camera_capture': 'false',
                'launch_aruco_detector': 'true',
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_fps': LaunchConfiguration('camera_fps'),
                'publish_camera_images': LaunchConfiguration('publish_camera_images'),
                'show_camera_images': LaunchConfiguration('show_camera_images'),
            }.items(),
        ),

        IncludeLaunchDescription(
            _launch_file('localisation.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'launch_encoder_odometry': 'true',
                'launch_fused_estimator': LaunchConfiguration('launch_cv_localization'),
                'launch_mock_odometry': 'false',
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'marker_world_map': LaunchConfiguration('marker_world_map'),
            }.items(),
        ),

        Node(
            package='asclinic_pkg',
            executable='arc_path_publisher.py',
            namespace=namespace,
            name='arc_path_publisher',
            output='screen',
            parameters=[{
                'center_x': LaunchConfiguration('center_x'),
                'center_y': LaunchConfiguration('center_y'),
                'radius': LaunchConfiguration('radius'),
                'start_angle': LaunchConfiguration('start_angle'),
                'end_angle': LaunchConfiguration('end_angle'),
                'clockwise': LaunchConfiguration('clockwise'),
                'point_spacing': LaunchConfiguration('point_spacing'),
                'path_topic': 'reference_path',
            }],
        ),

        IncludeLaunchDescription(
            _launch_file('control.launch.py'),
            condition=IfCondition(LaunchConfiguration('launch_tracking_controller')),
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
            executable='lqg_controller_test.py',
            namespace=namespace,
            name='lqg_controller_test',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_lqg_controller')),
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'path_topic': 'reference_path',
                'motor_cmd_topic': 'set_motor_duty_cycle',
                'v_ref': LaunchConfiguration('nominal_speed'),
                'v_max_mps': LaunchConfiguration('max_wheel_speed_at_full_duty'),
                'duty_cycle_limit': LaunchConfiguration('duty_cycle_limit'),
                'min_moving_duty': LaunchConfiguration('min_moving_duty'),
                'left_trim': LaunchConfiguration('left_trim'),
                'right_trim': LaunchConfiguration('right_trim'),
                'duty_slew_rate_percent_per_sec': LaunchConfiguration(
                    'duty_slew_rate_percent_per_sec'
                ),
                'command_filter_alpha': LaunchConfiguration('command_filter_alpha'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'control_period': '0.05',
                'verbose': LaunchConfiguration('control_verbose'),
            }],
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
