#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_PLANT_MODEL_PATH = PathJoinSubstitution(
    [FindPackageShare('asclinic_pkg'), 'models', 'plant_detector', 'best.pt']
)


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
    launch_planning = LaunchConfiguration('launch_planning')
    launch_lqg = LaunchConfiguration('launch_lqg')
    launch_plant_detector = LaunchConfiguration('launch_plant_detector')
    launch_map_viewer = LaunchConfiguration('launch_map_viewer')
    launch_system_status = LaunchConfiguration('launch_system_status')
    model_path = LaunchConfiguration('model_path')
    plant_detector_start_enabled = LaunchConfiguration('plant_detector_start_enabled')
    marker_world_map = LaunchConfiguration('marker_world_map')
    odom_topic = LaunchConfiguration('odom_topic')
    reference_path_topic = LaunchConfiguration('reference_path_topic')
    motor_cmd_topic = LaunchConfiguration('motor_cmd_topic')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('launch_planning', default_value='true'),
        DeclareLaunchArgument('launch_lqg', default_value='true'),
        DeclareLaunchArgument('launch_plant_detector', default_value='false'),
        DeclareLaunchArgument('launch_map_viewer', default_value='false'),
        DeclareLaunchArgument('launch_system_status', default_value='true'),
        DeclareLaunchArgument('plant_detector_start_enabled', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_PLANT_MODEL_PATH),

        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_fps', default_value='10'),
        DeclareLaunchArgument('publish_camera_images', default_value='true'),
        DeclareLaunchArgument('roboclaw_usb_port', default_value='/dev/ttyACM0'),

        DeclareLaunchArgument('initial_x', default_value='0.50'),
        DeclareLaunchArgument('initial_y', default_value='0.50'),
        DeclareLaunchArgument('initial_yaw', default_value='1.5708'),
        DeclareLaunchArgument('marker_world_map', default_value=FINAL_DEMO_MARKER_WORLD_MAP),

        DeclareLaunchArgument('odom_topic', default_value='pitt_fused_odometry'),
        DeclareLaunchArgument('reference_path_topic', default_value='reference_path'),
        DeclareLaunchArgument('motor_cmd_topic', default_value='set_motor_duty_cycle'),
        DeclareLaunchArgument('v_ref', default_value='0.12'),
        DeclareLaunchArgument('v_max_mps', default_value='1.50'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.35'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.8'),
        DeclareLaunchArgument('duty_cycle_limit', default_value='32.0'),
        DeclareLaunchArgument('min_moving_duty', default_value='18.0'),
        DeclareLaunchArgument('left_trim', default_value='1.0'),
        DeclareLaunchArgument('right_trim', default_value='0.95'),
        DeclareLaunchArgument('duty_slew_rate_percent_per_sec', default_value='45.0'),
        DeclareLaunchArgument('command_filter_alpha', default_value='0.45'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.55'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.20'),
        DeclareLaunchArgument('control_period', default_value='0.05'),
        DeclareLaunchArgument('control_verbose', default_value='false'),

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
            launch_arguments={
                'namespace': namespace,
                'launch_camera_capture': 'false',
                'launch_aruco_detector': 'true',
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_fps': LaunchConfiguration('camera_fps'),
                'publish_camera_images': LaunchConfiguration('publish_camera_images'),
            }.items(),
        ),

        IncludeLaunchDescription(
            _launch_file('localisation.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'launch_encoder_odometry': 'true',
                'launch_fused_estimator': 'true',
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'marker_world_map': marker_world_map,
            }.items(),
        ),

        IncludeLaunchDescription(
            _launch_file('planning.launch.py'),
            condition=IfCondition(launch_planning),
            launch_arguments={
                'namespace': namespace,
                'reference_path_topic': reference_path_topic,
            }.items(),
        ),

        IncludeLaunchDescription(
            _launch_file('plant_detector.launch.py'),
            condition=IfCondition(launch_plant_detector),
            launch_arguments={
                'namespace': namespace,
                'model_path': model_path,
                'image_topic': '/asc/camera_image',
                'enabled_topic': '/asc/plant_detector/enabled',
                'start_enabled': plant_detector_start_enabled,
                'publish_debug_image': 'false',
            }.items(),
        ),

        Node(
            package='asclinic_pkg',
            executable='lqg_controller_test.py',
            namespace=namespace,
            name='lqg_controller_test',
            output='screen',
            condition=IfCondition(launch_lqg),
            parameters=[{
                'odom_topic': odom_topic,
                'path_topic': reference_path_topic,
                'motor_cmd_topic': motor_cmd_topic,
                'v_ref': LaunchConfiguration('v_ref'),
                'v_max_mps': LaunchConfiguration('v_max_mps'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
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
                'control_period': LaunchConfiguration('control_period'),
                'verbose': LaunchConfiguration('control_verbose'),
            }],
        ),

        Node(
            package='asclinic_pkg',
            executable='system_status_publisher.py',
            namespace=namespace,
            name='jetson_status_publisher',
            output='screen',
            condition=IfCondition(launch_system_status),
            parameters=[{
                'topic': 'jetson_status',
                'publish_period_sec': 1.0,
                'disk_path': '/',
            }],
        ),

        Node(
            package='asclinic_pkg',
            executable='live_global_map_viewer.py',
            namespace=namespace,
            name='live_global_map_viewer',
            output='screen',
            condition=IfCondition(launch_map_viewer),
            parameters=[{
                'odom_topic': odom_topic,
                'wheel_odom_topic': 'wheel_odometry',
                'vision_odom_topic': 'pitt_vision_odometry',
                'path_topic': reference_path_topic,
                'stop_poses_topic': 'plant_stop_poses',
                'system_status_topic': 'jetson_status',
                'save_directory': '~/asclinic-ros2/ros2_ws/results/live_global_map',
                'show_gui': True,
                'save_live_png': True,
            }],
        ),
    ])
