#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    launch_camera_capture = LaunchConfiguration('launch_camera_capture')
    launch_aruco_detector = LaunchConfiguration('launch_aruco_detector')

    default_params = PathJoinSubstitution(
        [FindPackageShare('asclinic_pkg'), 'config', 'camera_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('launch_camera_capture', default_value='true'),
        DeclareLaunchArgument('launch_aruco_detector', default_value='false'),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_fps', default_value='5'),
        DeclareLaunchArgument('publish_camera_images', default_value='true'),
        DeclareLaunchArgument('show_camera_images', default_value='false'),

        Node(
            package='asclinic_pkg',
            executable='camera_capture.py',
            namespace=namespace,
            name='camera_capture',
            output='screen',
            condition=IfCondition(launch_camera_capture),
            parameters=[
                params_file,
                {
                    'camera_capture_usb_camera_device_number': LaunchConfiguration('camera_device'),
                    'camera_capture_desired_camera_fps': LaunchConfiguration('camera_fps'),
                    'camera_capture_should_publish_camera_images': LaunchConfiguration(
                        'publish_camera_images'
                    ),
                    'camera_capture_should_show_camera_images': LaunchConfiguration(
                        'show_camera_images'
                    ),
                },
            ],
        ),

        Node(
            package='asclinic_pkg',
            executable='aruco_detector',
            namespace=namespace,
            name='aruco_detector',
            output='screen',
            condition=IfCondition(launch_aruco_detector),
            parameters=[
                params_file,
                {
                    'aruco_detector_usb_camera_device_number': LaunchConfiguration('camera_device'),
                    'aruco_detector_desired_camera_fps': LaunchConfiguration('camera_fps'),
                    'aruco_detector_should_publish_camera_images': LaunchConfiguration(
                        'publish_camera_images'
                    ),
                    'aruco_detector_should_show_camera_images': LaunchConfiguration(
                        'show_camera_images'
                    ),
                },
            ],
        ),
    ])
