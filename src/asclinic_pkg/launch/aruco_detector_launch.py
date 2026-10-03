#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    verbosity = LaunchConfiguration('aruco_detector_verbosity')
    device_number = LaunchConfiguration('aruco_detector_usb_camera_device_number')
    frame_height = LaunchConfiguration('aruco_detector_desired_camera_frame_height')
    frame_width = LaunchConfiguration('aruco_detector_desired_camera_frame_width')
    cam_fps = LaunchConfiguration('aruco_detector_desired_camera_fps')
    marker_size = LaunchConfiguration('aruco_detector_marker_size')
    img_path = LaunchConfiguration('aruco_detector_save_image_path')
    save_all_img = LaunchConfiguration('aruco_detector_should_save_all_aruco_images')
    pub_all_img = LaunchConfiguration('aruco_detector_should_publish_camera_images')
    show_all_img = LaunchConfiguration('aruco_detector_should_show_camera_images')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('aruco_detector_verbosity', default_value='1'),
        DeclareLaunchArgument('aruco_detector_usb_camera_device_number', default_value='0'),
        DeclareLaunchArgument('aruco_detector_desired_camera_frame_height', default_value='1080'),
        DeclareLaunchArgument('aruco_detector_desired_camera_frame_width', default_value='1920'),
        DeclareLaunchArgument('aruco_detector_desired_camera_fps', default_value='5'),
        DeclareLaunchArgument('aruco_detector_marker_size', default_value='0.250'),
        DeclareLaunchArgument(
            'aruco_detector_save_image_path',
            default_value='saved_camera_images/',
        ),
        DeclareLaunchArgument('aruco_detector_should_save_all_aruco_images', default_value='false'),
        DeclareLaunchArgument('aruco_detector_should_publish_camera_images', default_value='true'),
        DeclareLaunchArgument('aruco_detector_should_show_camera_images', default_value='false'),

        Node(
            package='asclinic_pkg',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            namespace=namespace,
            parameters=[{
                'aruco_detector_verbosity': verbosity,
                'aruco_detector_usb_camera_device_number': device_number,
                'aruco_detector_desired_camera_frame_height': frame_height,
                'aruco_detector_desired_camera_frame_width': frame_width,
                'aruco_detector_desired_camera_fps': cam_fps,
                'aruco_detector_marker_size': marker_size,
                'aruco_detector_save_image_path': img_path,
                'aruco_detector_should_save_all_aruco_images': save_all_img,
                'aruco_detector_should_publish_camera_images': pub_all_img,
                'aruco_detector_should_show_camera_images': show_all_img,
            }],
        ),
    ])
