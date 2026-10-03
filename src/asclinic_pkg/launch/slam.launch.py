#!/usr/bin/env python3
"""
Launch SLAM toolbox with the correct TF tree for this robot.

Required TF chain:  odom -> base_link -> laser
  - odom -> base_link : published by liu_odometry_from_encoders (must be running)
  - base_link -> laser : static transform published here

Adjust lidar_x / lidar_y / lidar_z to match the physical lidar mount position
on the robot (measured from the centre of the robot base).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_params = PathJoinSubstitution(
        [FindPackageShare('asclinic_pkg'), 'config', 'slam_params.yaml']
    )

    # Physical offset of the LiDAR frame origin relative to base_link.
    # Measure these values on the real robot and update accordingly.
    lidar_x   = '0.0'   # metres forward  (+x = forward)
    lidar_y   = '0.0'   # metres left      (+y = left)
    lidar_z   = '0.15'  # metres up        (+z = up)
    lidar_roll  = '0.0'
    lidar_pitch = '0.0'
    lidar_yaw   = '0.0'

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params,
            description='Path to slam_toolbox parameters YAML',
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_tf',
            arguments=[
                '--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                '--roll', lidar_roll, '--pitch', lidar_pitch, '--yaw', lidar_yaw,
                '--frame-id', 'base_link', '--child-frame-id', 'laser',
            ],
        ),

        # SLAM toolbox (async)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': False,
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'scan_topic': '/scan',
                },
            ],
        ),
    ])
