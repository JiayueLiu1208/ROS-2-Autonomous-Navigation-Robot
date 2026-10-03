#!/usr/bin/env python3

# Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#
# See the root of the repository for license details.
#
# Launch file for the control_policy_skeleton Python node.



from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Namespace and node info
    namespace    = "asc"
    asclinic_pkg = "asclinic_pkg"
    node_exec    = "control_policy_skeleton.py" 

    # --- Parameters as LaunchConfigurations ---
    control_policy_verbosity = LaunchConfiguration(
        'control_policy_verbosity',
        default='1'
    )

    robot_wheel_base = LaunchConfiguration(
        'robot_wheel_base',
        default='0.22'
    )

    robot_wheel_radius = LaunchConfiguration(
        'robot_wheel_radius',
        default='0.072'
    )

    encoder_counts_per_wheel_revolution = LaunchConfiguration(
        'encoder_counts_per_wheel_revolution',
        default='4480'
    )

    # --- Declare launch arguments ---
    declare_control_policy_verbosity = DeclareLaunchArgument(
        'control_policy_verbosity',
        default_value=control_policy_verbosity,
        description='Verbosity level for control_policy_skeleton'
    )

    declare_robot_wheel_base = DeclareLaunchArgument(
        'robot_wheel_base',
        default_value=robot_wheel_base,
        description='Robot wheel base (meters)'
    )

    declare_robot_wheel_radius = DeclareLaunchArgument(
        'robot_wheel_radius',
        default_value=robot_wheel_radius,
        description='Robot wheel radius (meters)'
    )

    declare_encoder_counts_per_wheel_revolution = DeclareLaunchArgument(
        'encoder_counts_per_wheel_revolution',
        default_value=encoder_counts_per_wheel_revolution,
        description='Encoder counts per wheel revolution (RoboClaw quadrature: 64 CPR x 70 = 4480)'
    )

    # --- Node definition ---
    control_policy_node = Node(
        package    = asclinic_pkg,
        executable = node_exec,
        namespace  = namespace,
        name       = 'control_policy_skeleton',
        output     = 'screen',
        parameters = [{
            'control_policy_verbosity':             control_policy_verbosity,
            'robot_wheel_base':                     robot_wheel_base,
            'robot_wheel_radius':                   robot_wheel_radius,
            'encoder_counts_per_wheel_revolution':  encoder_counts_per_wheel_revolution,
        }]
    )

    # --- Build LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(declare_control_policy_verbosity)
    ld.add_action(declare_robot_wheel_base)
    ld.add_action(declare_robot_wheel_radius)
    ld.add_action(declare_encoder_counts_per_wheel_revolution)
    ld.add_action(control_policy_node)

    return ld
