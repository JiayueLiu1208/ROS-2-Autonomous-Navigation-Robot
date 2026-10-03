#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(value):
    return ParameterValue(value, value_type=float)


def _int(value):
    return ParameterValue(value, value_type=int)


def _bool(value):
    return ParameterValue(value, value_type=bool)


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    launch_map = LaunchConfiguration('launch_map')
    launch_known_map_planner = LaunchConfiguration('launch_known_map_planner')
    launch_trajectory_generator = LaunchConfiguration('launch_trajectory_generator')
    reference_path_topic = LaunchConfiguration('reference_path_topic')
    dynamic_obstacle_marker_topic = LaunchConfiguration('dynamic_obstacle_marker_topic')
    explored_map_topic = LaunchConfiguration('explored_map_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    mission_progress_topic = LaunchConfiguration('mission_progress_topic')
    plant_goal_standoff_m = LaunchConfiguration('plant_goal_standoff_m')
    order_deviation_threshold_m = LaunchConfiguration('order_deviation_threshold_m')
    rotation_cost_per_90deg_m = LaunchConfiguration('rotation_cost_per_90deg_m')
    turn_start_cost_m = LaunchConfiguration('turn_start_cost_m')
    obstacle_clearance_radius_m = LaunchConfiguration('obstacle_clearance_radius_m')
    obstacle_clearance_weight_m = LaunchConfiguration('obstacle_clearance_weight_m')
    hard_obstacle_clearance_m = LaunchConfiguration('hard_obstacle_clearance_m')
    fog_path_preference = LaunchConfiguration('fog_path_preference')
    fast_global_ordering = LaunchConfiguration('fast_global_ordering')
    global_order_grid_stride = LaunchConfiguration('global_order_grid_stride')
    include_coverage_stops = LaunchConfiguration('include_coverage_stops')
    fog_cleanup_enabled = LaunchConfiguration('fog_cleanup_enabled')
    fog_cleanup_trigger_ratio = LaunchConfiguration('fog_cleanup_trigger_ratio')
    fog_cleanup_min_cluster_ratio = LaunchConfiguration('fog_cleanup_min_cluster_ratio')
    fog_cleanup_max_cluster_ratio = LaunchConfiguration('fog_cleanup_max_cluster_ratio')
    fog_cleanup_max_goals = LaunchConfiguration('fog_cleanup_max_goals')
    planned_coverage_enabled = LaunchConfiguration('planned_coverage_enabled')
    planned_coverage_radius_m = LaunchConfiguration('planned_coverage_radius_m')
    planned_coverage_use_line_of_sight = LaunchConfiguration(
        'planned_coverage_use_line_of_sight'
    )
    planned_coverage_sample_step_m = LaunchConfiguration(
        'planned_coverage_sample_step_m'
    )
    planned_coverage_min_cluster_area_m2 = LaunchConfiguration(
        'planned_coverage_min_cluster_area_m2'
    )
    planned_coverage_max_goals = LaunchConfiguration('planned_coverage_max_goals')
    planned_coverage_goal_tolerance_m = LaunchConfiguration(
        'planned_coverage_goal_tolerance_m'
    )
    current_path_bias_m = LaunchConfiguration('current_path_bias_m')
    current_path_bias_radius_m = LaunchConfiguration('current_path_bias_radius_m')
    start_heading_bias_m = LaunchConfiguration('start_heading_bias_m')
    start_heading_bias_distance_m = LaunchConfiguration('start_heading_bias_distance_m')
    dynamic_obstacle_ttl_sec = LaunchConfiguration('dynamic_obstacle_ttl_sec')
    dynamic_obstacle_inflation_m = LaunchConfiguration('dynamic_obstacle_inflation_m')
    dynamic_obstacle_static_filter_m = LaunchConfiguration(
        'dynamic_obstacle_static_filter_m'
    )
    dynamic_obstacle_beam_stride = LaunchConfiguration('dynamic_obstacle_beam_stride')
    dynamic_obstacle_max_points = LaunchConfiguration('dynamic_obstacle_max_points')
    dynamic_obstacle_marker_latest_only = LaunchConfiguration(
        'dynamic_obstacle_marker_latest_only'
    )
    lidar_in_base_x = LaunchConfiguration('lidar_in_base_x')
    lidar_in_base_y = LaunchConfiguration('lidar_in_base_y')
    lidar_in_base_yaw_deg = LaunchConfiguration('lidar_in_base_yaw_deg')
    lidar_scan_angle_multiplier = LaunchConfiguration('lidar_scan_angle_multiplier')
    replan_cooldown_sec = LaunchConfiguration('replan_cooldown_sec')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('launch_map', default_value='true'),
        DeclareLaunchArgument('launch_known_map_planner', default_value='true'),
        DeclareLaunchArgument('launch_trajectory_generator', default_value='false'),
        DeclareLaunchArgument('reference_path_topic', default_value='reference_path'),
        DeclareLaunchArgument(
            'dynamic_obstacle_marker_topic',
            default_value='dynamic_obstacle_marker',
        ),
        DeclareLaunchArgument('explored_map_topic', default_value='explored_map'),
        DeclareLaunchArgument('odom_topic', default_value='pitt_fused_odometry'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('mission_progress_topic', default_value='mission_progress'),
        DeclareLaunchArgument('plant_goal_standoff_m', default_value='1.0'),
        DeclareLaunchArgument('order_deviation_threshold_m', default_value='5.0'),
        DeclareLaunchArgument('rotation_cost_per_90deg_m', default_value='0.3'),
        DeclareLaunchArgument('turn_start_cost_m', default_value='0.25'),
        DeclareLaunchArgument('obstacle_clearance_radius_m', default_value='0.30'),
        DeclareLaunchArgument('obstacle_clearance_weight_m', default_value='3.0'),
        DeclareLaunchArgument('hard_obstacle_clearance_m', default_value='0.0'),
        DeclareLaunchArgument('fog_path_preference', default_value='1.00'),
        DeclareLaunchArgument('fast_global_ordering', default_value='true'),
        DeclareLaunchArgument('global_order_grid_stride', default_value='1'),
        DeclareLaunchArgument('include_coverage_stops', default_value='true'),
        DeclareLaunchArgument('fog_cleanup_enabled', default_value='false'),
        DeclareLaunchArgument('fog_cleanup_trigger_ratio', default_value='0.50'),
        DeclareLaunchArgument('fog_cleanup_min_cluster_ratio', default_value='0.01'),
        DeclareLaunchArgument('fog_cleanup_max_cluster_ratio', default_value='0.10'),
        DeclareLaunchArgument('fog_cleanup_max_goals', default_value='1'),
        DeclareLaunchArgument('planned_coverage_enabled', default_value='false'),
        DeclareLaunchArgument('planned_coverage_radius_m', default_value='2.0'),
        DeclareLaunchArgument('planned_coverage_use_line_of_sight', default_value='true'),
        DeclareLaunchArgument('planned_coverage_sample_step_m', default_value='0.25'),
        DeclareLaunchArgument('planned_coverage_min_cluster_area_m2', default_value='0.60'),
        DeclareLaunchArgument('planned_coverage_max_goals', default_value='2'),
        DeclareLaunchArgument('planned_coverage_goal_tolerance_m', default_value='0.25'),
        DeclareLaunchArgument('current_path_bias_m', default_value='0.02'),
        DeclareLaunchArgument('current_path_bias_radius_m', default_value='0.30'),
        DeclareLaunchArgument('start_heading_bias_m', default_value='0.15'),
        DeclareLaunchArgument('start_heading_bias_distance_m', default_value='0.70'),
        DeclareLaunchArgument('dynamic_obstacle_ttl_sec', default_value='1.0'),
        DeclareLaunchArgument('dynamic_obstacle_inflation_m', default_value='0.20'),
        DeclareLaunchArgument('dynamic_obstacle_static_filter_m', default_value='0.0'),
        DeclareLaunchArgument('dynamic_obstacle_beam_stride', default_value='4'),
        DeclareLaunchArgument('dynamic_obstacle_max_points', default_value='1500'),
        DeclareLaunchArgument('dynamic_obstacle_marker_latest_only', default_value='true'),
        DeclareLaunchArgument('lidar_in_base_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_in_base_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_in_base_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('lidar_scan_angle_multiplier', default_value='1.0'),
        DeclareLaunchArgument('replan_cooldown_sec', default_value='2.0'),

        Node(
            package='asclinic_pkg',
            executable='map.py',
            namespace=namespace,
            name='final_demo_map_server',
            output='screen',
            condition=IfCondition(launch_map),
        ),

        Node(
            package='asclinic_pkg',
            executable='global_planner.py',
            namespace=namespace,
            name='known_map_path_planner',
            output='screen',
            condition=IfCondition(launch_known_map_planner),
            parameters=[{
                'map_topic': '/map',
                'path_topic': reference_path_topic,
                'stop_poses_topic': 'plant_stop_poses',
                'dynamic_obstacle_marker_topic': dynamic_obstacle_marker_topic,
                'explored_map_topic': explored_map_topic,
                'odom_topic': odom_topic,
                'scan_topic': scan_topic,
                'mission_progress_topic': mission_progress_topic,
                'plant_goal_standoff_m': _float(plant_goal_standoff_m),
                'order_deviation_threshold_m': _float(order_deviation_threshold_m),
                'rotation_cost_per_90deg_m': _float(rotation_cost_per_90deg_m),
                'turn_start_cost_m': _float(turn_start_cost_m),
                'obstacle_clearance_radius_m': _float(obstacle_clearance_radius_m),
                'obstacle_clearance_weight_m': _float(obstacle_clearance_weight_m),
                'hard_obstacle_clearance_m': _float(hard_obstacle_clearance_m),
                'fog_path_preference': _float(fog_path_preference),
                'fast_global_ordering': _bool(fast_global_ordering),
                'global_order_grid_stride': _int(global_order_grid_stride),
                'include_coverage_stops': _bool(include_coverage_stops),
                'fog_cleanup_enabled': _bool(fog_cleanup_enabled),
                'fog_cleanup_trigger_ratio': _float(fog_cleanup_trigger_ratio),
                'fog_cleanup_min_cluster_ratio': _float(fog_cleanup_min_cluster_ratio),
                'fog_cleanup_max_cluster_ratio': _float(fog_cleanup_max_cluster_ratio),
                'fog_cleanup_max_goals': _int(fog_cleanup_max_goals),
                'planned_coverage_enabled': _bool(planned_coverage_enabled),
                'planned_coverage_radius_m': _float(planned_coverage_radius_m),
                'planned_coverage_use_line_of_sight': _bool(planned_coverage_use_line_of_sight),
                'planned_coverage_sample_step_m': _float(planned_coverage_sample_step_m),
                'planned_coverage_min_cluster_area_m2': _float(planned_coverage_min_cluster_area_m2),
                'planned_coverage_max_goals': _int(planned_coverage_max_goals),
                'planned_coverage_goal_tolerance_m': _float(planned_coverage_goal_tolerance_m),
                'current_path_bias_m': _float(current_path_bias_m),
                'current_path_bias_radius_m': _float(current_path_bias_radius_m),
                'start_heading_bias_m': _float(start_heading_bias_m),
                'start_heading_bias_distance_m': _float(start_heading_bias_distance_m),
                'dynamic_obstacle_ttl_sec': _float(dynamic_obstacle_ttl_sec),
                'dynamic_obstacle_inflation_m': _float(dynamic_obstacle_inflation_m),
                'dynamic_obstacle_static_filter_m': _float(dynamic_obstacle_static_filter_m),
                'dynamic_obstacle_beam_stride': _int(dynamic_obstacle_beam_stride),
                'dynamic_obstacle_max_points': _int(dynamic_obstacle_max_points),
                'dynamic_obstacle_marker_latest_only': _bool(dynamic_obstacle_marker_latest_only),
                'lidar_in_base_x': _float(lidar_in_base_x),
                'lidar_in_base_y': _float(lidar_in_base_y),
                'lidar_in_base_yaw_deg': _float(lidar_in_base_yaw_deg),
                'lidar_scan_angle_multiplier': _float(lidar_scan_angle_multiplier),
                'replan_cooldown_sec': _float(replan_cooldown_sec),
            }],
        ),

        Node(
            package='asclinic_pkg',
            executable='trajectory_generation.py',
            namespace=namespace,
            name='trajectory_generation',
            output='screen',
            condition=IfCondition(launch_trajectory_generator),
            remappings=[
                ('sprint_review_path', reference_path_topic),
            ],
        ),
    ])
