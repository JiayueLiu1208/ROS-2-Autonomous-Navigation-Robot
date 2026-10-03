#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    localisation_params_file = LaunchConfiguration('localisation_params_file')
    launch_encoder_odometry = LaunchConfiguration('launch_encoder_odometry')
    launch_fused_estimator = LaunchConfiguration('launch_fused_estimator')
    launch_mock_odometry = LaunchConfiguration('launch_mock_odometry')
    launch_lidar_wall_localizer = LaunchConfiguration('launch_lidar_wall_localizer')
    launch_lidar_scan_match_localizer = LaunchConfiguration(
        'launch_lidar_scan_match_localizer'
    )
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    scan_topic = LaunchConfiguration('scan_topic')
    lidar_wall_odom_topic = LaunchConfiguration('lidar_wall_odom_topic')
    lidar_scanmatch_odom_topic = LaunchConfiguration('lidar_scanmatch_odom_topic')
    lidar_scanmatch_prior_odom_topic = LaunchConfiguration(
        'lidar_scanmatch_prior_odom_topic'
    )
    lidar_in_base_x = LaunchConfiguration('lidar_in_base_x')
    lidar_in_base_y = LaunchConfiguration('lidar_in_base_y')
    lidar_in_base_yaw_deg = LaunchConfiguration('lidar_in_base_yaw_deg')
    lidar_scan_angle_multiplier = LaunchConfiguration('lidar_scan_angle_multiplier')
    use_lidar_wall_update = LaunchConfiguration('use_lidar_wall_update')
    use_lidar_scanmatch_update = LaunchConfiguration('use_lidar_scanmatch_update')

    default_params = PathJoinSubstitution(
        [FindPackageShare('asclinic_pkg'), 'config', 'localisation_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('localisation_params_file', default_value=default_params),
        DeclareLaunchArgument('launch_encoder_odometry', default_value='true'),
        DeclareLaunchArgument('launch_fused_estimator', default_value='true'),
        DeclareLaunchArgument('launch_mock_odometry', default_value='false'),
        DeclareLaunchArgument('launch_lidar_wall_localizer', default_value='false'),
        DeclareLaunchArgument('launch_lidar_scan_match_localizer', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('encoder_counts_topic', default_value='encoder_counts'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='wheel_odometry'),
        DeclareLaunchArgument('aruco_topic', default_value='aruco_detections'),
        DeclareLaunchArgument('fused_odom_topic', default_value='pitt_fused_odometry'),
        DeclareLaunchArgument('vision_odom_topic', default_value='pitt_vision_odometry'),
        DeclareLaunchArgument('lidar_wall_odom_topic', default_value='pitt_lidar_wall_odometry'),
        DeclareLaunchArgument(
            'lidar_scanmatch_odom_topic',
            default_value='pitt_lidar_scanmatch_odometry',
        ),
        DeclareLaunchArgument(
            'lidar_scanmatch_prior_odom_topic',
            default_value='pitt_fused_odometry',
        ),
        DeclareLaunchArgument('marker_world_map', default_value=FINAL_DEMO_MARKER_WORLD_MAP),
        DeclareLaunchArgument('camera_in_base_x', default_value='0.0'),
        DeclareLaunchArgument('camera_in_base_y', default_value='0.0'),
        DeclareLaunchArgument('camera_in_base_yaw', default_value='0.041'),
        DeclareLaunchArgument('lidar_in_base_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_in_base_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_in_base_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('lidar_scan_angle_multiplier', default_value='1.0'),
        DeclareLaunchArgument('use_pitt_kalman_filter', default_value='true'),
        DeclareLaunchArgument('use_lidar_wall_update', default_value='true'),
        DeclareLaunchArgument('use_lidar_scanmatch_update', default_value='false'),
        DeclareLaunchArgument('off_axis_skip_deg', default_value='22.0'),
        DeclareLaunchArgument('incidence_skip_deg', default_value='60.0'),
        DeclareLaunchArgument('vision_yaw_gate_deg', default_value='25.0'),
        DeclareLaunchArgument('use_vision_yaw_for_position', default_value='false'),
        DeclareLaunchArgument('use_vision_yaw_update', default_value='true'),
        DeclareLaunchArgument('use_vision_xy_yaw_coupling', default_value='false'),
        DeclareLaunchArgument('use_multi_marker_fusion', default_value='true'),
        DeclareLaunchArgument('vision_min_markers', default_value='1'),
        DeclareLaunchArgument('vision_max_marker_range_m', default_value='7.0'),
        DeclareLaunchArgument('single_marker_max_range_m', default_value='5.5'),
        DeclareLaunchArgument('vision_outlier_rejection_m', default_value='2.50'),
        DeclareLaunchArgument('vision_update_period_s', default_value='1.5'),
        DeclareLaunchArgument('vision_filter_window_s', default_value='1.6'),
        DeclareLaunchArgument('vision_filter_min_samples', default_value='3'),
        DeclareLaunchArgument('vision_filter_max_std_m', default_value='0.35'),
        DeclareLaunchArgument('vision_filter_max_jump_m', default_value='0.75'),
        DeclareLaunchArgument('vision_correction_step_limit_m', default_value='0.45'),
        DeclareLaunchArgument('vision_r_x', default_value='0.05'),
        DeclareLaunchArgument('vision_r_y', default_value='0.05'),
        DeclareLaunchArgument('vision_r_yaw', default_value='0.08'),
        DeclareLaunchArgument('verbosity', default_value='2'),

        Node(
            package='asclinic_pkg',
            executable='mock_odometry_publisher.py',
            namespace=namespace,
            name='mock_odometry_publisher',
            output='screen',
            condition=IfCondition(launch_mock_odometry),
        ),

        Node(
            package='asclinic_pkg',
            executable='liu_odometry_from_encoders.py',
            namespace=namespace,
            name='liu_odometry_from_encoders',
            output='screen',
            condition=IfCondition(launch_encoder_odometry),
            parameters=[
                localisation_params_file,
                {
                    'x_initial': initial_x,
                    'y_initial': initial_y,
                    'phi_initial': initial_yaw,
                    'publish_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                },
            ],
            remappings=[
                ('encoder_counts', LaunchConfiguration('encoder_counts_topic')),
            ],
        ),

        Node(
            package='asclinic_pkg',
            executable='pitt_fused_odemetry',
            namespace=namespace,
            name='pitt_fused_odemetry',
            output='screen',
            condition=IfCondition(launch_fused_estimator),
            parameters=[
                localisation_params_file,
                {
                    'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                    'aruco_topic': LaunchConfiguration('aruco_topic'),
                    'fused_odom_topic': LaunchConfiguration('fused_odom_topic'),
                    'vision_odom_topic': LaunchConfiguration('vision_odom_topic'),
                    'lidar_wall_odom_topic': lidar_wall_odom_topic,
                    'lidar_scanmatch_odom_topic': lidar_scanmatch_odom_topic,
                    'marker_world_map': LaunchConfiguration('marker_world_map'),
                    'camera_in_base_x': LaunchConfiguration('camera_in_base_x'),
                    'camera_in_base_y': LaunchConfiguration('camera_in_base_y'),
                    'camera_in_base_yaw': LaunchConfiguration('camera_in_base_yaw'),
                    'use_pitt_kalman_filter': LaunchConfiguration('use_pitt_kalman_filter'),
                    'use_lidar_wall_update': use_lidar_wall_update,
                    'use_lidar_scanmatch_update': use_lidar_scanmatch_update,
                    'off_axis_skip_deg': LaunchConfiguration('off_axis_skip_deg'),
                    'incidence_skip_deg': LaunchConfiguration('incidence_skip_deg'),
                    'vision_yaw_gate_deg': LaunchConfiguration('vision_yaw_gate_deg'),
                    'use_vision_yaw_for_position': LaunchConfiguration('use_vision_yaw_for_position'),
                    'use_vision_yaw_update': LaunchConfiguration('use_vision_yaw_update'),
                    'use_vision_xy_yaw_coupling': LaunchConfiguration('use_vision_xy_yaw_coupling'),
                    'use_multi_marker_fusion': LaunchConfiguration('use_multi_marker_fusion'),
                    'vision_min_markers': LaunchConfiguration('vision_min_markers'),
                    'vision_max_marker_range_m': LaunchConfiguration('vision_max_marker_range_m'),
                    'single_marker_max_range_m': LaunchConfiguration('single_marker_max_range_m'),
                    'vision_outlier_rejection_m': LaunchConfiguration('vision_outlier_rejection_m'),
                    'vision_update_period_s': LaunchConfiguration('vision_update_period_s'),
                    'vision_filter_window_s': LaunchConfiguration('vision_filter_window_s'),
                    'vision_filter_min_samples': LaunchConfiguration('vision_filter_min_samples'),
                    'vision_filter_max_std_m': LaunchConfiguration('vision_filter_max_std_m'),
                    'vision_filter_max_jump_m': LaunchConfiguration('vision_filter_max_jump_m'),
                    'vision_correction_step_limit_m': LaunchConfiguration('vision_correction_step_limit_m'),
                    'vision_r_x': LaunchConfiguration('vision_r_x'),
                    'vision_r_y': LaunchConfiguration('vision_r_y'),
                    'vision_r_yaw': LaunchConfiguration('vision_r_yaw'),
                    'verbosity': LaunchConfiguration('verbosity'),
                },
            ],
        ),

        Node(
            package='asclinic_pkg',
            executable='lidar_wall_localizer',
            namespace=namespace,
            name='lidar_wall_localizer',
            output='screen',
            condition=IfCondition(launch_lidar_wall_localizer),
            parameters=[
                localisation_params_file,
                {
                    'scan_topic': scan_topic,
                    'odom_topic': LaunchConfiguration('fused_odom_topic'),
                    'wall_odom_topic': lidar_wall_odom_topic,
                    'lidar_in_base_x': lidar_in_base_x,
                    'lidar_in_base_y': lidar_in_base_y,
                    'lidar_in_base_yaw_deg': lidar_in_base_yaw_deg,
                    'lidar_scan_angle_multiplier': lidar_scan_angle_multiplier,
                },
            ],
        ),

        Node(
            package='asclinic_pkg',
            executable='lidar_scan_match_localizer.py',
            namespace=namespace,
            name='lidar_scan_match_localizer',
            output='screen',
            condition=IfCondition(launch_lidar_scan_match_localizer),
            parameters=[
                localisation_params_file,
                {
                    'scan_topic': scan_topic,
                    'map_topic': '/map',
                    'prior_odom_topic': lidar_scanmatch_prior_odom_topic,
                    'matched_odom_topic': lidar_scanmatch_odom_topic,
                    'lidar_in_base_x': lidar_in_base_x,
                    'lidar_in_base_y': lidar_in_base_y,
                    'lidar_in_base_yaw_deg': lidar_in_base_yaw_deg,
                    'lidar_scan_angle_multiplier': lidar_scan_angle_multiplier,
                },
            ],
        ),
    ])
