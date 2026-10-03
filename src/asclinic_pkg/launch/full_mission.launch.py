#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_PLANT_MODEL_PATH = PathJoinSubstitution(
    [FindPackageShare('asclinic_pkg'), 'models', 'plant_detector', 'best.pt']
)

DEFAULT_LOCALISATION_PARAMS_PATH = PathJoinSubstitution(
    [FindPackageShare('asclinic_pkg'), 'config', 'localisation_params.yaml']
)

DEFAULT_CONTROLLER_PARAMS_PATH = PathJoinSubstitution(
    [FindPackageShare('asclinic_pkg'), 'config', 'controller_params.yaml']
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


def _package_launch_file(package_name, name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), 'launch', name])
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    launch_plant_detector = LaunchConfiguration('launch_plant_detector')
    launch_planning = LaunchConfiguration('launch_planning')
    launch_control = LaunchConfiguration('launch_control')
    launch_map_viewer = LaunchConfiguration('launch_map_viewer')
    launch_exploration_map = LaunchConfiguration('launch_exploration_map')
    launch_system_status = LaunchConfiguration('launch_system_status')
    launch_aruco_detector = LaunchConfiguration('launch_aruco_detector')
    launch_fused_estimator = LaunchConfiguration('launch_fused_estimator')
    launch_rplidar_driver = LaunchConfiguration('launch_rplidar_driver')
    launch_lidar_wall_localizer = LaunchConfiguration('launch_lidar_wall_localizer')
    launch_lidar_scan_match_localizer = LaunchConfiguration(
        'launch_lidar_scan_match_localizer'
    )
    model_path = LaunchConfiguration('model_path')
    run_tag = LaunchConfiguration('run_tag')
    localisation_params_file = LaunchConfiguration('localisation_params_file')
    control_params_file = LaunchConfiguration('control_params_file')
    plant_detector_start_enabled = LaunchConfiguration('plant_detector_start_enabled')
    plant_save_positive_images = LaunchConfiguration('plant_save_positive_images')
    plant_publish_debug_image = LaunchConfiguration('plant_publish_debug_image')
    plant_positive_image_dir = LaunchConfiguration('plant_positive_image_dir')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    path_controller_backend = LaunchConfiguration('path_controller_backend')
    backend_delta_v_limit = LaunchConfiguration('backend_delta_v_limit')
    backend_delta_omega_limit = LaunchConfiguration('backend_delta_omega_limit')
    lqg_smooth_turn_enabled = LaunchConfiguration('lqg_smooth_turn_enabled')
    lqg_smooth_turn_angle_deg = LaunchConfiguration('lqg_smooth_turn_angle_deg')
    lqg_stop_turn_angle_deg = LaunchConfiguration('lqg_stop_turn_angle_deg')
    lqg_min_smooth_turn_speed_scale = LaunchConfiguration(
        'lqg_min_smooth_turn_speed_scale'
    )
    nominal_speed = LaunchConfiguration('nominal_speed')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    duty_cycle_limit = LaunchConfiguration('duty_cycle_limit')
    min_moving_duty = LaunchConfiguration('min_moving_duty')
    max_wheel_speed_at_full_duty = LaunchConfiguration('max_wheel_speed_at_full_duty')
    duty_slew_rate = LaunchConfiguration('duty_slew_rate_percent_per_sec')
    graceful_stop_enabled = LaunchConfiguration('graceful_stop_enabled')
    graceful_stop_decel_duty_per_sec = LaunchConfiguration(
        'graceful_stop_decel_duty_per_sec'
    )
    graceful_stop_turn_duty_per_sec = LaunchConfiguration(
        'graceful_stop_turn_duty_per_sec'
    )
    command_filter_alpha = LaunchConfiguration('command_filter_alpha')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    slowdown_distance = LaunchConfiguration('slowdown_distance')
    curvature_gain = LaunchConfiguration('curvature_gain')
    backward_target_turn_direction = LaunchConfiguration('backward_target_turn_direction')
    left_trim = LaunchConfiguration('left_trim')
    right_trim = LaunchConfiguration('right_trim')
    control_verbose = LaunchConfiguration('control_verbose')
    control_odom_topic = LaunchConfiguration('control_odom_topic')
    map_viewer_odom_topic = LaunchConfiguration('map_viewer_odom_topic')
    map_viewer_show_gui = LaunchConfiguration('map_viewer_show_gui')
    map_viewer_backend = LaunchConfiguration('map_viewer_backend')
    map_viewer_plot_period_sec = LaunchConfiguration('map_viewer_plot_period_sec')
    map_viewer_save_period_sec = LaunchConfiguration('map_viewer_save_period_sec')
    map_viewer_save_live_png = LaunchConfiguration('map_viewer_save_live_png')
    point_to_point_mode = LaunchConfiguration('point_to_point_mode')
    point_to_point_rotate_tolerance = LaunchConfiguration('point_to_point_rotate_tolerance')
    point_to_point_arrival_tolerance = LaunchConfiguration('point_to_point_arrival_tolerance')
    preserve_replan_motion_enabled = LaunchConfiguration(
        'preserve_replan_motion_enabled'
    )
    preserve_replan_heading_tolerance_deg = LaunchConfiguration(
        'preserve_replan_heading_tolerance_deg'
    )
    preserve_replan_start_tolerance_m = LaunchConfiguration(
        'preserve_replan_start_tolerance_m'
    )
    corner_turn_max_duty = LaunchConfiguration('corner_turn_max_duty')
    segment_straight_duty = LaunchConfiguration('segment_straight_duty')
    segment_min_straight_duty = LaunchConfiguration('segment_min_straight_duty')
    segment_yaw_kp = LaunchConfiguration('segment_yaw_kp')
    segment_max_correction = LaunchConfiguration('segment_max_correction')
    segment_lateral_kp = LaunchConfiguration('segment_lateral_kp')
    segment_lateral_deadband_m = LaunchConfiguration('segment_lateral_deadband_m')
    segment_lateral_max_heading_deg = LaunchConfiguration('segment_lateral_max_heading_deg')
    segment_deceleration_distance = LaunchConfiguration('segment_deceleration_distance')
    segment_velocity_profile_enabled = LaunchConfiguration('segment_velocity_profile_enabled')
    segment_profile_cruise_duty = LaunchConfiguration('segment_profile_cruise_duty')
    segment_profile_approach_duty = LaunchConfiguration('segment_profile_approach_duty')
    segment_profile_accel_duty_per_sec = LaunchConfiguration(
        'segment_profile_accel_duty_per_sec'
    )
    launch_obstacle_filter = LaunchConfiguration('launch_obstacle_filter')
    launch_fog_inspection_filter = LaunchConfiguration('launch_fog_inspection_filter')
    scan_topic = LaunchConfiguration('scan_topic')
    lidar_scan_angle_multiplier = LaunchConfiguration('lidar_scan_angle_multiplier')
    lidar_avoid_distance_m = LaunchConfiguration('lidar_avoid_distance_m')
    lidar_clear_distance_m = LaunchConfiguration('lidar_clear_distance_m')
    lidar_pre_stop_decel_enabled = LaunchConfiguration('lidar_pre_stop_decel_enabled')
    lidar_pre_stop_decel_scale = LaunchConfiguration('lidar_pre_stop_decel_scale')
    lidar_pre_stop_decel_distance_m = LaunchConfiguration(
        'lidar_pre_stop_decel_distance_m'
    )
    lidar_steer_emergency_stop_distance_m = LaunchConfiguration(
        'lidar_steer_emergency_stop_distance_m'
    )
    lidar_front_recovery_enabled = LaunchConfiguration('lidar_front_recovery_enabled')
    lidar_front_recovery_trigger_distance_m = LaunchConfiguration(
        'lidar_front_recovery_trigger_distance_m'
    )
    lidar_front_recovery_distance_m = LaunchConfiguration('lidar_front_recovery_distance_m')
    lidar_front_recovery_reverse_duty = LaunchConfiguration(
        'lidar_front_recovery_reverse_duty'
    )
    lidar_front_recovery_timeout_sec = LaunchConfiguration(
        'lidar_front_recovery_timeout_sec'
    )
    lidar_front_recovery_steer_duration_sec = LaunchConfiguration(
        'lidar_front_recovery_steer_duration_sec'
    )
    lidar_wall_odom_topic = LaunchConfiguration('lidar_wall_odom_topic')
    lidar_scanmatch_odom_topic = LaunchConfiguration('lidar_scanmatch_odom_topic')
    use_lidar_wall_update = LaunchConfiguration('use_lidar_wall_update')
    use_lidar_scanmatch_update = LaunchConfiguration('use_lidar_scanmatch_update')
    dynamic_obstacle_marker_topic = LaunchConfiguration('dynamic_obstacle_marker_topic')
    plant_goal_standoff_m = LaunchConfiguration('plant_goal_standoff_m')
    replan_cooldown_sec = LaunchConfiguration('replan_cooldown_sec')
    turn_start_cost_m = LaunchConfiguration('turn_start_cost_m')
    obstacle_clearance_radius_m = LaunchConfiguration('obstacle_clearance_radius_m')
    obstacle_clearance_weight_m = LaunchConfiguration('obstacle_clearance_weight_m')
    hard_obstacle_clearance_m = LaunchConfiguration('hard_obstacle_clearance_m')
    fog_path_preference = LaunchConfiguration('fog_path_preference')
    dynamic_obstacle_ttl_sec = LaunchConfiguration('dynamic_obstacle_ttl_sec')
    dynamic_obstacle_static_filter_m = LaunchConfiguration(
        'dynamic_obstacle_static_filter_m'
    )
    dynamic_obstacle_beam_stride = LaunchConfiguration('dynamic_obstacle_beam_stride')
    dynamic_obstacle_max_points = LaunchConfiguration('dynamic_obstacle_max_points')
    dynamic_obstacle_marker_latest_only = LaunchConfiguration(
        'dynamic_obstacle_marker_latest_only'
    )
    start_heading_bias_m = LaunchConfiguration('start_heading_bias_m')
    start_heading_bias_distance_m = LaunchConfiguration('start_heading_bias_distance_m')
    lidar_in_base_x = LaunchConfiguration('lidar_in_base_x')
    lidar_in_base_y = LaunchConfiguration('lidar_in_base_y')
    lidar_in_base_yaw_deg = LaunchConfiguration('lidar_in_base_yaw_deg')
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
    camera_auto_pan_enabled = LaunchConfiguration('camera_auto_pan_enabled')
    camera_pan_center_us = LaunchConfiguration('camera_pan_center_us')
    camera_pan_min_us = LaunchConfiguration('camera_pan_min_us')
    camera_pan_max_us = LaunchConfiguration('camera_pan_max_us')
    camera_pan_max_angle_deg = LaunchConfiguration('camera_pan_max_angle_deg')
    camera_pan_active_radius_m = LaunchConfiguration('camera_pan_active_radius_m')
    camera_pan_idle_center_period_sec = LaunchConfiguration(
        'camera_pan_idle_center_period_sec'
    )
    camera_tilt_center_us = LaunchConfiguration('camera_tilt_center_us')
    plant_bbox_servo_enabled = LaunchConfiguration('plant_bbox_servo_enabled')
    plant_capture_radius_m = LaunchConfiguration('plant_capture_radius_m')
    plant_positive_save_gate_topic = LaunchConfiguration('plant_positive_save_gate_topic')
    plant_detection_enable_radius = LaunchConfiguration('plant_detection_enable_radius')
    plant_detections_topic = LaunchConfiguration('plant_detections_topic')
    unknown_plant_capture_topic = LaunchConfiguration('unknown_plant_capture_topic')
    unknown_plant_known_merge_radius_m = LaunchConfiguration(
        'unknown_plant_known_merge_radius_m'
    )
    explored_map_topic = LaunchConfiguration('explored_map_topic')
    exploration_summary_topic = LaunchConfiguration('exploration_summary_topic')
    camera_reveal_half_angle_deg = LaunchConfiguration('camera_reveal_half_angle_deg')
    fog_scan_max_range_m = LaunchConfiguration('fog_scan_max_range_m')
    mark_static_prior_seen_on_start = LaunchConfiguration(
        'mark_static_prior_seen_on_start'
    )
    fog_inspection_status_topic = LaunchConfiguration('fog_inspection_status_topic')
    fog_slow_scale = LaunchConfiguration('fog_slow_scale')
    fog_attention_max_distance_m = LaunchConfiguration('fog_attention_max_distance_m')
    fog_hold_trigger_distance_m = LaunchConfiguration('fog_hold_trigger_distance_m')
    fog_pre_hold_decel_enabled = LaunchConfiguration('fog_pre_hold_decel_enabled')
    fog_pre_hold_decel_scale = LaunchConfiguration('fog_pre_hold_decel_scale')
    fog_pre_hold_decel_duration_sec = LaunchConfiguration(
        'fog_pre_hold_decel_duration_sec'
    )
    fog_hold_duration_sec = LaunchConfiguration('fog_hold_duration_sec')
    fog_cooldown_sec = LaunchConfiguration('fog_cooldown_sec')
    fog_min_frontier_area_m2 = LaunchConfiguration('fog_min_frontier_area_m2')
    fog_camera_pan_max_angle_deg = LaunchConfiguration('fog_camera_pan_max_angle_deg')
    fog_camera_command_period_sec = LaunchConfiguration('fog_camera_command_period_sec')
    plant_completion_confidence_threshold = LaunchConfiguration(
        'plant_completion_confidence_threshold'
    )
    plant_completion_max_stop_distance_m = LaunchConfiguration(
        'plant_completion_max_stop_distance_m'
    )
    plant_completion_match_mode = LaunchConfiguration('plant_completion_match_mode')
    plant_completion_require_valid_capture = LaunchConfiguration(
        'plant_completion_require_valid_capture'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('launch_plant_detector', default_value='false'),
        DeclareLaunchArgument('launch_planning', default_value='false'),
        DeclareLaunchArgument('launch_control', default_value='false'),
        DeclareLaunchArgument('launch_map_viewer', default_value='false'),
        DeclareLaunchArgument('launch_exploration_map', default_value='true'),
        DeclareLaunchArgument('launch_system_status', default_value='true'),
        DeclareLaunchArgument('launch_aruco_detector', default_value='true'),
        DeclareLaunchArgument('launch_fused_estimator', default_value='true'),
        DeclareLaunchArgument('launch_rplidar_driver', default_value='false'),
        DeclareLaunchArgument('launch_lidar_wall_localizer', default_value='true'),
        DeclareLaunchArgument('launch_lidar_scan_match_localizer', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_PLANT_MODEL_PATH),
        DeclareLaunchArgument('run_tag', default_value=''),
        DeclareLaunchArgument(
            'localisation_params_file',
            default_value=DEFAULT_LOCALISATION_PARAMS_PATH,
        ),
        DeclareLaunchArgument(
            'control_params_file',
            default_value=DEFAULT_CONTROLLER_PARAMS_PATH,
        ),
        DeclareLaunchArgument('plant_detector_start_enabled', default_value='true'),
        DeclareLaunchArgument('plant_save_positive_images', default_value='true'),
        DeclareLaunchArgument('plant_publish_debug_image', default_value='false'),
        DeclareLaunchArgument(
            'plant_positive_image_dir',
            default_value='data/plant_dataset/positive',
        ),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_fps', default_value='10'),
        DeclareLaunchArgument('roboclaw_usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('initial_x', default_value='0.50'),
        DeclareLaunchArgument('initial_y', default_value='0.40'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('path_controller_backend', default_value='direct'),
        DeclareLaunchArgument('backend_delta_v_limit', default_value='0.30'),
        DeclareLaunchArgument('backend_delta_omega_limit', default_value='1.50'),
        DeclareLaunchArgument('lqg_smooth_turn_enabled', default_value='true'),
        DeclareLaunchArgument('lqg_smooth_turn_angle_deg', default_value='30.0'),
        DeclareLaunchArgument('lqg_stop_turn_angle_deg', default_value='60.0'),
        DeclareLaunchArgument('lqg_min_smooth_turn_speed_scale', default_value='0.45'),
        DeclareLaunchArgument('nominal_speed', default_value='0.20'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.35'),
        DeclareLaunchArgument('duty_cycle_limit', default_value='40.0'),
        DeclareLaunchArgument('min_moving_duty', default_value='18.0'),
        DeclareLaunchArgument('max_wheel_speed_at_full_duty', default_value='1.50'),
        DeclareLaunchArgument('duty_slew_rate_percent_per_sec', default_value='55.0'),
        DeclareLaunchArgument('graceful_stop_enabled', default_value='true'),
        DeclareLaunchArgument(
            'graceful_stop_decel_duty_per_sec',
            default_value='35.0',
        ),
        DeclareLaunchArgument(
            'graceful_stop_turn_duty_per_sec',
            default_value='120.0',
        ),
        DeclareLaunchArgument('command_filter_alpha', default_value='0.0'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.45'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.18'),
        DeclareLaunchArgument('slowdown_distance', default_value='0.70'),
        DeclareLaunchArgument('curvature_gain', default_value='1.0'),
        DeclareLaunchArgument('backward_target_turn_direction', default_value='1.0'),
        DeclareLaunchArgument('left_trim', default_value='1.0'),
        DeclareLaunchArgument('right_trim', default_value='0.932'),
        DeclareLaunchArgument('control_verbose', default_value='false'),
        DeclareLaunchArgument('control_odom_topic', default_value='pitt_fused_odometry'),
        DeclareLaunchArgument('map_viewer_odom_topic', default_value='pitt_fused_odometry'),
        DeclareLaunchArgument('map_viewer_show_gui', default_value='false'),
        DeclareLaunchArgument('map_viewer_backend', default_value='Agg'),
        DeclareLaunchArgument('map_viewer_plot_period_sec', default_value='0.25'),
        DeclareLaunchArgument('map_viewer_save_period_sec', default_value='0.75'),
        DeclareLaunchArgument('map_viewer_save_live_png', default_value='true'),
        DeclareLaunchArgument('point_to_point_mode', default_value='true'),
        DeclareLaunchArgument('point_to_point_rotate_tolerance', default_value='0.06'),
        DeclareLaunchArgument('point_to_point_arrival_tolerance', default_value='0.04'),
        DeclareLaunchArgument('preserve_replan_motion_enabled', default_value='true'),
        DeclareLaunchArgument(
            'preserve_replan_heading_tolerance_deg',
            default_value='12.0',
        ),
        DeclareLaunchArgument(
            'preserve_replan_start_tolerance_m',
            default_value='0.45',
        ),
        DeclareLaunchArgument('corner_turn_max_duty', default_value='12.0'),
        DeclareLaunchArgument('segment_straight_duty', default_value='20.0'),
        DeclareLaunchArgument('segment_min_straight_duty', default_value='15.0'),
        DeclareLaunchArgument('segment_yaw_kp', default_value='14.0'),
        DeclareLaunchArgument('segment_max_correction', default_value='4.0'),
        DeclareLaunchArgument('segment_lateral_kp', default_value='0.0'),
        DeclareLaunchArgument('segment_lateral_deadband_m', default_value='0.03'),
        DeclareLaunchArgument('segment_lateral_max_heading_deg', default_value='18.0'),
        DeclareLaunchArgument('segment_deceleration_distance', default_value='0.70'),
        DeclareLaunchArgument('segment_velocity_profile_enabled', default_value='false'),
        DeclareLaunchArgument('segment_profile_cruise_duty', default_value='35.0'),
        DeclareLaunchArgument('segment_profile_approach_duty', default_value='25.0'),
        DeclareLaunchArgument('segment_profile_accel_duty_per_sec', default_value='50.0'),
        DeclareLaunchArgument('marker_world_map', default_value=FINAL_DEMO_MARKER_WORLD_MAP),
        DeclareLaunchArgument('launch_obstacle_filter', default_value='false'),
        DeclareLaunchArgument('launch_fog_inspection_filter', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('lidar_scan_angle_multiplier', default_value='1.0'),
        DeclareLaunchArgument('lidar_avoid_distance_m', default_value='0.55'),
        DeclareLaunchArgument('lidar_clear_distance_m', default_value='0.75'),
        DeclareLaunchArgument('lidar_pre_stop_decel_enabled', default_value='true'),
        DeclareLaunchArgument('lidar_pre_stop_decel_scale', default_value='0.70'),
        DeclareLaunchArgument('lidar_pre_stop_decel_distance_m', default_value='0.60'),
        DeclareLaunchArgument('lidar_steer_emergency_stop_distance_m', default_value='0.0'),
        DeclareLaunchArgument('lidar_front_recovery_enabled', default_value='false'),
        DeclareLaunchArgument('lidar_front_recovery_trigger_distance_m', default_value='0.0'),
        DeclareLaunchArgument('lidar_front_recovery_distance_m', default_value='0.20'),
        DeclareLaunchArgument('lidar_front_recovery_reverse_duty', default_value='22.0'),
        DeclareLaunchArgument('lidar_front_recovery_timeout_sec', default_value='1.5'),
        DeclareLaunchArgument(
            'lidar_front_recovery_steer_duration_sec',
            default_value='0.8',
        ),
        DeclareLaunchArgument('lidar_wall_odom_topic', default_value='pitt_lidar_wall_odometry'),
        DeclareLaunchArgument(
            'lidar_scanmatch_odom_topic',
            default_value='pitt_lidar_scanmatch_odometry',
        ),
        DeclareLaunchArgument('use_lidar_wall_update', default_value='true'),
        DeclareLaunchArgument('use_lidar_scanmatch_update', default_value='false'),
        DeclareLaunchArgument(
            'dynamic_obstacle_marker_topic',
            default_value='dynamic_obstacle_marker',
        ),
        DeclareLaunchArgument('plant_goal_standoff_m', default_value='1.0'),
        DeclareLaunchArgument('replan_cooldown_sec', default_value='2.0'),
        DeclareLaunchArgument('turn_start_cost_m', default_value='0.25'),
        DeclareLaunchArgument('obstacle_clearance_radius_m', default_value='0.30'),
        DeclareLaunchArgument('obstacle_clearance_weight_m', default_value='3.0'),
        DeclareLaunchArgument('hard_obstacle_clearance_m', default_value='0.0'),
        DeclareLaunchArgument('fog_path_preference', default_value='1.00'),
        DeclareLaunchArgument('dynamic_obstacle_ttl_sec', default_value='1.0'),
        DeclareLaunchArgument('dynamic_obstacle_static_filter_m', default_value='0.0'),
        DeclareLaunchArgument('dynamic_obstacle_beam_stride', default_value='4'),
        DeclareLaunchArgument('dynamic_obstacle_max_points', default_value='1500'),
        DeclareLaunchArgument('dynamic_obstacle_marker_latest_only', default_value='true'),
        DeclareLaunchArgument('start_heading_bias_m', default_value='0.15'),
        DeclareLaunchArgument('start_heading_bias_distance_m', default_value='0.70'),
        DeclareLaunchArgument('lidar_in_base_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_in_base_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_in_base_yaw_deg', default_value='0.0'),
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
        DeclareLaunchArgument('lidar_verbose', default_value='false'),
        DeclareLaunchArgument('camera_auto_pan_enabled', default_value='true'),
        DeclareLaunchArgument('camera_pan_center_us', default_value='1548'),
        DeclareLaunchArgument('camera_pan_min_us', default_value='500'),
        DeclareLaunchArgument('camera_pan_max_us', default_value='2500'),
        DeclareLaunchArgument('camera_pan_max_angle_deg', default_value='180.0'),
        DeclareLaunchArgument('camera_pan_active_radius_m', default_value='1.60'),
        DeclareLaunchArgument('camera_pan_idle_center_period_sec', default_value='0.0'),
        DeclareLaunchArgument('camera_tilt_center_us', default_value='1400'),
        DeclareLaunchArgument('plant_bbox_servo_enabled', default_value='true'),
        DeclareLaunchArgument('plant_capture_radius_m', default_value='0.60'),
        DeclareLaunchArgument(
            'plant_positive_save_gate_topic',
            default_value='/asc/plant_detector/save_positive_enabled',
        ),
        DeclareLaunchArgument('plant_detection_enable_radius', default_value='0.60'),
        DeclareLaunchArgument('plant_detections_topic', default_value='/asc/plant_detections'),
        DeclareLaunchArgument(
            'unknown_plant_capture_topic',
            default_value='unknown_plant_capture',
        ),
        DeclareLaunchArgument('unknown_plant_known_merge_radius_m', default_value='1.00'),
        DeclareLaunchArgument('explored_map_topic', default_value='explored_map'),
        DeclareLaunchArgument('exploration_summary_topic', default_value='exploration_summary'),
        DeclareLaunchArgument('camera_reveal_half_angle_deg', default_value='30.0'),
        DeclareLaunchArgument('fog_scan_max_range_m', default_value='5.0'),
        DeclareLaunchArgument('mark_static_prior_seen_on_start', default_value='true'),
        DeclareLaunchArgument(
            'fog_inspection_status_topic',
            default_value='fog_inspection/status',
        ),
        DeclareLaunchArgument('fog_slow_scale', default_value='0.30'),
        DeclareLaunchArgument('fog_attention_max_distance_m', default_value='2.50'),
        DeclareLaunchArgument('fog_hold_trigger_distance_m', default_value='1.20'),
        DeclareLaunchArgument('fog_pre_hold_decel_enabled', default_value='true'),
        DeclareLaunchArgument('fog_pre_hold_decel_scale', default_value='0.70'),
        DeclareLaunchArgument('fog_pre_hold_decel_duration_sec', default_value='0.40'),
        DeclareLaunchArgument('fog_hold_duration_sec', default_value='1.80'),
        DeclareLaunchArgument('fog_cooldown_sec', default_value='1.00'),
        DeclareLaunchArgument('fog_min_frontier_area_m2', default_value='0.04'),
        DeclareLaunchArgument('fog_camera_pan_max_angle_deg', default_value='0.0'),
        DeclareLaunchArgument('fog_camera_command_period_sec', default_value='0.10'),
        DeclareLaunchArgument('plant_completion_confidence_threshold', default_value='0.60'),
        DeclareLaunchArgument('plant_completion_max_stop_distance_m', default_value='1.60'),
        DeclareLaunchArgument('plant_completion_match_mode', default_value='any'),
        DeclareLaunchArgument('plant_completion_require_valid_capture', default_value='true'),

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
                'launch_aruco_detector': launch_aruco_detector,
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_fps': LaunchConfiguration('camera_fps'),
                'publish_camera_images': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            _package_launch_file('rplidar_ros', 'rplidar_a1_launch.py'),
            condition=IfCondition(launch_rplidar_driver),
        ),

        IncludeLaunchDescription(
            _launch_file('localisation.launch.py'),
            launch_arguments={
                'namespace': namespace,
                'launch_encoder_odometry': 'true',
                'launch_fused_estimator': launch_fused_estimator,
                'localisation_params_file': localisation_params_file,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_yaw': initial_yaw,
                'marker_world_map': LaunchConfiguration('marker_world_map'),
                'launch_lidar_wall_localizer': launch_lidar_wall_localizer,
                'launch_lidar_scan_match_localizer': launch_lidar_scan_match_localizer,
                'scan_topic': scan_topic,
                'lidar_wall_odom_topic': lidar_wall_odom_topic,
                'lidar_scanmatch_odom_topic': lidar_scanmatch_odom_topic,
                'lidar_scanmatch_prior_odom_topic': control_odom_topic,
                'use_lidar_wall_update': use_lidar_wall_update,
                'use_lidar_scanmatch_update': use_lidar_scanmatch_update,
                'lidar_in_base_x': lidar_in_base_x,
                'lidar_in_base_y': lidar_in_base_y,
                'lidar_in_base_yaw_deg': lidar_in_base_yaw_deg,
                'lidar_scan_angle_multiplier': lidar_scan_angle_multiplier,
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
                'publish_debug_image': plant_publish_debug_image,
                'save_positive_images': plant_save_positive_images,
                'save_positive_requires_valid_capture': 'false',
                'positive_image_dir': plant_positive_image_dir,
                'positive_save_gate_topic': plant_positive_save_gate_topic,
            }.items(),
        ),

        IncludeLaunchDescription(
            _launch_file('planning.launch.py'),
            condition=IfCondition(launch_planning),
            launch_arguments={
                'namespace': namespace,
                'odom_topic': control_odom_topic,
                'scan_topic': scan_topic,
                'lidar_scan_angle_multiplier': lidar_scan_angle_multiplier,
                'dynamic_obstacle_marker_topic': dynamic_obstacle_marker_topic,
                'explored_map_topic': explored_map_topic,
                'mission_progress_topic': 'mission_progress',
                'plant_goal_standoff_m': plant_goal_standoff_m,
                'replan_cooldown_sec': replan_cooldown_sec,
                'turn_start_cost_m': turn_start_cost_m,
                'obstacle_clearance_radius_m': obstacle_clearance_radius_m,
                'obstacle_clearance_weight_m': obstacle_clearance_weight_m,
                'hard_obstacle_clearance_m': hard_obstacle_clearance_m,
                'fog_path_preference': fog_path_preference,
                'dynamic_obstacle_ttl_sec': dynamic_obstacle_ttl_sec,
                'dynamic_obstacle_static_filter_m': dynamic_obstacle_static_filter_m,
                'dynamic_obstacle_beam_stride': dynamic_obstacle_beam_stride,
                'dynamic_obstacle_max_points': dynamic_obstacle_max_points,
                'dynamic_obstacle_marker_latest_only': dynamic_obstacle_marker_latest_only,
                'start_heading_bias_m': start_heading_bias_m,
                'start_heading_bias_distance_m': start_heading_bias_distance_m,
                'lidar_in_base_x': lidar_in_base_x,
                'lidar_in_base_y': lidar_in_base_y,
                'lidar_in_base_yaw_deg': lidar_in_base_yaw_deg,
                'fast_global_ordering': fast_global_ordering,
                'global_order_grid_stride': global_order_grid_stride,
                'include_coverage_stops': include_coverage_stops,
                'fog_cleanup_enabled': fog_cleanup_enabled,
                'fog_cleanup_trigger_ratio': fog_cleanup_trigger_ratio,
                'fog_cleanup_min_cluster_ratio': fog_cleanup_min_cluster_ratio,
                'fog_cleanup_max_cluster_ratio': fog_cleanup_max_cluster_ratio,
                'fog_cleanup_max_goals': fog_cleanup_max_goals,
                'planned_coverage_enabled': planned_coverage_enabled,
                'planned_coverage_radius_m': planned_coverage_radius_m,
                'planned_coverage_use_line_of_sight': planned_coverage_use_line_of_sight,
                'planned_coverage_sample_step_m': planned_coverage_sample_step_m,
                'planned_coverage_min_cluster_area_m2': planned_coverage_min_cluster_area_m2,
                'planned_coverage_max_goals': planned_coverage_max_goals,
                'planned_coverage_goal_tolerance_m': planned_coverage_goal_tolerance_m,
            }.items(),
        ),

        Node(
            package='asclinic_pkg',
            executable='semantic_exploration_mapper.py',
            namespace=namespace,
            name='semantic_exploration_mapper',
            output='screen',
            condition=IfCondition(launch_exploration_map),
            parameters=[{
                'odom_topic': control_odom_topic,
                'scan_topic': scan_topic,
                'aruco_topic': 'aruco_detections',
                'plant_detections_topic': plant_detections_topic,
                'unknown_plant_capture_topic': unknown_plant_capture_topic,
                'unknown_plant_known_merge_radius_m': unknown_plant_known_merge_radius_m,
                'camera_servo_topic': 'set_servo_pulse_width',
                'explored_map_topic': explored_map_topic,
                'exploration_summary_topic': exploration_summary_topic,
                'lidar_scan_angle_multiplier': lidar_scan_angle_multiplier,
                'lidar_in_base_yaw_deg': lidar_in_base_yaw_deg,
                'plant_bbox_image_width': 1920.0,
                'plant_bbox_fx_px': 1429.62,
                'camera_reveal_half_angle_deg': camera_reveal_half_angle_deg,
                'scan_max_range_m': fog_scan_max_range_m,
                'mark_static_prior_seen_on_start': mark_static_prior_seen_on_start,
                'camera_pan_center_us': camera_pan_center_us,
                'camera_pan_min_us': camera_pan_min_us,
                'camera_pan_max_us': camera_pan_max_us,
                'camera_pan_max_angle_deg': camera_pan_max_angle_deg,
            }],
        ),

        IncludeLaunchDescription(
            _launch_file('control.launch.py'),
            condition=IfCondition(launch_control),
            launch_arguments={
                'namespace': namespace,
                'control_params_file': control_params_file,
                'odom_topic': control_odom_topic,
                'aruco_topic': 'aruco_detections',
                'mission_progress_topic': 'mission_progress',
                'plant_detections_topic': plant_detections_topic,
                'unknown_plant_capture_topic': unknown_plant_capture_topic,
                'plant_completion_confidence_threshold': plant_completion_confidence_threshold,
                'plant_completion_max_stop_distance_m': plant_completion_max_stop_distance_m,
                'plant_completion_match_mode': plant_completion_match_mode,
                'plant_completion_require_valid_capture': plant_completion_require_valid_capture,
                'path_controller_backend': path_controller_backend,
                'backend_delta_v_limit': backend_delta_v_limit,
                'backend_delta_omega_limit': backend_delta_omega_limit,
                'lqg_smooth_turn_enabled': lqg_smooth_turn_enabled,
                'lqg_smooth_turn_angle_deg': lqg_smooth_turn_angle_deg,
                'lqg_stop_turn_angle_deg': lqg_stop_turn_angle_deg,
                'lqg_min_smooth_turn_speed_scale': lqg_min_smooth_turn_speed_scale,
                'nominal_speed': nominal_speed,
                'max_linear_speed': max_linear_speed,
                'duty_cycle_limit': duty_cycle_limit,
                'min_moving_duty': min_moving_duty,
                'max_wheel_speed_at_full_duty': max_wheel_speed_at_full_duty,
                'duty_slew_rate_percent_per_sec': duty_slew_rate,
                'graceful_stop_enabled': graceful_stop_enabled,
                'graceful_stop_decel_duty_per_sec': graceful_stop_decel_duty_per_sec,
                'graceful_stop_turn_duty_per_sec': graceful_stop_turn_duty_per_sec,
                'command_filter_alpha': command_filter_alpha,
                'lookahead_distance': lookahead_distance,
                'goal_tolerance': goal_tolerance,
                'slowdown_distance': slowdown_distance,
                'curvature_gain': curvature_gain,
                'backward_target_turn_direction': backward_target_turn_direction,
                'left_trim': left_trim,
                'right_trim': right_trim,
                'point_to_point_mode': point_to_point_mode,
                'point_to_point_rotate_tolerance': point_to_point_rotate_tolerance,
                'point_to_point_arrival_tolerance': point_to_point_arrival_tolerance,
                'preserve_replan_motion_enabled': preserve_replan_motion_enabled,
                'preserve_replan_heading_tolerance_deg': (
                    preserve_replan_heading_tolerance_deg
                ),
                'preserve_replan_start_tolerance_m': preserve_replan_start_tolerance_m,
                'corner_turn_max_duty': corner_turn_max_duty,
                'segment_straight_duty': segment_straight_duty,
                'segment_min_straight_duty': segment_min_straight_duty,
                'segment_yaw_kp': segment_yaw_kp,
                'segment_max_correction': segment_max_correction,
                'segment_lateral_kp': segment_lateral_kp,
                'segment_lateral_deadband_m': segment_lateral_deadband_m,
                'segment_lateral_max_heading_deg': segment_lateral_max_heading_deg,
                'segment_deceleration_distance': segment_deceleration_distance,
                'segment_velocity_profile_enabled': segment_velocity_profile_enabled,
                'segment_profile_cruise_duty': segment_profile_cruise_duty,
                'segment_profile_approach_duty': segment_profile_approach_duty,
                'segment_profile_accel_duty_per_sec': segment_profile_accel_duty_per_sec,
                'verbose': control_verbose,
                'launch_obstacle_filter': launch_obstacle_filter,
                'launch_fog_inspection_filter': launch_fog_inspection_filter,
                'scan_topic': scan_topic,
                'lidar_in_base_yaw_deg': lidar_in_base_yaw_deg,
                'lidar_scan_angle_multiplier': lidar_scan_angle_multiplier,
                'lidar_avoid_distance_m': lidar_avoid_distance_m,
                'lidar_clear_distance_m': lidar_clear_distance_m,
                'lidar_pre_stop_decel_enabled': lidar_pre_stop_decel_enabled,
                'lidar_pre_stop_decel_scale': lidar_pre_stop_decel_scale,
                'lidar_pre_stop_decel_distance_m': lidar_pre_stop_decel_distance_m,
                'lidar_steer_emergency_stop_distance_m': (
                    lidar_steer_emergency_stop_distance_m
                ),
                'lidar_front_recovery_enabled': lidar_front_recovery_enabled,
                'lidar_front_recovery_trigger_distance_m': (
                    lidar_front_recovery_trigger_distance_m
                ),
                'lidar_front_recovery_distance_m': lidar_front_recovery_distance_m,
                'lidar_front_recovery_reverse_duty': lidar_front_recovery_reverse_duty,
                'lidar_front_recovery_timeout_sec': lidar_front_recovery_timeout_sec,
                'lidar_front_recovery_steer_duration_sec': (
                    lidar_front_recovery_steer_duration_sec
                ),
                'lidar_verbose': LaunchConfiguration('lidar_verbose'),
                'camera_auto_pan_enabled': camera_auto_pan_enabled,
                'camera_pan_center_us': camera_pan_center_us,
                'camera_pan_min_us': camera_pan_min_us,
                'camera_pan_max_us': camera_pan_max_us,
                'camera_pan_max_angle_deg': camera_pan_max_angle_deg,
                'camera_pan_active_radius_m': camera_pan_active_radius_m,
                'camera_pan_idle_center_period_sec': camera_pan_idle_center_period_sec,
                'camera_tilt_center_us': camera_tilt_center_us,
                'plant_bbox_servo_enabled': plant_bbox_servo_enabled,
                'plant_capture_radius_m': plant_capture_radius_m,
                'inspection_standoff_distance_m': plant_goal_standoff_m,
                'plant_positive_save_gate_topic': plant_positive_save_gate_topic,
                'plant_detection_enable_radius': plant_detection_enable_radius,
                'explored_map_topic': explored_map_topic,
                'fog_inspection_status_topic': fog_inspection_status_topic,
                'fog_slow_scale': fog_slow_scale,
                'fog_attention_max_distance_m': fog_attention_max_distance_m,
                'fog_hold_trigger_distance_m': fog_hold_trigger_distance_m,
                'fog_pre_hold_decel_enabled': fog_pre_hold_decel_enabled,
                'fog_pre_hold_decel_scale': fog_pre_hold_decel_scale,
                'fog_pre_hold_decel_duration_sec': fog_pre_hold_decel_duration_sec,
                'fog_hold_duration_sec': fog_hold_duration_sec,
                'fog_cooldown_sec': fog_cooldown_sec,
                'fog_min_frontier_area_m2': fog_min_frontier_area_m2,
                'fog_camera_pan_max_angle_deg': fog_camera_pan_max_angle_deg,
                'fog_camera_command_period_sec': fog_camera_command_period_sec,
            }.items(),
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
            additional_env={
                'MPLBACKEND': map_viewer_backend,
            },
            parameters=[{
                'odom_topic': map_viewer_odom_topic,
                'wheel_odom_topic': 'wheel_odometry',
                'vision_odom_topic': 'pitt_vision_odometry',
                'lidar_wall_odom_topic': lidar_wall_odom_topic,
                'lidar_scanmatch_odom_topic': lidar_scanmatch_odom_topic,
                'path_topic': 'reference_path',
                'stop_poses_topic': 'plant_stop_poses',
                'dynamic_obstacle_marker_topic': dynamic_obstacle_marker_topic,
                'system_status_topic': 'jetson_status',
                'explored_map_topic': explored_map_topic,
                'exploration_summary_topic': exploration_summary_topic,
                'save_directory': '~/asclinic-ros2/ros2_ws/results/live_global_map',
                'show_gui': map_viewer_show_gui,
                'save_live_png': map_viewer_save_live_png,
                'plot_period_sec': map_viewer_plot_period_sec,
                'save_period_sec': map_viewer_save_period_sec,
                'run_tag': ParameterValue(run_tag, value_type=str),
            }],
        ),
    ])
