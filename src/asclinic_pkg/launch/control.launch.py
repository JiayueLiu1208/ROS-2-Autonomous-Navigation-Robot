#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(value):
    return ParameterValue(value, value_type=float)


def _int(value):
    return ParameterValue(value, value_type=int)


def _bool(value):
    return ParameterValue(value, value_type=bool)


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    control_params_file = LaunchConfiguration('control_params_file')
    default_params = PathJoinSubstitution(
        [FindPackageShare('asclinic_pkg'), 'config', 'controller_params.yaml']
    )
    odom_topic = LaunchConfiguration('odom_topic')
    reference_path_topic = LaunchConfiguration('reference_path_topic')
    motor_cmd_topic = LaunchConfiguration('motor_cmd_topic')
    mission_progress_topic = LaunchConfiguration('mission_progress_topic')
    plant_detector_enable_topic = LaunchConfiguration('plant_detector_enable_topic')
    plant_detections_topic = LaunchConfiguration('plant_detections_topic')
    unknown_plant_capture_topic = LaunchConfiguration('unknown_plant_capture_topic')
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
    aruco_topic = LaunchConfiguration('aruco_topic')
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
    lidar_in_base_yaw_deg = LaunchConfiguration('lidar_in_base_yaw_deg')
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
    explored_map_topic = LaunchConfiguration('explored_map_topic')
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
    inspection_standoff_distance_m = LaunchConfiguration('inspection_standoff_distance_m')
    plant_positive_save_gate_topic = LaunchConfiguration('plant_positive_save_gate_topic')
    plant_detection_enable_radius = LaunchConfiguration('plant_detection_enable_radius')

    # When a downstream filter is in the pipeline the controller publishes to the
    # raw topic; filters then forward toward the motor driver topic.
    effective_motor_cmd_topic = PythonExpression([
        "'set_motor_duty_cycle_raw' if ('", launch_obstacle_filter, "'.lower() == 'true' or '", launch_fog_inspection_filter, "'.lower() == 'true') else '", motor_cmd_topic, "'",
    ])
    obstacle_output_cmd_topic = PythonExpression([
        "'set_motor_duty_cycle_after_obstacle' if '", launch_fog_inspection_filter, "'.lower() == 'true' else '", motor_cmd_topic, "'",
    ])
    fog_input_cmd_topic = PythonExpression([
        "'set_motor_duty_cycle_after_obstacle' if '", launch_obstacle_filter, "'.lower() == 'true' else 'set_motor_duty_cycle_raw'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('control_params_file', default_value=default_params),
        DeclareLaunchArgument('odom_topic', default_value='pitt_fused_odometry'),
        DeclareLaunchArgument('reference_path_topic', default_value='reference_path'),
        DeclareLaunchArgument('motor_cmd_topic', default_value='set_motor_duty_cycle'),
        DeclareLaunchArgument('mission_progress_topic', default_value='mission_progress'),
        DeclareLaunchArgument('aruco_topic', default_value='aruco_detections'),
        DeclareLaunchArgument('nominal_speed', default_value='0.12'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.35'),
        DeclareLaunchArgument('duty_cycle_limit', default_value='28.0'),
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
        DeclareLaunchArgument('point_to_point_mode', default_value='false'),
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
        DeclareLaunchArgument(
            'plant_detector_enable_topic',
            default_value='plant_detector/enabled',
        ),
        DeclareLaunchArgument('plant_detections_topic', default_value='plant_detections'),
        DeclareLaunchArgument(
            'unknown_plant_capture_topic',
            default_value='unknown_plant_capture',
        ),
        DeclareLaunchArgument(
            'plant_completion_confidence_threshold',
            default_value='0.60',
        ),
        DeclareLaunchArgument(
            'plant_completion_max_stop_distance_m',
            default_value='1.60',
        ),
        DeclareLaunchArgument('plant_completion_match_mode', default_value='any'),
        DeclareLaunchArgument(
            'plant_completion_require_valid_capture',
            default_value='true',
        ),
        DeclareLaunchArgument('path_controller_backend', default_value='direct'),
        DeclareLaunchArgument('backend_delta_v_limit', default_value='0.30'),
        DeclareLaunchArgument('backend_delta_omega_limit', default_value='1.50'),
        DeclareLaunchArgument('lqg_smooth_turn_enabled', default_value='true'),
        DeclareLaunchArgument('lqg_smooth_turn_angle_deg', default_value='30.0'),
        DeclareLaunchArgument('lqg_stop_turn_angle_deg', default_value='60.0'),
        DeclareLaunchArgument('lqg_min_smooth_turn_speed_scale', default_value='0.45'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('launch_obstacle_filter', default_value='false'),
        DeclareLaunchArgument('launch_fog_inspection_filter', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('lidar_in_base_yaw_deg', default_value='0.0'),
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
        DeclareLaunchArgument('lidar_verbose', default_value='false'),
        DeclareLaunchArgument('explored_map_topic', default_value='explored_map'),
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
        DeclareLaunchArgument('inspection_standoff_distance_m', default_value='1.0'),
        DeclareLaunchArgument(
            'plant_positive_save_gate_topic',
            default_value='plant_detector/save_positive_enabled',
        ),
        DeclareLaunchArgument('plant_detection_enable_radius', default_value='0.60'),

        Node(
            package='asclinic_pkg',
            executable='tracking_controller_node.py',
            namespace=namespace,
            name='tracking_controller_node',
            output='screen',
            parameters=[
                control_params_file,
                {
                    'odom_topic': odom_topic,
                    'reference_path_topic': reference_path_topic,
                    'motor_cmd_topic': effective_motor_cmd_topic,
                    'mission_progress_topic': mission_progress_topic,
                    'plant_detector_enable_topic': plant_detector_enable_topic,
                    'plant_detections_topic': plant_detections_topic,
                    'unknown_plant_capture_topic': unknown_plant_capture_topic,
                    'plant_completion_confidence_threshold': _float(plant_completion_confidence_threshold),
                    'plant_completion_max_stop_distance_m': _float(plant_completion_max_stop_distance_m),
                    'plant_completion_match_mode': plant_completion_match_mode,
                    'plant_completion_require_valid_capture': _bool(plant_completion_require_valid_capture),
                    'scan_topic': scan_topic,
                    'lidar_in_base_yaw_deg': _float(lidar_in_base_yaw_deg),
                    'lidar_scan_angle_multiplier': _float(lidar_scan_angle_multiplier),
                    'aruco_topic': aruco_topic,
                    'path_controller_backend': path_controller_backend,
                    'backend_delta_v_limit': _float(backend_delta_v_limit),
                    'backend_delta_omega_limit': _float(backend_delta_omega_limit),
                    'lqg_smooth_turn_enabled': _bool(lqg_smooth_turn_enabled),
                    'lqg_smooth_turn_angle_deg': _float(lqg_smooth_turn_angle_deg),
                    'lqg_stop_turn_angle_deg': _float(lqg_stop_turn_angle_deg),
                    'lqg_min_smooth_turn_speed_scale': _float(lqg_min_smooth_turn_speed_scale),
                    'nominal_speed': _float(nominal_speed),
                    'max_linear_speed': _float(max_linear_speed),
                    'duty_cycle_limit': _float(duty_cycle_limit),
                    'min_moving_duty': _float(min_moving_duty),
                    'max_wheel_speed_at_full_duty': _float(max_wheel_speed_at_full_duty),
                    'duty_slew_rate_percent_per_sec': _float(duty_slew_rate),
                    'graceful_stop_enabled': _bool(graceful_stop_enabled),
                    'graceful_stop_decel_duty_per_sec': (
                        _float(graceful_stop_decel_duty_per_sec)
                    ),
                    'graceful_stop_turn_duty_per_sec': _float(graceful_stop_turn_duty_per_sec),
                    'command_filter_alpha': _float(command_filter_alpha),
                    'lookahead_distance': _float(lookahead_distance),
                    'goal_tolerance': _float(goal_tolerance),
                    'slowdown_distance': _float(slowdown_distance),
                    'curvature_gain': _float(curvature_gain),
                    'backward_target_turn_direction': _float(backward_target_turn_direction),
                    'left_trim': _float(left_trim),
                    'right_trim': _float(right_trim),
                    'point_to_point_mode': _bool(point_to_point_mode),
                    'point_to_point_rotate_tolerance': _float(point_to_point_rotate_tolerance),
                    'point_to_point_arrival_tolerance': _float(point_to_point_arrival_tolerance),
                    'preserve_replan_motion_enabled': _bool(preserve_replan_motion_enabled),
                    'preserve_replan_heading_tolerance_deg': (
                        _float(preserve_replan_heading_tolerance_deg)
                    ),
                    'preserve_replan_start_tolerance_m': (
                        _float(preserve_replan_start_tolerance_m)
                    ),
                    'corner_turn_max_duty': _float(corner_turn_max_duty),
                    'segment_straight_duty': _float(segment_straight_duty),
                    'segment_min_straight_duty': _float(segment_min_straight_duty),
                    'segment_yaw_kp': _float(segment_yaw_kp),
                    'segment_max_correction': _float(segment_max_correction),
                    'segment_lateral_kp': _float(segment_lateral_kp),
                    'segment_lateral_deadband_m': _float(segment_lateral_deadband_m),
                    'segment_lateral_max_heading_deg': _float(segment_lateral_max_heading_deg),
                    'segment_deceleration_distance': _float(segment_deceleration_distance),
                    'segment_velocity_profile_enabled': _bool(segment_velocity_profile_enabled),
                    'segment_profile_cruise_duty': _float(segment_profile_cruise_duty),
                    'segment_profile_approach_duty': _float(segment_profile_approach_duty),
                    'segment_profile_accel_duty_per_sec': _float(segment_profile_accel_duty_per_sec),
                    'camera_auto_pan_enabled': _bool(camera_auto_pan_enabled),
                    'camera_pan_center_us': _int(camera_pan_center_us),
                    'camera_pan_min_us': _int(camera_pan_min_us),
                    'camera_pan_max_us': _int(camera_pan_max_us),
                    'camera_pan_max_angle_deg': _float(camera_pan_max_angle_deg),
                    'camera_pan_active_radius_m': _float(camera_pan_active_radius_m),
                    'camera_pan_idle_center_period_sec': _float(
                        camera_pan_idle_center_period_sec
                    ),
                    'camera_tilt_center_us': _int(camera_tilt_center_us),
                    'plant_bbox_servo_enabled': _bool(plant_bbox_servo_enabled),
                    'plant_capture_radius_m': _float(plant_capture_radius_m),
                    'inspection_standoff_distance_m': _float(inspection_standoff_distance_m),
                    'plant_positive_save_gate_topic': plant_positive_save_gate_topic,
                    'plant_detection_enable_radius': _float(plant_detection_enable_radius),
                    'verbose': _bool(LaunchConfiguration('verbose')),
                },
            ],
        ),

        Node(
            package='asclinic_pkg',
            executable='obstacle_stop_motor_filter.py',
            namespace=namespace,
            name='obstacle_stop_motor_filter',
            output='screen',
            condition=IfCondition(launch_obstacle_filter),
            parameters=[{
                'input_cmd_topic': 'set_motor_duty_cycle_raw',
                'output_cmd_topic': obstacle_output_cmd_topic,
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'lidar_in_base_yaw_deg': _float(lidar_in_base_yaw_deg),
                'scan_angle_multiplier': _float(lidar_scan_angle_multiplier),
                'stop_distance_m': _float(lidar_avoid_distance_m),
                'clear_distance_m': _float(lidar_clear_distance_m),
                'front_sector_half_angle_deg': 30.0,
                'stop_on_stale_scan': False,
                'scan_timeout_sec': 0.75,
                'avoidance_mode': 'steer',
                'avoidance_max_steer_duty': 6.0,
                'steer_emergency_stop_distance_m': _float(
                    lidar_steer_emergency_stop_distance_m
                ),
                'front_recovery_enabled': _bool(lidar_front_recovery_enabled),
                'front_recovery_trigger_distance_m': _float(
                    lidar_front_recovery_trigger_distance_m
                ),
                'front_recovery_distance_m': _float(lidar_front_recovery_distance_m),
                'front_recovery_reverse_duty': _float(lidar_front_recovery_reverse_duty),
                'front_recovery_timeout_sec': _float(lidar_front_recovery_timeout_sec),
                'front_recovery_steer_duration_sec': _float(
                    lidar_front_recovery_steer_duration_sec
                ),
                'pre_stop_decel_enabled': _bool(lidar_pre_stop_decel_enabled),
                'pre_stop_decel_scale': _float(lidar_pre_stop_decel_scale),
                'pre_stop_decel_distance_m': _float(lidar_pre_stop_decel_distance_m),
                'latency_compensation_sec': 0.25,
                'forward_speed_m_s': 0.12,
                'lidar_verbose': _bool(LaunchConfiguration('lidar_verbose')),
            }],
        ),

        Node(
            package='asclinic_pkg',
            executable='fog_inspection_motor_filter.py',
            namespace=namespace,
            name='fog_inspection_motor_filter',
            output='screen',
            condition=IfCondition(launch_fog_inspection_filter),
            parameters=[{
                'input_cmd_topic': fog_input_cmd_topic,
                'output_cmd_topic': motor_cmd_topic,
                'odom_topic': odom_topic,
                'explored_map_topic': explored_map_topic,
                'mission_progress_topic': mission_progress_topic,
                'plant_positive_save_gate_topic': plant_positive_save_gate_topic,
                'camera_pan_topic': 'set_servo_pulse_width',
                'status_topic': fog_inspection_status_topic,
                'slow_scale': _float(fog_slow_scale),
                'attention_max_distance_m': _float(fog_attention_max_distance_m),
                'hold_trigger_distance_m': _float(fog_hold_trigger_distance_m),
                'pre_hold_decel_enabled': _bool(fog_pre_hold_decel_enabled),
                'pre_hold_decel_scale': _float(fog_pre_hold_decel_scale),
                'pre_hold_decel_duration_sec': _float(fog_pre_hold_decel_duration_sec),
                'hold_duration_sec': _float(fog_hold_duration_sec),
                'cooldown_sec': _float(fog_cooldown_sec),
                'min_frontier_area_m2': _float(fog_min_frontier_area_m2),
                'camera_pan_center_us': _int(camera_pan_center_us),
                'camera_pan_min_us': _int(camera_pan_min_us),
                'camera_pan_max_us': _int(camera_pan_max_us),
                'camera_pan_max_angle_deg': _float(camera_pan_max_angle_deg),
                'fog_camera_pan_max_angle_deg': _float(fog_camera_pan_max_angle_deg),
                'camera_command_period_sec': _float(fog_camera_command_period_sec),
            }],
        ),
    ])
