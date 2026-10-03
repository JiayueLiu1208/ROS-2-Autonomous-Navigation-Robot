#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_MODEL_PATH = PathJoinSubstitution(
    [FindPackageShare('asclinic_pkg'), 'models', 'plant_detector', 'best.pt']
)


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    enabled_topic = LaunchConfiguration('enabled_topic')
    start_enabled = LaunchConfiguration('start_enabled')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    save_positive_images = LaunchConfiguration('save_positive_images')
    save_positive_requires_valid_capture = LaunchConfiguration(
        'save_positive_requires_valid_capture'
    )
    positive_image_dir = LaunchConfiguration('positive_image_dir')
    positive_save_gate_topic = LaunchConfiguration('positive_save_gate_topic')
    device = LaunchConfiguration('device')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    inference_rate_hz = LaunchConfiguration('inference_rate_hz')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_capture = LaunchConfiguration('launch_capture')
    capture_mode = LaunchConfiguration('capture_mode')
    output_dir = LaunchConfiguration('output_dir')

    default_params = PathJoinSubstitution(
        [FindPackageShare('asclinic_pkg'), 'config', 'plant_detector_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='asc'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL_PATH),
        DeclareLaunchArgument('image_topic', default_value='/asc/camera_image'),
        DeclareLaunchArgument('enabled_topic', default_value='/asc/plant_detector/enabled'),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        DeclareLaunchArgument('publish_debug_image', default_value='false'),
        DeclareLaunchArgument('save_positive_images', default_value='false'),
        DeclareLaunchArgument(
            'save_positive_requires_valid_capture',
            default_value='false',
        ),
        DeclareLaunchArgument('positive_image_dir', default_value='data/plant_dataset/positive'),
        DeclareLaunchArgument('positive_save_gate_topic', default_value=''),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('inference_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('launch_camera', default_value='false'),
        DeclareLaunchArgument('launch_capture', default_value='false'),
        DeclareLaunchArgument('capture_mode', default_value='manual'),
        DeclareLaunchArgument('output_dir', default_value='data/plant_dataset/raw'),

        Node(
            package='asclinic_pkg',
            executable='camera_capture.py',
            namespace=namespace,
            name='camera_capture',
            output='screen',
            condition=IfCondition(launch_camera),
            parameters=[{
                'camera_capture_should_publish_camera_images': True,
                'camera_capture_should_show_camera_images': False,
                'camera_capture_should_save_all_chessboard_images': False,
            }],
        ),

        Node(
            package='asclinic_pkg',
            executable='plant_detector_node',
            namespace=namespace,
            name='plant_detector_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'model_path': model_path,
                    'image_topic': image_topic,
                    'enabled_topic': enabled_topic,
                    'start_enabled': start_enabled,
                    'publish_debug_image': publish_debug_image,
                    'save_positive_images': save_positive_images,
                    'save_positive_requires_valid_capture': save_positive_requires_valid_capture,
                    'positive_image_dir': positive_image_dir,
                    'positive_save_gate_topic': positive_save_gate_topic,
                    'device': device,
                    'confidence_threshold': confidence_threshold,
                    'inference_rate_hz': inference_rate_hz,
                },
            ],
        ),

        Node(
            package='asclinic_pkg',
            executable='plant_image_capture_node',
            namespace=namespace,
            name='plant_image_capture_node',
            output='screen',
            condition=IfCondition(launch_capture),
            parameters=[
                params_file,
                {
                    'image_topic': image_topic,
                    'capture_mode': capture_mode,
                    'output_dir': output_dir,
                },
            ],
        ),
    ])
