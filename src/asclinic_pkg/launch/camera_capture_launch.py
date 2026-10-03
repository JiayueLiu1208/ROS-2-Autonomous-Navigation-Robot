#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Namespace and node/executable
    namespace    = "asc"
    asclinic_pkg = "asclinic_pkg"
    node_exec    = "camera_capture.py"

    # --- Parameters (LaunchConfigurations) ---
    verbosity     = LaunchConfiguration('camera_capture_verbosity',                         default='1')
    device_number = LaunchConfiguration('camera_capture_usb_camera_device_number',         default='0')
    frame_height  = LaunchConfiguration('camera_capture_desired_camera_frame_height',      default='1080')
    frame_width   = LaunchConfiguration('camera_capture_desired_camera_frame_width',       default='1920')
    cam_fps       = LaunchConfiguration('camera_capture_desired_camera_fps',               default='5')
    chess_h       = LaunchConfiguration('camera_capture_chessboard_size_height',           default='9')
    chess_w       = LaunchConfiguration('camera_capture_chessboard_size_width',            default='6')
    img_path      = LaunchConfiguration('camera_capture_save_image_path',                  default='saved_camera_images/')
    save_all      = LaunchConfiguration('camera_capture_should_save_all_chessboard_images',default='True')
    pub_all       = LaunchConfiguration('camera_capture_should_publish_camera_images',     default='True')
    show_all      = LaunchConfiguration('camera_capture_should_show_camera_images',        default='False')

    # --- Declare launch arguments ---
    declare_args = [
        DeclareLaunchArgument('camera_capture_verbosity',                         default_value=verbosity),
        DeclareLaunchArgument('camera_capture_usb_camera_device_number',         default_value=device_number),
        DeclareLaunchArgument('camera_capture_desired_camera_frame_height',      default_value=frame_height),
        DeclareLaunchArgument('camera_capture_desired_camera_frame_width',       default_value=frame_width),
        DeclareLaunchArgument('camera_capture_desired_camera_fps',               default_value=cam_fps),
        DeclareLaunchArgument('camera_capture_chessboard_size_height',           default_value=chess_h),
        DeclareLaunchArgument('camera_capture_chessboard_size_width',            default_value=chess_w),
        DeclareLaunchArgument('camera_capture_save_image_path',                  default_value=img_path),
        DeclareLaunchArgument('camera_capture_should_save_all_chessboard_images',default_value=save_all),
        DeclareLaunchArgument('camera_capture_should_publish_camera_images',     default_value=pub_all),
        DeclareLaunchArgument('camera_capture_should_show_camera_images',        default_value=show_all),
    ]

    # --- Node definition ---
    camera_node = Node(
        package    = asclinic_pkg,
        executable = node_exec,
        namespace  = namespace,
        name       = "camera_capture",
        output     = "screen",
        parameters = [{
            'camera_capture_verbosity':                         verbosity,
            'camera_capture_usb_camera_device_number':          device_number,
            'camera_capture_desired_camera_frame_height':       frame_height,
            'camera_capture_desired_camera_frame_width':        frame_width,
            'camera_capture_desired_camera_fps':                cam_fps,
            'camera_capture_chessboard_size_height':            chess_h,
            'camera_capture_chessboard_size_width':             chess_w,
            'camera_capture_save_image_path':                   img_path,
            'camera_capture_should_save_all_chessboard_images': save_all,
            'camera_capture_should_publish_camera_images':      pub_all,
            'camera_capture_should_show_camera_images':         show_all,
        }]
    )

    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(camera_node)

    return ld
