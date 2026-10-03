cmake_minimum_required(VERSION 3.10)
project(asclinic_pkg)
# cmake_policy(SET CMP0148 OLD)
# Set C++14 standard
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-comment -Wno-vla)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Generate Message and Service File
set(msg_files
  "msg/FiducialMarker.msg"
  "msg/FiducialMarkerArray.msg"
  "msg/LeftRightFloat32.msg"
  "msg/LeftRightInt32.msg"
  "msg/MissionProgress.msg"
  "msg/PlantDetection.msg"
  "msg/PlantDetections.msg"
  "msg/ReferencePath.msg"
  "msg/RobotState.msg"
  "msg/ServoPulseWidth.msg"
  "msg/TemplateMessage.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs # Add packages that above messages depend on here
)





include_directories(
  include
  src/drivers/src/
  #include/nodes
)

## Create dependency variable for all executables
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  cv_bridge
  OpenCV
  Eigen3
)

# TEMPLATE CPP NODE:
# Creates executable from the C++ source file
add_executable(template_cpp_node_class               src/nodes/template/template_cpp_node_class.cpp)
add_executable(template_cpp_node_minimal             src/nodes/template/template_cpp_node_minimal.cpp)
add_executable(template_gpio_event_triggered         src/nodes/template/template_gpio_event_triggered.cpp)
add_executable(template_gpio_polling                 src/nodes/template/template_gpio_polling.cpp)
add_executable(i2c_for_servos                        src/nodes/hardware/i2c_for_servos.cpp
                                                     src/drivers/src/i2c_driver/i2c_driver.cpp
                                                     src/drivers/src/pca9685/pca9685.cpp)



# Links ROS2 packages (rclcpp, std_msgs, etc.) to the executable
ament_target_dependencies(template_cpp_node_minimal       ${dependencies})
ament_target_dependencies(template_cpp_node_class         ${dependencies})
ament_target_dependencies(template_gpio_event_triggered   ${dependencies})
ament_target_dependencies(template_gpio_polling           ${dependencies})
ament_target_dependencies(i2c_for_servos                  ${dependencies})



# Linking External Library
target_link_libraries(template_gpio_event_triggered   gpiod)
target_link_libraries(template_gpio_polling           gpiod)

# Links ROS2 Messages
# rosidl_target_interfaces() is deprecated since ROS 2 Humble; use
# rosidl_get_typesupport_target() + target_link_libraries() instead.
rosidl_get_typesupport_target(cpp_typesupport_target     ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(template_cpp_node_minimal       "${cpp_typesupport_target}")
target_link_libraries(template_cpp_node_class         "${cpp_typesupport_target}")
target_link_libraries(template_gpio_event_triggered   "${cpp_typesupport_target}")
target_link_libraries(template_gpio_polling           "${cpp_typesupport_target}")
target_link_libraries(i2c_for_servos                  "${cpp_typesupport_target}")


ament_export_dependencies(rosidl_default_runtime)

# Install executable
install(TARGETS 
  template_cpp_node_minimal
  template_cpp_node_class
  template_gpio_event_triggered
  template_gpio_polling
  i2c_for_servos
  DESTINATION lib/${PROJECT_NAME}
)

# Add the Python Node Here
install(PROGRAMS
  src/nodes/perception/camera_capture.py
  src/nodes/control/control_policy_skeleton.py
  src/nodes/control/tracking_controller_node.py
  src/nodes/control/obstacle_stop_motor_filter.py
  src/nodes/control/fog_inspection_motor_filter.py
  src/nodes/localization/liu_odometry_from_encoders.py
  src/nodes/localization/mock_odometry_publisher.py
  src/nodes/localization/lidar_wall_localizer.py
  src/nodes/localization/lidar_scan_match_localizer.py
  src/nodes/experiments/motion/liu_go_straight_distance.py
  src/nodes/experiments/motion/liu_avoidance_odometry_calibration.py
  src/nodes/experiments/motion/liu_turn_odometry_calibration.py
  src/nodes/experiments/motion/liu_rotate_in_place_angle.py
  src/nodes/experiments/motion/liu_rotate_both_wheels_by_angle.py
  src/nodes/experiments/motion/liu_rectangle_trajectory.py
  src/nodes/experiments/motion/liu_rectangle_trajectory2.py
  src/nodes/experiments/motion/liu_square_trajectory.py
  src/nodes/path_planning/global_planner.py
  src/nodes/path_planning/map.py
  src/nodes/path_planning/final_demo_layout.py
  src/nodes/path_planning/preview_route.py
  src/nodes/path_planning/arc_path_publisher.py
  src/nodes/path_planning/straight_path_publisher.py
  src/nodes/path_planning/test_trajectory.py
  src/nodes/path_planning/trajectory_generation.py
  src/nodes/tools/visualization/live_global_map_viewer.py
  src/nodes/tools/visualization/path_tracking_diagnostics.py
  src/nodes/tools/mapping/semantic_exploration_mapper.py
  src/nodes/tools/monitoring/system_status_publisher.py
  src/nodes/tools/marker_map_editor.py
  src/nodes/controllers/lqg_controller_test.py
  src/nodes/localization/pitt_fused_odemetry.py
  src/nodes/hardware/roboclaw_for_motors.py
  src/nodes/aruco_detection/pitt_aruco_calibration_collector.py
  src/nodes/aruco_detection/pitt_aruco_calibration_analysis.py
  src/nodes/aruco_detection/pitt_aruco_subscriber.py
  src/nodes/aruco_detection/pitt_aruco_localization_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch-facing executables without the .py suffix.
install(PROGRAMS
  src/nodes/aruco_detection/aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_detector
)

install(PROGRAMS
  src/nodes/localization/pitt_fused_odemetry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pitt_fused_odemetry
)

install(PROGRAMS
  src/nodes/localization/lidar_wall_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_wall_localizer
)

install(PROGRAMS
  src/nodes/perception/plant_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plant_detector_node
)

install(PROGRAMS
  src/nodes/perception/plant_image_capture_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plant_image_capture_node
)

install(PROGRAMS
  src/nodes/perception/standalone_yolo_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_yolo_tensorrt_camera.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_yolo_tensorrt_camera
)

install(DIRECTORY
  src/nodes/controllers
  src/nodes/control_models
  src/nodes/estimator
  src/nodes/utils
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  OPTIONAL
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/run_arc_path_tuned.sh
  scripts/run_final_lqg_cv.sh
  scripts/run_full_mission.sh
  scripts/run_full_mission_wheel_only.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)



ament_package()
