# ROS 2 Autonomous Navigation Robot

A ROS 2 mobile robot for autonomous plant inspection, combining LiDAR mapping, sensor-fusion localisation, route planning, motion control and YOLO object detection.

The system connects perception to navigation: detections inform inspection behaviour, the planner selects approach routes, and the controller coordinates movement, camera positioning and image capture. Configuration files and calibration utilities are included so the hardware interfaces and navigation parameters can be adapted to another robot.

## Capabilities

- **Mapping:** an asynchronous SLAM Toolbox launch for LiDAR-based mapping, plus a configured room map and a live explored-area layer for the inspection mission.
- **Localisation:** wheel-encoder odometry corrected by camera-based ArUco observations and LiDAR wall measurements in an extended Kalman filter. An optional scan-to-map correction node is also included.
- **Planning:** global plant-visit ordering and heading-aware local A* search, with costs for distance, turns, occupancy and clearance. New LiDAR obstacles can trigger local replanning.
- **Control:** segment-based path tracking with motor duty shaping, turn handling and stop/dwell behaviour. Experimental PID, LQG and MPC backends are retained for comparison.
- **Perception:** YOLO object detection integrated with inspection-image capture, confidence/quality filtering and camera-servo behaviour.
- **Exploration and diagnostics:** coverage-oriented search for unknown targets, live map visualisation, path-tracking diagnostics and onboard resource monitoring.

## Repository layout

```text
src/asclinic_pkg/
  config/       Robot, camera, localisation, controller and SLAM parameters
  launch/       Subsystem, full-mission and experiment launch files
  msg/          Custom ROS 2 interfaces
  scripts/      Mission wrappers, motion tests and YOLO training utilities
  Pitt_A1/      Chessboard capture and camera calibration utilities
  src/drivers/  I2C, servo and encoder hardware helpers
  src/nodes/
    aruco_detection/  Visual localisation and calibration
    localization/    Odometry, EKF and LiDAR corrections
    path_planning/   Maps, global/local planning and path generation
    control/         Tracking and obstacle-response coordination
    controllers/     PID, LQG and MPC implementations
    perception/      Camera, detector and image-capture nodes
    tools/           Mapping, logging, visualisation and monitoring
    experiments/     Motion and odometry calibration experiments
```

Only source code and configuration are distributed. Build/install directories, runtime logs, datasets, captured media, reports, model weights and editor/assistant metadata are excluded. YOLO weights must be supplied separately.

## Platform and dependencies

The original deployment used Linux with **ROS 2 Humble** and an NVIDIA Jetson onboard computer. Hardware includes a differential-drive base with wheel encoders, a RoboClaw motor controller, an RPLIDAR A1, a USB camera and a PCA9685 I2C servo driver. The C++ GPIO examples use the libgpiod 1.x API available in the original Ubuntu 22.04 environment.

Start with a working ROS 2 installation. Install `colcon`, `rosdep` and the package dependencies:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-pip libopencv-dev libeigen3-dev libgpiod-dev \
  ros-humble-slam-toolbox ros-humble-rplidar-ros

# Only needed once on a machine without rosdep initialised:
sudo rosdep init
rosdep update

git clone https://github.com/JiayueLiu1208/ROS-2-Autonomous-Navigation-Robot.git
cd ROS-2-Autonomous-Navigation-Robot
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
python3 -m pip install --user -r requirements.txt
```

OpenCV must include `cv2.aruco` for visual localisation. Use the ROS-compatible OpenCV build for your platform rather than replacing it blindly with a pip wheel. The supplemental requirements intentionally do not install another OpenCV distribution.

For YOLO, install Ultralytics and a PyTorch/Torchvision combination compatible with the target machine. On Jetson, use versions compatible with its JetPack/CUDA release. Supply a trained model explicitly with `--model-path`, or place it locally at `src/asclinic_pkg/models/plant_detector/best.pt` before building. Model files are ignored by Git. The detector's configured class labels must match the supplied model.

## Build

Run from the repository root, which is the colcon workspace:

```bash
colcon build --symlink-install --packages-select asclinic_pkg
source install/setup.bash
```

Model weights are optional at build time. The camera and hardware nodes still require their runtime dependencies and attached devices when launched. The source uses C++14 and Python 3.10-era syntax; other ROS releases have not been validated for this publication.

## Configure the robot

Before running a mission, check:

- RoboClaw and LiDAR device paths, serial settings and device permissions.
- Wheel radius, track width, encoder scale/signs and motor command signs.
- Camera intrinsics, marker dimensions/poses and sensor mounting transforms.
- Map layout, initial pose, plant goals, controller gains, duty limits and clearance settings.

Runtime image outputs default to `saved_camera_images/` or ignored `data/` and `results/` directories. The included calibration values and room map are examples from the original robot, not universal settings.

## Run

List the mission-wrapper options without starting the robot:

```bash
bash src/asclinic_pkg/scripts/run_full_mission.sh --help
ros2 launch asclinic_pkg full_mission.launch.py --show-args
```

After calibrating and connecting the robot, launch navigation without YOLO weights:

```bash
bash src/asclinic_pkg/scripts/run_full_mission.sh \
  --no-build --no-detector --safe-speed --viewer --lidar-stop
```

Enable object detection by supplying your local checkpoint:

```bash
bash src/asclinic_pkg/scripts/run_full_mission.sh \
  --no-build --model-path /absolute/path/to/best.pt \
  --safe-speed --viewer --lidar-stop
```

These mission commands drive the real motors. Use a supervised test area and a functioning emergency stop. `--safe-speed` reduces the configured speed and duty limits; it does not replace calibration or supervision. Add `--map-gui` for an interactive map window; otherwise the viewer writes ignored diagnostic images.

For separate online mapping, first bring up the LiDAR scan stream and encoder odometry, then run:

```bash
ros2 launch asclinic_pkg slam.launch.py
```

This launch expects `/scan` and an `odom -> base_link` transform. It publishes `base_link -> laser` and starts SLAM Toolbox with `config/slam_params.yaml`. Check mounting offsets and avoid duplicate `/map` or TF publishers when integrating it with planning.

## Development and validation

The publication preparation checks Python syntax, shell syntax, YAML/XML parsing, referenced build/install files and credential patterns. It does **not** constitute a fresh ROS build or a new hardware validation run. Full validation requires the target Linux/ROS environment and calibrated robot.

Useful experiment entry points include `straight_line_test.launch.py`, `arc_path_test.launch.py`, `run_arc_path_tuned.sh`, and `run_turn_45_deg.sh` / `run_turn_90_deg.sh` / `run_turn_180_deg.sh`. These are hardware experiments and may command motors. Generated results stay outside version control.

## Acknowledgements

The robot software was developed through collaborative teamwork. The contributions of all team members are gratefully acknowledged.

The repository incorporates existing robot-interface code and node templates. Credit remains with their original authors; copyright notices are preserved in the source files and [LICENSE](LICENSE), and the original package-maintainer attribution is retained in `package.xml`.

The system also uses open-source software from the [ROS 2](https://docs.ros.org/en/humble/), [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox), [OpenCV](https://opencv.org/), [Slamtec RPLIDAR](https://github.com/Slamtec/rplidar_ros/tree/ros2) and [Ultralytics](https://github.com/ultralytics/ultralytics) communities. Their maintainers and contributors are acknowledged for the libraries and tools that support this implementation.

## Licence

The package metadata declares MIT, reproduced in [LICENSE](LICENSE). External dependencies and any separately supplied model weights remain subject to their respective licences.

## References

- [ROS 2 colcon tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS dependency management with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Slamtec RPLIDAR ROS 2 driver](https://github.com/Slamtec/rplidar_ros/tree/ros2)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
