#!/usr/bin/env python3

# Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Python node to detect ArUco markers in the camera images
#
# ----------------------------------------------------------------------------



# ----------------------------------------------------------------------------
# A FEW USEFUL LINKS ABOUT ARUCO MARKER DETECTION
#
# > This link is most similar to the code used in this node:
#   https://aliyasineser.medium.com/aruco-marker-tracking-with-opencv-8cb844c26628
#
# > This online tutorial provides very detailed explanations:
#   https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
#
# > This link is the main Aruco website:
#   https://www.uco.es/investiga/grupos/ava/node/26
#
# > This link is an OpenCV tutorial for detection of ArUco markers:
#   https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
#
# > This link is an OpenCV explanation of the "solvePnP" function:
#   https://docs.opencv.org/4.7.0/d5/d1f/calib3d_solvePnP.html
#
# > As starting point for details about Rodrigues representation of rotations
#   https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
#
# ----------------------------------------------------------------------------


import rclpy
from rclpy.node import Node

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image

# Import asclinic_pkg message types (your custom messages)
from asclinic_pkg.msg import FiducialMarker
from asclinic_pkg.msg import FiducialMarkerArray

import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge, CvBridgeError

# Import numpy
import numpy as np

# DEFINE THE PARAMETERS
# > For the verbosity level of displaying info
DEFAULT_ARUCO_DETECTOR_VERBOSITY = 1
# Note: the levels of increasing verbosity are defined as:
# 0 : Info is not displayed. Warnings and errors are still displayed
# 1 : Startup info is displayed
# 2 : Info about detected Aruco markers is displayed

# > For the number of the USB camera device
#   i.e., for /dev/video0, this parameter should be 0
DEFAULT_USB_CAMERA_DEVICE_NUMBER = 0

# > Properties of the camera images captured
DEFAULT_DESIRED_CAMERA_FRAME_HEIGHT = 1080
DEFAULT_DESIRED_CAMERA_FRAME_WIDTH = 1920
DEFAULT_DESIRED_CAMERA_FPS = 5

# > For the size of the ArUco marker used in the A04 Sprint Review Demo.
DEFAULT_MARKER_SIZE = 0.250
# NOTE: the unit are arbitary and the units of "tvec" match
#       the units of the marker size.

# > For where to save images captured by the camera
#   Note: ensure that this path already exists
#   Note: one image is saved each time a message is received
#         on the "request_save_image" topic.
DEFAULT_SAVE_IMAGE_PATH = "saved_camera_images/"

# > A flag for whether to save all images that contain
#   an aruco marker
DEFAULT_SHOULD_SAVE_ALL_ARUCO_IMAGES = False

# > A flag for whether to publish the images captured
DEFAULT_SHOULD_PUBLISH_CAMERA_IMAGES = False

# > A flag for whether to display the images captured
DEFAULT_SHOULD_SHOW_CAMERA_IMAGES = False


class ArucoDetector(Node):

    def __init__(self):
        super().__init__("aruco_detector")

        # Get the namespace of the node
        node_namespace = self.get_namespace()
        node_name = self.get_name()

        # Declare all parameters with default values
        self.declare_parameter("aruco_detector_verbosity", DEFAULT_ARUCO_DETECTOR_VERBOSITY)
        self.declare_parameter("aruco_detector_usb_camera_device_number", DEFAULT_USB_CAMERA_DEVICE_NUMBER)
        self.declare_parameter("aruco_detector_desired_camera_frame_height", DEFAULT_DESIRED_CAMERA_FRAME_HEIGHT)
        self.declare_parameter("aruco_detector_desired_camera_frame_width", DEFAULT_DESIRED_CAMERA_FRAME_WIDTH)
        self.declare_parameter("aruco_detector_desired_camera_fps", DEFAULT_DESIRED_CAMERA_FPS)
        self.declare_parameter("aruco_detector_marker_size", DEFAULT_MARKER_SIZE)
        self.declare_parameter("aruco_detector_save_image_path", DEFAULT_SAVE_IMAGE_PATH)
        self.declare_parameter("aruco_detector_should_save_all_aruco_images", DEFAULT_SHOULD_SAVE_ALL_ARUCO_IMAGES)
        self.declare_parameter("aruco_detector_should_publish_camera_images", DEFAULT_SHOULD_PUBLISH_CAMERA_IMAGES)
        self.declare_parameter("aruco_detector_should_show_camera_images", DEFAULT_SHOULD_SHOW_CAMERA_IMAGES)

        # Get the parameters
        self.aruco_detector_verbosity     = self.get_parameter("aruco_detector_verbosity").value
        usb_camera_device_number          = self.get_parameter("aruco_detector_usb_camera_device_number").value
        desired_camera_frame_height       = self.get_parameter("aruco_detector_desired_camera_frame_height").value
        desired_camera_frame_width        = self.get_parameter("aruco_detector_desired_camera_frame_width").value
        desired_camera_fps                = self.get_parameter("aruco_detector_desired_camera_fps").value
        self.marker_size                  = self.get_parameter("aruco_detector_marker_size").value
        self.save_image_path              = self.get_parameter("aruco_detector_save_image_path").value
        self.should_save_all_aruco_images = self.get_parameter("aruco_detector_should_save_all_aruco_images").value
        self.should_publish_camera_images = self.get_parameter("aruco_detector_should_publish_camera_images").value
        self.should_show_camera_images    = self.get_parameter("aruco_detector_should_show_camera_images").value

        # Initialise a publisher for the details of detected markers
        self.marker_detections_publisher = self.create_publisher(FiducialMarkerArray, "aruco_detections", 10)

        # Initialise a publisher for the images
        self.image_publisher = self.create_publisher(Image, "camera_image", 10)

        # Initialise a subscriber for flagging when to save an image
        self.request_save_sub = self.create_subscription(UInt32, "request_save_image", self.requestSaveImageSubscriberCallback, 10)
        # > For convenience, the command line can be used to trigger this subscriber
        #   by publishing a message to the "request_save_image" as follows:
        #
        # ros2 topic pub /request_save_image std_msgs/msg/UInt32 "{data: 1}" 

        # Initialise variables for managing the saving of an image
        self.save_image_counter = 0
        self.should_save_image  = False

        # Specify the details for camera to capture from

        # > Put the desired video capture properties into local variables
        self.camera_frame_width  = desired_camera_frame_width
        self.camera_frame_height = desired_camera_frame_height
        self.camera_fps = desired_camera_fps

        # > For capturing from a USB camera:
        #   > List the contents of /dev/video* to determine
        #     the number of the USB camera
        #   > If "v4l2-ctl" command line tool is installed then list video devices with:
        #     v4l2-ctl --list-devices
        self.camera_setup = usb_camera_device_number

        # > For capture from a camera connected via the MIPI CSI cable connectors
        #   > This specifies the gstreamer pipeline for video capture
        #   > sensor-id=0 for CAM0 and sensor-id=1 for CAM1
        #   > This should work; it is not "optimized"; precise details depend on the camera connected
        #self.camera_setup = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, framerate=12/1, format=MJPG ! nvvidconv flip-method=0 ! video/x-raw, width = 800, height=600, format =BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

        # Initialise video capture from the camera
        # > For numeric device indices, force V4L2 backend.
        # > For non-numeric strings (e.g. explicit GStreamer pipeline), keep default behaviour.
        try:
            camera_device_index = int(self.camera_setup)
            self.cam = cv2.VideoCapture(camera_device_index, cv2.CAP_V4L2)
            if not self.cam.isOpened():
                self.get_logger().warn(
                    "[ARUCO DETECTOR] Failed to open camera with CAP_V4L2 for device index {}. Falling back to default backend.".format(
                        camera_device_index
                    )
                )
                self.cam = cv2.VideoCapture(self.camera_setup)
        except (TypeError, ValueError):
            self.cam = cv2.VideoCapture(self.camera_setup)

        if not self.cam.isOpened():
            raise RuntimeError(
                "[ARUCO DETECTOR] Could not open camera device/pipeline: {}".format(
                    self.camera_setup
                )
            )

        self.camera_backend_name = self.cam.getBackendName()

        # Display the properties of the camera upon initialisation
        # > A list of all the properties available can be found here:
        #   https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
        if (self.aruco_detector_verbosity >= 1):
            self.get_logger().info("\n[ARUCO DETECTOR] Camera properties upon initialisation:")
            self.get_logger().info("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            self.get_logger().info("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
            self.get_logger().info("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
            self.get_logger().info("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
            self.get_logger().info("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
            self.get_logger().info("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
            self.get_logger().info("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
            self.get_logger().info("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
            self.get_logger().info("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # Set the camera properties to the desired values
        # > Frame height and  width, in [pixels]
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_frame_height)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,  self.camera_frame_width)
        # > Frame rate, in [fps]
        self.cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
        # > Auto focus, [bool: 0=off, 1=on]
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # > Focus absolute, [int: min=0 max=250 step=5 default=0]
        #   0 corresponds to focus at infinity
        self.cam.set(cv2.CAP_PROP_FOCUS, 0)
        # > Buffer size, [int: min=1]
        #   Setting the buffer to zero ensures that we get that
        #   most recent frame even when the "timerCallbackForCameraRead"
        #   function takes longer than (1.self.camera_fps) seconds
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Display the properties of the camera after setting the desired values
        self.get_logger().info(f"[ARUCO DETECTOR] Using backend: {self.camera_backend_name}")
        if (self.aruco_detector_verbosity >= 1):
            self.get_logger().info("\n[ARUCO DETECTOR] Camera properties after setting desired values:")
            self.get_logger().info("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            self.get_logger().info("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
            self.get_logger().info("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
            self.get_logger().info("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
            self.get_logger().info("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
            self.get_logger().info("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
            self.get_logger().info("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
            self.get_logger().info("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
            self.get_logger().info("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))
        
        # The frame per second (fps) property cannot take any value,
        # hence compare the actual value and display any discrepancy
        camera_actual_fps = self.cam.get(cv2.CAP_PROP_FPS)
        if not(camera_actual_fps==self.camera_fps):
            self.get_logger().warn("[ARUCO DETECTOR] The camera is running at " + str(camera_actual_fps) + " fps, even though " + str(self.camera_fps) + " fps was requested.")
            self.get_logger().warn("[ARUCO DETECTOR] The fps discrepancy is normal behaviour as most cameras cannot run at arbitrary fps rates.")
            self.get_logger().warn("[ARUCO DETECTOR] Due to the fps discrepancy, updated the value: self.camera_fps = " + str(camera_actual_fps))
            self.camera_fps = camera_actual_fps
        
        # Initialise the OpenCV<->ROS bridge
        self.cv_bridge = CvBridge()

        # Get the ArUco dictionary to use
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Create an parameter structure needed for the ArUco detection
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_parameters = cv2.aruco.DetectorParameters()
            if (self.aruco_detector_verbosity >= 1):
                self.get_logger().info("[ARUCO DETECTOR] ArUco parameter structure created with: \"cv2.aruco.DetectorParameters()\"")
        elif hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
            if (self.aruco_detector_verbosity >= 1):
                self.get_logger().info("[ARUCO DETECTOR] ArUco parameter structure created with: \"cv2.aruco.DetectorParameters_create()\"")
        else:
            raise RuntimeError("[ARUCO DETECTOR] OpenCV ArUco parameters API not found.")
        # > Specify the parameter for: corner refinement
        self.aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Create an Aruco detector object
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_parameters)
            self.aruco_detection_api = "ArucoDetector"
            if (self.aruco_detector_verbosity >= 1):
                self.get_logger().info("[ARUCO DETECTOR] ArUco detection is using API object created with: \"cv2.aruco.ArucoDetector(...)\"")
        else:
            self.aruco_detector = None
            self.aruco_detection_api = "detectMarkers"
            if (self.aruco_detector_verbosity >= 1):
                self.get_logger().info("[ARUCO DETECTOR] The \"cv2.aruco.ArucoDetector(...)\" detection object is not available. Instead, the \"cv2.aruco.detectMarkers(...)\" function will be used.")
        self.get_logger().info("[ARUCO DETECTOR] ArUco API selected: " + self.aruco_detection_api)

        # Define the marker corner point in the marker frame
        marker_size_half = 0.5 * self.marker_size
        self.single_marker_object_points = np.array([  \
                [-marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half,-marker_size_half, 0.0], \
                [-marker_size_half,-marker_size_half, 0.0]  \
                ], dtype=np.float32 )
        
        # Specify the intrinsic parameters of the camera
        # > These parameters could either be hardcoded here;
        # > Or you can load then from a file that you may have
        #   saved during the calibration procedure.
        # > Calibrated on 2026-04-14 using 13 chessboard images (9x6 grid, 80-200cm)
        #   Reprojection error RMS = 0.2102 pixels
        self.intrinic_camera_matrix = np.array([[1429.62, 0, 931.99], [0, 1429.00, 471.67], [0, 0, 1]], dtype=float)
        self.intrinic_camera_distortion = np.array([[2.807578e-02, -1.709202e-01, 2.437027e-03, -4.612838e-03, -2.993165e-02]], dtype=float)

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        return_flag, current_frame = self.cam.read()
        if return_flag and current_frame is not None:
            # > Get the dimensions of the frame
            dimensions = current_frame.shape
            # > Display the dimensions

            if(self.aruco_detector_verbosity >= 1):
                self.get_logger().info("[ARUCO DETECTOR] As a double check of the camera properties set, a frame captured just now has dimensions = " + str(dimensions))
            # > Also check the values
            if (not(dimensions[0]==self.camera_frame_height) or not(dimensions[1]==self.camera_frame_width)):
                self.get_logger().warn("[ARUCO DETECTOR] ERROR: frame dimensions do NOT match the desired values.")
                # Update the variables
                self.camera_frame_height = dimensions[0]
                self.camera_frame_width  = dimensions[1]
        else:
            self.get_logger().warn(
                '[ARUCO DETECTOR] Could not read an initial camera frame. '
                'The timer callback will keep trying.'
            )
        
        # Initialize a sequence number for each camera frame that is read
        self.camera_frame_sequence_number = 0

        # Display command line command for publishing a
        # request to save a camera image
        if(self.aruco_detector_verbosity >= 1):
            self.get_logger().info("[ARUCO DETECTOR] publish request from command line to save a single camera image using: ros2 topic pub --once "+node_namespace+"/request_save_image std_msgs/msg/UInt32 \"{data: 1}\"")
        
        # Display the status
        if (self.aruco_detector_verbosity >= 1):
            self.get_logger().info("[ARUCO DETECTOR] Node initialisation complete")

        # Initialise a timer for capturing the camera frames
        timer_period = 1/self.camera_fps
        self.cam_timer = self.create_timer(timer_period, self.timerCallbackForCameraRead)

    # Respond to timer callback
    def timerCallbackForCameraRead(self):
        # Read the camera frame
        return_flag , current_frame = self.cam.read()
        # Note: return_flag is false if no frame was grabbed

        if(return_flag == True):

            # Increment the sequence number
            self.camera_frame_sequence_number += 1

            # Convert the image to gray scale
            current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers from the frame
            if self.aruco_detector is not None:
                aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = self.aruco_detector.detectMarkers(current_frame_gray)
            else:
                aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = cv2.aruco.detectMarkers(
                    current_frame_gray,
                    self.aruco_dict,
                    parameters=self.aruco_parameters
                )

            # Process any ArUco markers that were detected
            if aruco_ids is not None:
                # Display the number of markers found
                if (self.aruco_detector_verbosity >= 2):
                    if (len(aruco_ids) == 1):
                        self.get_logger().info("[ARUCO DETECTOR] Found " + "{:3}".format(len(aruco_ids)) + " marker  with id  = " + str(aruco_ids[:,0]))
                    else:
                        self.get_logger().info("[ARUCO DETECTOR] Found " + "{:3}".format(len(aruco_ids)) + " markers with ids = " + str(aruco_ids[:,0]))
                
                # Set the save image flag as appropriate
                if (self.should_save_all_aruco_images):
                    self.should_save_image = True

                # Outline all of the markers detected found in the image
                # > Only if this image will be used
                if (self.should_publish_camera_images or self.should_save_image or self.should_show_camera_images):
                    current_frame_with_marker_outlines = cv2.aruco.drawDetectedMarkers(current_frame.copy(), aruco_corners_of_all_markers, aruco_ids, borderColor=(0, 220, 0))
                
                # Initialize the message for publishing the markers
                msg_of_marker_detections = FiducialMarkerArray()

                # Fill in the "header" details of the message
                msg_of_marker_detections.num_markers = len(aruco_ids)
                msg_of_marker_detections.seq_num = self.camera_frame_sequence_number

                # Iterate over the markers detected
                for i_marker_id in range(len(aruco_ids)):
                    # Get the ID for this marker
                    this_id = aruco_ids[i_marker_id][0]
                    # Get the corners for this marker
                    corners_of_this_marker = np.asarray(aruco_corners_of_all_markers[i_marker_id][0], dtype=np.float32)
                    # Estimate the pose of this marker
                    # > Optionally use flags to specify solve method:
                    #   > SOLVEPNP_ITERATIVE Iterative method is based on a Levenberg-Marquardt optimization. In this case the function finds such a pose that minimizes reprojection error, that is the sum of squared distances between the observed projections "imagePoints" and the projected (using cv::projectPoints ) "objectPoints". Initial solution for non-planar "objectPoints" needs at least 6 points and uses the DLT algorithm. Initial solution for planar "objectPoints" needs at least 4 points and uses pose from homography decomposition.
                    #   > SOLVEPNP_P3P Method is based on the paper of X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang "Complete Solution Classification for the Perspective-Three-Point Problem". In this case the function requires exactly four object and image points.
                    #   > SOLVEPNP_AP3P Method is based on the paper of T. Ke, S. Roumeliotis "An Efficient Algebraic Solution to the Perspective-Three-Point Problem". In this case the function requires exactly four object and image points.
                    #   > SOLVEPNP_EPNP Method has been introduced by F. Moreno-Noguer, V. Lepetit and P. Fua in the paper "EPnP: Efficient Perspective-n-Point Camera Pose Estimation".
                    #   > SOLVEPNP_IPPE Method is based on the paper of T. Collins and A. Bartoli. "Infinitesimal Plane-Based Pose Estimation". This method requires coplanar object points.
                    #   > SOLVEPNP_IPPE_SQUARE Method is based on the paper of Toby Collins and Adrien Bartoli. "Infinitesimal Plane-Based Pose Estimation". This method is suitable for marker pose estimation. It requires 4 coplanar object points defined in the following order:
                    #         point 0: [-squareLength / 2, squareLength / 2, 0]
                    #         point 1: [ squareLength / 2, squareLength / 2, 0]
                    #         point 2: [ squareLength / 2, -squareLength / 2, 0]
                    #         point 3: [-squareLength / 2, -squareLength / 2, 0]
                    #   > SOLVEPNP_SQPNP Method is based on the paper "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem" by G. Terzakis and M.Lourakis. It requires 3 or more points.
                    
                    # Estimate the pose of this marker
                    solvepnp_method = cv2.SOLVEPNP_IPPE_SQUARE
                    success_flag, rvec, tvec = cv2.solvePnP(self.single_marker_object_points, corners_of_this_marker, self.intrinic_camera_matrix, self.intrinic_camera_distortion, flags=solvepnp_method)

                    # Note: the cv2.aruco.estimatePoseSingleMarkers" function was deprecated in OpenCV version 4.7
                    # > The recommended alternative "cv2.solvePnP" is used above.
                    #this_rvec_estimate, this_tvec_estimate, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners_of_this_marker, self.marker_size, self.intrinic_camera_matrix, self.intrinic_camera_distortion)
                    #rvec = this_rvec_estimate[0]
                    #tvec = this_tvec_estimate[0]

                    # Draw the marker's axes onto the image
                    # > Only if this image will be used
                    if (self.should_publish_camera_images or self.should_save_image or self.should_show_camera_images):
                        current_frame_with_marker_outlines = cv2.drawFrameAxes(current_frame_with_marker_outlines, self.intrinic_camera_matrix, self.intrinic_camera_distortion, rvec, tvec, 0.5*self.marker_size)
                    
                    # At this stage, the variable "rvec" and "tvec" respectively
                    # describe the rotation and translation of the marker frame
                    # relative to the camera frame, i.e.:
                    # tvec - is a vector of length 3 expressing the (x,y,z) coordinates
                    #        of the marker's center in the coordinate frame of the camera.
                    # rvec - is a vector of length 3 expressing the rotation of the marker's
                    #        frame relative to the frame of the camera.
                    #        This vector is an "axis angle" representation of the rotation.

                    # A vector expressed in the maker frame coordinates can now be
                    # rotated to the camera frame coordinates as:
                    # Rmat = cv2.Rodrigues(rvec)
                    # [x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}

                    # Note: the camera frame convention is:
                    # > z-axis points along the optical axis, i.e., straight out of the lens
                    # > x-axis points to the right when looking out of the lens along the z-axis
                    # > y-axis points to the down  when looking out of the lens along the z-axis

                    # Display the rvec and tvec
                    if (self.aruco_detector_verbosity >= 2):
                        self.get_logger().info("[ARUCO DETECTOR] for id = " + str(this_id) + ", tvec = [ " + str(tvec[0]) + " , " + str(tvec[1]) + " , " + str(tvec[2]) + " ]" )

                    # Create a message with the details of this marker
                    msg_for_this_marker = FiducialMarker()
                    msg_for_this_marker.id = int(this_id)
                    # Convert to float32 as required by the message definition
                    msg_for_this_marker.tvec = [float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])]
                    msg_for_this_marker.rvec = [float(rvec[0][0]), float(rvec[1][0]), float(rvec[2][0])]

                    # Append to the message list of all marker detections
                    msg_of_marker_detections.markers.append(msg_for_this_marker)

                # Publish the message with details of the detected markers
                self.marker_detections_publisher.publish(msg_of_marker_detections)

            else:
                # Display that no aruco markers were found
                #rospy.loginfo("[ARUCO DETECTOR] No markers found in this image")
                # Set the frame variable that is used for save/display/publish
                current_frame_with_marker_outlines = current_frame

            # Publish the camera frame
            if (self.should_publish_camera_images):
                try:
                    self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame_with_marker_outlines, "bgr8"))
                except CvBridgeError as cv_bridge_err:
                    print(cv_bridge_err)

            # Save the camera frame if requested
            if (self.should_save_image):
                # Increment the image counter
                self.save_image_counter += 1
                # Write the image to file
                temp_filename = self.save_image_path + "aruco_image" + str(self.save_image_counter) + ".jpg"
                cv2.imwrite(temp_filename,current_frame_with_marker_outlines)
                # Display the path to where the image was saved
                if (self.aruco_detector_verbosity >= 2):
                    self.get_logger().info("[ARUCO DETECTOR] Saved camera frame to: " + temp_filename)
                # Reset the flag to false
                self.should_save_image = False

            # Display the camera frame if requested
            if (self.should_show_camera_images):
                cv2.imshow("[ARUCO DETECTOR]", current_frame_with_marker_outlines)
                cv2.waitKey(1)  # Add this to properly display the window

        else:
            # Display an error message
            self.get_logger().warn("[ARUCO DETECTOR] ERROR occurred during \"self.cam.read()\"")

    def requestSaveImageSubscriberCallback(self, msg : UInt32):
        if (self.aruco_detector_verbosity >= 1):
            self.get_logger().info("[ARUCO DETECTOR] Request received to save the next image")
        # Set the flag for saving an image to true
        self.should_save_image = True

def main(args = None):
    rclpy.init(args=args)

    # Initialise an object of the aruco detector class
    aruco_detector_object = ArucoDetector()

    # Spin as a single-threaded node
    rclpy.spin(aruco_detector_object)

    # Release the camera
    aruco_detector_object.cam.release()
    
    # Close any OpenCV windows
    cv2.destroyAllWindows()
    aruco_detector_object.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
