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
# Python node to capture and publish video
#
# ----------------------------------------------------------------------------



# ----------------------------------------------------------------------------
# A FEW USEFUL LINKS ABOUT OPEN CV VIDEO CAPTURE
#
# > The "VideoCapture" class reference page:
#   https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html
#
# > Open CV tutorial describing the camera calibration procedure:
#   https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#
# > The Open CV function for finding "chessboard" corners:
#   https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a
#
# ----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image

# Import opencv
import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge, CvBridgeError



# DEFINE THE PARAMETERS
# > For the verbosity level of displaying info
DEFAULT_CAMERA_CAPTURE_VERBOSITY = 1
# Note: the levels of increasing verbosity are defined as:
# 0 : Info is not displayed. Warnings and errors are still displayed
# 1 : Startup info is displayed
# 2 : Info about detected chessboards is displayed

# > For the number of the USB camera device
#   i.e., for /dev/video0, this parameter should be 0
DEFAULT_USB_CAMERA_DEVICE_NUMBER = 0

# > Properties of the camera images captured
DEFAULT_DESIRED_CAMERA_FRAME_HEIGHT = 1080
DEFAULT_DESIRED_CAMERA_FRAME_WIDTH = 1920
DEFAULT_DESIRED_CAMERA_FPS = 5

# > For the size of the chessboard grid
DEFAULT_CHESSBOARD_SIZE_HEIGHT = 9
DEFAULT_CHESSBOARD_SIZE_WIDTH  = 6

# > For where to save images captured by the camera
#   IMPORTANT: ensure that this path already exists!
#   Note: one image is saved each time a message is received
#         on the "request_save_image" topic.
DEFAULT_SAVE_IMAGE_PATH = "saved_camera_images/"

# > A flag for whether to save any images that contains
#   a camera calibration chessboard
DEFAULT_SHOULD_SAVE_ALL_CHESSBOARD_IMAGES = True

# > A flag for whether to publish the images captured
DEFAULT_SHOULD_PUBLISH_CAMERA_IMAGES = False

# > A flag for whether to display the images captured
DEFAULT_SHOULD_SHOW_IMAGES = False



class CameraCapture(Node):

    def __init__(self):
        super().__init__("camera_capture")
        
        # Declare all parameters with default values
        self.declare_parameter('camera_capture_verbosity', DEFAULT_CAMERA_CAPTURE_VERBOSITY)
        self.declare_parameter('camera_capture_usb_camera_device_number', DEFAULT_USB_CAMERA_DEVICE_NUMBER)
        self.declare_parameter('camera_capture_desired_camera_frame_height', DEFAULT_DESIRED_CAMERA_FRAME_HEIGHT)
        self.declare_parameter('camera_capture_desired_camera_frame_width', DEFAULT_DESIRED_CAMERA_FRAME_WIDTH)
        self.declare_parameter('camera_capture_desired_camera_fps', DEFAULT_DESIRED_CAMERA_FPS)
        self.declare_parameter('camera_capture_chessboard_size_height', DEFAULT_CHESSBOARD_SIZE_HEIGHT)
        self.declare_parameter('camera_capture_chessboard_size_width', DEFAULT_CHESSBOARD_SIZE_WIDTH)
        self.declare_parameter('camera_capture_save_image_path', DEFAULT_SAVE_IMAGE_PATH)
        self.declare_parameter('camera_capture_should_save_all_chessboard_images', DEFAULT_SHOULD_SAVE_ALL_CHESSBOARD_IMAGES)
        self.declare_parameter('camera_capture_should_publish_camera_images', DEFAULT_SHOULD_PUBLISH_CAMERA_IMAGES)
        self.declare_parameter('camera_capture_should_show_camera_images', DEFAULT_SHOULD_SHOW_IMAGES)
        
        # Get the namespace of the node
        node_namespace = self.get_namespace()
        
        # Get the parameters
        self.camera_capture_verbosity = self.get_parameter('camera_capture_verbosity').value
        usb_camera_device_number = self.get_parameter('camera_capture_usb_camera_device_number').value
        desired_camera_frame_height = self.get_parameter('camera_capture_desired_camera_frame_height').value
        desired_camera_frame_width = self.get_parameter('camera_capture_desired_camera_frame_width').value
        desired_camera_fps = self.get_parameter('camera_capture_desired_camera_fps').value
        self.chessboard_size_height = self.get_parameter('camera_capture_chessboard_size_height').value
        self.chessboard_size_width = self.get_parameter('camera_capture_chessboard_size_width').value
        self.save_image_path = self.get_parameter('camera_capture_save_image_path').value
        self.should_save_all_chessboard_images = self.get_parameter('camera_capture_should_save_all_chessboard_images').value
        self.should_publish_camera_images = self.get_parameter('camera_capture_should_publish_camera_images').value
        self.should_show_images = self.get_parameter('camera_capture_should_show_camera_images').value

        # Initialise a publisher for the images
        self.image_publisher = self.create_publisher(Image, "camera_image", 10)

        # Initialise a subscriber for flagging when to save an image
        self.create_subscription(UInt32, "request_save_image", self.requestSaveImageSubscriberCallback, 10)
        # > For convenience, the command line can be used to trigger this subscriber
        #   by publishing a message to the "request_save_image" as follows:
        #
        # ros2 topic pub /<namespace>/request_save_image std_msgs/msg/UInt32 "{data: 1}"

        # Initialise variables for managing the saving of an image
        self.save_image_counter = 0
        self.should_save_image = False

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
        try:
            camera_device_index = int(self.camera_setup)
            self.cam = cv2.VideoCapture(camera_device_index, cv2.CAP_V4L2)
            if not self.cam.isOpened():
                self.get_logger().warn(
                    "[CAMERA CAPTURE] Failed to open camera with CAP_V4L2 for device index {}. Falling back to default backend.".format(
                        camera_device_index
                    )
                )
                self.cam = cv2.VideoCapture(self.camera_setup)
        except (TypeError, ValueError):
            self.cam = cv2.VideoCapture(self.camera_setup)

        # Display the properties of the camera upon initialisation
        # > A list of all the properties available can be found here:
        #   https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
        if (self.camera_capture_verbosity >= 1):
            self.get_logger().info("\n[CAMERA CAPTURE] Camera properties upon initialisation:")
            self.get_logger().info("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            self.get_logger().info("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
            self.get_logger().info("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
            self.get_logger().info("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
            self.get_logger().info("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
            self.get_logger().info("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
            self.get_logger().info("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
            self.get_logger().info("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
            #self.get_logger().info("CAP_PROP_HUE :             '{}'".format(self.cam.get(cv2.CAP_PROP_HUE)))
            #self.get_logger().info("CAP_PROP_CONVERT_RGB :     '{}'".format(self.cam.get(cv2.CAP_PROP_CONVERT_RGB)))
            #self.get_logger().info("CAP_PROP_POS_MSEC :        '{}'".format(self.cam.get(cv2.CAP_PROP_POS_MSEC)))
            #self.get_logger().info("CAP_PROP_FRAME_COUNT  :    '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_COUNT)))
            self.get_logger().info("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # Set the camera properties to the desired values
        # > Frame height and  width, in [pixels]
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_frame_height)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,  self.camera_frame_width)
        # > Frame rate, in [fps]
        self.cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
        # > Auto focus, [bool: 0=off, 1=on]
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # > Focus absolute, [int: min=0 max=250 step=5 default=0 value=0 flags=inactive]
        #   0 corresponds to focus at infinity
        self.cam.set(cv2.CAP_PROP_FOCUS, 0)
        # > Buffer size, [int: min=1]
        #   Setting the buffer to zero ensures that we get that
        #   most recent frame even when the "timerCallbackForCameraRead"
        #   function takes longer than (1.self.camera_fps) seconds
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Display the properties of the camera after setting the desired values
        if (self.camera_capture_verbosity >= 1):
            self.get_logger().info("\n[CAMERA CAPTURE] Camera properties after setting desired values:")
            self.get_logger().info("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            self.get_logger().info("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
            self.get_logger().info("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
            self.get_logger().info("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
            self.get_logger().info("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
            self.get_logger().info("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
            self.get_logger().info("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
            self.get_logger().info("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
            #self.get_logger().info("CAP_PROP_HUE :             '{}'".format(self.cam.get(cv2.CAP_PROP_HUE)))
            #self.get_logger().info("CAP_PROP_CONVERT_RGB :     '{}'".format(self.cam.get(cv2.CAP_PROP_CONVERT_RGB)))
            #self.get_logger().info("CAP_PROP_POS_MSEC :        '{}'".format(self.cam.get(cv2.CAP_PROP_POS_MSEC)))
            #self.get_logger().info("CAP_PROP_FRAME_COUNT  :    '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_COUNT)))
            self.get_logger().info("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # The frame per second (fps) property cannot take any value,
        # hence compare the actual value and display any discrepancy
        camera_actual_fps = self.cam.get(cv2.CAP_PROP_FPS)
        if not(camera_actual_fps==self.camera_fps):
            self.get_logger().warn("[CAMERA CAPTURE] The camera is running at " + str(camera_actual_fps) + " fps, even though " + str(self.camera_fps) + " fps was requested.")
            self.get_logger().warn("[CAMERA CAPTURE] The fps discrepancy is normal behaviour as most cameras cannot run at arbitrary fps rates.")
            self.get_logger().warn("[CAMERA CAPTURE] Due to the fps discrepancy, updated the value: self.camera_fps = " + str(camera_actual_fps))
            self.camera_fps = camera_actual_fps

        # Initialise the OpenCV<->ROS bridge
        self.cv_bridge = CvBridge()

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        return_flag , current_frame = self.cam.read()
        # > Get the dimensions of the frame
        dimensions = current_frame.shape
        # > Display the dimensions
        if (self.camera_capture_verbosity >= 1):
            self.get_logger().info("[CAMERA CAPTURE] As a double check of the camera properties set, a frame captured just now has dimensions = " + str(dimensions))
        # > Also check the values
        if (not(dimensions[0]==self.camera_frame_height) or not(dimensions[1]==self.camera_frame_width)):
            self.get_logger().warn("[CAMERA CAPTURE] ERROR: frame dimensions do NOT match the desired values.")
            # Update the variables
            self.camera_frame_height = dimensions[0]
            self.camera_frame_width  = dimensions[1]

        # Display command line command for publishing a
        # request to save a camera image
        if (self.camera_capture_verbosity >= 1):
            self.get_logger().info("[CAMERA CAPTURE] publish request from command line to save a single camera image using: ros2 topic pub --once " + node_namespace + "/request_save_image std_msgs/msg/UInt32 '{data: 1}'")

        # Display the status
        if (self.camera_capture_verbosity >= 1):
            self.get_logger().info("[CAMERA CAPTURE] Node initialisation complete")

        # Initialise a timer for capturing the camera frames
        self.create_timer(1/self.camera_fps, self.timerCallbackForCameraRead)



    # Respond to timer callback
    def timerCallbackForCameraRead(self):
        # Read the camera frame
        #self.get_logger().info("[CAMERA CAPTURE] Now reading camera frame")
        return_flag , current_frame = self.cam.read()
        # Note: return_flag is false if no frame was grabbed

        # get dimensions of image
        #dimensions = current_frame.shape
        #height = current_frame.shape[0]
        #width = current_frame.shape[1]
        #channels = current_frame.shape[2]

        # Check if the camera frame was successfully read
        if (return_flag == True):
            # Save camera images for which a chessboard is detected
            if (self.should_save_all_chessboard_images):
                # Check if the camera frame contains a calibration
                # chessboard that can be detected by OpenCV
                # > Convert the image to gray scale
                current_frame_as_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
                # > Specify the operation flags for the "find chessboard corners" function
                #   The flags can be zero, or any combination of the following:
                #   > CALIB_CB_ADAPTIVE_THRESH  Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).
                #   > CALIB_CB_NORMALIZE_IMAGE  Normalize the image gamma with equalizeHist before applying fixed or adaptive thresholding.
                #   > CALIB_CB_FILTER_QUADS     Use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads extracted at the contour retrieval stage.
                #   > CALIB_CB_FAST_CHECK       Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.
                find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH #+ cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
                # > Call the "find chessboard corners" function:
                #   > The second argument is the grid size of internal
                #     corner points that the function should search for
                #   > This function can be quite slow when a chessboard is not visible in the image
                internal_corner_grid_size = (self.chessboard_size_height,self.chessboard_size_width)
                chessboard_found, chessboard_corners = cv2.findChessboardCorners(current_frame_as_gray, internal_corner_grid_size, flags=find_flags)
                # If found, then set the save image flag to true
                if (chessboard_found == True):
                    if (self.camera_capture_verbosity >= 2):
                        self.get_logger().info("[CAMERA CAPTURE] Chessboard FOUND, this image will be saved")
                    self.should_save_image = True
                #else:
                #    self.get_logger().info("[CAMERA CAPTURE] Chessboard NOT found")

            # Publish the camera frame
            if (self.should_publish_camera_images):
                #self.get_logger().info("[CAMERA CAPTURE] Now publishing camera frame")
                try:
                    self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame, "bgr8"))
                except CvBridgeError as cv_bridge_err:
                    print(cv_bridge_err)

            # Save the camera frame if requested
            if (self.should_save_image):
                # Increment the image counter
                self.save_image_counter += 1
                # Write the image to file
                temp_filename = self.save_image_path + "image" + str(self.save_image_counter) + ".jpg"
                cv2.imwrite(temp_filename,current_frame)
                # Display the path to where the image was saved
                if (self.camera_capture_verbosity >= 2):
                    self.get_logger().info("[CAMERA CAPTURE] Saved camera frame to: " + temp_filename)
                # Reset the flag to false
                self.should_save_image = False

            # Display the camera frame
            if (self.should_show_images):
                #self.get_logger().info("[CAMERA CAPTURE] Now displaying camera frame")
                cv2.imshow("[CAMERA CAPTURE]", current_frame)
                cv2.waitKey(1)  # Essential for cv2.imshow to work
        else:
            # Display an error message
            self.get_logger().warn("[CAMERA CAPTURE] ERROR occurred during \"self.cam.read()\"")



    # Respond to subscriber receiving a message
    def requestSaveImageSubscriberCallback(self, msg):
        if (self.camera_capture_verbosity >= 1):
            self.get_logger().info("[CAMERA CAPTURE] Request received to save the next image")
        # Set the flag for saving an image
        self.should_save_image = True



def main(args=None):
    # Initialise the node
    rclpy.init(args=args)

    # Initialise an object of the camera capture class
    camera_capture_object = CameraCapture()

    # Spin as a single-threaded node
    rclpy.spin(camera_capture_object)

    # Release the camera
    camera_capture_object.cam.release()
    
    # Close any OpenCV windows
    cv2.destroyAllWindows()
    
    # Destroy the node explicitly
    camera_capture_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
