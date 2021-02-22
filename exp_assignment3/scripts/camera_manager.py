#!/usr/bin/env python

## @file camera_manager.py
## @brief Manages the camera, then sends relevant information (related to the ball) of the environment.

import roslib
import rospy
import smach
import smach_ros
import time
#import random
from std_msgs.msg import String
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped
# from nav_msgs.msg import Odometry
# from gazebo_msgs.msg import LinkState
import math
import actionlib
import actionlib.msg
import sys
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int64MultiArray  # Float64, UInt32,
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import signal
# import subprocess
# import roslaunch
from tf import transformations
from exp_assignment3.msg import camera_msg

# Color limits and meanings
blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)
redLower = (0, 50, 50)
redUpper = (5, 255, 255)
yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)
greenLower = (50, 50, 50)
greenUpper = (70, 255, 255)
blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)
magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)
lowerValues = [blackLower, redLower, yellowLower,
               greenLower, blueLower, magentaLower]
upperValues = [blackUpper, redUpper, yellowUpper,
               greenUpper, blueUpper, magentaUpper]
colorName = ['entrance', 'closet', 'kitchen',
             'livingRoom', 'bedroom', 'bathroom']
colors = ['black', 'red', 'yellow', 'green', 'blue', 'magenta']

# Balls positions
blackPos = [0, 0]
redPos = [0, 0]
yellowPos = [0, 0]
greenPos = [0, 0]
bluePos = [0, 0]
magentaPos = [0, 0]
ballsPos = [blackPos, redPos, yellowPos, greenPos, bluePos, magentaPos]

# Variables for balls
justDetected = -1
closeBall = -1
desiredRoom = -1

# Robot positions
position_ = Point()
pose_ = Pose()
yaw_ = 0
yaw_precision_2_ = 1

# Desired robot position
moveTo = [0, 0, 0]

# Variables to move towards the balll
radius = 0
circCenter = 0


## Class to manage the camera
class camera_manager_fnct:

    ## Init function
    def __init__(self):

        # Init publishers and subscribers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.cam_pub = rospy.Publisher(
            "/camera_info", camera_msg, queue_size=1)

        self.to_send = camera_msg()

        # Subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)

    ## Callback function for the camera image
    def callback(self, ros_data):
        global ballsPos, justDetected, closeBall, circCenter, radius

        # Convert to cv2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # For each color
        for numMask in range(0, 6):

            # Create masks
            mask = cv2.inRange(hsv, lowerValues[numMask], upperValues[numMask])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                # Compute area around ball
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                circCenter = center[0]

                # Ball is seen at distance
                if radius > 10 and radius < 50:
                    justDetected = numMask
                    closeBall = -1

                # Ball is seen at close distance
                elif radius >= 50:
                    justDetected = numMask
                    closeBall = 1

                # Ball isn't in sight
                else:
                    justDetected = -1
                    closeBall = -1

            else:
                justDetected = -1

            # Publish info as a camera_msg
            self.to_send.justDetected.data = justDetected
            self.to_send.closeBall.data = closeBall
            self.to_send.radius.data = radius
            self.to_send.circCenter.data = circCenter
            self.cam_pub.publish(self.to_send)

            cv2.imshow('window', image_np)
            cv2.waitKey(2)


## Ros node that subscribes to the target_pos and odom topic and publishes on the cmd_vel topic.
def camera_manager():

    rospy.init_node('camera_manager', anonymous=True)

    camera_manager_fnct()

    rospy.spin()

    pass


if __name__ == '__main__':
    camera_manager()
