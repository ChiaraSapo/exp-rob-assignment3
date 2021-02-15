#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg
#import exp_assignment3.msg
import sys
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, UInt32
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import signal
import subprocess
import roslaunch
'''
VERBOSE = False
LOOPS = 2
vel_camera = Float64()
vel_Norm = Twist()
MAX_COUNTER = 25
SEARCH_FOR_BALL = 35
global counter
global subscriberNORM
global subscriberPLAY
'''
position_ = Point()
pose_ = Pose()
yaw_ = 0

# Color limits
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
               greenLower, blackLower, magentaLower]
upperValues = [blackUpper, redUpper, yellowUpper,
               greenUpper, blackUpper, magentaUpper]

#colorName = ['blue', 'red', 'yellow', 'green', 'black', 'magenta']
colorName = ['entrance', 'closet', 'kitchen',
             'livingRoom', 'bedroom', 'bathroom']

blackPos = [0, 0]
redPos = [0, 0]
yellowPos = [0, 0]
greenPos = [0, 0]
bluePos = [0, 0]
magentaPos = [0, 0]

ballsPos = [blackPos, redPos, yellowPos, greenPos, bluePos, magentaPos]

lastDetected = -1

moveTo = [0, 0, 0]


class find_and_follow_ball:

    def __init__(self):

        # Init publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # self.camera_pub = rospy.Publisher("/joint_position_controller/command", Float64, queue_size=1)

        # Subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)

    def callback(self, ros_data):

        global upperValues
        global lowerValues
        global colorName
        global ballsPos, lastDetected
        global position_

        # Convert to cv2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Create masks
        for numMask in range(0, 6):
            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lowerValues[numMask], upperValues[numMask])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                vel_Play = Twist()

                # Publish robot vel
                vel_Play.angular.z = -0.002*(center[0]-400)
                vel_Play.linear.x = -0.01*(radius-100)
                # self.vel_pub.publish(vel_Play)

                if radius > 10:
                    lastDetected = numMask
                    #rospy.set_param('color', numMask)
                    rospy.logerr('%s ball detected', colorName[numMask])

                else:
                    lastDetected = -1

            cv2.imshow('window', image_np)
            cv2.waitKey(2)

# This function is a client for the robot motion server. It sends the desired position.


def main():

    rospy.init_node('cameraManager')

    find_and_follow_ball()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    cv2.destroyAllWindows()
    sis.stop()


if __name__ == '__main__':
    main()
