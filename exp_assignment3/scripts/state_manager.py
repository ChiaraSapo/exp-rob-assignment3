#!/usr/bin/env python

# @file state_manager.py
# @brief

# SLEEP
# Go to kennel
# Stay still for 3 seconds
# Exit NORMAL

# NORMAL
# In a loop:
# Listen to human: play command?
# - Yes: exit PLAY
# - No: Continue
# Move around: ball?
# - Yes: exit N_TRACK
# - No: Continue
# End of the loop: exit SLEEP

# N_TRACK
# Go close to the ball: did you know its position yet?
# - Yes: continue
# - No: save it
# Exit NORMAL

# PLAY
# In a loop:
# Go to human
# Wait for a goto command
# Compare command to the known ball positions: is position known?
# - Yes: go to position
# - No: exit FIND
# Wait to be arrived
# End of the loop: exit NORMAL

# FIND
# In a loop:
# Move towards goal (may change it): ball?
# - Yes: exit F_TRACK
# - No: continue
# End of the loop: exit PLAY

# F_TRACK
# Go close to the ball: is it the desired position? (no need to check if saved: u enter here only if it's not)
# - Yes: exit PLAY
# - No: exit FIND

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
import exp_assignment2.msg
import sys
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, UInt32
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

VERBOSE = False
LOOPS = 2
vel_camera = Float64()
vel_Norm = Twist()
MAX_COUNTER = 25
SEARCH_FOR_BALL = 35
global counter
global subscriberNORM
global subscriberPLAY

# Publisher

vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


# Simulates the user's voice commands.
# @param stateCalling: which state the robot is in
# @return userVoice: user command
def user_says(stateCalling):

    comm = "go to %d %d" % (random.randrange(0, 11), random.randrange(0, 11))
    if stateCalling == 0:  # normal
        userVoice = random.choice(['play', '', 'hey buddy'])
    if stateCalling == 1:  # play
        userVoice = comm
    return userVoice

# This function is the callback for the normal state.
# It checks if the ball is visible from the camera. If it is: it sets the ros parameter ball=1.
# If it  isn't: it sets the ros parameter ball=0.


def find_ball(ros_data):

    # Convert to cv2
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

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

    # Create masks
    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, greenLower, greenUpper)
    mask1 = cv2.erode(mask, None, iterations=2)
    mask1 = cv2.dilate(mask, None, iterations=2)

    # Find contour
    cnts = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # Only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(image_np, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(image_np, center, 5, (0, 0, 255), -1)
            rospy.set_param('color', 'green')


'''
# This class is used to detect and follow the green ball in the arena.
class find_and_follow_ball:

    # It initializes a publisher to the image, one to the robot velocity and one to the camera motor.
    # It subscribes to the camera image
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # Init publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)
        self.camera_pub = rospy.Publisher(
            "/joint_position_controller/command", Float64, queue_size=1)

        # Subscribed Topic
        self.subscriber = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    # This function is the callback function of the subscription to the camera image.
    # It looks for a green contour in the image, plots a circle around it and makes the robot approach it.
    # When the robot has the object at a specified distance, it stops, turns its head twice and again looks for the object.
    # If the robot doesn't see the ball for 10 iterations in a row, it sets the ros parameter counter to 10 and then waits
    # for it to be zero again.
    def callback(self, ros_data):
        global counter
        global vel_camera
        global MAX_COUNTER

        # Read counter ros parameter: proceed only if it's not max
        counter = rospy.get_param('counter')
        while counter == MAX_COUNTER:
            time.sleep(1)

        if VERBOSE:
            print('received image of type: "%s"' % ros_data.format)

        # Convert to cv2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Color limits
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        # Create masks
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # Only proceed if at least one contour was found
        if len(cnts) > 0:

            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Only proceed if the radius meets a minimum size
            if radius > 10:

                # Draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel_Play = Twist()

                # Publish robot vel
                vel_Play.angular.z = -0.002*(center[0]-400)
                vel_Play.linear.x = -0.01*(radius-100)
                self.vel_pub.publish(vel_Play)

                # When robot has arrived: turn head
                if vel_Play.linear.x <= 0.05 and vel_Play.angular.z <= 0.05:

                    # Stop robot completely
                    vel_Play.angular.z = 0
                    vel_Play.linear.x = 0
                    self.vel_pub.publish(vel_Play)

                    # Rotate camera
                    rospy.set_param('rotate_camera', 1)

            # Go near ball
            else:
                vel_Play = Twist()
                vel_Play.linear.x = 0.5
                self.vel_pub.publish(vel_Play)

        # Look for ball by turning on the spot
        else:
            vel_Play = Twist()
            vel_Play.angular.z = 0.5
            self.vel_pub.publish(vel_Play)

            # Increase counter of iterations without seeing the ball
            counter = counter+1
            rospy.set_param('counter', counter)
            time.sleep(1)
            rospy.loginfo('counter incremented')

            # If counter is max: stop
            if counter == MAX_COUNTER:
                vel_Play.angular.z = 0
                self.vel_pub.publish(vel_Play)
                self.subscriber.unregister()

        # Show camera image
        cv2.imshow('window', image_np)
        cv2.waitKey(2)
'''

# This function is a client for the robot motion server. It sends the desired position.


def move_dog(target):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    rospy.loginfo('Going to %d %d with angle %d',
                  target[0], target[1], target[2])

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()

# Sleep state of the smach machine.


class MIRO_Sleep(smach.State):

    # Init function for smach machine sleep state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    # Go to kennel
    # Stay still for 3 seconds
    # Exit NORMAL

    def execute(self, userdata):
        time.sleep(3)
        move_dog([0, 0, 0])
        return 'normal_command'


# Normal state of the smach machine.


class MIRO_Normal(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['sleep_command', 'play_command', 'n_track_command'])

    # In a loop:
    # Listen to human: play command?
    # - Yes: exit PLAY
    # - No: Continue
    # Move around: ball?
    # - Yes: exit N_TRACK
    # - No: Continue
    # End of the loop: exit SLEEP

    def execute(self, userdata):
        time.sleep(2)
        return random.choice(['sleep_command', 'play_command', 'n_track_command'])


class N_Track(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    # Go close to the ball: did you know its position yet?
    # - Yes: continue
    # - No: save it
    # Exit NORMAL

    def execute(self, userdata):
        time.sleep(2)
        return 'normal_command'


# Play state of the smach machine.


class MIRO_Play(smach.State):

    # Init function for smach machine play state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command', 'find_command'])

        self.camera_pub = rospy.Publisher("/joint_position_controller/command",
                                          Float64, queue_size=1)

    # In a loop:
    # Go to human
    # Wait for a goto command
    # Compare command to the known ball positions: is position known?
    # - Yes: go to position
    # - No: exit FIND
    # Wait to be arrived
    # End of the loop: exit NORMAL

    def execute(self, userdata):
        time.sleep(2)
        return random.choice(['find_command', 'normal_command'])


class MIRO_Find(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['play_command', 'f_track_command'])

    # In a loop:
    # Move towards goal (may change it): ball?
    # - Yes: exit F_TRACK
    # - No: continue
    # End of the loop: exit PLAY

    def execute(self, userdata):
        time.sleep(2)
        return random.choice(['f_track_command', 'play_command'])


class F_Track(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['find_command', 'play_command'])

    # Go close to the ball: is it the desired position? (no need to check if saved: u enter here only if it's not)
    # - Yes: exit PLAY
    # - No: exit FIND

    def execute(self, userdata):
        time.sleep(2)
        return random.choice(['find_command', 'play_command'])


def main():

    rospy.init_node('state_manager')

    time.sleep(3)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', MIRO_Sleep(),
                               transitions={'normal_command': 'NORMAL'})
        ###
        smach.StateMachine.add('NORMAL', MIRO_Normal(),
                               transitions={'sleep_command': 'SLEEP',
                                            'play_command': 'PLAY',
                                            'n_track_command': 'N_TRACK'})

        smach.StateMachine.add('N_TRACK', N_Track(),
                               transitions={'normal_command': 'NORMAL'})
        ###
        smach.StateMachine.add('PLAY', MIRO_Play(),
                               transitions={'normal_command': 'NORMAL',
                                            'find_command': 'FIND'})

        smach.StateMachine.add('FIND', MIRO_Find(),
                               transitions={'play_command': 'PLAY',
                                            'f_track_command': 'F_TRACK'})

        smach.StateMachine.add('F_TRACK', F_Track(),
                               transitions={'find_command': 'FIND',
                                            'play_command': 'PLAY'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    cv2.destroyAllWindows()
    sis.stop()


if __name__ == '__main__':
    main()
