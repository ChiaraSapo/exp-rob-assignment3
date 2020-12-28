#!/usr/bin/env python

# @file state_manager.py
# @brief

# Sleep stays the same but with obstacle avoidance.
# Normal stays the same but with obstacle avoidance and vision and a sub-state Track
#   Track: go close to ball, store info about its position, then go back to normal.
# Play: activated by the human. Go to human, wait for a goto location.
# if location has already been found: go there
# else: go in find state.
# When robot has reached position, stay there a few seconds then go back
# After a while, go to normal state.
# Find: explore environment to find balls. If a ball is detected go to sub-state Track
#   Track: go close to ball, store info about its position, then:
# if the position of the ball is the desired position, go to play state and back to human.
# else: go to find state again and move
# If robot doesn't find the ball for a while, go back to Play state in any case and back to human.
# Also switch from play to normal and vice versa randomly.

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

vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)


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

    global subscriberNORM, vel_Norm, SEARCH_FOR_BALL

    # Init velocity
    vel_Norm.linear.x = 0
    vel_Norm.linear.y = 0
    vel_Norm.linear.z = 0
    vel_Norm.angular.x = 0
    vel_Norm.angular.y = 0
    vel_Norm.angular.z = 0

    # rospy.loginfo('entered img NORM fnct')

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
        rospy.loginfo('callback norm fnct: ball')
        # Ball was found
        rospy.set_param('ball', 1)
        subscriberNORM.unregister()

    else:

        vel_Norm.angular.z = 0.3
        vel_pub.publish(vel_Norm)

        # Ball was not found
        rospy.loginfo('callback norm fnct: no ball')
        rospy.set_param('ball', 0)
        subscriberNORM.unregister()


# This class is used to detect and follow the green ball in the arena.
class find_and_follow_ball:

    # It initializes a publisher to the image, one to the robot velocity and one to the camera motor.
    # It subscribes to the camera image
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # Init publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)
        self.camera_pub = rospy.Publisher(
            "/robot/joint_position_controller/command", Float64, queue_size=1)

        # Subscribed Topic
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
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

# This function is a client for the robot motion server. It sends the desired position.


def move_dog(target):

    time.sleep(3)

    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

    client.wait_for_server()

    rospy.loginfo('Going to %d %d %d', target[0], target[1], target[2])

    # Creates a goal to send to the action server.
    goal = exp_assignment2.msg.PlanningGoal()
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.position.z = target[2]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    rospy.loginfo('arrived, exiting dog fnct')

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

        self.camera_pub = rospy.Publisher("/robot/joint_position_controller/command",
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
