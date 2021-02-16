#!/usr/bin/env python

# @file state_manager.py
# @brief

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
import sys
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, UInt32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import signal
import subprocess
import roslaunch


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

# Balls positions
blackPos = [0, 0]
redPos = [0, 0]
yellowPos = [0, 0]
greenPos = [0, 0]
bluePos = [0, 0]
magentaPos = [0, 0]
ballsPos = [blackPos, redPos, yellowPos, greenPos, bluePos, magentaPos]

# Last detected ball, close ball
lastDetected = -1
closeBall = -1
desiredRoom = -1

# Robot positions
position_ = Point()
pose_ = Pose()
yaw_ = 0

# Desired robot position
moveTo = [0, 0, 0]

# Velocity publisher to go near ball
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


def user_says(stateCalling):

    if stateCalling == 0:  # Normal behaviour
        userVoice = 'play'  # random.choice(['play', ''])
        rospy.logerr('user said: %s', userVoice)

    elif stateCalling == 1:  # Play behaviour
        i = random.randrange(0, 6)
        comm = "go to %s" % (colorName[0])
        userVoice = comm
        rospy.logerr('user said to go to %s', colorName[0])

    else:
        rospy.logerr('user_says function called without input')
        return 0

    return userVoice


class camera_manager:

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
        global position_, closeBall

        # Convert to cv2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Create masks
        for numMask in range(0, 6):

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
                # vel_Play.angular.z = -0.002*(center[0]-400)
                # vel_Play.linear.x = -0.01*(radius-100)
                # self.vel_pub.publish(vel_Play)

                rospy.logerr('radius %d', radius)

                if radius > 10 and radius < 50:
                    lastDetected = numMask
                    closeBall = -1

                elif radius >= 50:
                    lastDetected = numMask
                    closeBall = 1

                else:
                    lastDetected = -1
                    closeBall = -1

            cv2.imshow('window', image_np)
            cv2.waitKey(2)

# This function is a client for the robot motion server. It sends the desired position.


def clbk_odom(msg):

    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def move_dog(target):  # then add orientation

    # Source: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

    rospy.logerr('move dog started with data %d %d', target[0], target[1])

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # rospy.loginfo('Going to %d %d with angle %d', target[0], target[1], target[2])

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()

    rospy.logerr('Almost there')

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


# Sleep state of the smach machine.


class MIRO_Sleep(smach.State):

    # Init function for smach machine sleep state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    # Go to kennel
    # Stay still for 5 seconds
    # Exit NORMAL

    def execute(self, userdata):
        rospy.logerr('sleep')
        # move_dog([0, 0, 0])
        time.sleep(1)

        rospy.logerr('exit sleep')
        return 'normal_command'


# Normal state of the smach machine.


class MIRO_Normal(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['sleep_command', 'play_command', 'n_track_command'])

    # Move around: ball?
    # - Yes: exit N_TRACK
    # - No: Continue
    # Listen to human: play command?
    # - Yes: exit PLAY
    # - No: Continue
    # End of the loop: exit SLEEP

    def execute(self, userdata):

        rospy.logerr('normal')
        time.sleep(2)

        for loops in range(0, 10):

            # Launch explore for autonomous exploration
            command = ["roslaunch", "explore_lite", "explore.launch"]
            p = subprocess.Popen(command)

            # Check if ball is detected
            for loops2 in range(0, 20):
                time.sleep(5)
                if lastDetected != -1:
                    p.terminate()
                    time.sleep(7)
                    return 'n_track_command'

            # Listen to user
            if user_says(0) == 'play':
                p.terminate()
                time.sleep(7)
                return 'play_command'

        p.terminate()
        return 'sleep_command'


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
        global vel_pub, closeBall

        rospy.logerr('N_track')
        time.sleep(2)

        # Move closer to the ball
        rospy.logerr('Moving closer to ball')
        vel_Play = Twist()
        while closeBall == -1:
            vel_Play.linear.x = 0.1
            vel_pub.publish(vel_Play)

        # Stop dog
        rospy.logerr('Stopping in front of ball')
        vel_Play.linear.x = 0
        vel_pub.publish(vel_Play)

        # If not saved yet, save ball's position
        if ballsPos[lastDetected] == [0, 0]:
            ballsPos[lastDetected] = [position_.x, position_.y]

            rospy.logerr('Saved position of %s ball as %d, %d approximately',
                         colorName[lastDetected], position_.x, position_.y)

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
        global moveTo, ballsPos, colorName

        rospy.logerr('play')

        for loops in range(0, 10):

            # Move to the human
            rospy.logerr('moving to human')
            move_dog([-5, 8, 0])

            # Listen to human
            rospy.logerr('listen to user')
            user_command = user_says(1)

            # Save the desired room
            if 'go' in user_command and 'to' in user_command:
                for i in range(0, 6):
                    if colorName[i] in user_command:
                        desiredRoom = i
                        break

            else:
                rospy.logerr('Wrong command received')
                return 'normal_command'

            # If position of room (i.e. ball) is known, go there...
            if ballsPos[desiredRoom] != [0, 0]:
                rospy.logerr(
                    'i know this ball! i will go to it right away! it is in')
                moveTo = np.append(np.asarray(ballsPos[desiredRoom]), 0)
                move_dog(moveTo)

            # ... if not: look for it
            else:
                rospy.logerr('i do not know this position, i will look for it')
                return 'find_command'

        return 'normal_command'


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
        global lastDetected

        rospy.logerr('find')

        for loops in range(0, 10):

            # Launch explore for autonomous exploration
            command = command = ["roslaunch", "explore_lite", "explore.launch"]
            p = subprocess.Popen(command)

            for loops2 in range(0, 10):

                # Check if ball is detected
                if lastDetected != -1:
                    p.terminate()
                    time.sleep(7)
                    return 'f_track_command'

            p.terminate()
            time.sleep(7)

        return 'play_command'


class F_Track(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['find_command', 'play_command'])

    # Go close to the ball: is it the desired position? (no need to check if saved: u enter here only if it's not)
    # - Yes: exit PLAY
    # - No: exit FIND

    def execute(self, userdata):
        global moveTo, vel_pub, closeBall, lastDetected, position_, ballsPos, desiredRoom

        rospy.logerr('f_track')

        # Move closer to the ball
        vel_Play = Twist()

        while closeBall == -1:
            vel_Play.linear.x = 0.1
            vel_pub.publish(vel_Play)

        # Stop dog
        vel_Play.linear.x = 0
        vel_pub.publish(vel_Play)

        # If the ball is the one the human asked for, go back to play...
        if colorName[lastDetected] == colorName[desiredRoom]:
            rospy.logerr('found the right ball')

            # If not saved yet, save ball's position
            if ballsPos[lastDetected] == [0, 0]:
                ballsPos[lastDetected] = [position_.x, position_.y]
                rospy.logerr('Saved position of the ball, will not forget it!')

            return 'play_command'

        # ... if it is not, save its position anyway
        else:
            rospy.logerr(
                'found the wrong ball, saving its position anyway, may turn out useful')

            # If not saved yet, save ball's position
            if ballsPos[lastDetected] == [0, 0]:
                ballsPos[lastDetected] = [position_.x, position_.y]

            return 'find_command'


def main():

    rospy.init_node('state_manager')

    camera_manager()

    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)

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
