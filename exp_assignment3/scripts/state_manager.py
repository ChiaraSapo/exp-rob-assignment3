#!/usr/bin/env python

## @file state_manager.py
## @brief Manges the dog behaviour states and the human commands.

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
import math
import actionlib
import actionlib.msg
import sys
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, UInt32, Int64MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import signal
import subprocess
import roslaunch
from tf import transformations
from exp_assignment3.msg import camera_msg

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
lastDetected = -2

LOOPS_EXPLORE = 20

# Robot positions
position_ = Point()
pose_ = Pose()
yaw_ = 0
yaw_precision_2_ = 0.5

# Desired robot position
moveTo = [0, 0, 0]

# Velocity publisher to go near ball
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
vel_to_ball = Twist()
vel_to_ball.linear.x = 0
vel_to_ball.linear.y = 0
vel_to_ball.linear.z = 0
vel_to_ball.angular.x = 0
vel_to_ball.angular.y = 0
vel_to_ball.angular.z = 0

# Variables to move towards the balll
radius = 0
circCenter = 0


## Callback function for the camera information

def clbk_cam(msg):
    global justDetected, closeBall, radius, circCenter
    justDetected = msg.justDetected.data
    closeBall = msg.closeBall.data
    radius = msg.radius.data
    circCenter = msg.circCenter.data


## Function to simulate the human speaking

def user_says(stateCalling):

    # Called from Normal behaviour
    if stateCalling == 0:
        userVoice = random.choice(['play', ''])
        # rospy.loginfo('user said: %s', userVoice)

    # Called from Play behaviour
    elif stateCalling == 1:
        i = random.randrange(0, 6)
        comm = "go to %s" % (colorName[i])
        userVoice = comm

    else:
        rospy.loginfo('user_says function called without input')
        return 0

    return userVoice


## Callback function for the odometry of the robot

def clbk_odom(msg):

    global position_
    global pose_
    global yaw_

    # Read position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # Read yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


## Function to normalize the yaw angle

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


## Function to move the dog to a target position. It first sets the right yaw angle
## by directly publishing on the /cmd_vel topic then implements a client to the
## MoveBase service to move the robot to the target position while avoiding obstacles.

def move_dog(target):
    global yaw_, pub, yaw_precision_2_, vel_pub, position_

    kp_a = 3.0
    ub_a = 0.6
    lb_a = -0.5

    rospy.loginfo('move dog started with data %d %d', target[0], target[1])

    # Adjust yaw angle
    while True:
        desired_yaw = math.atan2(
            target[1] - position_.y, target[0] - position_.x)
        err_yaw = normalize_angle(desired_yaw - yaw_)

        twist_msg = Twist()

        if math.fabs(err_yaw) > yaw_precision_2_:
            twist_msg.angular.z = kp_a*err_yaw
            if twist_msg.angular.z > ub_a:
                twist_msg.angular.z = ub_a
            elif twist_msg.angular.z < lb_a:
                twist_msg.angular.z = lb_a

        # Publish velocity directly on cmd_vel topic
        vel_pub.publish(twist_msg)

        if math.fabs(err_yaw) <= yaw_precision_2_:
            break

    # Move to target position
    rospy.loginfo('now set target')

    # Call MoveBase service
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.cancel_goal
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)

    while math.fabs(position_.x - target[0]) > 0.3 or math.fabs(position_.y - target[1]) > 0.3:
        time.sleep(1)
    rospy.loginfo('arrived')
    client.cancel_all_goals()
    return 1


## Sleep state of the smach machine.

class MIRO_Sleep(smach.State):

    ## Init function for smach machine sleep state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    ## Execute function of the state:
    ## - Go to kennel
    ## - Stay still for 5 seconds
    ## - Exit NORMAL

    def execute(self, userdata):
        global lastDetected
        rospy.loginfo('sleep')

        # Go to kennel
        move_dog([-5, 6, 0])
        time.sleep(1)

        # Reset previously seen ball variable
        lastDetected = -2

        rospy.loginfo('exit sleep')
        return 'normal_command'


## Normal state of the smach machine.


class MIRO_Normal(smach.State):

    ## Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['sleep_command', 'play_command', 'n_track_command'])

    ## Execute function of the state:
    ## - Move around: ball?
    ## 	- Yes: exit N_TRACK
    ## 	- No: Continue
    ## - Listen to human: play command?
    ## 	- Yes: exit PLAY
    ## 	- No: Continue
    ## - End of the loop: exit SLEEP

    def execute(self, userdata):
        global justDetected, lastDetected

        rospy.loginfo('normal')
        time.sleep(2)

        # Launch explore for autonomous exploration
        command = ["roslaunch", "explore_lite", "explore.launch"]
        p = subprocess.Popen(command)

        for loops in range(0, 10):
            time.sleep(1)

            for loops2 in range(0, LOOPS_EXPLORE):
                time.sleep(3)

                # Check if ball is detected
                if justDetected != -1:
                    rospy.loginfo('a ball was detected')

                    # Check if it's a new ball or the same as before
                    if justDetected != lastDetected:
                        lastDetected = justDetected
                        rospy.loginfo('and it is %s', colors[justDetected])

                        # Go to track state
                        p.terminate()
                        time.sleep(7)

                        return 'n_track_command'

                    else:
                        rospy.loginfo('but i already knew it')

            # Listen to user
            if user_says(0) == 'play':
                rospy.loginfo('user said play')

                p.terminate()
                time.sleep(7)
                return 'play_command'

        p.terminate()
        time.sleep(7)
        return 'sleep_command'

## Normal track state of the smach machine.

class N_Track(smach.State):

    ## Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    ## Execute function of the state:
    ## - Go close to the ball: did you know its position yet?
    ## 	- Yes: continue
    ## 	- No: save it
    ## - Exit NORMAL

    def execute(self, userdata):
        global vel_pub, closeBall, colorName, circCenter

        rospy.loginfo('N_track')
        time.sleep(2)

        # Move closer to the ball
        rospy.loginfo('Moving closer to ball')
        while closeBall == -1:
            vel_to_ball.angular.z = -0.002*(circCenter-400)
            vel_to_ball.linear.x = -0.01*(radius-100)
            vel_pub.publish(vel_to_ball)

        # Stop dog
        rospy.loginfo('Stopping in front of ball')
        for i in range(0, 3):
            vel_to_ball.linear.x = 0
            vel_pub.publish(vel_to_ball)
        time.sleep(3)

        # If not saved yet, save ball's position
        if ballsPos[justDetected] == [0, 0]:
            ballsPos[justDetected] = [position_.x, position_.y]

            rospy.loginfo('Saved position of %s ball as %d, %d approximately',
                         colors[justDetected], position_.x, position_.y)

        return 'normal_command'


## Play state of the smach machine.


class MIRO_Play(smach.State):

    ## Init function for smach machine play state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command', 'find_command'])

    ## Execute function of the state:
    ## - In a loop:
    ## - Go to human
    ## - Wait for a goto command
    ## - Compare command to the known ball positions: is position known?
    ## 	- Yes: go to position
    ## 	- No: exit FIND
    ## - Wait to be arrived
    ## - End of the loop: exit NORMAL

    def execute(self, userdata):
        global moveTo, ballsPos, colorName, justDetected

        rospy.loginfo('play')
        lastDetected = -2

        if ballsPos[0] != [0, 0] and ballsPos[1] != [0, 0] and ballsPos[2] != [0, 0] and ballsPos[3] != [0, 0] and ballsPos[4] != [0, 0] and ballsPos[5] != [0, 0]:
            rospy.loginfo('ALL BALLS HAVE BEEN DETECTED! Positions:')
            rospy.loginfo(ballsPos)

        for loops in range(0, 10):

            # Move to the human, unless dog is already near him
            rospy.loginfo('moving to human')
            if (position_.x > -7 or position_.x < -3) and (position_.y > 10 or position_.y < 6):
                move_dog([-5, 8, 0])
                time.sleep(1)

                # Listen to human
            rospy.loginfo('listen to user')
            user_command = user_says(1)

            # Save the desired room
            if 'go' in user_command and 'to' in user_command:
                for i in range(0, 6):
                    if colorName[i] in user_command:
                        desiredRoom = i
                        break

            else:
                rospy.loginfo('Wrong command received')
                return 'normal_command'

            rospy.loginfo('user said to go to %s which is color %s',
                         colorName[desiredRoom], colors[desiredRoom])

            # If position of room (i.e. ball) is known, go there...
            if ballsPos[desiredRoom] != [0, 0]:
                rospy.loginfo(
                    'i know this ball! i will go to it right away! it is in')
                moveTo = np.append(np.asarray(ballsPos[desiredRoom]), 0)
                move_dog(moveTo)

            # ... if not: look for it
            else:
                rospy.loginfo('i do not know this position, i will look for it')
                return 'find_command'

        return 'normal_command'


## Find state of the smach machine.

class MIRO_Find(smach.State):

    ## Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['play_command', 'f_track_command'])

    ## Execute function of the state:
    ## - In a loop:
    ## - Move towards goal (may change it): ball?
    ## 	- Yes: exit F_TRACK
    ## 	- No: continue
    ## - End of the loop: exit PLAY

    def execute(self, userdata):

        global justDetected

        rospy.loginfo('find')

        # Launch explore for autonomous exploration
        command2 = ["roslaunch", "explore_lite", "explore.launch"]
        p = subprocess.Popen(command2)

        for loops in range(0, 10):
            time.sleep(1)

            for loops2 in range(0, LOOPS_EXPLORE):
                time.sleep(3)

                # Check if ball is detected
                if justDetected != -1:
                    rospy.loginfo('I found something!')
                    p.terminate()
                    time.sleep(7)

                    return 'f_track_command'

        p.terminate()
        time.sleep(7)

        return 'play_command'

## Find track state of the smach machine.

class F_Track(smach.State):

    ## Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['find_command', 'play_command'])

    ## Execute function of the state:
    ## - Go close to the ball: is it the desired position? (no need to check if saved: u enter here only if it's not)
    ##	 - Yes: exit PLAY
    ##	 - No: exit FIND

    def execute(self, userdata):
        global moveTo, vel_pub, closeBall, justDetected, position_, ballsPos, desiredRoom

        rospy.loginfo('f_track')

        # Move closer to the ball
        while closeBall == -1:
            vel_to_ball.angular.z = -0.002*(circCenter-400)
            vel_to_ball.linear.x = -0.01*(radius-100)
            vel_pub.publish(vel_to_ball)

        # Stop dog
        rospy.loginfo('Stopping in front of ball')
        for i in range(0, 3):
            vel_to_ball.linear.x = 0
            vel_pub.publish(vel_to_ball)

        # If the ball is the one the human asked for, go back to play...
        if colorName[justDetected] == colorName[desiredRoom]:
            rospy.loginfo('I found the right ball')

            # If not saved yet, save ball's position
            if ballsPos[justDetected] == [0, 0]:
                ballsPos[justDetected] = [position_.x, position_.y]
                rospy.loginfo(
                    'Saved position of the %s ball, will not forget it!', colors[justDetected])

            return 'play_command'

        # ... if it is not, save its position anyway
        else:
            rospy.loginfo(
                'I found the wrong ball')

            # If not saved yet, save ball's position
            if ballsPos[justDetected] == [0, 0]:
                ballsPos[justDetected] = [position_.x, position_.y]
                rospy.loginfo(
                    'Saved position of the %s ball anyway, may turn out useful', colors[justDetected])

            return 'find_command'

## Ros node that subscribes to camera_info and odometry
def main():

    rospy.init_node('state_manager')

    sub_cam = rospy.Subscriber('camera_info', camera_msg, clbk_cam)

    # Control odometery
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
