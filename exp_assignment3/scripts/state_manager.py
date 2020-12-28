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
