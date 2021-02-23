# Experimental robotics final assignment 
This repository contains two packages. The explore package (https://github.com/CarmineD8/m-explore), which is a ROS package for robot exploration, and my package exp_asssignment3, which simulates a robot moving in a complex environment composed of multiple rooms charachterized by differently colored balls. The correlation between balls and rooms is:
- Blue : Entrance
- Red: Closet
- Green: Living Room
- Yellow : Kitchen
- Orange: Bathroom
- Black: Bedroom

<p align="center">
  <img height="400" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment3/blob/master/exp_assignment3/images/Screenshot%20from%202021-02-16%2016-25-48.png?raw=true "Title"">
</p>

The robot is a dog that moves on two wheels with differential drive control. It perceives the environment through a **hokuyo laser sensor** and a **RGBD camera**. It can "interact" in a simple way with a human, represented as still in the environment.

The dog has different **behavioural states**, the main ones being sleep, normal, play and find:
- During **sleep**, the dog simply goes to the kennel, stays there for a while then wakes up. 
- During **normal** state it wanders around.If it sees a ball it enters a substate **normal track**, goes closer to it, and saves the ball's approximate position as its own position. Of course it saves it only if it didn't prevoiusly know the ball's position yet. If it hears the human call him to play, it enters play state. 
- During **play** state the dog first goes close to the human, then waits for a command, which must be of type "go to *room*". If the dog knows this room, it goes there, then goes back to the human. If it doesn't, it enters in Find state.
- During **find** state, the robot goes around looking for the ball linked to the command room. If it finds any ball during search, it enters substate **find track**: if it is the ball it was looking for, it has succeded in its mission and can re-enter play state. If it isn't, it saves its position and continues in find state. Of course it saves it only if it didn't prevoiusly know the ball's position yet.

To implement these behaviours I used some ROS packages:
- smach state machine to implement the switch between states
- vision_opencv and cv_bridge for vision through camera
- gmapping to create a map of the environment 
- move_base as a local and global planner to reach a goal position with obstacle avoidance
- explore_lite for autonomous exploration of the environment

The code is written entirely in Python. I used ROS Kinetic, Rviz (as visualization tool) and Gazebo (as simulation tool).

# Software architecture 
The general architecture is simply composed by 2 nodes:
- state_manager node that manages the robot's behavioural states and the human commands.
- camera_manager node that manages the camera information and extracts relevant information about the balls positioned in te environment.

The two nodes communicate via a topic (published by camera_manager) on which information is sent. 

The state_manager implements a state machine represented in the following (To visualize it: *rosrun smach_viewer smach_viewer.py*):
<p align="center">
  <img height="400" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment3/blob/master/exp_assignment3/images/Screenshot%20from%202020-12-28%2016-05-37.png?raw=true "Title"">
</p>
The states will be analyzed in detail in the next section.

# Main folders

## launch 
- **exp3**: launches the useful launch files.
- **gmapping**: implements gmapping.
- **move_base**: implements movebase.
- **sim_w1**: creates the gazebo enviroment.

## msg
- camera_msg: custom message to send information about the images received from the camera. It comprises:
  - **justDetected** indicates the ball that has just been detected
  - **closeBall** indicates that the ball is close (threshold of proximity was chosen manually)
  - **radius**, **center** of the ball as seen from the camera.

## param
Costmap and moveBase parameters. Some default MoveBase parameters are changed:
  - sample time = 2
  - Velocity samples vx, vy (here they are the same), vth . This last one should be chosen higher: rotating is more complicated. Here: vx-vy = 20 , vth = 40. 
  - cost function to be minimized depends on 3 params 
    - pdist_scale (dist to endpoint of traj) = 5
    - gdist_scale (dist endpoint to local goal) = 0
    - occdist_scale (max obstacle cost along the traj) = 5
  - yaw goal tolerance (in rad) = 1  - xy goal tolerance (in meters) = 0.3

## src
Contains the slam_gmapping nodes that provide laser-based SLAM (Simultaneous Localization and Mapping). They create a 2D occupancy grid map from laser and pose data collected by the mobile robot. 

## Scripts
- **State manager**
  This file implements a ROS node that subscribes to **/camera_info**, **/odom** and publishes on **/cmd_vel**.
  It implements a smach machine where different states are described:

  - **sleep** : Dog goes to kennel (position chosen in [-5, 6, 0]) via MoveBase, stays still for a few seconds, then enters the Normal behaviour.

  - **normal** : In a loop: It starts an autonomous wandering phase via Explore_lite. In the meanwhile it continuously checks whether it sees the ball. In case it actually sees it, it enters in the Normal_track phase. Then the dog listens to human: if it hears a play command it enters in play behaviour. At the end of the loop, if nothing has happened, the dog goes to Sleep.

  - **n_track** : The dog gets close to the ball and checks if it already knew its position. In case it didn't, it saves the new position. Then it goes back to Normal state. Note: to understand the balls posion and proximity it reads the camera_info data.

  - **play** : In a loop: the dog goes to the human (position chosen in [-5, 8, 0]), waits for a goto command and, if it hears it, it compares it to the known ball positions to check if it already knows the position the user has said to him. If it knew it, it goes toward that position, and then goes to Normal. However, if it didn't know the room yet, it goes to Find.

  - **find** : In a loop: the dog starts an autonomous wandering phase via Explore_lite. In the meanwhile it continuously checks whether it sees the ball. In case it actually sees it, it enters in the Find_track phase. At the end of the loop, if nothing has happened, the dog goes to Play.

  - **f_track** : The dog gets close to the ball and checks if it is the desired ball. In case it isn't, it saves its position (if needed) and goes back to Find. If it is, it goes to Play.
   

- **Camera_manager**
  This file implements a ROS node that subscribes to **/camera1/image_raw/compressed** and publishes on **/output/image_raw/compressed**, **/camera_info**.

  The camera was used by implementing cv bridge in class camera_manager_fnct. 
  It subscribes to /camera1/image_raw/compressed to receive images and publishes on /output/image_raw/compressed to show them.

  In particular, the camera_manager_fnct class continuously checks the environment for the colored balls and, if one is seen, it sets some variables to be read by the smach machine:
  - **justDetected** indicates the ball that has just been detected
  - **closeBall** indicates that the ball is close (threshold of proximity was chosen manually)
  - **radius**, **center** of the ball as seen from the camera.

  It then publishes on /camera_info an array of integers that contains, in order: lastDetected, closeBall, radius, center.

## urdf 
- human.urdf (given)
- robot.gazebo and robot.xacro: describe a robot dog with a differential drive control for the two wheels and a fixed head. As sensors it has a hokuyo laser sensor and a camera. 
<p align="center">
  <img height="400" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment3/blob/master/exp_assignment3/images/robot_xacro_page-0001.jpg?raw=true "Title"">
</p>

## worlds
- **house2.world** (given, slightly modified): simulation with a custom-built world: an environment divided into 6 rooms. In each room, there is a ball of a different colour. Each colour is therefore associated to a different room. The robot has an initial position of: x = -5.0, y = 8.0, and with a yaw of -1.57 rad. 


# Installation and running procedure
## Installation
Download the package in your_catkin_ws/src folder. 
```sh
cd "Your catkin workspace"/src/exp_assignment3
chmod +x install.sh
./install.sh
```
## Running procedure
```sh
roslaunch exp_assignment3 exp3.launch
```

# System's features
## Vision though camera: OpenCV
Computer vision library for ROS: package vision_opencv and cv_bridge. This last one transforms openCV output format to ROS format adn vice versa: works as interface between ROS and openCV. 

Learn more at http://wiki.ros.org/vision_opencv and http://wiki.ros.org/cv_bridge.

## Creating a map: gmapping

The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot. 
As a launch file for gmapping I modified the sim_w1.launch file (given) to adapt it to my world and robot. The parameters of the gmapping node were also modified to accept the link_chassis link as base_frame.

Main topics needed by gmapping: 
- tf (odom) 
- scan (laser).
Main topics  published in gmapping: 
- map (occupancy grid)
- map_metadata
- entropy (precision).

Learn more at http://wiki.ros.org/gmapping.

## Planning (global + local): MoveBase
The move_base is a major component of the navigation stack.
The move_base package provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner to accomplish its global navigation task. 

- **Default global planners**
  - carrot: simple, doesn't work well in complex environments.
  - navfn (based on the Dijkstra to find the path with lowest cost).
  - global: similar to navfn but more flexible. 
  
  Here the navfn was used.

- **Default local planners** 
  - dinamic window approach
  - elastic band approach
  - teb approach
  
  Here the dwa was used. It relies on some steps: 
  - sample in robot's control space, obtaining dx, dy, dth,
  - perform forward simulation and score results of different trajectories depending on some predefined metrics,
  - choose the highest-scoring trajectory (i.e. no obstacles on the way),
  - apply the chosen velocity, then repeat.

Learn more at: http://wiki.ros.org/move_base


In this project, MoveBase was used by creating a simple action client with a MoveBaseAction in the **function move_dog**. This function first sets the right angle towards the target by directly publishing on the cmd_vel topic, then it calls the MoveBase service to go there. MoveBase assures obstacle avoidance but I have seen that it works better if the angle is previously set. This means that I firstly set the angle so that the robot faces the goal, then I ask MoveBase service to go to that position. 

## Explore autonomously: explore-lite package
It implements a greedy frontier based exploration: explore greedily until no frontiers can be found, by sending goals to moveBase server.
Explore_lite was used by launching and then stopping the explore.launch file from within the states that needed it.

Learn more at:http://wiki.ros.org/explore_lite


## Other important features
The user is implemented by the **function user_says** in which he can interact by sending a Play command to the robot, followed by a GoTo command + target location (Entrance, Closet, Living room, Kitchen, Bathroom, Bedroom).

When the dog saves a ball's position, it actually saves its own position, thus making an approximation. It is assumed that it can do so because:
- the robot is forced to be very close to the ball
- each room has one ball only 
- rooms are big with respect to the dog 

# System's limitations
- Since explore_lite is launched and terminated everytime the dog enters in Normal or Find state, the frontiers are not saved properly.
- In order to check if the dog has arrived in the goal position, the code evaluates the odometry output and, if the difference between actual and goal positions is small, all goals sent to MoveBase are cancelled and the dog can switch state. This was implemented because, by using MoveBase only, the code took between 10 and 20 seconds to realize that the dog had arrived. After modifying the MoveBase paramters by trial and error, and still obtaining bad results, I decided to solve the porblem in this way.
- The robot recognizes balls only if it is in certain states. In all other cases, even if it sees a ball in front of it, it doesn't recognize it nor save its position.
- Because of how the environment is built, the code doesn't account for multiple balls seen at the same moment.
- The yaw control of the dog is not implemented. 

# Author and contacts
Chiara Saporetti (**chiara.saporetti@gmail.com**)
