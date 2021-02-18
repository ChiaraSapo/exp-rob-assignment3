Next time: maybe implement the yaw, run for a long time.

# A little bit of theory

## OpenCV
Computer vision library for ROS: package vision_opencv and cv_bridge. This last one transforms openCV output format to ROS format adn vice versa: works as interface between ROS and openCV. 

## Creating a map: gmapping
The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot. 
As a launch file for gmapping I modified the sim_w1.launch file (given) to adapt it to my world and robot. The parameters of the gmapping node were also modified to accept the link_chassis link as base_frame.

### Main topics
Main topics needed by gmapping: 
- tf (odom) 
- scan (laser).
Main topics  published in gmapping: 
- map (occupancy grid)
- map_metadata
- entropy (precision).

## Planning (global + local):MoveBase
http://wiki.ros.org/move_base
The move_base is a major component of the navigation stack.
The move_base package provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner to accomplish its global navigation task. 

### Default global planners
- carrot: simple, doesn't work well in complex environments.
- navfn (based on the Dijkstra to find the path with lowest cost).
- global: similar to navfn but more flexible. 
Here the navfn was used.

### Default local planners: 
- dinamic window approach
- elastic band approach
- teb approach
Here the dwa was used. It relies on some steps: 
- sample in robot's control space, obtaining dx, dy, dth,
- perform forward simulation and score results of different trajectories depending on some predefined metrics,
- choose the highest-scoring trajectory (i.e. no obstacles on the way),
- apply the chosen velocity, then repeat.

### Some parameters to be set

- sample time (here: 2) that is a tradeoff between: 
  - high (approx. 5): longer paths but with high computational time
  - low (approx. 1): shorter paths but with lowe computational time
- Velocity samples vx, vy (here they are the same), vth . This last one should be chosen higher: rotating is more complicated. Here: 20 vx-vy, 40 vth. 
- cost function to be minimized depends on 3 params (here: 5,0,5.) 
  - pdist_scale (dist to endpoint of traj)
  - gdist_scale (dist endpoint to local goal)
  - occdist_scale (max obstacle cost along the traj). 
- yaw goal tolerance (in rad): here 1
- xy goal tolerance (in meters) Here 0.3

Costmap is composed of 3 layers:
- static map: interprets the static slam map provided to the navigation stack
- obstacle map: includes 2D and 3D obstacles
- inflation layer: has inflated obstacles to calculate cost of each 2D cell (0-255). Depends on two parameters: 
  - inflation radius (how far from an obstacle is the zero cost of the obstacle itself) 
  - cost scaling factor (inversely proportional to cost of a cell. If low: robot stays farther from obstacles). 
Then we need to set 
- costmap resolution (for both global and local costmap)
- footprint of the robot (here indicated as a circle of which we indicate the radius)
Finally, MoveBase allows some recovery behaviours (when robot gets stuck): 
- clear costmap recovery (set local costmap state to the same of global)
- rotate recovery (rotate of 360°)

## Explore autonomously: explore-lite package (+moveBase)
http://wiki.ros.org/explore_lite
It implements a greedy frontier based exploration: explore greedily until no frontiers can be found, by sending goals to moveBase server.

# Folders
# worlds
- house2.world (given, slightly modified): simulation with a custom-built world: an environment divided into 6 rooms. In each room, there is a ball of a different colour. Each colour is therefore associated to a different room. The robot has an initial position of: x = -5.0, y = 8.0, and with a yaw of -1.57 rad. 

# launch 
- exp3.launch: sets my params, launches the other useful launch files 
- gmapping.launch: implements gmapping
- move_base: implements movebase
- sim_w1: creates the gazebo enviroment

# urdf 
- human.urdf (given): person
- robot.gazebo and robot.xacro: improvements of the robot of assignment 2 (a head hokuyo laser sensor was added to the previous robot, the head revolute joint was substituted with a fixed joint since the robot didn't need to rotate its head anymore). Now the robot comprises of: camera + laser scan

# Scripts
## State manager
It implements a smach machine where different states are described:
<p align="center">
  <img height="400" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment3/blob/master/exp_assignment3/images/Screenshot%20from%202020-12-28%2016-05-37.png?raw=true "Title"">
</p>
- **Sleep** : Dog goes to kennel via MoveBase, stays still for a few seconds, then enters the Normal behaviour.
- **Normal** : In a loop: It starts an autonomous wandering phase via Explore_lite. In the meanwhile it continuously checks whether it sees the ball. In case it actually sees it, it enters in the Normal_track phase. Then the dog listens to human: if it hears a play command it enters in play behaviour. At the end of the loop, if nothing has happened, the dog goes to Sleep.
- **N_track** : The dog gets close to the ball and checks if it already knew its position. In case it didn't, it saves the new position. Then it goes back to Normal state.
- **Play** : In a loop: the dog goes to the human, waits for a goto command and, if it hears it, it compares it to the known ball positions to check if it already knows the position the user has said to him. If it knew it, it goes toward that position, and then goes to Normal. However, if it didn't know the room yet, it goes to Find.
- **Find** : In a loop: the dog starts an autonomous wandering phase via Explore_lite. In the meanwhile it continuously checks whether it sees the ball. In case it actually sees it, it enters in the Find_track phase. At the end of the loop, if nothing has happened, the dog goes to Play.
- **F_track** : The dog gets close to the ball and checks if it ais the desired ball. In case it isn't, it saves its position (if needed) and goes back to Find. If it is, it goes to Play.
   
### Other important features
Explore_lite was used by launching and then stopping the explore.launch file from within the states that needed it.

MoveBase was used by creating a simple action client with a MoveBaseAction in the function move_dog. This function first sets the right angle towards the target by directly publishing on the cmd_vel topic, then calls the MoveBase service to go there. MoveBase assures obstacle avoidance but works better if the angle is previously set (problems in moving towards the human during play). 

The camera info is read by subscribing to the topic camera_info, and the odometry is read by subscribing to the topic odom.

The user is implemented by the function user_says in which he can interact by sending a Play command to the robot, followed by a GoTo command + target location (Entrance, Closet, Living room, Kitchen, Bathroom, Bedroom)

## Camera_manager

The camera was used by implementing cv bridge in class camera_manager_fnct. It subscribes to /camera1/image_raw/compressed to receive images and publishes on /output/image_raw/compressed to show them.

In particular, the camera_manager_fnct class continuously checks the environment for the colored balls and, if one is seen, it sets some parameters to be read by the smach machine:
- **justDetected** indicates the ball that has just been detected
- **closeBall** indicates that the ball is close (threshold of proximity was chosen manually)
- **radius**, **center** of the ball as seen from the camera.

It then publishes on the topic "camera_info" an array of integers that contains, in order: lastDetected, closeBall, radius, center

