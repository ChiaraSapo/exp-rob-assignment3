
<p align="center">
  <img height="400" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment3/blob/master/exp_assignment3/images/Screenshot%20from%202020-12-28%2016-05-37.png?raw=true "Title"">
</p>


To see the states: rosrun smach_viewer smach_viewer.py


The human can interact by:
Sending a Play command to the robot, followed by a GoTo command + target location (Entrance,
Closet, Living room, Kitchen, Bathroom, Bedroom)

# A bit of theory

## Creating a map: gmapping
The robot creates a map of the environment by using the ROS package gmapping, that applies a filter based approach to build the map. As a launch file for gmapping I modified the sim_w1.launch file (given) to adapt it to my world and robot. The parameters of the gmapping node were also modified to accept the link_chassis link as base_frame.

Main topics needed by gmapping: tf (odom) and scan (laser).

Main topics  published in gmapping: map (occupancy grid), map_metadata, entropy (precision).

To see the map: in rviz -> add -> by topic -> map. To move via keyboard: rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

## Planning: global + local (MoveBase)
Default global planners: carrot, navfn (Dijkstra), global. I use navfn. 

Default local planners: dinamic window approach, elastic band and teb. I use dwa, that makes some steps: 
- sample in robot's control space
- perform forward simulation and score results of different trajectories
- choose the highest-scoring (i.e. no obstacles on the way) velocity of the mobile base
- apply it

The parameters for the cost map are in the folder param. 

We also need to set the simulation time for the second step: trade-off between longer paths (high) and computational time (low) and also  between flexible trajectories (low) and optimal trajectories (high). Here: time 2. Velocity samples vx, vy, vth (take vth higher: rotating is more complicated). Here: 10 vx-vy, 40 vth. 
To decide trajectory scoring, local planner maximizes an objective fnct to obtain optimal velocity pairs. The function depends on 3 params you choose: pdist_scale (dist to endpoint of traj), gdist_scale (dist endpoint to local goal), occdist_scale (max obstacle cost along the traj). Here: 5,0,5.
Other params: yaw goal tolerance (in rad). Here 3.14 (then change!). xy goal tolerance (in meters) Here 0.3

Costmap is composed of 3 layers:
- static map: interprets the static slam map provided to the navigation stack
- obstacle map: includes 2D adn 3D obstacles
- inflation layer: inflated obstacles to calculate cost of each 2D cell (0-255). Params: inflation radius (how far from obstalce is the zero cost of the obstacle itself) + cost scaling factor (inversely proportional to cost of a cell, if low: robot stays farther from obstacles). Costmap resolution.
Also, there is a global costmap (built before) and a local costmap genreated by using data from robot's sensors in runtime.

Recovery behaviours (when robot gets stuck): clear costmap recovery (set local costmap state to the same of global), rotate recovery (rotate of 360°). If they don't work, implement other methods (ex: go back to previous pas, or set a temporary goal close to the robot).

## Explore autonomously: explore-lite package (+moveBase)
Greedy frontier based exploration: explore greedily until no frontiers can be found, by ending goals to moveBase server. Teh costmap is the one of movebase, not another. Link chassis as base frame also here.
Sets frontiers to explore


# Folders
# worlds
- house2.world (given, slightly modified): simulation with a custom-built world: an environment divided into 6 rooms. In each room, there is a ball of a different colour. Each colour is therefore associated to a different room. The has an initial position of: x = -5.0, y = 8.0, and with a yaw of -1.57 rad.

# launch 
- exp3.launch: sets my params, launches the other useful launch files 
- gmapping.launch: implements gmapping
- move_base: implements movebase
- sim_w1: creates the gazebo enviroment

# urdf 
- human.urdf (given): person
- robot.gazebo and robot.xacro: improvements of the files of assignment  (a head hokuyo laser sensor was added to the previous robot)

# Scripts
- go_to_point_server
- move_dog_client
- state_manager: Different states are described:
  - SLEEP: 
  
    Go to kennel
    
    Stay still for 3 seconds
    
    Exit: NORMAL
    
  - NORMAL: In a loop:
  
    Listen to human: play command?
    
    Yes: exit: PLAY
    
    No: Continue.
    
    Move around: ball?
    
    Yes: exit: N_TRACK
    
    No: Continue.
    
    End of the loop: exit: SLEEP

  - N_TRACK
  
    Go close to the ball: did you know its position yet?
    
    Yes: continue.

    No: save it.
    
    Exit NORMAL

  - PLAY
   
    In a loop:
    
    Go to human,
    
    Wait for a goto command,
    
    Compare command to the known ball positions: is position known?
    
    Yes: go to position.
    
    No: exit FIND.
    
    Wait to be arrived.
    
    End of the loop: exit: NORMAL

  - FIND
    
    In a loop:
    
    Move towards goal (may change it): ball?
    
    Yes: exit F_TRACK
    
    No: continue
    
    End of the loop: exit PLAY

  - F_TRACK
    
    Go close to the ball: is it the desired position? (no need to check if saved: u enter here only if it's not)
    
    Yes: exit PLAY
    
    No: exit FIND
