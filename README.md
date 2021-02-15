TODO nect time: check why cv bridge is so slow. implement the "go near ball" in track states.


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

The move_base ROS Node, is a major component of the navigation stack which allows to configure, run and interact with the latter. The move_base node implements a SimpleActionServer, an action server with a single goal policy, taking in goals of geometry_msgs/PoseStamped message type. To communicate with this node, the SimpleActionClient interface is used. The move_base node tries to achieve a desired pose by combining a global and a local motion planners to accomplish a navigation task which includes obstacle avoidance. (source https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/)


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
  - SLEEP: Dog goes to kennel via MoveBase, stays still for a few seconds, then enters the Normal behaviour.
  - NORMAL: In a loop:
    Dog listens to human: if it hears a play command it enters in play behaviour. Else, it starts an autonomous wandering phase via Explore_lite. In the meanwhile it continuously checks whether it sees the ball. In case it actually sees it, it enters in the Normal_track phase. At the end of the loop, if nothing has happened, the dog goes to Sleep.
   - N_TRACK: The dog gets close to the ball and checks if it already knew its position. In case it didn't, it saves the new position. Then it goes back to Normal state.
  - PLAY: In a loop: the dog goes to the human, waits for a goto command and, if it hears it, it compares it to the known ball positions to check if it already knows the position the user has said to him. If it knew it, it goes toward that position, and then goes to Normal. However, if it didn't know the room yet, it goes to Find.
  - FIND: In a loop: the dog starts an autonomous wandering phase via Explore_lite. In the meanwhile it continuously checks whether it sees the ball. In case it actually sees it, it enters in the Find_track phase. At the end of the loop, if nothing has happened, the dog goes to Play.
  - F_TRACK: he dog gets close to the ball and checks if it ais the desired ball. In case it isn't, it goes back to Find. If it is, it goes to Play.
   
Explore_lite was used by launching and then stopping the explore.launch file from within the states that needed it.

MoveBase was used by creating a simple action client with a MoveBaseAction in function move_dog.

The camera was used by implementing cv bridge in function camera_manager.
