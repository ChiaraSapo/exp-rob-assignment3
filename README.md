
<p align="center">
  <img height="400" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment3/blob/master/exp_assignment3/images/Screenshot%20from%202020-12-28%2016-05-37.png?raw=true "Title"">
</p>

The human can interact by:
Sending a Play command to the robot, followed by a GoTo command + target location (Entrance,
Closet, Living room, Kitchen, Bathroom, Bedroom)

## Gmapping
The robot creates a map of the environment by using the ROS package gmapping, that applies a filter based approach to build the map. As a launch file for gmapping I modified the sim_w1.launch file (given) to adapt it to my world and robot. The parameters of the gmapping node were also modified to accept the link_chassis link as base_frame.

Main topics needed by gmapping: tf (odom) and scan (laser).

Main topics  published in gmapping: map (occupancy grid), map_metadata, entropy (precision).

## Folders
worlds: 
- house2.world (given, slightly modified): simulation with a custom-built world: an environment divided into 6 rooms. In each room, there is a ball of a different colour. Each colour is therefore associated to a different room. The has an initial position of: x = -5.0, y = 8.0, and with a yaw of -1.57 rad.
launch:
urdf: 
- human.urdf (given): person
- robot.gazebo and robot.xacro: improvements of the files of assignment  (a head hokuyo laser sensor was added to the previous robot)
Scripts: 
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
