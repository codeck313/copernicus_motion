# dwa_planner - Motion Planner for Copernicus

## Overview
Motion planner for copernicus using DWA

## Setup
### Pre-requisite
1. Have ROS Melodic installed on Ubuntu 18.
2. Download and install all the Copernicus packages from [here](http://wiki.ros.org/Robots/Copernicus)
4. Do  `catkin build`  in your [catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Configuration and starting the nodes

1. Open Gazebo world by ` roslaunch copernicus_simulation gazebo.launch `. To get better RTF you can turn off shadows in Scene settings.
2. Start Copernicus Simulation ` roslaunch copernicus_simulation simulation.launch `
3. [OPTIONAL] You can start the teleop by `roslaunch copernicus_teleoperator teleoperator.launch keyboard:=true`
4.  To start the dwa planner first make sure you have configured all the config files. Then use `roslaunch dwa_planner local_planner.launch`.
5. You can start Rviz simulation as well. The config file for it is in the config folder of the dwa_planner node. This will help in viz possible and selected trajectory.
6. Additionally python implementation of the DWA planner is given in `py_dwa.py`

## Config file Setup
### dwa_param.yaml
```yaml

GOAL_X: #Goal position
GOAL_Y: #Goal position

VELOCITY_RESOLUTION: # Resolution used for iterating through possible linear velocities
YAWRATE_RESOLUTION: # Resolution used for iterating through possible angular velocities
PREDICT_TIME: # Look ahead time for which traj needs to be calculated for
HZ: # Frequency with which trajectory needs to be calculated
TO_GOAL_COST_GAIN: # Cost factor on how far robot is wrt goal position
SPEED_COST_GAIN: # Cost factor for the speed, used to maximize the speed of the robot
OBSTACLE_COST_GAIN: # Applied on the distance to the nearest obstacle
HEADING_COST_GAIN: # Difference in heading between the traj final orientation and goal orientation at present
GOAL_THRESHOLD: #error threhold for distance
TURN_DIRECTION_THRESHOLD: #error threhold for heading


LENGTH_ROBOT: #Length of the robot (currently used as the radius for circular rep of the robot)
```

### robot_param.yaml
```yaml
LENGTH_ROBOT: #Length of the robot (currently used as the radius for circular rep of the robot)
MAX_VELOCITY: #Max vel of robot
MIN_VELOCITY: #Min vel of robot
MAX_ACCELERATION: #Max acc of robot

MAX_YAWRATE: #Max angular vel of robot
MAX_D_YAWRATE: #Max angular acceleration of robot

```

## References
- D. Fox,  W. Burgard, and S.Thrun, "The dynamic window approach to collision avoidance", IEEE Robotics Automation Magazine, 1997.

(https://ieeexplore.ieee.org/abstract/document/580977)
