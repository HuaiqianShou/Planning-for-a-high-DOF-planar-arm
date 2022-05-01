# Planning-for-a-high-DOF-planar-arm

## Overview

Implemented four different sampling-based planners (RRT, RRT-Connect, RRT*, PRM) for the arm to move from its start joint angles to the goal joint angles. Each planner can retun a plan that is collision-free.Note that all the joint angles are given as an angle with X-axis, clockwise rotation being positive (and in radians). So, if the second joint angle is π/2, then it implies that this link is pointing exactly downward, independently of how the previous link is oriented. 

## Getting Started

### Prerequisites
MATLAB

Compile /mex

### To Run
You can run matlab without the GUI using:
```sh
matlab -nodesktop
```

In MATLAB:

To compile the cpp code (Has to be run from MATLAB command window)
```sh
>> mex planner.cpp
```
Two map files are provided   
Start and goal position can be user defined   
To execute the planner on map1 (Has to be run from MATLAB command window)   
```sh
>> startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
>> goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
>> planner_id = 1 % (1 is RRT, 2 is RRT-Connect, 3 is RRT*, 4 is PRM)
>> runtest('map1.txt',startQ, goalQ, planner_id);
```
## Demo

### RRT

### RRT-Connect

### RRT*

### PRM


