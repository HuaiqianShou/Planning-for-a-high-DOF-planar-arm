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
![testnew81](https://user-images.githubusercontent.com/90527682/166164943-945b94d6-3b28-4845-a154-b6de2e6cefbd.gif)

### RRT-Connect
![testnew52](https://user-images.githubusercontent.com/90527682/166164965-38265ee2-e97b-4406-9dc6-26f954717893.gif)

### RRT*
![testnew61](https://user-images.githubusercontent.com/90527682/166164976-e0d93274-2b64-4b79-b9c8-cea52e8e3662.gif)

### PRM
![testnew71](https://user-images.githubusercontent.com/90527682/166164981-edcaa3d4-a8a7-491f-9c56-e3a1e0685dc2.gif)


