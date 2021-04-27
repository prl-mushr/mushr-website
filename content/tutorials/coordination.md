---
title: "Coordination Planner"
date: 2021-04-25T15:14:54+10:00
featured: true
draft: false
duration: 30
active: true
difficulty: Intermediate
summary: Initialize and operate multiple MuSHRs car to coordinationally complete tasks  
weight: 3
---

<h2> By: <a href=www.linkedin.com/in/p-chanrun>Podshara Chanrungmaneekul</a> </h2>

## Introduction

### Goal 

This tutorial will teach you to set up and planning the navigation paths for set of MuSHR's car to complete certain set of tasks using Enhanced Conflict-Based Search(ECBS). By the end of the tutorial, a set of cars would be able to generate path that avoiding each other and complete the given set of tasks.

### Requirements

- If in sim, complete the [quickstart tutorial](https://mushr.io/tutorials/quickstart/)
- If on real car, complete the [first steps tutorial](https://mushr.io/tutorials/first_steps/)
- Navigation System [`mushr_navigation_system`](https://github.com/naughtyStark/nhttc_ros)

## Coordination Planner Problem Statement

There is a team of `num_agents` MuSHR race cars and a set of `num_goals` tasks. Each tasks requires one same race car to follow all `num_waypoints` intermediate points and stop at the last one. The environment is based on a gridworld domain where the movement of the car is restricted to orthogonal movement. Also, it is possible for one car not taking any job or taking multiple jobs, but it have to finish the previous one before executing the next task. 
## Install
Clone repo: 
``` 
cd ~/catkin_ws/src/ && git clone https://github.com/prl-mushr/mushr_coordination.git
cd mushr_coordination && git clone https://github.com/whoenig/libMultiRobotPlanning.git
```

## API
For adjusting params see `launch/mushr_coordination.launch` it has the environment params for the planner. Change number of waypoints for each goal by setting parameter `num_waypoint`. Change the car team setting by editing or creating new config file. 

### [Example](https://github.com/prl-mushr/mushr_coordination/blob/main/config/4cars1.yaml)
```
num_agent: 4
car1:
    name: "car1"
    color: "FF0000" #red
car2:
    name: "car2"
    color: "00FF00" #green
car3:
    name: "car3"
    color: "0000FF" #blue
car4:
    name: "car4"
    color: "DB921D" #orange
```

### Publishers
Topic | Type | Description
------|------|------------
`/{car_1's name}/waypoints` | [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll)| Pathway assigned to car_1
...
`/{car_n's name}/waypoints`| [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll) | Pathway assigned to car_n
`/{car_1's name}/marker` | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)| Intermediate points of task(s) assigned to car_1 
...
`/{car_n's name}/marker`| [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | Intermediate points of task(s) assigned to car_n 
`/mushr_coodination/border` | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | Borderline of the current setup environment for the planner


### Subscribers
Topic | Type | Description
------|------|------------
`/{car_1's name}/init_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| Initial position of car_1
...
`/{car_n's name}/init_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| Initial position of car_n
`/mushr_coordination/obstacles` | [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll)| List of obstacles in the map
`/mushr_coordination/goals` | [/mushr_coordination/GoalPoseArray](#mushr_coordination/GoalPoseArray ) | List of goals for {n} cars to complete

### Message Definition
#### mushr_coordination/GoalPoseArray  
[`std_msgs/Header`](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) Header \
`float64` scale `#scala for converting continous space to grid space`   
`float64` minx \
`float64` miny \
`float64` maxx \
`float64` maxy \
[`geometry_msgs/PoseArray`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html)[] goals 

## Running the coordination planner

```
roslaunch mushr_coordination mushr_coordination.launch
```
for sample environment setup and tasks run:
```
rviz
roslauncuh mushr_coordination init_planner.launch
```


