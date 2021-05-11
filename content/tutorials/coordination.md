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

<h2> By: <a href='https://www.linkedin.com/in/p-chanrun/'>Podshara Chanrungmaneekul</a> </h2>
<img src="/tutorials/coordination_planner/header.gif" width="800" >

## Introduction

### Goal 

This tutorial will teach you how to setup the Enhance Conflict-Based Search with Optimal Task Assignment (ECBS-TA) planning algorithm on a set of MuSHR cars. ECBS-TA produces a set of collision free trajectories, one for each car, from start to goal. By the end of the tutorial, a set of cars will be able to generate path that avoiding each other and complete the given set of tasks.

### Requirements

- If in sim, complete the [quickstart tutorial](https://mushr.io/tutorials/quickstart/)
- If on real car, complete the [first steps tutorial](https://mushr.io/tutorials/first_steps/)
- Navigation System [mushr_navigation_system](https://github.com/naughtyStark/nhttc_ros)

## Coordination Planner Problem Statement

There is a team of `num_agents` MuSHR cars and a set of `num_goals` tasks. Each task requires one car to follow all of the `num_waypoints` points and stop at the last one. The environment is a grid world where the movement of the car is restricted to orthongonal movement. This approximate approach can work well when combined with the [nhttc controller](https://github.com/naughtyStark/nhttc_ros). It is possible that some cars will not have any tasks and others will have multiple to complete sequentially.

## Install
Clone repo and install the package: 
{{< highlight bash >}}
$ cd ~/catkin_ws/src/
$ git clone https://github.com/prl-mushr/mushr_coordination.git
$ cd mushr_coordination
$ git clone https://github.com/whoenig/libMultiRobotPlanning.git
$ cd ~/catkin_ws/
$ catkin_make
{{< / highlight >}}

## API
For adjusting params see `launch/mushr_coordination.launch` it has the environment params for the planner. Change number of waypoints for each goal by setting parameter `num_waypoint`. Change the car team setting by editing or creating new config file. 

## Running the coordination planner

Run the planner that will set up the environment publishers and subscribers.
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:={car's team config file}
{{< / highlight >}}

For simulation, run `rviz` then and subscribe to planner's topics. 
{{< highlight bash >}}
$ rviz
{{< / highlight >}}

Set up the car's initial position and give planner the task to complete. 
{{< highlight bash >}}
$ roslauncuh mushr_coordination init_planner.launch cars_file:={car's team config file} task_file:={task benchmark file}
{{< / highlight >}}

### [Car's team configuration Example](https://github.com/prl-mushr/mushr_coordination/blob/main/config/4cars1.yaml)
This `4cars1.yaml` file shows how to edit and create a config file. The file will have the number of agent (`num_agent`) following with `car{n}`'s `name` and `color` in hex. 

Note that, It would be advised to use the same config file for both `mushr_coordination` and `init_planner`.
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

## Example of Initializing the Planner

### Example #1
In this environment, there are 4 cars and have been given 4 tasks to complete within a 5 by 3 space. 
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
$ rviz
$ roslauncuh mushr_coordination init_planner.launch cars_file:=4cars.yaml task_file:=4cars4tasks5x3.yaml
{{< / highlight >}}

Each car get assigned to their respective task, in which their accumulated distanse would be minimal.

<img src="/tutorials/coordination_planner/4cars4task5x3.png" width="800" >

### Example #2
In this environment, there are 4 cars and have been given 6 tasks to complete within a 6 by 6 space. 
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
$ rviz
$ roslauncuh mushr_coordination init_planner.launch cars_file:=4cars.yaml task_file:=4cars6tasks6x6.yaml
{{< / highlight >}}

Number of tasks is more than number of cars, so the blue car and the green car have been given one task more than the others to complete. 

<img src="/tutorials/coordination_planner/4cars6task6x6.png" width="800" >

### Example #3
In this environment, there are 4 cars and have been given 2 tasks to complete within a 5 by 3 space. 
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
$ rviz
$ roslauncuh mushr_coordination init_planner.launch cars_file:=4cars.yaml task_file:=4cars2tasks5x3.yaml
{{< / highlight >}}

There is not enough task to assign to every cars. In this case, the red car and green car are idle and stay in place.

<img src="/tutorials/coordination_planner/4cars2task5x3.png" width="800" >

