---
title: "Multi-Agent Coordination Planner for Task Execution"
date: 2021-04-25T15:14:54+10:00
featured: true
draft: false
duration: 30
active: true
difficulty: Intermediate
summary: Plan and control multiple MuSHR cars to different goals without collisions  
weight: 3
---

<h2> By: <a href='https://www.linkedin.com/in/p-chanrun/'>Podshara Chanrungmaneekul</a>, <a href='https://www.linkedin.com/in/aditmjha/'> Adit Jha</a> </h2>
<img src="/tutorials/coordination_planner/4car4task.gif" width="1000" >

## Introduction

### Goal 

This tutorial will teach you how to setup the [Enhanced Conflict-Based Search with Optimal Task Assignment](https://act.usc.edu/publications/Hoenig_AAMAS2018a.pdf) (ECBS-TA) planning algorithm on a set of MuSHR cars. ECBS-TA produces a set of collision free trajectories, one for each car, from start to goal. By the end of the tutorial, a set of cars will be able to generate paths that avoid each other.

### Requirements

- If in sim, complete the [quickstart tutorial](https://mushr.io/tutorials/quickstart/)
- If on real car, complete the [first steps tutorial](https://mushr.io/tutorials/first_steps/)
- MuSHR Navigation System [mushr_navigation_system](https://mushr.io/tutorials/mushr_navigation_system/)

## Coordination Planner Problem Statement

There is a team of `num_agents` MuSHR cars and a set of `num_goals` tasks. Each task requires one car to follow all of the `num_waypoints` points and stop at the last one. The environment is a grid world where the movement of the car is restricted to orthongonal movement. This approximate approach can work well when combined with the [nhttc controller](https://github.com/naughtyStark/nhttc_ros). The coordination planner has the ability to create an initial path for each car that is avoiding all possible collisions. This path is leveraged with the nhttc controller to adapt a car's path and avoid collisions in real-time if need be. The combination of these two services is more powerful than just the nhttc controller by itself as potential conflicts are sorted out prior to declaring each car's path. It is possible that some cars will not have any tasks and others will have multiple to complete sequentially.

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
For adjusting params see `launch/mushr_coordination.launch` it has the environment params for the planner. Change the number of waypoints for each goal by setting parameter `num_waypoint`. Change the car team setting by editing or creating new config file. 

## Running the coordination planner

Run the planner that will set up the environment publishers and subscribers.
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:={car's team config file}
{{< / highlight >}}

For simulation, run `rviz` then and subscribe to planner's topics. Download the [rviz config file](/tutorials/coordination_planner/nhttc.rviz) for an out-of-box rviz setup. This setup subscribes to all the important rostopics and organizes them in categories. 
{{< highlight bash >}}
$ rviz
{{< / highlight >}}

Below is the format for setting the initial position for a team of cars and a set of tasks to complete. For out-of-box examples, look at the **Example of Initializing the Planner** section which utilizes files already part of the repository.
{{< highlight bash >}}
$ roslaunch mushr_coordination init_planner.launch cars_file:={car's team config file} tasks_file:={task benchmark file}
{{< / highlight >}}

### Configuration File Example for Car Information
This `4cars1.yaml` file shows how to edit and create a config file. The file will have the number of agents (`num_agent`) following with `car{n}`'s `name` and `color` in hex. 

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

## Examples of Initializing the Planner

### Example #1
In this environment, there are 4 cars and have been given 4 tasks to complete within a 5 by 3 space. 
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
$ rviz
$ roslaunch mushr_coordination init_planner.launch cars_file:=4cars.yaml tasks_file:=4cars4tasks5x3.yaml
{{< / highlight >}}

Each car get assigned to their respective task, in which their accumulated distance would be minimal.

<img src="/tutorials/coordination_planner/example1.jpg" width="800" >

### Example #2
In this environment, there are 4 cars and have been given 6 tasks to complete within a 6 by 6 space. 
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
$ rviz
$ roslaunch mushr_coordination init_planner.launch cars_file:=4cars.yaml tasks_file:=4cars6tasks6x6.yaml
{{< / highlight >}}

Number of tasks is more than number of cars, so the blue car and the green car have been given one task more than the others to complete. 

<img src="/tutorials/coordination_planner/example2.jpg" width="800" >

### Example #3
In this environment, there are 4 cars and have been given 2 tasks to complete within a 5 by 3 space. 
{{< highlight bash >}}
$ roslaunch mushr_coordination mushr_coordination.launch cars_file:=4cars.yaml
$ rviz
$ roslaunch mushr_coordination init_planner.launch cars_file:=4cars.yaml tasks_file:=4cars2tasks5x3.yaml
{{< / highlight >}}

There is not enough task to assign to every cars. In this case, the red car and green car are idle and stay in place.

<img src="/tutorials/coordination_planner/example3.jpg" width="800" >


