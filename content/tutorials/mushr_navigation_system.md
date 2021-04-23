---
title: "MuSHR_navigation_system_tutorial"
date: 2021-04-18T22:17:25+05:30
summary: "This tutorial covers running the MuSHR multi-agent navigation stack in simulation"
difficulty: "Advanced(?)"
duration: 0
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: true      # whether Hugo considers this a draft
weight: 3        # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://mushr.io/>Sidharth Talia</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/intro.jpg" width="800" >}} <br>                           
<br>

### Introduction
A navigation system enables a robot to navigate quickly between different poses while avoiding collisions with the environment or other agents. The navigation system in this project uses the NHTTC backend [link](https://github.com/davisbo/NHTTC), which is a decentralized navigation system meant for multi-agent navigation. The title NH-TTC stands for “Non-Holonomic-Time-To-Collision". The reader may peruse the [paper](http://motion.cs.umn.edu/r/NH-TTC/arxiv-NHTTC.pdf) to better understand how it works.

### Goal
The goal of this tutorial is to get the reader familiar with the multi-agent navigation framework used for the MuSHR car.

### Requirements
Successfully completed the [quickstart](https://mushr.io/tutorials/quickstart/) tutorial (familiarity with ROS and python is assumed).

## Environment setup
Clone the nhttc repository into the catkin workspace: 
{{< highlight bash >}}
$ cd catkin_ws/src
$ git clone --branch devel https://github.com/naughtyStark/nhttc_ros.git
$ cd nhttc_ros
$ git submodule init
$ git submodule update --force --recursive --init --remote
$ cd ~/catkin_mushr
$ catkin_make
{{< / highlight >}}

If everything compiles, you should be ready to try out the simulation example. Launch the nhttc_demo.launch:
{{< highlight bash >}}
$ roslaunch nhttc_ros nhttc_demo.launch
{{< / highlight >}}

In a new tab, open rviz:
Open rviz
{{< highlight bash >}}
$ rviz
{{< / highlight >}}

Select the rviz configuration corresponding to the nhttc 

run the route publisher node:
{{< highlight bash >}}
$ rosrun nhttc_ros route_publisher.py
{{< / highlight >}}

You should see something like this on rviz (this should be considered the default case henceforth):
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif" width="800" >}} <br>                           
<br>

## Using a global planner for publishing route (TBD)
In the demo example, the route-publisher node publishes a [poseArray message](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html). The user can look at the route_publisher.py node to see how to publish the waypoint messages. 

## Tuning the parameters
The multi-agent navigation system can work out of the box for most applications, however, it is possible to tune it should the user feel that it needs to be tuned. The parameters can be changed inside the launch file (in this case the nhttc_demo.launch file). Please note that the navigation system uses a solver which has some level of stochasticity to it, which can lead to different behavior. The demonstrations shown here are merely to explain the effect of changing the parameters.

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/params.png" width="800" >}} <br>                           
<br>

* **1) Carrot-goal ratio:** The ROS wrapper around the NHTTC backend does not simply feed the NHTTC backend the next closest waypoint. It looks for the farthest waypoint within a certain “lookahead” distance. Essentially, this keeps the goal far enough away from the car that the car doesn’t get stuck going in circles around a waypoint due to it’s limited steering angle. The look-ahead distance is also called the carrot-goal distance. The reason for that name is that the way the system works is akin to sitting on top of a donkey (the car) and keeping a carrot(waypoint) hung from a stick that you(navigation system) are holding:

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/carrot_goal_meme.jpg" width="800" >}} <br>                           
<br>

Since it is easier to think in terms of ratios than absolute values, we use a carrot-goal-ratio instead of the actual carrot-goal distance to tune the system. The carrot-goal-ratio is defined as the ratio of the carrot-goal distance to the turning radius of the car. A value of 1.0 means the carrot goal distance is the same as the turning radius. A value of 2 indicates that the carrot goal distance is twice the turning radius. larger numbers result in smoother navigation, however, they come with the drawback of greater path-deviation as the system will tend to "round off" corners a lot sooner. The following demo shows the behaviour when carrot-goal-ratio is 1.5 instead of 1.0:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_lookahead.gif" width="800" >}} <br>                           
<br>

If the car tends to get stuck around turns, increase the carrot-goal ratio in increments of 0.1. If the car appears to be rounding off the turns too much or significantly deviating from the path near turns, causing issues with other agents, reduce the carrot-goal-ratio in decrements of 0.1. 

* **2) Max_ttc:** Stands for maximum time to collison. This parameter decides which agents to consider and which to not consider when optimizing for the next control action. The time to collison is calculated using the current state (pose as well as twist) of all agents. A larger max_ttc results in a larger time horizon for optimization. A larger max_ttc will make the car respond earlier to other agents, but can result in the car deviating from it's path too early. A smaller max_ttc will make the car less sensitive to agents far away but may result in the car responding too late to the other agents and ending up in deadlocks more often. The following demo shows what happens when the max_ttc is increased from 3.0 to 6.0: 

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_maxttc.gif" width="800" >}} <br>                           
<br>

In this particular example, it may appear to be helping the agents. However, had there been another agent above/below these agents, it may have affected the performance adversely.

* **3) Solution time:** The amount of time in milliseconds to be used for calculating the solution and is not to be condused with the time to collision. Increasing the solution time may lead to better solution, however, as the system has to operate in real-time, this will reduce the control-loop frequency. A slower control-loop frequency results in a delayed response. A multi-agent navigation system will respond to the actions of the agents around it. If the control loop is slower, the other agents may appear out of sync at times as they aren't running the control loop fast enough to respond to the other agent's actions. In multi-agent situations when the intent of the other agent isn't clear, all the agents are essentially guessing what they should do. If the first guess is wrong, it may lead to sub-optimal behavior, as shown in the following demo where the solution time is increased from 20 milliseconds(default) to 30 milliseconds:

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_SolT.gif" width="800" >}} <br>                           
<br>

However, reducing the time may lead to the solution not converging at all, again, leading to sub-optimal solutions. Keeping the solution time between 20-40 milliseconds will yield feasible solutions, with larger times resulting in the above behaviour appearing more often.

* **4) Obey_time:** The navigation system generally takes a waypoint array from a global planner. In multi-agent systems, the global planner will specify both space and time coordinates, as in, where the agent is supposed to be and when it is supposed to be there. If this parameter is set to false (as in the default demo), the car will disregard the timing. The following demo shows what happens when the parameter is set to true:

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_time.gif" width="800" >}} <br>                           
<br>

As you can see, the blue car stops at the turn for a while before continuing. This happens because the blue car rounded the turn and arrived a little too early at the next waypoint, and therefore slowed down before continuing.

* **5) allow_reverse:** In some applications, it may be important to prevent the car from ever moving backwards. This parameter allows you to decide whether the car can or can not go backwards in a given situation. This parameter is fairly self-explanatory and is thus not accompanied by a demo.


## Troubleshooting
* **the multi_teleop.launch node crashes when rviz is started simultaneously:** Wait for all the cars to initialize first before starting rviz. This usually takes about 5-10 seconds. 