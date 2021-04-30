---
title: "MuSHR multi-agent navigation system tutorial"
date: 2021-04-18T22:17:25+05:30
summary: "This tutorial covers running the MuSHR multi-agent navigation stack in simulation"
difficulty: "Advanced"
duration: 30
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: true      # whether Hugo considers this a draft
weight: 3        # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://www.sidharthtalia.com/>Sidharth Talia</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/intro.jpg" width="800" >}} <br>                           
<br>

### Introduction
A navigation system enables a robot to navigate quickly between different poses while avoiding collisions with the environment or other agents. The navigation system in this project uses the NH-TTC [backend](https://github.com/davisbo/NHTTC), which is a decentralized multi-agent navigation system. The Non-holonomic-Time-To-Collision (NH-TTC) backend considers the Non-holonomic constraints of the car and considers the time to collision with other cars in the optimization process. To learn more about how it works, checkout the paper [here](http://motion.cs.umn.edu/r/NH-TTC/arxiv-NHTTC.pdf)!

### Goal
The goal of this tutorial is to get the multi-agent navigation system up and running on your system.

### Requirements
Completed the [quickstart](https://mushr.io/tutorials/quickstart/) tutorial (familiarity with ROS and python is assumed).

## Environment setup
Clone the nhttc_ros repository into your catkin workspace: 
{{< highlight bash >}}
$ cd catkin_ws/src
$ git clone --branch devel https://github.com/naughtyStark/nhttc_ros.git
$ cd nhttc_ros
$ git submodule init
$ git submodule update --force --recursive --init --remote
{{< / highlight >}}

install python requirements (assuming you are already in the nhttc_ros directory):
{{< highlight bash >}}
$ pip install -r requirements.txt
{{< / highlight >}}

Compile using catkin_make:
{{< highlight bash >}}
$ cd ~/catkin_ws
$ catkin_make
{{< / highlight >}}

If everything compiles, you should be ready to try out the simulation example. Launch the nhttc_demo.launch:
{{< highlight bash >}}
$ roslaunch nhttc_ros nhttc_demo.launch
{{< / highlight >}}

Wait till you see the highlighted text on the terminal:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/start-up-message.jpg" width="800" >}} <br>                           
<br>

In a new tab, open rviz:
{{< highlight bash >}}
$ rosrun rviz rviz
{{< / highlight >}}

Select the rviz configuration corresponding to the nhttc_ros. Run the route publisher node:
{{< highlight bash >}}
$ rosrun nhttc_ros route_publisher.py
{{< / highlight >}}

You should see something like this on rviz:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif" width="800" >}} <br>                           
<br>
This should be considered the default case henceforth.

# API

The nhttc_ros wrapper has the following parameters:
#### car_name (string):
The name assigned to the car (can be any string but prefer car{index number})

#### solver_time (int):
The maximum time in milliseconds for which the solver is allowed to run

#### max_ttc (float):
The maximum time-to-collision used by the solver: any agents that have a time to collision larger than this will not be considered in the cost function

#### carrot_goal_ratio (float):
The ratio of the carrot-goal/lookahead distance to the turning radius. value of 1 means that the lookahead distance is the same as the turning radius.

#### obey_time" value(boolean):
Set to true if the car is supposed to adhere to the time coordinate of the waypoints (as in, to arrive at a waypoint at a given time and not before/after).

#### allow_reverse (boolean):
Set to true if the car is allowed to go in reverse.

### Publishers
Topic | Type | Description
------|------|------------
`/car_name/mux/ackermann_cmd_mux/input/navigation` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)| steering and speed control of car corresponding to car_name.
`/car_name/cur_goal` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| topic on which current waypoint is published.
`/car_name/time_goal` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| waypoint being used for timing purposes.

### Subscribers
Topic | Type | Description
------|------|------------
`/car1/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of car 1
...
`/car{n}/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of car {n}
`/car_name/waypoints` | [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll)| waypoint array corresponding to car_name. The z axis coordinate represents the time difference between 2 waypoints.

Note that the z axis data in `/car/waypoints` topic represents the time difference between two consecutive waypoints in a unitless fashion, and the number should be pre-multiplied by 0.001 before publishing (so that the visualization on rviz does not look elevated). 1 unit of time here is equal to (distance between two waypoints/expected speed of the agent). If the car moves at 0.5 m/s and the distance between two waypoints is 1 meter, then 1 unit of time would equal to 2 seconds. 

## Tuning the parameters
The multi-agent navigation system _can_ work out of the box for most applications, however, it is possible to tune it should the user feel that it needs to be tuned. The parameters can be changed inside the launch file (in this case the nhttc_demo.launch file). Please note that the navigation system uses a solver which has some level of stochasticity to it, which can lead to slightly different behavior. The demonstrations shown here are to explain the effect of changing the parameters.

{{<highlight xml >}}
<param name="carrot_goal_ratio" value="1.5"/>
<param name="max_ttc" value="3.0"/>
<param name="solver_time" value="20"/>
<param name="obey_time" value = "true" />
<param name="allow_reverse" value = "false" />
{{</highlight>}}

* **1) Carrot-goal ratio:** The ROS wrapper implements a carrot-goal navigation system where waypoints are selected from a prescribed path. The waypoints are selected such that they are some “lookahead” distance away from the car. Keeping the car aimed at a waypoint farther away prevents it from getting stuck in a local minimum. Keeping this lookahead closer to the car makes sure the car does not deviate too far away from the prescribed path while getting to the point farther down the line. The ratio of this lookahead distance or carrot-goal distance to the turning radius has been defined as the carrot-goal ratio. The reason for that name is that the way the system works is akin to sitting on top of a donkey (the car) and keeping a carrot(waypoint) hung from a stick that you(navigation system) are holding:

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/carrot_goal_meme.jpg" width="800" >}} <br>                           
<br>

A value of 1.0 means the carrot goal distance is the same as the turning radius. A value of 2 indicates that the carrot goal distance is twice the turning radius. larger numbers result in smoother navigation, however, they come with the drawback of greater path-deviation as the system will tend to "round off" corners a lot sooner. The following demo shows the behavior when carrot-goal-ratio is 1.5 instead of 1.0:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_lookahead.gif" width="800" >}} <br>                           
<br>

If the car tends to get stuck around turns, increase the carrot-goal ratio in increments of 0.1. If the car appears to be rounding off the turns too much or significantly deviating from the path near turns, causing issues with other agents, reduce the carrot-goal-ratio in decrements of 0.1. 

* **2) Max_ttc:** Stands for maximum time to collision. This parameter decides which agents to consider and which to not consider when optimizing for the next control action. The time to collision is calculated using the current state (pose as well as twist) of all agents. A larger max_ttc results in a larger time horizon for optimization. A larger max_ttc will make the car respond earlier to other agents but can result in the car deviating from its path too early. A smaller max_ttc will make the car less sensitive to agents far away but may result in the car responding too late to the other agents and ending up in deadlocks more often. The following demo shows what happens when the max_ttc is increased from 3.0 to 6.0: 

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_maxttc.gif" width="800" >}} <br>                           
<br>

In this particular example, it may appear to be helping the agents. However, had there been another agent above/below these agents, it may have affected the performance adversely.

* **3) Solution time:** The amount of time in milliseconds to be used for calculating the solution and is not to be confused with the time to collision. Increasing the solution time may lead to a better solution, however, as the system has to operate in real-time, this will reduce the control-loop frequency. A slower control-loop frequency results in a delayed response. A multi-agent navigation system will respond to the actions of the agents around it. If the control loop is slower, the other agents may appear out of sync at times as they aren't running the control loop fast enough to respond to the other agent's actions. In multi-agent situations when the intent of the other agent isn't clear, all the agents are essentially guessing what they should do. If the first guess is wrong, it may lead to sub-optimal behavior, as shown in the following demo where the solution time is increased from 20 milliseconds(default) to 30 milliseconds:

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_SolT.gif" width="800" >}} <br>                           
<br>

However, reducing the time may lead to the solution not converging at all, again, leading to sub-optimal solutions. Keeping the solution time between 20-40 milliseconds will yield feasible solutions, with larger times resulting in the above behavior appearing more often.

* **4) Obey_time:** The navigation system generally takes a waypoint array from a global planner. In multi-agent systems, the global planner will specify both space and time coordinates, as in, where the agent is supposed to be and when it is supposed to be there. If this parameter is set to false (as in the default demo), the car will disregard the timing. The following demo shows what happens when the parameter is set to true:

<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_time.gif" width="800" >}} <br>                           
<br>

As you can see, the blue car stops at the turn for a while before continuing. This happens because the blue car rounded the turn and arrived a little too early at the next waypoint, and therefore slowed down before continuing.

* **5) allow_reverse:** In some applications, it may be important to prevent the car from ever moving backward. This parameter allows you to decide whether the car can or can not go backward in a given situation. This parameter is fairly self-explanatory and is thus not accompanied by a demo.


## Troubleshooting
* **the multi_teleop.launch or nhttc_demo.launch node crashes when rviz is started simultaneously:** Wait for all the cars to initialize first before starting rviz. This usually takes about 5-10 seconds. 

