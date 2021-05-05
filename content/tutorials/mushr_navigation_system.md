---
title: "Multi-agent navigation system"
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
A navigation system enables a robot to navigate quickly between different poses while avoiding collisions with the environment or other agents. The navigation system in this project uses a slightly modified version of the NH-TTC [backend](https://github.com/davisbo/NHTTC), which is a decentralized multi-agent navigation system. The Non-holonomic-Time-To-Collision (NH-TTC) backend considers the Non-holonomic constraints of the car and the time to collision with other cars when finding the optimal thing to do. To learn more about how it works, check out the paper [here](http://motion.cs.umn.edu/r/NH-TTC/arxiv-NHTTC.pdf)!

### Goal
The goal of this tutorial is to get the multi-agent navigation system up and running on your system.

### Requirements
Completed the [quickstart](https://mushr.io/tutorials/quickstart/) tutorial (familiarity with ROS and python is assumed).
Make sure you’re using the sandbox map. The quickstart tutorial explains how to change the map.

## Environment setup
Clone the [nhttc_ros](https://github.com/naughtyStark/nhttc_ros.git) repository into your catkin workspace: 
{{< highlight bash >}}
$ cd catkin_ws/src
$ git clone --branch devel https://github.com/naughtyStark/nhttc_ros.git
$ cd nhttc_ros
$ git submodule init
$ git submodule update --force --recursive --init --remote
{{< / highlight >}}

Install python requirements (assuming you are already in the nhttc_ros directory):
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
```
[ INFO] [1619972044.806652208]: node started
[ INFO] [1619972044.806919877]: max_ttc: 6.000000, carrot_goal_ratio: 1.000000
[ INFO] [1619972044.806943270]: carrot_goal_dist: 1.092528
[ INFO] [1619972044.806955298]: obey_time:1
```

In a new tab, open rviz:
{{< highlight bash >}}
$ rviz -d ~/catkin_ws/src/nhttc_ros/nhttc_ros/rviz/nhttc.rviz
{{< / highlight >}}

Select the rviz configuration corresponding to the nhttc_ros. Run the route publisher node:
{{< highlight bash >}}
$ rosrun nhttc_ros route_publisher.py
{{< / highlight >}}

You should see something like this on rviz:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif" width="800" >}}                           

This should be considered the default case henceforth. In this demo, the blue car is trying to follow the blue arrows and the green car is trying to follow the green arrows. The green car is supposed to stop a small distance after crossing the intersection of the two paths whereas the blue car is supposed to move towards the top after the intersection of the two paths. The two paths coincide to force the navigation system to display its capabilities.

## Using the wrapper for your own intents and purposes:
The system takes a set of waypoints rather than single points. The message to publish for this is:

{{<highlight bash>}} /car_name/waypoints 
{{</highlight>}}
of type: [geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.htmll). Note that "car_name" refers to the name of the car, such as car1, car2, and so on.

The z axis coordinate represents the time difference between 2 waypoints. Note that a z axis value of 0.001 equals a time difference of 1 unit. This is done so that the waypoints don't look like they're floating off the ground when visualized in rviz. The unit of time is equal to the time it takes for the car to cover the distance between two waypoints in a straight line at the rated speed. The reason for this is to allow the system's speed to be scaled up or down without changing the global plan timing itself. 

To adjust the number of cars used, their colors and so on, you can modify the launch file (or create your own launch file using the ones provided as a template). For instance, if you wish to add another car, simply add these lines to the launch file:
{{<highlight xml>}}
<group ns="car_name">
    <include file="$(find mushr_sim)/launch/single_car.launch" >
        <arg name="car_name" value="car_name"/>
        <arg name="racecar_version" value="racecar-uw-nano"/>
        <arg name="racecar_color" value="-red-white" />
    </include>
    <node pkg="nhttc_ros" type="nhttc_ros_node" name="controller3">
        <param name="car_name" value="car_name"/>
        <param name="solver_time" value="20"/>
        <param name="max_ttc" value="3.0"/>
        <param name="carrot_goal_ratio" value="1.0"/>
        <param name="obey_time" value = "false" />
        <param name="allow_reverse" value = "true" />
    </node>
</group>
{{</highlight>}}

Note that the route_publisher.py file used for the demo only publishes paths for two cars (`car1` and `car2`). If you add another car, you will have to modify the route-publisher to publish a route for the third car. We recommend writing a separate route publishing file by taking inspiration from the route_publisher.py).

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

A value of 1.0 means the carrot goal distance is the same as the turning radius. A value of 2 indicates that the carrot goal distance is twice the turning radius. larger numbers result in smoother navigation, however, they come with the drawback of greater path-deviation as the system will tend to "round off" corners a lot sooner. The first figure shows the performance with carrot-goal-ratio of 1.0, the second figure shows performance with a ratio of 1.5:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif"   width="600" height="400" >}}
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_lookahead.gif" width="600" height="400" >}}                           

If the car tends to get stuck around turns, increase the carrot-goal ratio in increments of 0.1. If the car appears to be rounding off the turns too much or significantly deviating from the path near turns, causing issues with other agents, reduce the carrot-goal-ratio in decrements of 0.1. 

* **2) Max_ttc:** Stands for maximum time to collision. This parameter decides which agents to consider and which to not consider when optimizing for the next control action. The time to collision is calculated using the current state (pose as well as twist) of all agents. A larger max_ttc results in a larger time horizon for optimization. A larger max_ttc will make the car respond earlier to other agents but can result in the car deviating from its path too early. A smaller max_ttc will make the car less sensitive to agents far away but may result in the car responding too late to the other agents and ending up in deadlocks more often. The first figure shows the performance with a max_ttc of 3.0 and the second shows the performance with max_ttc of 6.0 seconds. Notice how the cars start avoiding each other earlier in the second example.
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif" width="600" height="400" >}}
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_maxttc.gif"  width="600" height="400" >}}                           

In this particular example, it may appear to be helping the agents. However, had there been another agent above/below these agents, it may have affected the performance adversely.

* **3) Solution time:** The amount of time in milliseconds to be used for calculating the solution and is not to be confused with the time to collision. Increasing the solution time may lead to a better solution, however, as the system has to operate in real-time, this will reduce the control-loop frequency. A slower control-loop frequency results in a delayed response. A multi-agent navigation system responds to the actions of the agents around it. If the control loop is slower, the other agents may appear out of sync at times as they aren't running the control loop fast enough to respond to the other agent's actions. In multi-agent situations when the intent of the other agent isn't clear, all the agents are essentially guessing what they should do. If the first guess is wrong, it may lead to sub-optimal behavior. In the first figure, the solution time is 20 milliseconds and 30 milliseconds for the second case. Notice how the blue car takes the wrong course of action initially in the second case.
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif" width="600" height="400" >}}
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_SolT.gif"    width="600" height="400" >}}

However, reducing the time may lead to the solution not converging at all, again, leading to sub-optimal solutions. Keeping the solution time between 20-40 milliseconds will yield feasible solutions, with larger times resulting in the above behavior appearing more often.

* **4) Obey_time:** The navigation system generally takes a waypoint array from a global planner. In multi-agent systems, the global planner will specify both space and time coordinates, as in, where the agent is supposed to be and when it is supposed to be there. If this parameter is set to false (as in the default demo), the car will disregard the timing. The first figure shows the behaviour for when this parameter is false and the second shows what happens when it is true:
<br>
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_default.gif" width="600" height="400" >}}
{{< figure src="/tutorials/MuSHR_multiagent_navigation/demo_time.gif"    width="600" height="400" >}}

As you can see, the blue car stops at the turn for a while before continuing. This happens because the blue car rounded the turn and arrived a little too early at the next waypoint, and therefore slowed down before continuing.

* **5) allow_reverse:** In some applications, it may be important to prevent the car from ever moving backward. This parameter allows you to decide whether the car can or can not go backward in a given situation. This parameter is fairly self-explanatory and is thus not accompanied by a demo.


## Troubleshooting
* **the multi_teleop.launch or nhttc_demo.launch node crashes when rviz is started simultaneously:** Wait for all the cars to initialize first before starting rviz. This usually takes about 5-10 seconds. 

