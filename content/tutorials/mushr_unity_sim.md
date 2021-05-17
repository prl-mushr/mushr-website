---
title: "Using unity to simulate MuSHR"
date: 2021-05-17T17:51:18+05:30
summary: "Using a unity-based simulator in place of the default state prediction system that comes with mushr_base"
difficulty: "Intermediate"
duration: 0
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: true      # whether Hugo considers this a draft
weight: 3        # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://www.sidharthtalia.com/>Sidharth Talia</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/mushr_unity_sim/thumbnail.gif" width="800" >}} <br>                           
<br>

### Introduction
The default state prediction system [racecar_state.py](https://github.com/prl-mushr/mushr_base/blob/master/mushr_base/src/racecar_state.py) works well for low-speed situations where the wheel-slippage and body roll aren't significant. However, once you get to higher speeds (3-4 m/s and above), you want to be able to simulate those things as well so that you know your system is robust to slippage, body roll and so on. For this purpose, we use a unity-based simulator to simulate the state of the car with higher physics fidelity. The system provided by the [mushr_unity_sim](https://github.com/naughtyStark/mushr_unity_sim.git) is a drop in replacement for the default state predictor.

<br>
{{< figure src="/tutorials/mushr_unity_sim/thumbnail.png" width="800" >}} <br>                           
<br>

### Goal
The goal of this tutorial is to get the user acquainted to the unity-based simulation system.


### Requirements
Completed the [quickstart](https://mushr.io/tutorials/quickstart/) tutorial.
Make sure you’re using the sandbox map. The quickstart tutorial explains how to change the map.


## Installation:
Below are the instructions for installing the unity simulation system.

#### Cloning the repository:
In a terminal, enter the following commands:

{{<highlight bash>}}
$ cd ~/catkin_ws/src
$ git clone https://github.com/naughtyStark/mushr_unity_sim.git
$ cd ~/catkin_ws
$ catkin_make
{{</highlight>}}

The last catkin_make command is necessary to make the system recognize mushr_unity_sim as a valid ros package.

Python dependencies: 

{{<highlight bash>}}
$ pip install -r requirements.txt
{{</highlight>}}


### Running the base example:
In a new terminal, enter:

{{<highlight bash>}}
$ cd ~/catkin_ws/src/mushr_unity_sim/unity
$ ./donkey_sim.x86_64 -batchmode
{{</highlight>}}

If you have nvidia driver support on your linux machine (god bless), you can run it without the "-batchmode" tag. The tag makes the sim run in the headless mode which allows for higher fps if you don't have said driver support. Since the simulation surroundings/environment have been simplified to reduce the size of the simulator, there isn't much to see except the car itself and other cars if they are spawned.


In a new tab:

{{<highlight bash>}}
$ roslaunch mushr_unity_sim unity_multi.launch
{{</highlight>}}
In another new tab:
{{<highlight bash>}}
$ rviz -d ~/catkin_ws/src/mushr_unity_sim/rviz/mushr_unity_sim.rviz
{{</highlight>}}

You should see 4 cars by default. The poses of these cars are set by the pose_init.py file. The car's initial pose is set the same way as done for the default mushr_sim; by publishing a message on the topic: `/car_name/initialpose` of type: [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) where `car_name` corresponds to the name of the car. 


### Using this simulator for your own purposes:
In order to use this simulation backend in your system, all you need to do is replace the default call to the single_car.launch (as shown below)
{{<highlight xml>}}
    <group ns="$(arg car_name)">
        <include file="$(find mushr_unity_sim)/launch/single_car.launch">
            <arg name="car_name" value="$(arg car_name)"/>
            <arg name="racecar_version" value="racecar-uw-nano"/>
            <arg name="racecar_color" value="" />
        </include>
    </group>
{{</highlight>}}

with a call to the unity_single.launch (as show below):

{{<highlight xml>}}
    <group ns="$(arg car_name)">
        <include file="$(find mushr_unity_sim)/launch/unity_single.launch"> <!-- this is the only line that needs to be changed -->
            <arg name="car_name" value="$(arg car_name)"/>
            <arg name="racecar_version" value="racecar-uw-nano"/> <!-- simulator only supports the MuSHR car! --> 
            <arg name="racecar_color" value="" />
        </include>
    </group>
{{</highlight>}}

Note that the `group ns` is the transform prefix used for the corresponding car. Make sure it is unique to each car. With this done, you should be able to run everything else just the way you did with the standard stack.

## Troubleshooting
* **The launch file crashes on launch:** You probably forgot to run the donkey_sim.x86_64
