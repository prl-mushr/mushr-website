---
title: "MuJoCo Simulation"
date: 2020-07-13T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
duration: 30
active: true
difficulty: Beginner
summary: Run a MuSHR simulation with the MuJoCo physics engine.
weight: 5
---

<h2> By: <a href=https://github.com/21WANGC>Caelen Wang</a></h2>

{{< figure src="/tutorials/mujoco/mushr_mujoco.png" width="800" >}}
<br>

## Introduction

### Goal 

This tutorial will introduce you to the MuSHR MuJoCo simulation. MuJoCo is a 
physics engine that provides accurate simulations for real-world interactions.
By the end of the tutorial, you would be able to run a basic MuSHR simulation in
the MuJoCo environment.

### Requirements

* The full MuSHR stack from the 
[Quickstart](https://mushr.io/tutorials/quickstart/) tutorial

* ROS, follow the [Intro to ROS](https://mushr.io/tutorials/intro-to-ros/) 
tutorial for a refresher

* A licensed copy of [MuJoCo](https://www.roboti.us/index.html). Place the 
extracted folder `mujoco200_linux` along with the license `mjkey.txt` in the directory `~/.mujoco/`

* [Cmake](https://cmake.org/download/) miminum version 3.13.2

### Setup

Clone the MuSHR MuJoCo repository into your catkin workspace:
{{< highlight bash >}}
# Go to your catkin workspace
$ cd ~/catkin_ws/src
# Clone the MuSHR MuJoCo repository
$ git clone git@github.com:prl-mushr/mushr_mujoco_ros.git
{{</ highlight >}}

Note that in `CMakeLists.txt`, you can specify the MuJoCo directory with 
the `MUJOCO_LOCATION` environment variable. The default path is `~/.mujoco/`. Optionally, if you want to use OpenGL for enhanced graphics, 
set the `USE_GL` environment variable to 1, which is the default value. To
compile with GL install GLFW:
{{< highlight bash >}}
$ sudo apt-get install libglfw3-dev
{{< / highlight >}}

Compile your workspace:
{{< highlight bash >}}
$ cd .. && catkin_make
{{</ highlight >}}

### Running the Simulator
To demonstrate the MuSHR MuJoCo environment, we will run a simple simulation 
of a car and a block that can be pushed. Each simulation requires a ROS launch
file, a MuJoCo model XML file, and a configuration YAML file with the same name.
Take a look at `launch/block.launch`, `models/block.xml`, and 
`config/block.yaml`.

In the launch file, we can use a specific map file. The environment variable 
should match the name of the three files mentioned above:
{{< highlight xml >}}
<arg name="map_server" default="1"/>
<arg name="map_file" default="$(find mushr_mujoco_ros)/maps/empty.yaml" />
<arg name="environment" default="block" />
{{< / highlight >}}

The node parameters specify the MuJoCo key, XML file, and YAML file paths, as 
well as visualization. The default paths follow that in the previous step:
{{< highlight xml >}}
<node pkg="mushr_mujoco_ros" name="mushr_mujoco_ros" type="mushr_mujoco_ros_node" output="screen">
    <param name="mj_key" value="~/.mujoco/mjkey.txt" />
    <param name="model_file_path" value="$(find mushr_mujoco_ros)/models/$(arg environment).xml" />
    <param name="config_file_path" value="$(find mushr_mujoco_ros)/config/$(arg environment).yaml" />
    <param name="viz" value="true" />
</node>
{{< / highlight >}}

The model XML file contains MuJoCo constants that define the bodies and actuator parameters for the simulate car. To add a car to the simulation, use an 
"include" tag with the path from the `models` directory:
{{< highlight xml >}}
<include file="cars/pusher_car/buddy.xml"/>
{{< / highlight >}}

The configuration YAML file is a list of bodies and cars whose position will be published to different topics. The MuSHR vehicles are listed under "cars" while 
other objects are listed under "bodies". In this simulation, the car `buddy` 
receives the control, pose, and initpose messages on the `controls`, `pose`, and `initialpose` ROS topics:
{{< highlight xml >}}
cars:
- name: buddy
  control_topic : control
  pose_topic: pose
  initpose_topic: initialpose
{{< / highlight >}}

With a grasp of the basic structure, we can start the simulation:
{{< highlight bash >}}
$ roslaunch mushr_mujoco_ros block.launch
{{</ highlight >}}

To control the car with WASD keyboard input, open another terminal. Enter the 
line below, replacing the name of the car as needed:
{{< highlight bash >}}
$ rosrun topic_tools relay /mux/ackermann_cmd_mux/input/teleop /mushr_mujoco_ros/buddy/control
{{</ highlight >}}

### Experimenting with MuJoCo

Try to drive the car around and push the block. You might notice that it's 
somewhat difficult to push the block with precision. Let's change the MuJoCo
model parameters to facilitate our task. Open `models/block.xml` and find 
the `body` tag that describes the block. Double the mass to "2.4", and increase 
the size to "0.1 0.1 0.1", which should provide a larger block to push. Increase 
the friction on the block by setting friction to be "0.8 0.05 0.001". The 3 values 
control tangential, torsional, and rolling friction, respectively. Feel free to 
mess around with the values for a desired result.

{{< highlight xml >}}
<body pos="1.000000 0.000000 0.049" name="block" euler="0 0 0.000000">
  <joint type="free"/>
  <geom type="box" mass="2.4" contype="1" friction="0.8 0.05 0.001" 
  conaffinity="1" size="0.1 0.1 0.1" rgba="0.247 0.772 0.760 1"/>
</body>
{{< / highlight >}}

After changing model parameters, remember to recompile the workspace with 
`catkin_make` for the changes to take effect. Have fun with MuJoCo!
To autonomously drive figure-8s in MuJoCo, checkout the 
[MuJoCo Figure 8](https://mushr.io/tutorials/mujoco_figure8/) tutorial.