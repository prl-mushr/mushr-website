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

## Introduction

### Goal 

This tutorial will introduce you to the MuSHR MuJoCo simulation. MuJoCo is a 
physics engine that provides accurate simulations for real-world interactions.
By the end of the tutorial, you would be able to run a basic MuSHR simulation in
the MuJoCo environment.

### Requirements

Install the full MuSHR stack by following the 
[Quickstart](https://mushr.io/tutorials/quickstart/) tutorial.

For a quick refresher on ROS, complete the [Intro to ROS](https://mushr.io/tutorials/intro-to-ros/) tutorial.

You need a licensed copy of MuJoCo. Download [mujoco200 linux](https://www.roboti.us/index.html). Place the extracted folder `mujoco200_linux` along with
the license `mjkey.txt` in the directory `~/.mujoco`.

### Setup

Clone the MuSHR MuJoCo repository into your catkin workspace:
{{< highlight bash >}}
# Go to your catkin workspace
$ cd ~/catkin_ws/src
# Clone the MuSHR MuJoCo repository
$ git clone git@github.com:prl-mushr/mushr_mujoco_ros.git
{{</ highlight >}}

Note that in `CMakeLists.txt`, you can specify the MuJoCo directory with 
the `MUJOCO_LOCATION` environment variable. The default path follows that in the
previous section. Additonally, if you want to use OpenGL for enhanced graphics, 
set the `USE_GL` environment variable to 1, which is the default value. To
compile with GL install GLFW:
{{< highlight bash >}}
$ sudo apt-get install libglfw3-dev
{{< / highlight >}}

Compile your workspace:
{{< highlight bash >}}
$ catkin_make
{{</ highlight >}}

### Running the Simulator
To demonstrate the MuSHR MuJoCo environment, we will run a simple simulation 
of a car and a block that can be pushed. Each simulation requires a ROS launch
file, a MuJoCo model XML file, and a configuration YAML file with the same name.
Take a look at `block.launch`, `block.xml`, and `block.yaml` in the `launch`, 
`models`, and `config` directories, respectively.

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

The model XML file contains mainly MuJoCo constants. To add a car to the 
simulation, use an "include" tag with the path from the `models` directory:
{{< highlight xml >}}
<include file="cars/pusher_car/buddy.xml"/>
{{< / highlight >}}

The configuration YAML file is a straightfoward list of the simulation objects.
The MuSHR vehicles are listed under "cars" while other objects are listed under
"bodies". In this simulation, the car `buddy` receives the control, pose, and 
initpose messages on the `controls`, `pose`, and `initialpose` ROS topics:
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

For more MuSHR MuJoCo, check out our more advanced tutorials!