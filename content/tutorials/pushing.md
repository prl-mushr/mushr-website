---
title: "MuJoCo Simulation"
date: 2020-07-13T15:14:54+10:00
image: "/services/default.png"
featured: false
draft: false
duration: 30
active: true
difficulty: Beginner
summary: Run a MuSHR simulation with the MuJoCo physics engine!
weight: 3
---

## Introduction

### Goal 

This tutorial will introduce you to the MuSHR MuJoCo simulation. MuJoCo is a 
physics engine that provides accurate simulations for real-world interactions.
By the end of the tutorial, you would be able to run a basic MuSHR simulation in
the MuJoCo environment.

### Requirements

Install the full MuSHR stack by following the 
[Quickstart tutorial](https://mushr.io/tutorials/quickstart/).

You will need a licensed copy of MuJoCo. Download [mujoco200 linux](https://www.roboti.us/index.html). Place the extracted folder `mujoco200_linux` along with
the license `mjkey.txt` in the directory `~/.mujoco`.

To run MuJoCo with OpenGL for photorealistic graphics, install GLFW.
{{< highlight bash >}}
$ sudo apt-get install libglfw3-dev
{{< / highlight >}}

### Setup

Clone the MuSHR MuJoCo repository into your catkin workspace:
{{< highlight bash >}}
# Go to your catkin workspace
$ cd ~/catkin_ws/src
# Clone the MuSHR MuJoCo repository
$ git clone git@github.com:prl-mushr/mushr_mujoco_ros.git
{{</ highlight >}}

Compile your workspace:
{{< highlight bash >}}
$ catkin_make
{{</ highlight >}}

Start the simulation:
{{< highlight bash >}}
$ roslaunch mushr_mujoco_ros block.launch
{{</ highlight >}}

To control the car with keyboard input, enter in another terminal:
{{< highlight bash >}}
$ rosrun topic_tools relay /mux/ackermann_cmd_mux/input/teleop /mushr_mujoco_ros/buddy/control
{{</ highlight >}}