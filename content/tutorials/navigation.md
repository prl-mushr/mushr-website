---
title: "Basic Autonomous Navigation"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
duration: 15
difficulty: Intermediate
summary: This tutorial will teach you how to initialize and operate MuSHRs out-of-the-box autonomous navigation stack.
weight: 3
---

## Introduction

### Goal 

This tutorial will teach you to set up and operating MuSHR's baseline autonomous navigation stack. By the end of the tutorial, the car will be able to autonomously navigate around known obstacles on a known map.

### Requirements

If you intend to run this tutorial on the real car,  complete the Mapping Tutorial first. We will also assume you have built your car with the LiDaR. We recommend access to a secondary linux computer for viewing the visualizations of the localization module, as well as for initializing the particle filter.

## Navigation Stack Overview

At the highest level MuSHR's navigation stack consists of two principal components:

1. **Receding Horizon Controller (RHC) Node:** This node is responsible for planning the motions and generating controls for the car. The implementation we ship with the car uses Model Predictive Control (MPC) to generate control signals which are sent to the car's motor controller (VESC).
2. **Localization Node:** In order for the controller to know whether it is in the proximity of obstacles, it must know its location on a known map. Solving this problem is called "localization". The Localization Node is implemented using a method called Particle Filtering which in this case relies primarily on a data stream from the laser scanner.

This tutorial does not cover Model Predictive Control and Particle Filtering in depth. However we recommend this tutoral to learn more about MPC and this one for Particle Filtering.

## Installing the Navigation Stack

First we will install the RHC and Localization nodes on your robot. If you have already installed them, skip this step.

Ssh onto your racecar.

{{< highlight bash >}}
$ ssh robot@RACECAR_IP
{{< / highlight >}}

Ensure your racecar has a connection to the internet:
{{< highlight bash >}}
$ ping google.com
{{< / highlight >}}

This should return a result such as:
```
64 bytes from 172.217.3.174: icmp_seq=0 ttl=53 time=7.327 ms
```

Then download the RHC and localization nodes:
{{< highlight bash >}}
# Go to your catkin workspace
$ cd ~/catkin_ws/src
# Clone the RHC node
$ git clone git@github.com:prl-mushr/mushr_rhc.git 
# Clone the localization node
$ git clone git@github.com:prl-mushr/mushr_pf.git
{{< / highlight >}}

Both repositories contain ROS packages that reproduce the desired functionality. However, you need only concern yourself with each package's launch files to use them effectively. You can find the lauch files in each package's `launch` directory.

## Running the navigation stack

Now we will launch the navigation stack directly on the robot. To learn about strategies for effectively operating and experimenting with the MuSHR car, visit the workflow tutorial series. We suggest using [`tmux`](https://hackernoon.com/a-gentle-introduction-to-tmux-8d784c404340) to manage multiple ROSlaunch sessions.

Once you've ssh'd into your robot, activate `tmux`:
{{< highlight bash >}}
$ tmux
{{< / highlight >}}

Then, to create two vertical panes, type `ctrl+b` (`ctrl` and `b`) then `%` (or alternatively `"` to split horizontally). We will need three panes for this tutorial.

First, we will launch `teleop.launch`, which activates the robot's sensors and hardware, including the motor controller. You will need to activate this launch file for any project which requires using the car's sensors:
{{< highlight bash >}}
$ roslaunch mushr_base teleop.launch
{{< / highlight >}}

Then, to go to the next tmux pane type `ctrl+b` then `[arrow key]`. Now, we will launch the localization node in the second tmux pane:
{{< highlight bash >}}
$ roslaunch mushr_pf ParticleFilter.launch
{{< / highlight >}}

Then activate the RHC node:
{{< highlight bash >}}
roslaunch mushr_rhc real.launch
{{< / highlight >}}

## Operating the navigation stack

Now it's time to initialize the particle filter, giving it an initial estimate of the distribution of possible poses. On your separate workstation, we will initialize `rviz`. [Rviz](http://wiki.ros.org/rviz/UserGuide) allows us to visualize the robot's telemetry, such as position, laser scan messages, etc. You've likely already used it in the simulator.

_Note: be sure that your `ROS_IP` and `ROS_MASTER_URI` environment variables are correctly set before initializaing `rviz`. See the workflow tutorial for more details._
{{< highlight bash >}}
$ rosrun rviz rviz -d $MUSHR/mushr_utils/rviz/real.rviz
{{< / highlight >}}

If you are running this tutorial in sim, run:
{{< highlight bash >}}
$ rosrun rviz rviz -d $MUSHR/mushr_utils/rviz/sim.rviz
{{< / highlight >}}

Initializing `rviz` with the `.rviz` files allows you to configure RVIZ's settings, including which ROS topics, in advance. This is handy if you're working on a specific task, or have preferences in how you would like to view the car's telemetry. You can always modify the existing `.rviz` files and save new ones to taste.

You may now set the initial pose of the car on the map, in a similar position to where you would expect it to be. Press the button labeled "2D Pose Estimate". The cursor will become an arrow, and you can press it where you think the car is. Try driving the car around using the joystick, and notice to what extent the localization is able to track the car's position.

Now, we will choose a goal for the car to navigate towards. We recommend starting with simple goal poses and gradually increasing the complexity. To select a goal position, choose the button labeled "2D Nav Goal" and select the goal pose.

Once you've seen the session for the rhc node output the text `Goal set`, hold the left-hand deadman switch to allow the car to track towards its goal. Release the button if you suspect the car is close to a collision.

If the car loses localization, simply re-click with the "2D Pose Estimate" button. You can set a new goal at any time, even if the car has not reached the goal you specified.

That's it, now you have basic autonomous navigation!