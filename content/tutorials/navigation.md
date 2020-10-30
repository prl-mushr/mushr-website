---
title: "Autonomous Navigation"
date: 2018-11-28T15:14:54+10:00
featured: true
draft: false
duration: 30
active: true
difficulty: Intermediate
summary: Initialize and operate MuSHRs out-of-the-box autonomous navigation stack!
weight: 3
---

<h2> By: <a href=https://www.linkedin.com/in/michalove/>Johan Michalove</a> & <a href=https://github.com/Rockett8855>Matthew Rockett</a></h2>   

{{< figure src="/tutorials/autonomous-navigation/auto.gif" >}}<br>

## Introduction

### Goal 

This tutorial will teach you to set up and operating MuSHR's baseline autonomous navigation stack. By the end of the tutorial, the car will be able to autonomously navigate around known obstacles on a known map.

### Requirements

- If in sim, complete the [quickstart tutorial](https://mushr.io/tutorials/quickstart/)
- If on real car, complete the [first steps tutorial](https://mushr.io/tutorials/first_steps/)
- Python dependencies for [`mushr_rhc`](https://github.com/prl-mushr/mushr_rhc)

If you intend to run this tutorial on the real car, construct a map of the environment in which you'll be building the car. Our team recommends using [gmapping](https://wiki.ros.org/slam_gmapping) or [cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/) We will also assume you have built your car with the LiDaR. We recommend access to a secondary linux computer for viewing the visualizations of the localization module, as well as for initializing the particle filter.

If you are in sim we don't recommend testing with the `sandbox.yaml` default map because the localization struggles in an open environment (all positions look the same!). See the [quickstart tutorial](https://mushr.io/tutorials/quickstart/) for how to change maps.

## Navigation Stack Overview

At the highest level MuSHR's navigation stack consists of two principal components:

1. **Receding Horizon Controller (RHC) Node:** This node is responsible for planning the motions and generating controls for the car. The implementation we ship with the car uses Model Predictive Control (MPC) to generate control signals which are sent to the car's motor controller (VESC).
2. **Localization Node:** In order for the controller to know whether it is in the proximity of obstacles, it must know its location on a known map. Solving this problem is called "localization". The Localization Node is implemented using a method called Particle Filtering which in this case relies primarily on a data stream from the laser scanner.

This tutorial does not cover Model Predictive Control and Particle Filtering in depth.

## Installing the Navigation Stack

*If you intend to run this tutorial on the simulator, start from downloading the [`mushr_rhc`](https://github.com/prl-mushr/mushr_rhc) and [`mushr_pf`](https://github.com/prl-mushr/mushr_pf).*

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
# Re-make to update paths 
$ cd ~/catkin_ws && catkin_make
{{</ highlight >}}

*You also need to download all dependencies packages for the [`mush_rhc`](https://github.com/prl-mushr/mushr_rhc)*.

Both repositories contain ROS packages that reproduce the desired functionality. However, you need only concern yourself with each package's launch files to use them effectively. You can find the launch files in each package's `launch` directory.

## Running the navigation stack

Now we will launch the navigation stack directly on the robot. To learn about strategies for effectively operating and experimenting with the MuSHR car, visit the [workflow tutorial](). We suggest using [`tmux`](https://hackernoon.com/a-gentle-introduction-to-tmux-8d784c404340) to manage multiple ROSlaunch sessions.

Once you've ssh'd into your robot, activate `tmux`:
{{< highlight bash >}}
$ tmux
{{< / highlight >}}

Then, to create two vertical panes, type `ctrl+b` (`ctrl` and `b`) then `%` (or alternatively `"` to split horizontally). We will need three panes for this tutorial.

**Note:** If the map you are using is very large (greater than 100 x 100 meters) including the unknown region than the controller will be sampling points for a really long time causing it to seem like it is not working. Save yourself the headache and shrink/crop your map ([Gimp](https://www.gimp.org/) is a good tool) before beginning.

First, we will launch `teleop.launch`, 

### On Real Car
To enable the robot's sensors and hardware including the motor controller, you will need to activate this launch file for any project which requires using the car's sensors:
{{< highlight bash >}}
$ roslaunch mushr_base teleop.launch
{{< / highlight >}}

Then, to go to the next tmux pane type `ctrl+b` then `[arrow key]`. Now launch the `map_server`:
{{< highlight bash >}}
# Make sure mushr/mushr_base/mushr_base/mushr_base/maps has your map 
# and mushr_base/launch/includes/map_server.launch is set to your map
$ roslaunch mushr_base map_server.launch
{{< / highlight >}}

Now, we will launch the localization node:
{{< highlight bash >}}
$ roslaunch mushr_pf real.launch
{{< / highlight >}}

Then activate the RHC node,

{{< highlight bash >}}
$ roslaunch mushr_rhc_ros real.launch
{{< / highlight >}}

### In Sim
If you run this tutorial with the simulator, you need the simulation version:
{{< highlight bash >}}
$ roslaunch mushr_sim teleop.launch
{{< / highlight >}}

Then, to go to the next tmux pane type `ctrl+b` then `[arrow key]`. Now, we will launch the RHC node in the second tmux pane:

{{< highlight bash >}}
$ roslaunch mushr_rhc_ros sim.launch
{{< / highlight >}}

The default setting is to use the ground truth sim pose for localization. But if you would like to use the particle filter (noisier) we have to change one default value. Open `mushr_rhc/mushr_rhc_ros/launch/sim/sim.launch` using your favorite text editor. Replace `car_pose` with `particle_filter/inferred_pose`.
{{< highlight bash >}}
$ roslaunch mushr_pf sim.launch
{{< / highlight >}}



## Operating the navigation stack

Now it's time to initialize the particle filter, giving it an initial estimate of the distribution of possible poses. On your separate workstation, we will initialize `rviz`. [Rviz](http://wiki.ros.org/rviz/UserGuide) allows us to visualize the robot's telemetry, such as position, laser scan messages, etc. You've likely already used it in the simulator.

_Note: be sure that your `ROS_IP` and `ROS_MASTER_URI` environment variables are correctly set before initializaing `rviz`. See the workflow tutorial for more details._
{{< highlight bash >}}
$ rosrun rviz rviz -d $MUSHR/mushr_utils/rviz/default.rviz
{{< / highlight >}}

Initializing `rviz` with the `.rviz` files allows you to configure RVIZ's settings, including which ROS topics, in advance. This is handy if you're working on a specific task, or have preferences in how you would like to view the car's telemetry. You can always modify the existing `.rviz` files and save new ones to taste.

Add the navigation topics if they are not selected already by clicking `ADD` &rarr; `By Topic` in rviz. Add the following:
- `/car/particle_filter/inferred_pose`: The particle filter's estimate of the car position
- `/car/particle_filter/particles`: The particle filter's distribution of particles (OPTIONAL) 
- `/car/rhccontroller/traj_chosen`: The trajectory the controller is choosing 

You may now set the initial pose of the car on the map, in a similar position to where you would expect it to be. Press the button labeled "2D Pose Estimate". The cursor will become an arrow, and you can press it where you think the car is. Try driving the car around using the joystick, and notice to what extent the localization is able to track the car's position.

Now, we will choose a goal for the car to navigate towards. We recommend starting with simple goal poses and gradually increasing the complexity. To select a goal position, choose the button labeled "2D Nav Goal" and select the goal pose.

Once you've seen the session for the rhc node output the text `Goal set`, hold the left-hand deadman switch to allow the car to track towards its goal. Release the button if you suspect the car is close to a collision.

If the car loses localization, simply re-click with the "2D Pose Estimate" button. You can set a new goal at any time, even if the car has not reached the goal you specified.

That's it, now you have basic autonomous navigation!
