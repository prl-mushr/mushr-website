---
title: "Autonomous Navigation"
date: 2022-06-08T13:31:34-07:00
featured: true
draft: false
duration: 30
active: true
difficulty: Intermediate
summary: Initialize and operate MuSHRs out-of-the-box autonomous navigation stack!
weight: 3
---

<h2> By: <a href=https://www.linkedin.com/in/markusschiffer/ />Markus Schiffer</a> & /h2>   

{{< mp4gif src="/tutorials/autonomous-navigation/auto.mp4" figure=true >}}<br/>

## Introduction

### Goal 

This tutorial will teach you to set up and operating MuSHR's baseline autonomous navigation stack. By the end of the tutorial, the car will be able to autonomously navigate around known obstacles on a known map.

### Requirements

- If in sim, complete the [quickstart tutorial](https://mushr.io/tutorials/quickstart/)
- If on real car, complete the [first steps tutorial](https://mushr.io/tutorials/first_steps/)
- Python dependencies for [`mushr_rhc`](https://github.com/prl-mushr/mushr_rhc)

#### Note on dependecies: 
This tutorial is designed for the python3 ROS noetic image. It assumes that this accounts for any dependencies needed.

If you are in sim we don't recommend testing with the `sandbox.yaml` default map because the localization struggles in an open environment (all positions look the same!). See the [quickstart tutorial](https://mushr.io/tutorials/quickstart/) for how to change maps.

## Navigation Stack Overview

At the highest level MuSHR's navigation stack consists of two principal components:

1. **Receding Horizon Controller (RHC) Node:** This node is responsible for planning the motions and generating controls for the car. The implementation we ship with the car uses Model Predictive Control (MPC) to generate control signals which are sent to the car's motor controller (VESC).
2. **Localization Node:** In order for the controller to know whether it is in the proximity of obstacles, it must know its location on a known map. Solving this problem is called "localization". The Localization Node is implemented using a method called Particle Filtering which in this case relies primarily on a data stream from the laser scanner.
3. **Planner Node** This node generates a plan that your controller will try to follow. Unlike the controller, our planner does not consider dynamic obstacles when constructing its plan; rather it uses a static map of the environment. It chains the car's motion primitives together into a plan, computer using a search algorithm (such as A*).

This tutorial does not cover Model Predictive Control, Particle Filtering, and A* in depth.

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
# Clone the planner node
Depending on your setup, you may want to clone different MuSHR planners. If you plan to run entirely in sim, or have the Pro MuSHR car (uses NVidia Jeson TODO), you may want to use this version of the planner:
{{< highlight bash>}}
$ git clone git@github.com:prl-mushr/mushr_gp.git
# Re-make to update paths 
$ cd ~/catkin_ws && catkin_make
{{</ highlight >}}
If you have a standard MuSHR car, the above planner is unfortunately too resource intensive. Instead, use this global planner.
{{< highlight bash>}}
$ git clone git@github.com:prl-mushr/mushr_gprm.git
{{</ highlight >}}


*You also need to download all dependencies packages for the [`mush_rhc`](https://github.com/prl-mushr/mushr_rhc)*.

Both repositories contain ROS packages that reproduce the desired functionality. However, you need only concern yourself with each package's launch files to use them effectively. You can find the launch files in each package's `launch` directory.

## Running the navigation stack

Now we will launch the navigation stack directly on the robot. To learn about strategies for effectively operating and experimenting with the MuSHR car, visit the [workflow tutorial](). We suggest using [`tmux`](https://hackernoon.com/a-gentle-introduction-to-tmux-8d784c404340) to manage multiple ROSlaunch sessions.

Once you've ssh'd into your robot, activate `tmux`:
{{< highlight bash >}}
$ tmux
{{< / highlight >}}

Then, to create two vertical panes, type `ctrl+b` (`ctrl` and `b`) then `%` (or alternatively `"` to split horizontally). You can use `ctrl+b` then your left and right arrow keys to switch between windows. We will need five panes for this tutorial on the real car, and four on the simulator.

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

Finally, go to the next tmux pane, and launch the global planner node. Remember, there are two planners that come with MuSHR so pick the one which best suits your needs. 
{{< highlight bash >}}
$ roslaunch mushr_gp sim.launch
{{< / highlight >}}
Or alternatively
{{< highlight bash >}}
$ roslaunch mushr_gprm sim.launch
{{< / highlight >}}




## Operating the navigation stack

Now it's time to initialize the particle filter, giving it an initial estimate of the distribution of possible poses. On your separate workstation, we will use Foxglove. [Foxglove](https://foxglove.dev) allows us to visualize the robot's telemetry, such as position, laser scan messages, etc. You've likely already used it in the simulator.

Start the foxglove app on your computer. In another terminal, launch the mushr_noetic Docker image.
{{< highlight bash >}}
$ mushr_noetic
{{< / highlight >}}

Add the navigation topics if they are not selected already by clicking `ADD` &rarr; `By Topic` in rviz. Add the following:
- `/car/particle_filter/inferred_pose`: The particle filter's estimate of the car position
- `/car/particle_filter/particles`: The particle filter's distribution of particles (OPTIONAL) 
- `/car/rhccontroller/traj_chosen`: The trajectory the controller is choosing 
- `/car/global_planner/path`: The path created by the global planner

If using the sim, you can now set the initial position of the car make sure the "Set Pose" button is selected, and then hold the click to publish button. Ensure "Publish pose" is selected, and then click on the map where you want the car to start from. Then you can use the mouse to click another point on the screen; the direction of the car will be from the first click toward the second one.

If on the real car, you may see that the car is already in the correct place. This means that the localization node has correctly determined the location of the car based on its surroundings. If you're on a map with a lot of similar sections, the localization may be unable to correctly determine an initial postion with high probability. You may want to help it do so. You can select the "Set Pose Estimate" button, and follow the same steps as above to set the pose of the car. After this, you're localization node should be able to track the car's postion accurately. Try driving the car around using the joystick, and notice to what extent the localization is able to track the car's position.

Now, we will choose a goal for the car to navigate towards. We recommend starting with simple goal poses and gradually increasing the complexity. To select a goal position, choose the button labeled "Set Goal" and set the goal pose by clicking the "Click to publish" button. After this, click once on the goal location, and using a second click to specify a direction.

Once you've seen the session for the rhc node output the text `Goal set` and the planner should output `Solution found`. Your car should start moving in sim. In real, hold the right-hand deadman switch (R1) to allow the car to track towards its goal. Release the button if you suspect the car is close to a collision.

If the car loses localization, simply re-click with the "Set Pose Estimate" button. You can set a new goal at any time, even if the car has not reached the goal you specified.

That's it, now you have basic autonomous navigation!