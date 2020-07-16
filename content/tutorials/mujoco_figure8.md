---
title: "Figure 8 in Mujoco Simulator."
date: 2020-07-15T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: true (change to false)
active: false (change to true)
duration: (need to calculate)
difficulty: Beginner
summary: Run the MuSHR platform on your machine!
weight: (Needs to be changed)
---

{{< figure src="/tutorials/quickstart/quickstart_header.gif" width="800" >}}
<br>

### Introduction
This tutorial will introduce you to controlling the mushr bot in the MuJoCo environment.

### Goal 
To command the MuSHR bot to make a figure 8 in the MuJoCo simulator.

### Requirements
A Ubuntu Linux machine. If you don't run linux natively then get a Ubuntu VM: [OSX](https://www.instructables.com/id/How-to-Create-an-Ubuntu-Virtual-Machine-with-Virtu/), [Windows](https://itsfoss.com/install-linux-in-virtualbox/). 

We also provide a virtual machine image that already has the MuSHR stack setup, it can be downloaded [here](https://drive.google.com/a/cs.washington.edu/file/d/1mOzSzVx9BF_e2U1OeK58NS42UIPcnIZq/view?usp=sharing). The username is **robot** and the password is **prl_robot**. If you use this image, you can start the VM and then skip to the [**Running the Simulator**](#running-the-simulator) section.

Window Subsystem for Linux (WSL): There has also been success getting the quickstart to run using WSL. When running `rivz` you'll need to disable native `opengl`. There will be a note ([**Note for WSL**](#wsl-users-note)) in the section running `rviz`.

## Setup

Make sure you have completed the MuSHR Quickstart tutorial (link: ) and MuJoCo setup tutorial (link: ) before you proceed.

Let's create our "figure 8" plan text files.

{{< highlight bash >}}
$ cd ~/catkin_ws/src/mushr_mujoco_ros
$ mkdir plans
$ cd plans
{{< / highlight >}}

Next is to create a file called figure8.txt and save the below coordinate commands in it.

{{< highlight bash >}}
  $ nano figure8.txt
{{< / highlight >}}

Copy the below contents in figure8.txt



-----------------------------------------------------------------------------------------------------------------

{{< highlight bash >}}
$ sudo pip install Cython 
{{< / highlight >}}

Now that Cython's installed, you can install rangelibc:

{{< highlight bash >}}
$ cd ~/catkin_ws/src/range_libc/pywrapper
$ sudo python setup.py install
$ cd ~/catkin_ws/src && rm -rf range_libc
{{< / highlight >}}

We will now run `catkin_make` to setup all the packages:
{{< highlight bash >}}
$ cd ~/catkin_ws && catkin_make
{{< / highlight >}}

### Setting up our environment

To make sure our environment is setup we run:

*If you are using a shell other than bash, be sure to put these "source" commands in the analagous file for your shell.*

#### Kinetic (Ubuntu 16.04)

{{< highlight bash >}}
$ echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
$ echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
$ . ~/.bashrc
{{< / highlight >}}

#### Melodic (Ubuntu 18.04)

{{< highlight bash >}}
$ echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
$ echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
$ . ~/.bashrc
{{< / highlight >}}

Putting these lines in the `~/.bashrc` guarantee they run on the startup of a new shell.

Finally, move the "Outrun" themed `.rviz` file to `~/.rviz` to get our default setup:
{{< highlight bash >}}
$ cp ~/catkin_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/
{{< / highlight >}}

That's it! Time to run it.

## Running the Simulator
To start the sim run:

{{< highlight bash >}}
$ roslaunch mushr_sim teleop.launch
{{< / highlight >}}

{{< figure src="/tutorials/quickstart/teleop_window.png" caption="Teleop window that should appear after starting the sim" width="200">}}

And in another terminal window launch rviz:

{{< highlight bash >}}
$ rviz
{{< / highlight >}}

<span id="wsl-users-note">**WSL Users Note:**</span> In order for Open GL to find a display you'll need to do an extra step to get `rviz` to work. See [Install VcXsrv](https://janbernloehr.de/2017/06/10/ros-windows#install-vcxsrv) to get it working.

The `rviz` window with the car model should appear (see below). `Rviz` is useful for visualizing what the car is thinking/seeing. Currently it is set to visualize the car, map, and laserscan but `rviz` can be used for much [more](http://wiki.ros.org/rviz/Tutorials).

{{< figure src="/tutorials/quickstart/rviz_docker-update.png" caption="This is an image of the `rviz` window that should pop up." width="800">}}

### Setting an Initial Position

Give the car an initial position by clicking
{{< figure src="/tutorials/quickstart/2d_pose_estimate.png" width="150">}}

in `rviz` and clicking and dragging in the main window. Now you can click on the small gray window and use the WASD keys to drive the car around! You can set the position at any time to reset the pose.

### Navigating rviz

The main pane will show a map of an empty square space. The ligher areas are the space where the simulated car can drive around. The darker areas are walls.

Clicking and dragging will change the perspective of `rviz`, while `Shift + Click` and draging will move the map around.

## Going Further
To learn about programming the car, continue to the [Intro to ROS Tutorial](/tutorials/intro-to-ros).
