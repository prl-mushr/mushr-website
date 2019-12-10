---
title: "Quickstart"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
duration: 30
difficulty: Beginner
summary: Run the MuSHR platform on your machine!
weight: 1
---

{{< figure src="/tutorials/quickstart/quickstart_header.gif" width="800" >}}
<br>

### Introduction
This tutorial will get you started with MuSHR in simulation!

### Goal 
To get the simulator running on your machine so that you can begin hacking immediately!

### Requirements
A Ubuntu Linux machine. If you don't run linux natively then get a Ubuntu VM: [OSX](https://www.instructables.com/id/How-to-Create-an-Ubuntu-Virtual-Machine-with-Virtu/), [Windows](https://itsfoss.com/install-linux-in-virtualbox/).

Window Subsystem for Linux (WSL): There has also been success getting the quickstart to run using WSL. When running `rivz` you'll need to disable native `opengl`. There will be a note ([**Note for WSL**](#wsl-users-note)) in the section running `rviz`.

## Setup
First we need to make sure you have a few dependencies installed. All commands are to be executed in a terminal (CTRL + ALT + T). Here is what you need:

- [ROS Melodic Desktop Full](http://wiki.ros.org/melodic/Installation) (for Ubuntu 18.04) or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) (for Ubuntu 16.04)
*You could also try installing ROS on another supported platform, but as of right now this tutorial has not been tested on non-Ubuntu machines.*
- A [catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- git:

{{< highlight bash >}}
$ sudo apt install git-all
{{< / highlight >}}

- tkinter:

{{< highlight bash >}}
$ sudo apt install python-tk
{{< / highlight >}}

- A github account. You can sign up for one [here](https://github.com/join?source=header-home).
- [vcstool](https://github.com/dirk-thomas/vcstool.git)


Once you have these, you're good to go!

## Install Sim
Now that we have the dependencies, lets get started! We'll start by making sure we have all the necessary ROS packages. Select **one** of the following, based off the version of ROS you installed.

**Melodic:**
{{< highlight bash >}}
$ sudo apt install -y ros-melodic-ackermann-msgs ros-melodic-map-server ros-melodic-serial ros-melodic-urg-node ros-melodic-robot-state-publisher ros-melodic-xacro
{{< / highlight >}}

**Kinetic:**
{{< highlight bash >}}
$ sudo apt install -y ros-kinetic-ackermann-msgs ros-kinetic-map-server ros-kinetic-serial ros-kinetic-urg-node ros-kinetic-robot-state-publisher ros-kinetic-xacro
{{< / highlight >}}

Now, let's clone the necessary repos. First go to your `catkin_ws/src` directory:

{{< highlight bash >}}
$ cd ~/catkin_ws/src
{{< / highlight >}}

Download [repos.yaml](/tutorials/quickstart/repos.yaml) into `~/catkin_ws/src`.

And clone the necessary repos using vcstool:

{{< highlight bash >}}
$ vcs import < repos.yaml
{{< / highlight >}}

We need the realsense2_description directory only:

{{< highlight bash >}}
$ mv ~/catkin_ws/src/mushr/mushr_hardware/realsense/realsense2_description ~/catkin_ws/src/mushr/mushr_hardware/realsense2_description
$ rm -rf ~/catkin_ws/src/mushr/mushr_hardware/realsense
{{< / highlight >}}

We need to also install rangelibc. First, if you don't have Cython installed, install it now:

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
