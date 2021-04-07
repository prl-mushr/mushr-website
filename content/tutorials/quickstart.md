---
title: "Quickstart"
date: 2018-11-28T15:14:54+10:00
featured: true
draft: false
active: true
duration: 30
difficulty: Beginner
summary: Run the MuSHR platform on your machine!
weight: 1
---

<h2> By: <a href=https://www.mattschmittle.com/>Matt Schmittle</a></h2>                              
{{< mp4gif src="/tutorials/quickstart/quickstart_header.mp4" width="800" figure=true >}}
<br>

### Introduction
This tutorial will get you started with MuSHR in simulation!

### Goal 
To get the simulator running on your machine so that you can begin hacking immediately!

### Requirements
* **A Ubuntu 16.04 or 18.04 Linux machine** (20.04 not supported yet!). If you don't run linux natively you can use an [Ubuntu VM](https://www.freecodecamp.org/news/how-to-install-ubuntu-with-oracle-virtualbox/).

We also provide a virtual machine image that already has the MuSHR stack setup, it can be downloaded [here](https://drive.google.com/a/cs.washington.edu/file/d/1mOzSzVx9BF_e2U1OeK58NS42UIPcnIZq/view?usp=sharing). The username is **robot** and the password is **prl_robot**. If you use this image, you can start the VM and then skip to the [**Running the Simulator**](#running-the-simulator) section. If you're using VirtualBox, here is a tutorial for [setting up the virtual machine image](https://docs.oracle.com/cd/E26217_01/E26796/html/qs-import-vm.html).

For good performance, it's a good idea to set the RAM used by your VM to at least 2048 MB. If using VirtualBox, navigate to Settings -> System and you should see a slider
which allows you to modify RAM.

* **Window Subsystem for Linux (WSL):** There has also been success getting the quickstart to run using WSL. When running `rivz` you'll need to disable native `opengl`. There will be a note ([**Note for WSL**](#wsl-users-note)) in the section running `rviz`.

* **Python2.7:** If you have Python3 as your default version, download [Conda](https://docs.conda.io/en/latest/) to easily create/manage Python2.7 virtual environments.
* To learn how to use conda environments please see the following [tutorial](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html)
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
- [pip](https://pip.pypa.io/en/stable/installing/)


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

### Changing the map

The current map is fine for driving around, but to do more interesting things you may want more interesting maps. To change the map follow these steps.

{{< highlight bash >}}
# To list what maps are available
$ ls ~/catkin_ws/src/mushr_base/mushr_base/mushr_base/maps/
# Open map_server.launch
$ nano ~/catkin_ws/src/mushr/mushr_base/mushr_base/mushr_base/launch/includes/map_server.launch
{{< / highlight >}}

Edit `map_server.launch` to set the map to the name of the .yaml file. It should look like this, replacing map_name with the name
of the map you want to use.
{{< highlight python "linenos=table" >}}
<launch>
    <arg name="map" default="$(find mushr_base)/maps/map_name.yaml" />
    <node pkg="map_server" name="map_server" type="map_server" args="$(arg map)" />
</launch>
{{< / highlight >}}

You can make your own maps using SLAM based techniques like [gmapping](http://wiki.ros.org/gmapping) or [cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/). But if you just want something quick and simple, you can make your own `.yaml` and `.png` files for the map. Use the current map files as a template. We use [Gimp](https://www.gimp.org/) to edit the map `.png` file but you can use any image editor you like.

## Going Further
To learn about programming the car, continue to the [Intro to ROS Tutorial](/tutorials/intro-to-ros).

## Troubleshooting
Commonly encountered problems during this tutorial.

### Virtual Machines
#### Failed to open ovf descriptor when importing the virtual machine image
This is a known issue with .ova file extensions downloading as .ovf from Google Suite. To fix, rename the file extension to .ova.
#### Can't copy and paste into VirtualBox
Power down the virtual machine. Select Settings -> General -> Advanced -> SharedClipboard and select bidirectional. Restart the virtual machine.
#### Ubuntu runs slowly/freezes on VirtualBox
This likely occured because the default settings for new virtual machines
are low. For better performance, it's a good idea to double the default memory
(from 1024 to 2048 MB). Select Settings -> System and you should see a slider
which allows you to increase RAM.
