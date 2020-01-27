---
title: "Global Planner"
date: 2020-01-25T15:14:54+10:00
image: "/services/default.png"
featured: false
draft: true
duration: 30
active: true
difficulty: Intermediate
weight: 3
---

## Introduction

### Goal
This tuturial will teach you to setup and plan a path from one start position in the map to a goal position. 

### Requirement
To complete this tutorial, you need to:
- Complete the [quickstart](/tutorials/quickstart) tutorial.
- 
## Setup
- Install ompl from source [here](https://ompl.kavrakilab.org/installation.html) with python bindings or downloading the script [here](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh)
    - Change python version setting in the script from `$PYTHONV=3` to `$PYTHONV=2.7`
    - make the script executable
    ```
    $chmod u+x install-ompl-ubuntu.sh
    ```

    - run the script

    ```
    $ ./install-ompl-ubuntu.sh --python
    ```
    - If you have a segfault with CastXML on Ubuntu 16.04, install the binaries from [here](https://data.kitware.com/#collection/57b5c9e58d777f126827f5a1/folder/57b5de948d777f10f2696370). You will need to put the contents of `.../castxml/bin` into `/usr/local/bin/` and `.../castxml/share` into `/usr/local/share`. Also, uninstall your current version of cast xml `sudo apt remove castxml` 
    - make sure you have the [following](https://ompl.kavrakilab.org/installPyPlusPlus.html) installed
- Clone the necessary repo to your 
    ```
    # Go to your catkin workspace
    $ cd ~/catkin_ws/src
    # Clone the global planner node
    $ git clone https://github.com/thompsonmax/mushr_global_planner.git
    ```
- Make and source your workspace. (Assuming source commands in .bashrc)
    ```
    $ cd ~/catkin_ws
    $ catkin_make
    $ source ~/.bashrc
    ```
## Running Demo
We have a pre-built demo script that include start and goal position. To use this just run
```
roslaunch mushr_global_planner demo.launch
```
Then open rviz
```
rviz
```
Then subscribe to the `/mushr_global_planner_result` topic, you should see a red path of `PoseArray`

## Using Mushr Global Planner 
To use the global planner simply launch
```
roslaunch mushr_global_planner planner.launch
```
\* Note that the defualt launcher does not include a map server.

To call the planner, simply make a service request using the API:
 - start position is in world frame
 - goal position is in world frame
 - turning radius is the turning radius of the car in meters. You can calculate your turning radius although we recommend you tune your car to match your expected turning radius. You should match the variables to the vesc settings preset [car length](https://github.com/prl-mushr/vesc/blob/master/vesc_main/config/racecar-uw-nano/vesc.yaml) and [delta](https://github.com/prl-mushr/mushr_base/blob/master/mushr_base/config/joy_teleop.yaml) (steering angle).  
    ![equation image](https://drive.google.com/uc?export=view&id=12Fe6HDtbWj7XZcV6HvmQ-qeWHpfCcDX0)
 - planning time is the planning time cutoff. E.g. give me your best solution after 30 seconds
#### Publishers
Topic | Type | Description
------|------|------------
`/mushr_global_planner_start`|[geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html)| Start position
`/mushr_global_planner_goal`|[geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html)|Goal position
`/mushr_global_planner_result`|[geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Planned path of poses in world coordinates from start to goal

#### Services
Topic | Type | Description
------|------|------------
`/mushr_global_planner`|[mushr_global_planner/MushrGlobalPlanner](srv/MushrGlobalPlanner.srv)| Calls for a path to be created with a given start, goal, turning radius, and planning time