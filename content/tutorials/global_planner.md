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
The [Open Montion Planner Library(OMPL)](http://ompl.kavrakilab.org/index.html) contains modules for computing motion plans using sampling-based algorithm. The OMPL is designed to be easily integrated into other systems that provide necessary component such as ROS, and Moveit. In this tutorial, we will be using the ROS package that provides [ROS services](http://wiki.ros.org/rospy/Overview/Services) as an API for planning a path from a start position to a goal position.

### Requirement
In order to successfully finish this tutorial you need to be familar with python and bash command, complete the [quickstart](/tutorials/quickstart) and [Intro to Ros](/tutorials/intro-to-ros) tutorial, and have a basic understanding of [ROS topics](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).
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
    $ git clone https://github.com/schmittlema/mushr_global_planner.git
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

#### The code 
The code below examplify how to use a ROS service, which will be explained in greater detail later. Save this file in `mushr_ros_intro/src/global_planner.py`. 

{{< highlight python "linenos=table" >}}

    #!/usr/bin/env python
    import rospy
    from gpnode import MushrGlobalPlanner
    from geometry_msgs.msg import (
        Pose, 
        PoseArray,
        Point,
        Quaternion
    )

    if __name__ == "__main__":
        rospy.wait_for_service('mushr_global_planner')
        mushrt_global_planner = rospy.ServiceProxy('mushr_global_planner', MushrGlobalPlanner)
        start_pose = Pose(Point(-1.887, 26.930,0.0), Quaternion(0,0,0,1)) 
        goal_pose = Pose(Point(41.804, 0.761, 0.0), Quaternion(0,0,0,1))
        response = mushr_global_planner(header=None, start=start_pose, goal=goal_pose, turning_radius=5.0, planning_time=30.0)

{{< / highlight >}}


First, we have to make sure that service that we want to use, `mushr_global_planner` in this case, is running and ready. `rospy.wait_for_service` will wait until a service with given name becomes avaiable. You can limit the waiting time by specify the timeout(in seconds)
```
rospy.wait_for_service('mushr_global_planner')
```
Then, we could create a callable instance giving the service name and class. The instance could be use just like a regular python method.  
```
rospy.ServiceProxy('mushr_global_planner', MushrGlobalPlanner)
```
`global_planner` takes parameter as listed below: 
 - start position is in world frame
 - goal position is in world frame
 - turning radius is the turning radius of the car in meters. You can calculate your turning radius although we recommend you tune your car to match your expected turning radius. You should match the variables to the vesc settings preset [car length](https://github.com/prl-mushr/vesc/blob/master/vesc_main/config/racecar-uw-nano/vesc.yaml) and [delta](https://github.com/prl-mushr/mushr_base/blob/master/mushr_base/config/joy_teleop.yaml) (steering angle).  
    ![equation image](https://drive.google.com/uc?export=view&id=12Fe6HDtbWj7XZcV6HvmQ-qeWHpfCcDX0)
 - planning time is the planning time cutoff. E.g. give me your best solution after 30 seconds
 ```
 start_pose = Pose(Point(-1.887, 26.930,0.0), Quaternion(0,0,0,1)) 
goal_pose = Pose(Point(41.804, 0.761, 0.0), Quaternion(0,0,0,1))
response = mushr_global_planner(header=None, start=start_pose, goal=goal_pose, turning_radius=5.0, planning_time=30.0)
 ```

#### running globel planner
With python files, we have to change the permissions to be able to execute and run.
```
$ chmod +x mushr_ros_intro/src/global_planner.py
```
Then initiate the `mushr_global_planner` server to be ready for client to invoke and plan the path.
```
roslaunch mushr_global_planner planner.launch
```
Since the launch file of the mushr global planner does not include the map server by default, we have to manually launch it.
```
roslaunch mushr_global_planner map_server.launch
```
Next, we run the `global_planner.py` to create the path.
```
rosrun mushr_ros_intro global_planner.py
```
Finally, open the rviz
```
rviz
```
Then subscribe to the `/mushr_global_planner_result` topic, you should see the same red path as the one generated by the demo.

