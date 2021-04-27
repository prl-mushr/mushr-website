---
title: "Global Planner"
date: 2020-01-25T15:14:54+10:00
featured: false
draft: false
active: false
duration: 60
difficulty: Advanced 
summary: Plan paths from start to goal in known map
weight: 3
---

<h2> By: <a href=https://www.mattschmittle.com/>Matt Schmittle</a></h2>                              
<br>
{{< figure src="/tutorials/global_planner/header.png" width="1000" >}}
<br>

### Introduction
The [Open Montion Planner Library(OMPL)](http://ompl.kavrakilab.org/index.html) contains modules for computing motion plans using sampling-based algorithms. OMPL is designed to be easily integrated into other systems and provide necessary components such as ROS, and Moveit. In this tutorial, we will be using the mushr_global_planner that provides [ROS services](http://wiki.ros.org/rospy/Overview/Services) as an API for planning a path from a start position to a goal position.

### Goal
Plan paths from start to goal in known map

### Requirements
  - Complete the [quickstart](/tutorials/quickstart) tutorial  
  - Complete the [Intro to Ros](/tutorials/intro-to-ros) tutorial  

## Setup
- Install ompl from source [here](https://ompl.kavrakilab.org/installation.html) with python bindings or downloading the script [here](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh)

    Change python version setting in the script from `$PYTHONV=3` to `$PYTHONV=2.7`
     and make the script executable.
    ```
    $ chmod u+x install-ompl-ubuntu.sh
    ```
    Make sure you have the [following](https://ompl.kavrakilab.org/installPyPlusPlus.html) installed. Then run the script.

    ```
    $ ./install-ompl-ubuntu.sh --python
    ```
    If you have a segfault with CastXML on Ubuntu 16.04, install the binaries from [here](https://data.kitware.com/#collection/57b5c9e58d777f126827f5a1/folder/57b5de948d777f10f2696370). You will need to put the contents of `.../castxml/bin` into `/usr/local/bin/` and `.../castxml/share` into `/usr/local/share` and uninstall your current version of cast xml.
    ```
    $ sudo apt remove castxml
    ``` 
- Clone the necessary repo to your 
    ```
    # Go to your catkin workspace
    $ cd ~/catkin_ws/src
    # Clone the global planner node
    $ git clone https://github.com/prl-mushr/mushr_global_planner
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
$ roslaunch mushr_global_planner demo.launch
```
Then open rviz
```
$ rviz
```
Then subscribe to the `/mushr_global_planner_result` topic, you should see a red path of `PoseArray`

## Using Mushr Global Planner 

#### The code 
The code below (`global_planner.py`) demonstrate how to use the ROS service.

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


Let's unpack this. First, we have to make sure that service that we want to use, `mushr_global_planner` in this case, is running and ready. `rospy.wait_for_service` will wait until a service with given name becomes avaiable.
```
rospy.wait_for_service('mushr_global_planner')
```
Then, we could create a callable instance giving the service name and class. The instance could be use just like a regular python method.  
```
rospy.ServiceProxy('mushr_global_planner', MushrGlobalPlanner)
```
`global_planner` takes parameter as listed below: 
Lastly, we call the server with parameters to plan a path from the start position to the goal position.

```
start_pose = Pose(Point(-1.887, 26.930,0.0), Quaternion(0,0,0,1)) 
goal_pose = Pose(Point(41.804, 0.761, 0.0), Quaternion(0,0,0,1))
response = mushr_global_planner(header=None, start=start_pose, goal=goal_pose, turning_radius=5.0, planning_time=30.0)
```

### Running Globel Planner
With python files, we have to change the permissions to be able to execute and run.
```
$ chmod +x mushr_ros_intro/src/global_planner.py
```
Set parameter necessary for the global planner node by using the command [rosparam](http://wiki.ros.org/rosparam), which will be explained further below.
```
$ rosparam set reuse_plans True
$ rosparam set kernal_size 31
$ rosparam set interpolation_density 1000
$ rosparam set validity_resolution 0.0005
```
Then initiate the `mushr_global_planner` server to be ready for client to invoke and plan the path.
```
$ roslaunch mushr_global_planner planner.launch
```
Since the launch file of the mushr global planner does not include the map server by default, we have to manually launch it.
```
$ roslaunch mushr_global_planner map_server.launch
```
Next, we run the `global_planner.py` to create the path.
```
$ rosrun global_planner.py
```
Finally, open the rviz
```
$ rviz
```
Then subscribe to the `/mushr_global_planner_result` topic, you should see the same red path as the one generated by the demo.

### Parameters for the service
Parameter | Type | Description
----------|------|-------------
`start` | [geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html) | Start position of the path in the world frame
`goal` | [geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html) | Goal position of the path in the world frame
`turning_radius` | [Float](https://docs.python.org/2/tutorial/floatingpoint.html) | Turning radius of the car in meters. You can calculate your turning radius although we recommend you tune your car to match your expected turning radius. You should match the variables to the vesc settings preset [car length](https://github.com/prl-mushr/vesc/blob/master/vesc_main/config/racecar-uw-nano/vesc.yaml) and [delta](https://github.com/prl-mushr/mushr_base/blob/master/mushr_base/config/joy_teleop.yaml) (steering angle). \![equation image](https://drive.google.com/uc?export=view&id=12Fe6HDtbWj7XZcV6HvmQ-qeWHpfCcDX0)
`planning_time` | [Float](https://docs.python.org/2/tutorial/floatingpoint.html) | Planning time cutoff in seconds E.g. give me your best solution after 30 seconds

### How the planner works
The planner is essentially a wrapper around OMPL doing Dubin’s path planning using BIT*. The input map is used to get the bounds of the state space and for determining if a sampled state is invalid (either by being out of bounds or within an obstacle). The planner uses the optimization objective of minimizing the path length of the solution. It then converts the solution path to a non-ompl format (list of tuples) and returns it to the user. The path is filled in with interpolated points via solutionPath.interpolate to create a dense path, but this can be tuned to vary the density of the output path.

#### Publishers
Topic | Type | Description
------|------|------------
`/mushr_global_planner_start` | [geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html) | Start position
`/mushr_global_planner_goal` | [geometry_msgs/Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html) | Goal position
`/mushr_global_planner_result` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html) | Planned path of poses in world coordinates from start to goal

#### Services
Topic | Type | Description
------|------|------------
`/mushr_global_planner` | [mushr_global_planner/MushrGlobalPlanner](srv/MushrGlobalPlanner.srv) | Calls for a path to be created with a given start, goal, turning radius, and planning time

#### Parameters for the planner
Parameter | Type | Description
----------|------|-------------
`reuse_plans` | [Boolean](https://docs.python.org/2/c-api/bool.html) | Option to use the same previously generated path if the parameter is set to True, otherwise create a new path from the global planner service. This is used when the planner node is called multiple times with the same settings. 
`kernal_size` | [Integer](https://docs.python.org/2/c-api/int.html) | The kernal that convolves over the map. A higher kernal size results in a planner that has a larger buffer from walls but may not find a path through tight spaces. Smaller sizes result in less of a buffer from walls.
`interpolation_density` | [Integer](https://docs.python.org/2/c-api/int.html) | A number of states in a path so that the path is made up of exactly `interpolation_density` states. 
`validity_resolution` | [Float](https://docs.python.org/2/tutorial/floatingpoint.html) | The resolution at which state validity needs to be verified in order for a motion between two states to be considered valid (collision free). This value is specified as a fraction of the space's extent.
