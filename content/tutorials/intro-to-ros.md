---
title: "Intro to ROS"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
difficulty: Beginner
duration: 60
summary: Learn basic ROS concepts.
weight: 2
---

## Introduction

### Goal
This tutorial will help you get familiar with ROS concepts in reference to the MuSHR software stack. Afterwards you should start to become comfortable with ROS [Publishers and Subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

### Prerequisites
In order to successfully complete this tutorial you will need: 

1. to have a working instalation of ROS. (get a link to relevant installation tools. quickstart?)
+  to have completed the [quickstart](/tutorials/quickstart) tutorial.
+  (*should*) have familiarity with `bash` and `python`.


### Notes
This tutorial assumes your catkin workspaces is located at `~/catkin_ws`. If you followed the [quickstart](/tutorials/quickstart) tutorial, this will be the case. If your workspace is in a different directory, adjust the provided command accordingly.

## Creating a package

First start by creating a catkin package for our code:

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg mushr_ros_intro std_msgs rospy ackermann_msgs
```

Our package depends on `std_msgs`, `rospy`, and `ackermann_msgs`. All new ROS packages should depend on `std_msgs` and `rospy` (if you plan to use C++, then your package should also depend on `roscpp`). We include `ackermann_msgs` because it is the way we send velocities and steering angles to be applied on the car (more on this later in the tutorial).

Now we will build our empty package:

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscd mushr_ros_intro
```

Sourcing `devel/setup.bash` sets up the ROS environment as well as sets up some useful auto-complete rules, easing the traversal of multiple ROS packages. We can see this power with the `roscd` command. Once you use the `roscd` with `mushr_ros_intro` once, you should be able to tab complete the name. *Sometimes, when you encounter packages not being found, all you need it to rerun the source command.*

## A simple plan specification

Now, that we have a package, we want create a source file to run our node. We will be creating a simply ROS node to read commands (velocity and steering angle) from a file line by line, and sending them to the simulator to be applied to the simulated car. Each line will denote a command to be applied for a second. The first line is a message to send as the "starting pose" of the car. The input files will be of the form:

```
0,0,0.0
2.0,0.09
3.0,-0.15
```

The first line is the initial position, of the form `x, y, theta`, where `x` and `y` are the starting coordinates in the map, and `theta` is the initial angle of the car. The following two commands tell the car how fast to go, and at what steering angle. The first to run at `2.0 meters per second`, with a steering angle of `0.09 radians`. The second, to run at `3.0 meters per second`, with a steering angle of `-0.15 radians`. We will be applying each command for 1 second. A positive steering angle corresponds to a left turn and a negative steering angle corresponds to a right turn.

Create a directory `plans` in the package:
```bash
$ roscd mushr_ros_intro # if you aren't in the intro package directory
$ mkdir plans
```

Here are two plans you can use (add these two files to the directory we just created):

`plans/straight_line.txt`
```txt
0,0,0
2,0.0
3,0.0
4,0.0
5,0.0
6,0.0
6,0.0
6,0.0
5,0.0
4,0.0
3,0.0
2,0.0
```

`plans/figure_8.txt`
```txt
0,0,0.785
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
2.0,-0.09
```

Try and figure out what these plans will do (hint: look at the file names :)).

## The code

Below the entire code is listed. Each section will be explained in greater detail below the listing. Save this file in `src/path_publisher.py`

{{< highlight python "linenos=table" >}}
#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler


def run_plan(pub_init_pose, pub_controls, plan):
    init = plan.pop(0)
    send_init_pose(pub_init_pose, init)

    for c in plan:
        send_command(pub_controls, c)


def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))


def send_command(pub_controls, c):
    cmd = c.split(",")
    assert len(cmd) == 2
    v, delta = float(cmd[0]), float(cmd[1])

    dur = rospy.Duration(1.0)
    rate = rospy.Rate(10)
    start = rospy.Time.now()

    drive = AckermannDrive(steering_angle=delta, speed=v)

    while rospy.Time.now() - start < dur:
        pub_controls.publish(AckermannDriveStamped(drive=drive))
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("path_publisher", anonymous=True, disable_signals=True)

    control_topic = rospy.get_param("~control_topic", "/mux/ackermann_cmd_mux/input/navigation")
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    plan_file = rospy.get_param("~plan_file")

    with open(plan_file) as f:
        plan = f.readlines()

    # Publishers sometimes need a warm-up time, you can also wait until there
    # are subscribers to start publishing see publisher documentation.
    rospy.sleep(1.0)
    run_plan(pub_init_pose, pub_controls, plan)
{{< / highlight >}}

### Includes

We define functions and modules we need first:

{{< highlight python "linenos=table" >}}
#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler

{{< / highlight >}}

`rospy` is the main python interface to the ROS API. [Ackermann steering](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) is the geometry of our wheels, the [`ackermann_msgs`](http://wiki.ros.org/ackermann_msgs) defines a common interface to send drive commands. The imports from [`geometry_msgs`](http://wiki.ros.org/geometry_msgs) are for sending the initial pose of the car to the simulator.

## Writing the launch file

In order to run out code in a convenient and extendable way, ROS has the notion of launch files. These are XML files that describe how different software components should be started. These file allow us to start a large number of ROS nodes with few commands. We will write a launch file for our node to conform to this standard. First we make a launch directory:
```bash
$ roscd mushr_ros_intro # if you aren't in the directory
$ mkdir launch
```

Create the file `launch/path_publisher.launch`, containing:
{{< highlight xml "linenos=table" >}}
<launch>
    <arg name="control_topic" default="/mux/ackermann_cmd_mux/input/navigation" />
    <arg name="init_pose_topic" default="/initialpose" />
    <arg name="plan_file" default="$(find mushr_ros_intro)/plans/straight_line.txt" />

    <node pkg="mushr_ros_intro" type="path_publisher.py" name="path_publisher" output="screen">
        <param name="control_topic" value="$(arg control_topic)" />
        <param name="init_pose_topic" value="$(arg init_pose_topic)" />
        <param name="plan_file" value="$(arg plan_file)" />
    </node>
</launch>
{{< / highlight >}}

The launch tags:
```xml
<launch> ... </launch>
```
are required preable to wrap any content in your launch files.

The `<arg ...>` tags allow you to pass arguments in from the command line (or from other launch files). The `default` attribute specifies a default if no argument is passed in. To change an argument at runtime from the command line use the follwing syntax:
```bash
$ roslaunch <package> <launch file> plan_file:='/path/to/plan.txt'
```
It is good practice to use argument for state that can be set at runtime so users can choose values that make sense for their applications. It is also good practice to provide sensible defaults. If there is no sensible default, you should omit the default attribute. This will require the user to specify an argument at runtime.

In the third argument (line 4), the default uses a `roslaunch` command to locate a ROS package programatically. In order to keep your code portable, whenever you want to use files located in ROS packages, you can use the `$(find ...)` command to get the location of a ROS package. Then, no matter where the catkin workspace is located, `roslaunch` will populate the argument with the correct path.

The `<node ...>` tags denotes a single ROS node to be launched. ROS nodes are individual processes that run on a host. There are three attributes:

1. `pkg="mushr_ros_intro"`: The package to find the executable for the node.
+  `type="path_publisher.py"`: The entry file for the node. For python, you have to provide the name of the file in the `src` directory.
+  `name="path_publisher"`: The name of the node. This will be used for other nodes to reference your node. For now we will just use the same name as the executable, as this is a uniquely identifying name.

Line 7-9 define parameters for the node. Parameters are accessed programatically by the node (think `rospy.get_param(...)`). This is different than an argument, which is only used by `roslaunch` to pass values into the launch file. It is often convenient to use the same names for arguments and parameters, although this is entirely up to you. To define the parameters using arguments passed into the launch file, we use another `roslaunch` macro `$(arg <arg name>)`.

## Putting it all together

With python files, we have to change the execution permissions so `roslaunch` can file the correct files to run.
```bash
$ chmod a+x src/path_publisher.py
```

"`chmod`" stands for change mode, "`a+x`" says, "let all users execute this program". **If you don't do this**, you will likely see the following message:

```txt
ERROR: cannot launch node of type [mushr_ros_intro/path_publisher.py]: can't locate node [path_publisher.py] in package [mushr_ros_intro]
```
So make sure to make your files executable!

Once this is done, all that's left to do is launch the file and the simulator. In one terminal run the simulator:
```bash
$ roslaunch mushr_sim teleop.launch
```

In another, start `rviz` (a ROS tool that allows you to visualize simulated environments):
```bash
$ rosrun rviz rviz -d $MUSHR/mush_utils/rviz/mushr_ros_intro.rviz
```
This will launch rviz with a configuration that has all the right topics visualized.

Now, finally, in another terminal, run the path publisher we created:
```bash
$ roslaunch mushr_ros_intro path_publisher.launch
```

This will start the path publisher immediately, so make sure you are watching the `rviz` screen.

## Wrap-up
This concluded the introductory tutorial. This tutorial was meant to get you hands on experience with both ROS and the MuSHR environment. This means many of the topics were glossed over in order to make the tutorial managable. If you are interested in diving deeper, have a look at the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials), and then try the challenge problems below.


## Challenges
Below are a few challenges to get you more familiar with the tutorial, MuSHR, and ROS in general:

1. Create new paths! You can can add new paths to `$(find mushr_ros_intro)/paths/`, and pass the path into the launch file as an argument. The console window you opened `rviz` in will show "clicked points", which you can use to find `x`, `y`, and `theta` values for the inital pose.
+  Adapt `path_publisher.py` to take a "duration" parameter as the third comma separated value. This will allow a much richer set of path specifications.
+  We we launched the simulator with a sandbox square map. Use another map in the map directory, or a map you create (more on this in a different tutorial).
+  Update the argument to take a file name instead of a path, this way we won't have to specifiy the entire path (as long as the file exists in `$(find mushr_ros_intro)/paths`.
+  When the plan is done being executed, restart from the the begining until the node is killed.
+  Try and get this to run on the car. How does hard-coding in predefined velocities and steering angles work in the real world?