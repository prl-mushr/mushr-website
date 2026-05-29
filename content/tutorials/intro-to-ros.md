---
title: "Intro to ROS"
date: 2018-11-28T15:14:54+10:00
featured: true
draft: false
active: true
difficulty: Beginner
duration: 60
summary: Fundamental Robot Operating System (ROS) concepts using MuSHR.
weight: 2
---

<h2> By: <a href=https://github.com/Rockett8855>Matthew Rockett</a></h2>                              
<br>
{{< figure src="/tutorials/intro-to-ros/ros_logo.png" width="1000" >}}
<br>

### Introduction

The [Robot Operating System (ROS)](https://en.wikipedia.org/wiki/Robot_Operating_System) is a robotics middleware framework (not actually an operating system) that is commonly used across robotics platforms. MuSHR uses ROS for several utilities, the most significant of which is communication between "nodes" which each contain seperate logical roles. To learn more about these nodes, see the [MuSHR System Overview](/tutorials/overview). Gaining a grasp of fundamental ROS concepts will help you make the most of the MuSHR system, and the [many other robots](https://robots.ros.org/) which use ROS. 

### Goal
This tutorial will help you get familiar with ROS concepts in reference to the MuSHR software stack. Upon completion, you will be familiar with ROS 2 [Publishers and Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

### Prerequisites
In order to successfully complete this tutorial you will need: 

1. to have completed the [quickstart](/tutorials/humble_quickstart) tutorial.
+  *(should)* have familiarity with `bash` and `python`.


### Notes
This tutorial assumes your colcon workspace is located at `~/colcon_ws`. If you followed the [quickstart](/tutorials/humble_quickstart) tutorial, this will be the case. If your workspace is in a different directory, adjust the provided command accordingly.

## Creating a package

First start by creating an `ament_python` package for our code:

```bash
$ cd ~/colcon_ws/src
$ ros2 pkg create --build-type ament_python mushr_ros_intro --dependencies rclpy std_msgs ackermann_msgs geometry_msgs tf_transformations
```

Our package depends on `rclpy`, `std_msgs`, `ackermann_msgs`, and `geometry_msgs`, plus `tf_transformations` for the quaternion helper. We include `ackermann_msgs` because it is the way we send velocities and steering angles to the car (more on this later in the tutorial). (If you plan to use C++, create the package with `--build-type ament_cmake` and depend on `rclcpp` instead of `rclpy`.)

Now we will build our empty package:

```bash
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select mushr_ros_intro
$ source install/setup.bash
$ cd src/mushr_ros_intro
```

Sourcing[^1] `install/setup.bash` sets up the ROS environment and some useful auto-complete rules, easing the traversal of multiple ROS packages. *Sometimes, when you encounter packages not being found, all you need is to rerun the source command.*

## A simple plan specification

Now that we have a package we want to create a source code file to run our ROS node[^3]. We will be creating a simple ROS node to read commands (in this case, velocity and steering angle) from a file line by line, and send them to the simulator to be applied to the simulated car. Each line will denote a command to be applied for one second. The first line is a message to send as the "starting pose" of the car. 

The input files will be of the form:

```
0,0,0.0
2.0,0.09
3.0,-0.15
```

The first line is the initial position, of the form `x, y, theta`, where `x` and `y` are the starting coordinates in the map, and `theta` is the initial angle of the car. The following two lines describe commands that tell the car how fast to go, and at what steering angle. The first says to run at `2.0 meters per second`, with a steering angle of `0.09 radians`. The second, to run at `3.0 meters per second`, with a steering angle of `-0.15 radians`. We will be applying each command for 1 second. Note that a positive steering angle corresponds to a left turn and a negative steering angle corresponds to a right turn.

Create the directory `plans` in the package:
```bash
$ cd ~/colcon_ws/src/mushr_ros_intro # if you aren't in the intro package directory
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

Try and figure out what these plans will do (hint: look at the file names :)). We encourage the extra adventurous can create their own "plan" files.

## The code

**Spoiler alert!** Below the entire code is listed. Each section will be explained in greater detail below the listing. Save this file in `mushr_ros_intro/path_publisher.py` (the Python module directory `ros2 pkg create` made inside your package). We suggest reading through the code first to gain an understanding of how it works before moving on to the explanations.


If you're curious to attempt solving the task on your own, give it a try (the section "[Writing the launch file](#writing-the-launch-file)" will help you launch your code) and return to this section afterwards.

{{< highlight python "linenos=table" >}}
import time

import rclpy
from rclpy.duration import Duration
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf_transformations import quaternion_from_euler


def run_plan(node, pub_init_pose, pub_controls, plan):
    init = plan.pop(0)
    send_init_pose(pub_init_pose, init)

    for c in plan:
        send_command(node, pub_controls, c)


def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
    q = Quaternion(x=qx, y=qy, z=qz, w=qw)
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))


def send_command(node, pub_controls, c):
    cmd = c.split(",")
    assert len(cmd) == 2
    v, delta = float(cmd[0]), float(cmd[1])

    dur = Duration(seconds=1.0)
    start = node.get_clock().now()

    drive = AckermannDrive(steering_angle=delta, speed=v)

    while node.get_clock().now() - start < dur:
        pub_controls.publish(AckermannDriveStamped(drive=drive))
        time.sleep(0.1)


def main():
    rclpy.init()
    node = rclpy.create_node("path_publisher")

    control_topic = node.declare_parameter(
        "control_topic", "/car/mux/ackermann_cmd_mux/input/navigation"
    ).value
    pub_controls = node.create_publisher(AckermannDriveStamped, control_topic, 1)

    init_pose_topic = node.declare_parameter("init_pose_topic", "/initialpose").value
    pub_init_pose = node.create_publisher(PoseWithCovarianceStamped, init_pose_topic, 1)

    plan_file = node.declare_parameter("plan_file", "").value

    with open(plan_file) as f:
        plan = f.readlines()

    # Publishers sometimes need a warm-up time, you can also wait until there
    # are subscribers to start publishing see publisher documentation.
    time.sleep(1.0)
    run_plan(node, pub_init_pose, pub_controls, plan)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
{{< / highlight >}}

## The code, explained

Below we will break down the code. The code blocks are explained out of order in the large code block to explain the concepts in a more logical way.

### Includes

We import functions and modules we need first:

{{< highlight python "linenos=table" >}}
import time

import rclpy
from rclpy.duration import Duration
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf_transformations import quaternion_from_euler

{{< / highlight >}}

`rclpy` is the main python interface to the ROS 2 API. [Ackermann steering](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) is the geometry of our car chassis. For this reason, the MuSHR system uses [`ackermann_msgs`](https://index.ros.org/p/ackermann_msgs/) to define a common interface for sending drive commands. The imports from [`geometry_msgs`](https://docs.ros.org/en/humble/p/geometry_msgs/) are for sending the initial pose of the car to the simulator.


### Entrypoint

The `main` function is where the thread of execution starts; the guard `if __name__ == "__main__"` calls it when the file is run directly. (When launched with `ros2 run`/`ros2 launch`, the `main` entry point we register in `setup.py` is called instead.)

{{< highlight python "linenos=table,linenostart=51" >}}
def main():
    rclpy.init()
    node = rclpy.create_node("path_publisher")

    control_topic = node.declare_parameter(
        "control_topic", "/car/mux/ackermann_cmd_mux/input/navigation"
    ).value
    pub_controls = node.create_publisher(AckermannDriveStamped, control_topic, 1)

    init_pose_topic = node.declare_parameter("init_pose_topic", "/initialpose").value
    pub_init_pose = node.create_publisher(PoseWithCovarianceStamped, init_pose_topic, 1)

    plan_file = node.declare_parameter("plan_file", "").value

    with open(plan_file) as f:
        plan = f.readlines()

    # Publishers sometimes need a warm-up time, you can also wait until there
    # are subscribers to start publishing see publisher documentation.
    time.sleep(1.0)
    run_plan(node, pub_init_pose, pub_controls, plan)

    node.destroy_node()
    rclpy.shutdown()
{{< / highlight >}}

`rclpy.init()` starts up the ROS 2 client library, and `rclpy.create_node(...)` creates our node. Every program must do this to register itself into the ROS environment. You must pass a name for the node (a good practice is to use the same name as the script, but it's up to you). The next lines set up communication streams for this node to talk to others. `node.declare_parameter(...)` declares a [parameter](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) we define at runtime (more below in 'Writing the launch file') and returns it; we read its `.value`. The first argument is the name of a parameter. The second is a default value. In ROS 2, every parameter must be declared before it is used.

In ROS 1, node-private parameters used a `~parameter_name` form. In ROS 2 parameters belong to the node directly, so we just use the plain name (e.g. `control_topic`). If you name them the same in the launch file, the values will line up (more on launch files in the next section).

[Publishers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) are part of the ROS message passing paradigm. `node.create_publisher(...)` takes three arguments. The first is the message type (note the order is different from ROS 1). The second is the topic name. There are a wide range of predefined messages you can use, and you also have the ability to [define custom ROS messages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) if needed (although that is not needed in this tutorial). The third is the queue size. This defines how many messages to buffer when waiting to send (or recieve) messages. In general, a large queue size will be useful when many messages will be sent. In our case, we are infrequently sending messages, so a queue size of one is okay. [^4]

### Functions

Below each function is briefly explained.

{{< highlight python "linenos=table,linenostart=16" >}}
def run_plan(node, pub_init_pose, pub_controls, plan):
    init = plan.pop(0)
    send_init_pose(pub_init_pose, init)

    for c in plan:
        send_command(node, pub_controls, c)
{{< / highlight >}}

`run_plan` is the main loop of the node. Once the initialization code is complete, the main function calls this. Given a list of strings (the commands), it calls relevant functions to run the car.

{{< highlight python "linenos=table,linenostart=24" >}}
def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
    q = Quaternion(x=qx, y=qy, z=qz, w=qw)
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))
{{< / highlight >}}

`send_init_pose` takes the string representation of the intial pose and converts it into an `x`, `y`, `theta` float representation. Instead of using Euler angles to describe orientations, many ROS packages use the more expressive [Quaternions](https://en.wikipedia.org/wiki/Quaternion). The `quaternion_from_euler` function provided by the `tf_transformations` package converts Euler angles to a quaternion. ROS 2 message constructors take keyword arguments, so we unpack the result into `x, y, z, w`.

`/initialpose` takes a `PoseWithCovarianceStamped` message. In ROS, `Stamped` messages are used when the ordering of data by time is important. Due to randomness in the network, messages can be passed out of order, and in time-critical systems, it's important for the most recently *sent* message to be used. For our setting, this is not as important, so we leave the header unset; our simulator does not require the timestamp. (If you did need it, you would set `msg.header.stamp = node.get_clock().now().to_msg()`.) Typically, the `PoseWithCovariance` message requires a `Pose` and `Covariance` matrix. However our simulator doesn't use the covariance matrix, so we don't need to provide it. The `Pose` is composed of an `x`, `y` point and an orientation (which must be in quaternion form).

After constructing the message, we publish it to the `pub_init_pose`, signalling to the simulator where to place the car.

{{< highlight python "linenos=table,linenostart=36" >}}
def send_command(node, pub_controls, c):
    cmd = c.split(",")
    assert len(cmd) == 2
    v, delta = float(cmd[0]), float(cmd[1])

    dur = Duration(seconds=1.0)
    start = node.get_clock().now()

    drive = AckermannDrive(steering_angle=delta, speed=v)

    while node.get_clock().now() - start < dur:
        pub_controls.publish(AckermannDriveStamped(drive=drive))
        time.sleep(0.1)
{{< / highlight >}}

`send_command` is very similar to `send_init_pose`, with a few differences. First, we define a one second `Duration`. ROS 2 durations are a convenient way to make durations that can be compared and combined using math operators. Subtracting the `start` time from the node clock's current time yields a comparable duration. Once a second has elapsed, the loop will terminate.

Inside the loop we sleep for 0.1 s between publishes, so the loop runs about 10 times a second. This limits how often the loop runs, conserving cycles when they are not needed. 10 Hertz is plenty fast for our application. This is necessary because if we only sent one command per second the car would "lurch" and then wait a second for the next command.


## Writing the launch file

In order to run our code in a convenient and extendable way, ROS 2 has the notion of [launch files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html). In ROS 2 these are Python files that describe how different software components should be started. They allow us to start a large number of ROS nodes with few commands.

### Registering the executable

Because this is an `ament_python` package, we first expose our script as an executable by adding a console-script entry point. Open `setup.py` and add to the `entry_points` dictionary:
{{< highlight python >}}
entry_points={
    'console_scripts': [
        'path_publisher = mushr_ros_intro.path_publisher:main',
    ],
},
{{< / highlight >}}

So the launch file can find your plan and launch files, also install those directories. Near the top of `setup.py` add `import os` and `from glob import glob`, then add to `data_files`:
{{< highlight python >}}
data_files=[
    # ... entries ros2 pkg create already added ...
    (os.path.join('share', 'mushr_ros_intro', 'plans'), glob('plans/*.txt')),
    (os.path.join('share', 'mushr_ros_intro', 'launch'), glob('launch/*.launch.py')),
],
{{< / highlight >}}

### The launch file

First we make a launch directory:
```bash
$ cd ~/colcon_ws/src/mushr_ros_intro # if you aren't in the directory
$ mkdir launch
```

Create the file `launch/path_publisher.launch.py`, containing:
{{< highlight python "linenos=table" >}}
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    plans_dir = os.path.join(
        get_package_share_directory("mushr_ros_intro"), "plans"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_topic",
            default_value="/car/mux/ackermann_cmd_mux/input/navigation",
        ),
        DeclareLaunchArgument("init_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument(
            "plan_file", default_value=os.path.join(plans_dir, "straight_line.txt")
        ),
        Node(
            package="mushr_ros_intro",
            executable="path_publisher",
            name="path_publisher",
            output="screen",
            parameters=[{
                "control_topic": LaunchConfiguration("control_topic"),
                "init_pose_topic": LaunchConfiguration("init_pose_topic"),
                "plan_file": LaunchConfiguration("plan_file"),
            }],
        ),
    ])
{{< / highlight >}}

`generate_launch_description()` is the function ROS 2 calls to build the launch description.

The `DeclareLaunchArgument(...)` actions allow you to pass arguments in from the command line (or from other launch files). The `default_value` specifies a default if no argument is passed in. To change an argument at runtime from the command line use the following syntax:
```bash
$ ros2 launch <package> <launch file> plan_file:='/path/to/plan.txt'
```
It is good practice to use arguments for state that can be set at runtime so users can choose values that make sense for their applications. It is also good practice to provide sensible defaults. If there is no sensible default, omit `default_value`; this will require the user to specify an argument at runtime.

The `plan_file` default uses `get_package_share_directory(...)` to locate the package's installed share directory programatically. In order to keep your code portable, whenever you want to use files located in ROS packages, use this helper to get the location instead of hard-coding a path. (This is why we installed `plans` into the package share above.)

The `Node(...)` action denotes a single ROS node to be launched. ROS nodes are individual processes that run on a host. The key fields are:

1. `package="mushr_ros_intro"`: The package to find the executable for the node.
+  `executable="path_publisher"`: The console-script entry point we registered in `setup.py` (not the file name).
+  `name="path_publisher"`: The name of the node. This will be used for other nodes to reference your node. For now we will just use the same name as the executable, as this is a uniquely identifying name.

The `parameters` field defines parameters for the node. Parameters are accessed programatically by the node (think `node.declare_parameter(...).value`). This is different than an argument, which is only used by `ros2 launch` to pass values into the launch file. It is often convenient to use the same names for arguments and parameters. To set a parameter from a launch argument, we use `LaunchConfiguration("<arg name>")`.

## Putting it all together

After editing `setup.py`, rebuild and re-source so the new executable and data files are installed:
```bash
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select mushr_ros_intro
$ source install/setup.bash
```

Once this is done, all that's left to do is launch the file and the simulator. In one terminal run the simulator:
```bash
$ ros2 launch mushr_sim teleop.launch.py foxglove_teleop:=1
```

In another, start `rviz2` (a ROS tool that allows you to visualize simulated environments):
```bash
$ ros2 run rviz2 rviz2 -d $HOME/colcon_ws/src/mushr/mushr_utils/rviz/default.rviz
```
This will launch rviz2 with a configuration that has all the right topics visualized.

Now, finally, in another terminal, run the path publisher we created:
```bash
$ ros2 launch mushr_ros_intro path_publisher.launch.py
```

This will start the path publisher immediately, so make sure you are watching the `rviz` screen. 

Once your node is running, try passing a different plan file in using the `plan_file` commandline argument.

## Wrap-up
This concludes the introductory tutorial. This tutorial was meant to get you hands on experience with both ROS and the MuSHR environment. This means many of the topics were glossed over in order to make the tutorial managable. If you are interested in diving deeper, have a look at the [ROS 2 tutorials](https://docs.ros.org/en/humble/Tutorials.html), and then try the challenge problems below.


## Challenges
Below are a few challenges to get you more familiar with the tutorial, MuSHR, and ROS in general:

1. Create new paths! You can add new plan files to your package's `plans/` directory (rebuild so they install), and pass the path into the launch file as an argument. The console window you opened `rviz2` in will show "clicked points", which you can use to find `x`, `y`, and `theta` values for the inital pose.
+  Adapt `path_publisher.py` to take a "duration" parameter as the third comma separated value. This will allow a much richer set of path specifications.
+  We launched the simulator with the "sandbox" square map. Use another map in the map directory, or a map you create (more on this in a different tutorial).
+  Update the argument to take a file name instead of a path, this way we won't have to specifiy the entire path (as long as the file exists in your package's `plans/` directory).
+  When the node finishes executing the plan, restart from the the begining until the node is killed.
+  Try and get this code to run on the actual car. How does hard-coding in predefined velocities and steering angles work in the real world?

[^1]: That is, calling [`source`](https://en.wikipedia.org/wiki/Source_(command)) on a file.

[^2]: In other words, you can press the `tab` key to auto-complete the package name.

[^3]: A node is an executable that uses ROS to communicate with other nodes. See [this article](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) for more details.

[^4]: In ROS 2 the queue size is part of the topic's Quality of Service (QoS) settings. See [About Quality of Service settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html).
