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
```
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
0.0,-0.09
```
## Rostopic of interest

(Note: This section is only for better understanding, feel free to skip it.)

Before jumping into the code lets find out which rostopics we need to publish.

{{< highlight bash >}}
  $ roslaunch mushr_mujoco_ros two_cars.launch
{{< / highlight >}}

Open a new terminal window, and type the folloing.

{{< highlight bash >}}
 $ rostopic list -v 
{{< / highlight >}}

Locate the highlighted msg of interest as highlighted in the image below. This is the topic we will publish navigation commands to.

## Code

The entire code is written below. Feel free to try to code it by yourself, as this code is very similar to the MuSHR Introductory navigation (link: ), and in the earlier section we know our rostopic of interest. The code will be explained later on in chunks.

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
    rospy.init_node("path_publisher")
    control_topic = rospy.get_param("~control_topic", "/mushr_mujoco_ros/buddy/control")
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)
    init_pose_topic = rospy.get_param("~init_pose_topic", \
                                    "/mushr_mujoco_ros/buddy/initialpose")
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)
    plan_file = rospy.get_param("~plan_file")
    with open(plan_file) as f:
        plan = f.readlines()
    rospy.sleep(1.0)
    run_plan(pub_init_pose, pub_controls, plan)
{{< / highlight >}}

## The code explained

## Writing the launch file

## Executing the figure 8

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
