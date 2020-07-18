---
title: "Navigating in the Mujoco Simulator."
date: 2020-07-15T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
duration: (need to calculate)
difficulty: Beginner
summary: Execute a plan/trajectory in the MuJoCo simulator.
weight: (Needs to be changed)
---

{{< figure src="/tutorials/mujoco_figure8/figure8.gif" width="800" >}}
<br>

### Introduction
This tutorial will introduce you to controlling the mushr bot in the MuJoCo environment.

### Goal 
To command the MuSHR bot to execute a figure 8 plan in the MuJoCo simulator.

### Requirements
A Ubuntu Linux machine. If you don't run linux natively then get a Ubuntu VM: [OSX](https://www.instructables.com/id/How-to-Create-an-Ubuntu-Virtual-Machine-with-Virtu/), [Windows](https://itsfoss.com/install-linux-in-virtualbox/). 

We also provide a virtual machine image that already has the MuSHR stack setup, it can be downloaded [here](https://drive.google.com/a/cs.washington.edu/file/d/1mOzSzVx9BF_e2U1OeK58NS42UIPcnIZq/view?usp=sharing). The username is **robot** and the password is **prl_robot**. If you use this image, you can start the VM and then skip to the [**Running the Simulator**](#running-the-simulator) section.

Window Subsystem for Linux (WSL): There has also been success getting the quickstart to run using WSL. When running `rivz` you'll need to disable native `opengl`. There will be a note ([**Note for WSL**](#wsl-users-note)) in the section running `rviz`.

## Setup

Make sure you have completed the MuSHR Quickstart tutorial (link: [MuSHR Setup](https://mushr.io/tutorials/intro-to-ros/)) and MuJoCo setup tutorial (link: Link from Caemen) before you proceed.

Let's first start by creating our "figure 8" plan text file.

```bash
$ cd ~/catkin_ws/src/mushr_mujoco_ros
$ mkdir plans
$ cd plans
```

Next is to create a file called figure8.txt and save the below coordinate commands in it.

```bash
$ nano figure8.txt
```

Paste below initial pose and drive commands in figure8.txt
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

Before jumping into the code lets find out which rostopics we need to publish to. In a new terminal window type, 

```bash
$ roslaunch mushr_mujoco_ros two_cars.launch
```

Open a new terminal window, and type the following.

```bash
$ rostopic list -v 
```

Locate the highlighted msg of interest as highlighted in the image below. This is the topic we will publish navigation commands to.

{{< figure src="/tutorials/mujoco_figure8/rostopic.png" width="800" >}}
<br>


## Code

The entire code is written below. Feel free to try to code it by yourself, as this code is very similar to the MuSHR Introductory navigation (link: [MuSHR Setup](https://mushr.io/tutorials/intro-to-ros/)), and from the earlier section we now know our rostopic of interest. The code will be explained later on.

```bash
$ cd ~/catkin_ws/src/mushr_mujoco_ros/src
$ nano path_publisher.py
```

Paste the below code in path_publisher.py.

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

The detailed explanations of the python functions are given in the introductory navigation tutorial (link: [MuSHR Setup](https://mushr.io/tutorials/intro-to-ros/)).

{{< highlight python "linenos=table" >}}
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

Line 2 initializes the ros node. Lines 4-5 initializes the publisher node to control the MuSHR bot. `rospy.get_param("~control_topic", "/mushr_mujoco_ros/buddy/control")` calls the rostopic we want to publish messages to (will be more clear while writing the launch file, notice this is the highlighted rostopic from before). Similarly lines 7-9 initializes the publisher node to set the initial position of the MuSHR bot. Lines 11-13 reads the figure8.txt file we just created.

## Writing the launch file

We need to create a launch file to launch our path_publisher.py code. Again the entire code is pasted below, feel free to write your own launch file, (Note: Make sure to implement relevant changes in path_publisher.py in case you decide to make changes).

```bash
$ cd ~/catkin_ws/src/mushr_mujoco_ros/launch
$ nano path_publisher.launch
```

Paste the below code in path_publisher.launch.

{{< highlight python "linenos=table" >}}
<launch>
    <arg name="control_topic" default="/mushr_mujoco_ros/buddy/control" />
    <arg name="init_pose_topic" default="/mushr_mujoco_ros/buddy/initialpose" />
    <arg name="plan_file" default="$(find mushr_mujoco_ros)/plans/figure8.txt" />

    <node pkg="mushr_mujoco_ros" type="path_publisher.py" name="path_publisher" output="screen">
        <param name="control_topic" value="$(arg control_topic)" />
        <param name="init_pose_topic" value="$(arg init_pose_topic)" />
        <param name="plan_file" value="$(arg plan_file)" />
    </node>
</launch>
{{< / highlight >}}

## The launch file explained

Lines 2-4 create arguments with default values (note: these values can be set externally while running the launch file). Lines 6-9, initialize a rosnode with parameters "control_topic", "init_pose_topic" and "plan_file", which if you recall are used in rospy.get_param() in path_publisher.py. We can set values to arguments (arg) while executing the launch file. If no value was assigned to the argument, the default is used. In case you decide to make modifications and add some more arguments, make sure you always assign default values to the arguments.

## Catkin_make

You are all set. Lets catkin_make our files to start executing the figure 8.

```bash
$ cd ~/catkin_ws/
$ catkin_make
```

## Executing the figure 8

Now time to execute our code. First lets launch the MuSHR cars. In a new terminal enter the below commands.

```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch mushr_mujoco_ros two_cars.launch
```

After this step, two cars spawned in the MuJoCo simulator as shown below.

{{< figure src="/tutorials/mujoco_figure8/Mujoco.png" caption="MuJoCo simulator with two MuSHR cars spawned" width="800" >}}
<br>
Next, in a new terminal enter the following roslaunch command.

```bash
$ roslaunch mushr_mujoco_ros path_publisher.launch
```

If instead, you want to manually add a plan.txt file, you can write it as

```bash
$ roslaunch mushr_mujoco_ros path_publisher.launch plan_file:='~/catkin_ws/src/mushr_mujoco_ros/plans/figure8.txt'
```

Since the default plan file is our plan file of interest, we dont need to add the path to the plan_file manually. Both methods should work.

Enjoy how one MuSHR bot makes an 8 without colliding into the other bot. Awesome, you have successfully completed this tutorial, feel free to play around and design your own plans and execute them.

-----------------------------------------------------------------------------------------------------------------
