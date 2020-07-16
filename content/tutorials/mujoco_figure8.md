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

{{<higlight bash>}}
  $ cd ~/catkin_ws/src/mushr_mujoco_ros/src
  $ nano path_publisher.py
{{</ highlight>}}

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

The detailed explanations of the python functions are given in the introductory navigation tutorial (link: ).

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

(Correctly mention the line numbers)
Line 51 initializes the ros node. Lines 53-54 initializes the publisher node to control the MuSHR bot. ``rospy.get_param("~control_topic", "/mushr_mujoco_ros/buddy/control")`` gets the rostopic we want to publish messages to (will be more clear while writing the launch file, notice this is the highlighted topic from before). Similarly lines 56-58 initializes the publisher node to set the initial position of the MuSHR bot. Lines 60-62 reads the figure8.txt file we just created.

## Writing the launch file

We need to create a launch file to launch our path_publisher.py code. Again the entire code is pasted below, feel free to write your own launch file, (Note: Make sure to implement relevant changes in path_publisher.py in case you decide to make changes).

{{< highlight bash>}}
  $ cd ~/catkin_ws/src/mushr_mujoco_ros/launch
  $ nano path_publisher.launch
{{< / highlight >}}

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

Lines 12-14 create arguments with default values (note: these values can be set externally while running the launch file). Lines 16-20, initialize a node with parameters "control_topic", "init_pose_topic" and "plan_file", which if you recall are used in rospy.get_param() in path_publisher.py. ``value=$(arg control_topic)``, sets value as the value assigned to control_topic. If no value was assigned to control_topic, the default is used. In case you decide to make modifications and add some parameters, make sure you always assign default values to the arguments. (Might be confusing, make it clearer.)

## Catkin_make

You are all set. Lets catkin_make our files to start executing the figure 8.

{{< highlight bash >}}
  $ cd ~/catkin_ws/
  $ catkin_make
{{< / highlight >}}

## Executing the figure 8

Now time to execute our code. First lets launch our file cars. In a new terminal enter the below commands

{{< highlight bash >}}
  $ source ~/catkin_ws/devel/setup.bash
  $ roslaunch mushr_mujoco_ros twa_cars.launch
{{< / highlight >}}

After this step, the MuJoCo simulator must launch and two cars standing next to each other should be visible. 

(Add image of the MuJoCo simulator)

Next, in a new terminal enter the follow commands.

{{< highlight bash >}}
  $ roslaunch mushr_mujoco_ros path_publisher.launch
{{< / highlight >}}

If instead, you want to manually add a plan.txt file, you can write it as below, since the default plan file is our plan file of interest, we dont require to add it manually (like below)

{{< highlight bash >}}
  $ roslaunch mushr_mujoco_ros path_publisher.launch plan_file:='~/catkin_ws/src/mushr_mujoco_ros/plans/figure8.txt'
{{< / highlight >}}

Enjoy how one MuSHR bot makes an 8 without colliding with the other MuSHR bot. Once you have reached here, feel free to play around and design your own plans and you can then execute them as shown above. 

-----------------------------------------------------------------------------------------------------------------

{{< figure src="/tutorials/quickstart/rviz_docker-update.png" caption="This is an image of the `rviz` window that should pop up." width="800">}}
