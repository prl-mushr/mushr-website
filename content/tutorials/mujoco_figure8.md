---
title: "Navigating in the MuJoCo Simulator"
date: 2020-07-15T15:14:54+10:00
image: "/services/default.png"
featured: false
draft: false
active: true
duration: 30
difficulty: Beginner
summary: Execute a plan/trajectory in the MuJoCo simulator.
weight: 5
---

<h2> By: <a href=https://github.com/Alrick11>Alrick Dsouza</a></h2>

{{< figure src="/tutorials/mujoco_figure8/side_shot.gif" width="800" caption="MuSHR car making a figure 8" >}}
<br>

### Introduction
This tutorial will introduce you to controlling the mushr car in the MuJoCo environment. Why MuJoCo??? MuJoCo is a physics simulator which provides a unique combination of speed, accuracy and modeling power for the purpose of model based optimization, and in particular optimization through contacts. MuJoCo is your goto simulator for RL and Deep RL projects. A well-known example would be DeepMind's humanoid simulations. You can find some very interesting OpenAI+MuJoCo projects [here](https://gym.openai.com/envs/#mujoco).

### Goal 
To command the MuSHR car to execute a figure 8 plan in the MuJoCo simulator.

### Requirements
* Complete the [Quickstart Tutorial](https://mushr.io/tutorials/quickstart/)  
* Complete the [MuJoco Simulation Tutorial](https://mushr.io/tutorials/mujoco/)  

## Setup

Let's first start by creating a new rospackage.

```bash
cd ~/catkin_ws/src
catkin_create_pkg mushr_mujoco_figure8 rospy geometry_msgs ackermann_msgs
```

Now let's make the directories we will be needing.

```bash
cd mushr_mujoco_figure8
mkdir launch plans
```

We will now create a text file containing a sequence of control commands designed to produce a trajectory of figure-8 shape when executed by the vehicle

```bash
$ cd plans
$ nano figure8.txt
```

Paste the following sequence of control commands into the figure8.txt file.  
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
$ roslaunch mushr_mujoco_ros one_car.launch
```

Open a new terminal window, and type the following.

```bash
$ rostopic list -v 
```

You will see a list of Published and Subscribed topics like below.

```bash
Published topics:
 * /map_metadata [nav_msgs/MapMetaData] 1 publisher
 * /mushr_mujoco_ros/buddy/velocimeter [geometry_msgs/Vector3Stamped] 1 publisher
 * /mushr_mujoco_ros/buddy/car_imu [sensor_msgs/Imu] 1 publisher
 * /rosout [rosgraph_msgs/Log] 3 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /mux/ackermann_cmd_mux/input/teleop [ackermann_msgs/AckermannDriveStamped] 1 publisher
 * /mushr_mujoco_ros/buddy/pose [geometry_msgs/PoseStamped] 1 publisher
 * /mushr_mujoco_ros/body_state [mushr_mujoco_ros/BodyStateArray] 1 publisher
 * /map [nav_msgs/OccupancyGrid] 1 publisher

Subscribed topics:
 * /mushr_mujoco_ros/buddy/initialpose [geometry_msgs/PoseWithCovarianceStamped] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /mushr_mujoco_ros/buddy/control [ackermann_msgs/AckermannDriveStamped] 1 subscriber
```

**/mushr_mujoco_ros/buddy/control** in Subscribed topics is our rostopic of interest.

## Code

The entire code is written below. Feel free to try to code it by yourself, as this code is very similar to the MuSHR Introductory navigation (link: [MuSHR Intro to ROS](https://mushr.io/tutorials/intro-to-ros/)), and from the earlier section we now know our rostopic of interest. The code will be explained later on.

```bash
$ cd ~/catkin_ws/src/mushr_mujoco_figure8/src
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

Make the python script executable.

```bash
chmod +x path_publisher.py
```

## The code explained

The detailed explanations of the python functions (run_plan, send_init_pose, send_command) are given in the introductory navigation tutorial (link: [MuSHR Intro to ROS](https://mushr.io/tutorials/intro-to-ros/)).

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

Line 2 initializes the rosnode. Lines 4-5 initializes the publisher node to control the MuSHR car. `rospy.get_param("~control_topic", "/mushr_mujoco_ros/buddy/control")` calls the rostopic we want to publish messages to (will be more clear while writing the launch file, notice this is the highlighted rostopic from before). Similarly lines 7-9 initializes the publisher node to set the initial position of the MuSHR bot. Lines 11-13 reads the figure8.txt file we just created.

## Writing the launch file

We need to create a launch file to launch our path_publisher.py code. Again the entire code is pasted below, feel free to write your own launch file (Note: Make sure to implement relevant changes in path_publisher.py in case you decide to make changes).

```bash
$ cd ~/catkin_ws/src/mushr_mujoco_figure8/launch
$ nano path_publisher.launch
```

Paste the below code in path_publisher.launch.

{{< highlight python "linenos=table" >}}
<launch>
    <arg name="control_topic" default="/mushr_mujoco_ros/buddy/control" />
    <arg name="init_pose_topic" default="/mushr_mujoco_ros/buddy/initialpose" />
    <arg name="plan_file" default="$(find mushr_mujoco_figure8)/plans/figure8.txt" />

    <node pkg="mushr_mujoco_figure8" type="path_publisher.py" name="path_publisher" output="screen">
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

Now it's time to execute our code. First let's launch the MuSHR car. In a new terminal, enter the below commands.

```bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch mushr_mujoco_ros one_car.launch
```

After this step, a MuSHR car is spawned in the MuJoCo simulator as shown below.

{{< figure src="/tutorials/mujoco_figure8/mujoco.png" caption="MuJoCo simulator a MuSHR car spawned" width="600" >}}
<br>
Next, in a new terminal enter the following roslaunch command.

```bash
$ roslaunch mushr_mujoco_figure8 path_publisher.launch
```

Since the default plan file is our plan file of interest, we don't need to add the path to figure8.txt manually. But, if you would like to call a different plan file you can write:

```bash
$ roslaunch mushr_mujoco_figure8 path_publisher.launch plan_file:='~/catkin_ws/src/mushr_mujoco_figure8/plans/{name of text file}'
```

Awesome, if your MuSHR car makes an 8, you have successfully completed this tutorial! Feel free to play around and design your own plans and execute them.

-----------------------------------------------------------------------------------------------------------------
