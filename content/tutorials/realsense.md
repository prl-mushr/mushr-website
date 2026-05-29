---
title: "Using the RealSense Camera"
date: 2018-11-28T15:14:54+10:00
featured: false
draft: false
active: true
duration: 20 
difficulty: Beginner 
summary: Setup and launch the camera
weight: 2
---

<h2> By: Akkshaj Singh, Ramon Qu, Rosario Scalise</h2>

<br>
{{< figure src="/tutorials/realsense/cameras.jpg" width="1000" >}}
<br>

### Introduction
This tutorial will get your car's camera operational and teach you to modify your camera data.

### Goal 
To get you viewing your car's live camera feed.

### Requirements
  - Complete the [hardware](/hardware/build_instructions) setup with your car
  - Complete the [quickstart](/tutorials/humble_quickstart) tutorial. (Required for the MuSHR stack)
  - Complete the [first_steps](/tutorials/first_steps/) tutorial.
  - A desktop/laptop computer that can ssh into the car.
  - An SSH-capable text editor, like Vim or Visual Studio Code (requires the SSH plugin)

## SSH into the Car
{{< highlight bash >}}
$ ssh <user>@<car ip> 
{{< / highlight >}}

Power on the Jetson, and SSH into the car, like in the [first_steps](/tutorials/first_steps/) tutorial. Do this in three separate terminal windows. If you're familiar with tools like tmux or GNU Screen, feel free to use those, but they are outside the scope of the tutorial.

## ROS Setup
Now that you're connected to the car, source the workspace.

{{< highlight bash >}}
$ source ~/colcon_ws/install/setup.bash
{{< / highlight >}}

Normally, we would launch tele-op here in order to be able to drive the car around, but let's try launching just the camera.

## Launching the Intel RealSense Camera
Ensure your RealSense camera is connected to the Jetson's USB port. You can verify this with the `lsusb` command. Then launch the camera driver:

{{< highlight bash >}}
$ ros2 launch realsense2_camera rs_launch.py
{{< / highlight >}}

If the camera fails to start, try reconnecting it to the Jetson and running the command again.

{{< highlight bash >}}
$ ros2 topic list
{{< / highlight >}}

Run the above command, and you should be able to see the topics for the camera, like this:

{{< highlight bash >}}
/camera/color/camera_info
/camera/color/image_raw
{{< / highlight >}}

## Launching RViz2 on your computer
Note: the following commands in this section should be run in a terminal window connected to your local device, NOT the SSH window to the car.

Set `ROS_DOMAIN_ID` on both car and laptop to a shared, unique value. The car and your laptop find each other over DDS if they share the same `ROS_DOMAIN_ID`.

{{< highlight bash >}}
$ export ROS_DOMAIN_ID=0
{{< / highlight >}}

Now, launch RViz2.

{{< highlight bash >}}
$ rviz2
{{< / highlight >}}

If you get errors make sure the following are correct:

- Teleop is running
- Your laptop is connnected properly
{{< highlight bash >}}
$ ros2 topic list
{{< / highlight >}}
This should output a bunch of camera-related topics. If not, check that `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` matches on both machines and that they are on the same network.

{{< highlight bash >}}
/camera/color/camera_info
/camera/color/image_raw
{{< / highlight >}}


## Viewing Camera Output in RViz2

{{< figure src="/tutorials/realsense/rvizadd.png" caption="">}}

In your RViz2 window, you can add topics to view various camera feeds from the RealSense. It is able to publish RGB, Depth, and Infrared camera data. Clicking add will allow you to view those feeds, like so:
{{< figure src="/tutorials/realsense/rvizcamtopics.png" caption="Adding the Color Input from the RealSense. Other inputs are also visible.">}}
After selecting the topic, you should be able to see the camera feed in RViz2.

## Changing Camera Settings

{{< figure src="/tutorials/realsense/camparams.png" caption="">}}

It is also possible to change camera settings and parameters, like resolution, framerate, image compression, etc. You can do this by editing the launch arguments in `rs_launch.py`, or pass them on the command line, for example:

{{< highlight bash >}}
$ ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
{{< / highlight >}}

Run `ros2 launch realsense2_camera rs_launch.py --show-args` to see every available parameter.
