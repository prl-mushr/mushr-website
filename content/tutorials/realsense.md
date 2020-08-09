---
title: "Using the RealSense Camera"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: false
draft: false
active: true
duration: 20 
difficulty: Beginner 
summary: Setup and launch the camera
weight: 1
---

### Introduction
This tutorial will get your car's camera operational and teach you to modify your camera data.

### Goal 
To get you viewing your car's live camera feed.

### Requirements
  - Complete the [hardware](/hardware/build_instructions) setup with your car
  - Complete the [quickstart](/tutorials/quickstart) tutorial. (Required for rviz)
  - Complete the [first_steps](/tutorials/first_steps/) tutorial.
  - A desktop/laptop computer that can ssh into the car.
  - An SSH-capable text editor, like Vim or Visual Studio Code (requires the SSH plugin)

## SSH into the Car
{{< highlight bash >}}
$ ssh <user>@<car ip> 
{{< / highlight >}}

Power on the Jetson, and SSH into the car, like in the [first_steps](/tutorials/first_steps/) tutorial. Do this in three separate terminal windows. If you're familiar with tools like tmux or GNU Screen, feel free to use those, but they are outside the scope of the tutorial.

## ROS Setup
Now that you're connected to the car, let's launch ROS.

{{< highlight bash >}}
$ roscore 
{{< / highlight >}}

Normally, we would launch tele-op here in order to be able to drive the car around, but let's try launching just the camera.

## Launching the Intel RealSense Camera
First, navigate to the directory containing the <code> realsense2 <\code> package for ROS.

{{< highlight bash >}}
$ cd ~/catkin_ws/src/realsense2/realsense2_camera/launch/
{{< / highlight >}}

Here, you can now launch the camera by running the launchfile. Ensure your RealSense camera is connected to the Jetson's USB port. You can verify this with the <code> lsusb <\code> command.

{{< highlight bash >}}
$ roslaunch rs_camera.launch
{{< / highlight >}}

If you get an error along the lines of `failed to find nodelet to unload` try reconnecting the camera to the Jetson and running the command again.

{{< highlight bash >}}
$ rostopic list
{{< / highlight >}}

Run the above command, and you should be able to see the topics for the camera, like this:

{{< highlight bash >}}
/camera/color/camera_info
/camera/color/image_raw
{{< / highlight >}}

## Launching RViz on your computer
Set the `ROS_IP` to your IP. Your IP can be found in a variety of ways: [Linux](https://www.howtogeek.com/howto/17012/how-to-find-your-ip-address-in-ubuntu/), [Mac](http://osxdaily.com/2010/08/08/lan-ip-address-mac/), [Windows](https://kb.netgear.com/20878/Finding-your-IP-address-without-using-the-command-prompt).

Note: the following commands in this section should be run in a terminal window connected to your local device, NOT the SSH window to the car.

Set `ROS_IP` with:

{{< highlight bash >}}
$ export ROS_IP=YOUR-IP
{{< / highlight >}}

Set the `ROS_MASTER_URI` to the IP of the car. (You used this to SSH into it earlier.)

{{< highlight bash >}}
$ export ROS_MASTER_URI=http://CAR_IP_GOES_HERE:11311
{{< / highlight >}}

Now, launch RViz.

{{< highlight bash >}}
$ rviz
{{< / highlight >}}

If you get errors make sure the following are correct:  

- Teleop is running  
- Your laptop is connnected properly
{{< highlight bash >}}
$ rostopic list
{{< / highlight >}}
This should output a bunch of camera-related topics. If not, check your `ROS_MASTER_URI` and `ROS_IP` to ensure they are correct.

{{< highlight bash >}}
/camera/color/camera_info
/camera/color/image_raw
{{< / highlight >}}


## Viewing Camera Output in RViz

{{< figure src="/tutorials/realsense/rvizadd.png" caption="">}}

In your RViz window, you can add topics to view various camera feeds from the RealSense. It is able to publish RGB, Depth, and Infrared camera data. Clicking add will allow you to view those feeds, like so:
{{< figure src="/tutorials/realsense/rvizcamtopics.png" caption="Adding the Color Input from the RealSense. Other inputs are also visible.">}}
After selecting the topic, you should be able to see the camera feed in RViz.

## Changing Camera Settings

{{< figure src="/tutorials/realsense/camparams.png" caption="">}}

It is also possible to change camera settings and parameters, like resolution, framerate, image compression, etc. You can do this by editing the launchfile we used previously to launch the camera in the <code> ~/catkin_ws/src/realsense2/realsense2_camera/launch/ <\code> directory. Make sure that it is not running when you do this. Open it with your text editor of choice and edit the parameters in the .launch file.