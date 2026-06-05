---
title: "Workflow & Troubleshooting"
date: 2018-11-28T15:14:54+10:00
active: true
featured: false
draft: false
duration: 30
difficulty: Beginner 
summary: Techniques for devloping and troubleshooting with the cars.
weight: 2
---

<h2> By: <a href=https://www.mattschmittle.com/>Matt Schmittle</a></h2> 
{{< figure src="/tutorials/workflow/work.jpg" >}}  <br>

## Introduction

### Goal 
To learn common workflows with the software stack to speed up development on your own projects. It will also cover common problems and troubleshooting tips and tricks.

### Requirements
You can choose to use this tutorial as a reference or to follow along on your car to see for yourself how the various tools work.

### Table of Contents
This tutorial consists of four core parts:

1. [Common Workflows](#common-workflows) introduces you to effective strategies for setting up and using the car in simulation and the real world for the purposes of developing new algorithms and collecting data.
2. [Troubleshooting](#troubleshooting) helps speed up the diagnostic process for when things go wrong. We give you a series of questions to help narrow down your issue.
3. [Debugging Tools](#debugging-tools) provides a summarized reference for ROS diagnostic tools we can't do without. We share the commands we frequent and their functions.
4. [Common Issues & How to Fix](#common-issues-how-to-fix) contains issues you're sure to encounter and proposals for fixing them.

_See something that's missing or have a suggestion for improving this guide? [Contact us](/contact/) with your suggestion!_

## Common Workflows
There are two common workflows, in sim and in the real world. Sim is easier because you do not need to connect and communicate with the car over the WiFi. We recommend getting comfortable working in sim before trying to test on the car.

### Simulation Workflow
To work in sim you need to have either have the docker image working or an Ubuntu machine with ROS 2 Humble that has `mushr_sim`, `mushr`, `mushr_base`, `vesc` and `rviz2` installed. The Docker container is straight-forward to set up, but if you don't like working in a container we recommend the latter. Each has a slightly different workflow covered below.

##### Docker Container
If you set up the docker container through the [quickstart](/tutorials/humble_quickstart/) tutorial, enter it with:
{{< highlight bash >}}
$ mushr_humble
{{< / highlight >}}

The launcher creates a persistent container the first time and attaches to it on every subsequent run, so any number of terminals can share one ROS 2 environment. Inside, build and source your workspace:

{{< highlight bash >}}
$ cd ~/colcon_ws && colcon build --symlink-install && source install/setup.bash
{{< / highlight >}}

We recommend storing your code on a git repository so it is easier to move it from container to container. Because the container persists between runs, packages you install with apt are kept. If you want a clean slate, run `mushr_humble rm` and the next `mushr_humble` will create a fresh container.

Docker has a bunch of resources for going further with containers and images. Checkout their [docs](https://docs.docker.com/) for further resources!


##### Regular Install 

Use these instructions to operate the simulator on an Ubuntu system where ROS 2 Humble and `mushr_sim`, `mushr`, `mushr_base`, `vesc` and `rviz2` are installed.

Source your workspace, then start rviz2:

{{< highlight bash >}}
$ source ~/colcon_ws/install/setup.bash
$ rviz2
{{< / highlight >}}

Ideally you want to open your previously saved `.rviz` file (file &rarr; save as) that has your most commonly used topics all set up. But if not then wait until everything is running, subscribe, then save a file. Also, because you are working with a 2D map, make sure the camera is set to TopDownOrtho.

Launch the sim and the map server:

{{< highlight bash >}}
$ ros2 launch mushr_sim teleop.launch.py
{{< / highlight >}}

Launch your code:

{{< highlight bash >}}
$ ros2 launch your_package your_launchfile.launch.py
{{< / highlight >}}

Subscribe to all the necessary topics (and save a .rviz file!). All topics can be found by clicking **Add &rarr; By topic**. To get the robot model: **Add &rarr; By display type &rarr; RobotModel**. Use the gray box and the W, A, S, D keys to drive

If you need to restart or edit your code then make sure to reset the rviz topics so you get the most up to date data. This can be done by pressing the `reset` button in the bottom left corner. When finished with your session, press `q` in the gray box. 

### Real World Workflow
Operating the robot in the real world has a similar process to that of simulation, but with added hardware setup and connecting remotely. The key to a solid workflow is to make sure to separate hardware and software failures clearly which is something that we will discuss more in the next section.

The first step is to make sure both batteries have sufficient charge (not dead at least). When the batteries are charged the VESC will blink 3 times when connected and powered. The below image show batteries in a charged state. When charging there should be a solid red light (hold start/stop to toggle). The solid green light indicates full. 

{{< figure src="/tutorials/workflow/battery.jpg" caption="Batteries in a charged state. A solid red light indicates charging; solid green light indicates full." width="600">}}

Once you have comfirmed that your batteries are charged, you have eliminated many of the most common issues. The next thing to do is to plug in the batteries starting with the VESC. If you want to start right into teleoperation then hold the front button until you see the lidar spin (approximately 1 minute). Once you have plugged both batteries in, you should see the vesc and the Jetson Nano light up. If the Nano does not light up (and the battery is charged) check to make sure the barrel connector is plugged in.

{{< figure src="/tutorials/workflow/nano_light.jpg" caption="Green light on nano indicates the computer is powered and on." width="600">}}

{{< figure src="/tutorials/workflow/vesc_light.jpg" caption="Blue light on vesc indicates vesc is on. The wheels will also straighten." width="600">}}

Now it is time to connect to the computer! If you have the default network setting where the robot makes its own network then connect to "Robot AP" and ssh into the car.

{{< highlight bash >}}
$ ssh robot@10.42.0.171
{{< / highlight >}}

If you have the robot setup connect to a local network (see [Robot Setup Tutorial](/tutorials/robot_setup)), then connect to the local network yourself and ssh but replace the IP with the robot's static IP that you set. 

If you are having trouble connecting see [Troubleshooting](#troubleshooting). For the car and your computer to see each other, they must be on the same network, share the same `ROS_DOMAIN_ID`, and share the same `RMW_IMPLEMENTATION` (defaults to FastDDS, CycloneDDS is recommended). Set the same id on both devices (any integer 0&ndash;101, default 0):

{{< highlight bash >}}
$ export ROS_DOMAIN_ID=0
{{< / highlight >}}

You can put this command at the bottom of your `~/.bashrc` (on both the car and your computer) so it runs every time you log in. You can check that it is set using `echo`:

{{< highlight bash >}}
$ echo $ROS_DOMAIN_ID
{{< / highlight >}}

Now we are ready to run stuff!

On the base:
{{< highlight bash >}}
$ rviz2
{{< / highlight >}}

rviz2 should be setup to listen to the same topics as the simulator 

On the car:

{{< highlight bash >}}
$ ros2 launch mushr_base teleop.launch.py
{{< / highlight >}}

The `teleop.launch.py` launch file activates the car's hardware, sensors and remote control. Make sure you can drive the car and steer. On the base, visualize topics in rviz2.

On the car:
{{< highlight bash >}}
$ ros2 launch your_package your_launch.launch.py
{{< / highlight >}}

It is good practice to make a ros package for your code that is separate from the mushr_base package. That way if you need to update mushr code, your code remains unaffected. This can be done by making a [separate ROS 2 package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).
 
## Troubleshooting
Troubleshooting is what 80% of a roboticist's time is spent on. If we know it is inevitable, we need to design systems and use tools to narrow down a diagnosis for the problem as fast as possible. Diagnosis can usually be the hardest part because it could be hardware or software or both. In addition, a robotic system is highly interconnected so a weird behavior in one component may only manifest itself in another component down the road. This section will cover the main debugging tools you should use on the car and some common problems and fixes.

### Debugging Questions
You should try answering the following questions in order to work towards a diagnosis.

#### Is the issue hardware or software?
Very important question, as your fix will change drastically. There are usually clues into this problem. Let's look at an example:

{{< highlight bash >}}
[FATAL] [1455208235.408745600]: Failed to connect to the VESC, SerialException Failed to open the serial port to the VESC. IO Exception (2): No such file or directory, in serial port impl, line 151. failed..
{{< / highlight >}}

Now this issue is hardware. There is a key clue here, the word "IO Exception." We know the VESC is not connected because we can't start a [serial connection](https://en.wikipedia.org/wiki/Serial_communication) with the VESC. Here is another example:

{{< highlight bash >}}
$ ros2 topic list
/parameter_events
/rosout
{{< / highlight >}}

Now this is also a connection issue, but this time it is software. We expected to see the car's topics (like `/car/scan`), but only the default topics appear. ROS 2 nodes find each other over DDS, so either the two machines are on different networks, or their `ROS_DOMAIN_ID` values don't match. We know the WiFi is functional because we can ssh into the car, so it is most likely a `ROS_DOMAIN_ID` mismatch!

#### What component is causing the issue?
So now that you have determined that the issue is in hardware vs. software, we need to narrow down the problem. Sometimes, in the above examples, it explicitly tells you, but we aren't always that lucky. Take the following example:

{{< highlight bash >}}
$ ros2 launch labx teleop.launch.py
file 'teleop.launch.py' was not found in the share directory of package 'labx'
{{< / highlight >}}

So this points to labx being the component (package in this case) that is not working. Turns out that is slightly a red herring. So this error means ROS cannot find the launchfile or the package. So it could be one of two things or both. `teleop.launch.py` is not installed in labx in which case labx was the culprit. And/or you haven't sourced your workspace since making this package so ROS does not have it in its package list. We can narrow this down by doing the following:

{{< highlight bash >}}
$ ros2 pkg prefix labx
/home/nvidia/colcon_ws/install/labx
{{< / highlight >}}

It found the package! Which means you don't have a `teleop.launch.py` file installed in your labx package. To fix this, you would need to go to the labx package.

{{< highlight bash >}}
$ cd ~/colcon_ws/src/labx/launch
{{< / highlight >}}

And see if you have mispelled `teleop.launch.py` (and that it is installed via `data_files` in `setup.py`). If there is no file matching `teleop.launch.py` then you would have to make one.


#### Is the error ROS related or pure code related?
A helpful thing to determine is if the problem has anything to do with ROS. If the error looks like a standard python/C++ error then great, you can rule out all ROS stuff. If not, then it could be either your ROS interface (publishers/subscribers) in your code, or your launchfiles, or your ROS environment. This question is relatively easy to answer if you answer the previous questions. But the one difficulty is that ROS will add a bunch of node failure gibberish even if your code logic is the problem. So just make sure to scroll the error to the very beginning to find the core issue.

## Debugging Tools
ROS provides a suite of tools to help debug issues. We'll cover each a bit and when to use.

#### ros2 topic
This tool is really useful for checking if topics are publishing, get a sense of latency, see what is being published, and more info about specific topics. 

| Command                      | Function                                                                        |
|------------------------------|---------------------------------------------------------------------------------|
| `ros2 topic list`            | allows you to see all the topics                                                |
| `ros2 topic echo topic_name` | allows you to see what is actually being published                              |
| `ros2 topic info topic_name` | lets you see the message type and other important info about a topic            |
| `ros2 topic hz topic_name`   | lets you see the publish rate of the topic. A quick way to detect a bottleneck. |

<br>
Give this a try as you're getting acquainted with your system! <br>

#### ros2 node
This tool works very similar to `ros2 topic` except on a node level. It is useful to see what nodes are publishing/subscribing to.  

| Command                    | Function                                                                |
|----------------------------|-------------------------------------------------------------------------|
| `ros2 node list`           | list all the ROS nodes                                                  |
| `ros2 node info node_name` | see what the node is publishing/subscribing to and other important info |
<br>
  
#### ros2 param
If you have params that are set dynamically (erpm gain) then this a good tool to make sure a param is what you expect it to be and if not change it.  

| Command                                   | Function                  |
|-------------------------------------------|---------------------------|
| `ros2 param list`                         | list all params (by node) |
| `ros2 param get node_name param_name`     | get param value           |
| `ros2 param set node_name param_name val` | set param value           |
<br>

#### ros2 pkg
This tool is useful if you need to find a package.  

| Command                       | Function                                                       |
|-------------------------------|----------------------------------------------------------------|
| `ros2 pkg list`               | list all packages                                              |
| `ros2 pkg prefix package_name`| gives you the install location of the specified package        |
<br>

#### tf2
If you are having transform issues, these tools are a good way to debug.  

| Command                                       | Function                                            |
|-----------------------------------------------|-----------------------------------------------------|
| `ros2 run tf2_ros tf2_echo frame1 frame2`     | will output the transform from frame1 to frame2     |
| `ros2 run tf2_tools view_frames`              | will create a pdf diagram of the transforms present |
| `ros2 run tf2_ros tf2_monitor`                | show all frames and publish rates                   |
<br>

#### rqt_graph
This will give you a sense of the overall system of nodes and topics connecting them.  

| Command                       | Function                                |
|-------------------------------|-----------------------------------------|
| `ros2 run rqt_graph rqt_graph`| shows a live graph of nodes and topics  |
<br>

#### ssh
While this is not really a debugging tool, it is a tool commonly used and should be touched on. ssh stands for Secure Shell. It is an encrypted network protocol that amongst other things gives you a shell session on a remote machine. It is super useful for remote work and connecting to robots because then you do not need an additional monitor and keyboard. So when you connect, you supply the username (robot) and the IP address of the car and it will connect you to that user specifically. There are some other potentially useful things you can do with ssh in addition to a standard shell.  
  
{{< highlight bash >}}
$ ssh robot@10.42.0.171 -X
{{< / highlight >}}

This connects an X session so you can run graphical applications remotely, but it will be very slow because everything needs to be encrypted. 


{{< highlight bash >}}
$ sftp nvidia@10.42.0.171
{{< / highlight >}}

Secure file transfer protocol is the best way to get files from another machine. It allows you to `cd` and `ls` like a shell but also `get` files and `put` files from your local machine on the remote machine.  

#### tmux 
The terminal multiplexer is a famous and commonly used tool for working with multiple shell sessions from one window. For us, this is particularly useful when you ssh into the car. You don't have to keep sshing in for each new window, but instead ssh in once then use tmux to have multiple sessions. This [tutorial](https://hackernoon.com/a-gentle-introduction-to-tmux-8d784c404340) goes into more detail.

## Common Issues & How to Fix
Alright, now that we know how to narrow down issues, let's look at the most common issues on the cars and how to fix them.

#### Vesc Failure

{{< highlight bash >}}
[FATAL] [1455208235.408745600]: Failed to connect to the VESC, SerialException Failed to open the serial port to the VESC. IO Exception (2): No such file or directory, in serial port impl, line 151. failed..
{{< / highlight >}}

**Hardware/Software:** Hardware  
**Component:** Vesc  
**ROS Related:** No, but it manifests itself through ROS  
**Fix:** Make sure the Vesc battery is charged and plugged in. The Vesc should have a blue light. If you have to change the battery, make sure to re-source your workspace so the computer can detect the vesc. If continues, check connections to the computer.  

#### ROS Workspace Not Setup (bash not recognizing ROS commands): 
```
-bash: ros2: command not found
```
**Hardware/Software:** Software  
**Component:** ROS  
**ROS Related:** Yes  
**Fix:** `source /opt/ros/humble/setup.bash` then `source ~/colcon_ws/install/setup.bash`. We recommend putting these at the end of your `~/.bashrc` so you never experience this issue.  

#### Car Only Has Steering

**Hardware/Software:** Hardware  
**Component:** Vesc Battery  
**ROS Related:** No  
**Fix:** Charge the Vesc battery, it doesn't have enough juice to power the motor.  

#### Car Drifts When Driving Straight
This can also manifest when using a particle filter that works in sim but not on the car because the expected control is to go straight when commanded straight. It could also manifest in a controller, when the car cannot seem to follow a trajectory because what it is commanding to the car is not what the car is doing.  
**Hardware/Software:** Software  
**Component:** Vesc config  
**ROS Related:** No  
**Fix:** edit the `steering_angle_to_servo_offset` in  `~/colcon_ws/src/vesc/vesc_main/config/racecar-uw-nano/vesc.yaml` to a value that when the car is commanded straight it goes straight.  

#### ROS Topics Not Appearing in rviz2
**Hardware/Software:** Software  
**Component:** Your component/rviz2  
**ROS Related:** Yes  
**Fix:** Either your component is not publishing the topic (use `ros2 topic` to double check!) or rviz2 needs to be refreshed or the transform is not publishing (`ros2 run tf2_ros tf2_echo` to double check!). Click "Reset" in rviz2 and try restarting your node. If the error says no transform from x to map then you need to make sure the transform is being published (use `ros2 run tf2_ros tf2_echo`! See [Debugging Tools](#debugging-tools))  

#### ssh Not Connecting
**Hardware/Software:** Hardware  
**Component:** WiFi <br>
**ROS Related:** No  
**Fix:** If ssh is not working first do a dumby check to make sure your IP and username are correct. Then try to `ping` the car IP. If it does not respond, then make sure the car is powered. If powered and still not pinging, then it likely is struggling to pickup wifi. Plug a HDMI cable into the jetson and use `ifconfig` to confirm wifi. If nothing still, then use the graphical interface to try to connect to wifi. Also, make sure your base computer is on the same network as the car.  

#### Nodes on Two Machines Can't See Each Other

{{< highlight bash >}}
$ ros2 topic list   # missing the topics you expect from the other machine
{{< / highlight >}}

**Hardware/Software:** Software  
**Component:** ROS / network  
**ROS Related:** Yes  
**Fix:** ROS 2 discovery needs both machines on the same network with a matching `ROS_DOMAIN_ID`. Confirm `echo $ROS_DOMAIN_ID` matches on both, and that they can `ping` each other.

[^1]: `CONTAINER-ID` is just a stand in for the value provided by the command `docker ps`.
