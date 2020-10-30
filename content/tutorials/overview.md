---
title: "MuSHR System Overview"
date: 2018-11-28T15:14:54+10:00
featured: true
draft: false
active: true
duration: 15
difficulty: Intermediate
summary: Basic components of the car and their associated software counterparts.
weight: 2
---

<h2> By: <a href=https://www.mattschmittle.com/>Matt Schmittle</a></h2>                              

### Introduction
This tutorial is largely informative and intended to read more like a manual. We will cover all the major software and hardware components of the car.

### Goal 
To be able to identify and understand the purpose of the major software and hardware components of the car.

### Requirements
A car that you can connect to if you wish to follow along.

## Hardware Overview
This section will cover an overview of the physical racecar Below is a diagram of the physical racecar components.

{{< figure src="/tutorials/overview/hardware_overview.png" caption="This diagram shows all the components and their data & power connections." width="600">}}

* __Chasis (Redcat Racing Blackout SC 1/10) :__ The car chasis. It has adjustable 4x4 suspension and non-flat tires. It is also used for mounting things to it, including the housing which contains the computer and sensors.
* __Computer (Jetson Nano):__ This the computer that runs the software on the car. You can connect to the computer in 3 ways primarily: ssh through local network to static IP, connecting to car's network and using ssh, or plugging a monitor, keyboard, and mouse into the computer directly. 
* __Motor (Jrelecs F540 3930KV):__  The single DC motor that powers all four wheels. The motor makes the car move and is controlled by the VESC.
* __Servo (ZOSKAY 1X DS3218):__ A servo is another type of motor but is better at going to specific angles along its rotation than rotating continuously. The servo's job is to steer the front wheels by taking in steering angle commands. The servo is also controlled by the VESC.
* __VESC (Turnigy SK8-ESC):__ The VESC is responsible for taking high level control commands like steering angle and velocity and converting that to power/angle commands for the motor/servo. The VESC has an associated ROS node. This node takes your ROS `ackermann_msgs/AckermannDriveStamped` message and converts that to `VescStateStamped` (power), and `Float64` (steering angle) messages. These messages are something the physical VESC can use to control the motor and servo.
* __NiMH Battery (Redcat Racing HX-5000MH-B):__ There are 2 batteries. One to power the motor and in turn the VESC (because the motor power flows through the VESC). And a second to power the computer and sensors. Many of the issues (particularly with the VESC) can stem from one of these batteries having a low battery, so it is good to check them regularly.
* __Power Converter (DZS Elec LM2596):__ This power converter converts the higher voltage of the battery to the necessary 5V max for the computer.
* __RGBD Camera (Realsense D435i):__ This Intel Realsense camera can publish both rgb and depth. It has a associated node so you can just subscribe to the topic to use the images! OpenCV even has a function for converting ROS `Image` messages to something OpenCV can use. Note depth cameras are different from stereo cameras (how humans do it). They both provide the same output, but depth cameras project a IR pattern and use a IR camera to see the deformation of that pattern to compute depth. Depth cameras will also sometimes use stereo in addition to make their measurements more accurate. It publishes to the `/camera` topics. This camera can also publish IMU measurments, but they currently are not being used.
* __Laser Scanner (YDLIDAR X4):__ This 360 degree sensor works by having a 1D laser range finder spin around. With each distance reading their is a associated angle the range finder was at. It publishes an array of distance and angle measurements to the `/scan` topic.
* __Wireless Controller (Logitech F710):__ This provides controls to the cars! It doesn't just have to be use for teleop it can be used by your programs for most anything. It publishes on the `/joy` topic.
* __Bumber Switch (Vex Bumper Switch):__ This is a button on the front of the car that you can use to indicate a collision. It publishes a binary signal to **/push_button_state**.
* __Micro SD Card:__ Storage for the OS and any logs. What's great about this is if you want to switch cars you can simply switch SD cards.


## Software Components
 Now that we have a better understanding of the hardware components let's checkout the software components and see how it all ties together.
On the car go to your catkin workspace  
{{< highlight bash >}}
 cd ~/catkin_ws/
{{< / highlight >}}

 and list the directories

{{< highlight bash >}}
$ ls
build  devel  src
{{< / highlight >}}

So what we have is 3 directories. `build` is where code is compiled to. `devel` has setup files for your environment. The most important thing about `devel` is that it contains `setup.**` which sets up environment variables and paths amongst other things. 
 {{< highlight bash >}}
robot@digger:~/catkin_ws/devel$ ls
bin  env.sh  include  lib  setup.bash  setup.sh  _setup_util.py  setup.zsh  share
 {{< / highlight >}}

 So `setup.**` ("`**`" because you can run .bashrc, .sh, or .zsh and get the same effect) sets up ROS environment variables and paths. So if you ever see an error that says that it can't find a package, or that roscore isn't a command, it is because you have not sourced your `setup.**` like so
 `source ~/catkin_ws/devel/setup.bash`
We usually prevent this by putting the above in your `~/.bashrc` a file that is run everytime you log in. 
So now if you change directories to `~/catkin_ws/src/` you will see all the ros packages. When you want to make new ROS software you make a package and you put it here. Why? Because when you run `catkin_make` it looks in this directory for the packages it needs to build. `catkin_make` needs to be run in the `~/catkin_ws/` directory and should be run after every update in code. Although, since python is an interpreted language, we do not have to run `catkin_make` everytime we edit code, but in C++ we would.

Alright, now that we have a high level view of our workspace let's checkout the system components. Each package (located in `src`) creates ROS nodes. This tutorial will not dive into the details of how ROS works but this [post](https://robohub.org/ros-101-intro-to-the-robot-operating-system/) gives a good overview with diagrams. Below is a simplified diagram of the system at a high level:

{{< figure src="/tutorials/overview/software_overview.png" caption="The full MuSHR stack with each ROS node as a box and each line as a ROS topic." width="980">}}

Let's unpack this, starting from the top. The sensor nodes take the raw input, do some processing, and  convert it to ros friendly topics. The map server converts a `.yaml` map file to a ros topic. Your controller (orange) takes in sensor topics coming in from the each sensors' ros node and the map. It then outputs some command to drive the car. That command is put into the mux which listens on multiple `/mux/ackermann_cmd_mux/input` channels and selects the highest priority. `*/input/default` is a zero throttle/steering command that is passed whenever your controller and teleop are not publishing. 
Currently, MuSHR does not have a explicit safety controller publishing to the `*/input/safety` topic. The mux priorities can be found `mushr_base/ackermann_cmd_mux/param/mux.yaml` and they are listed in order of priority below.  

	1. Safety  
	2. Teleop  
	3. Navigation  
	4. Default  

Once the highest priority command is output it goes to the VESC. The VESC smoothes the command by clipping the min/max of the steering/throttle so we don't try to turn the wheels 180 degrees for example. It then provides that to the vesc driver which directly controls the motors.

These components are in the following locations all within `~/catkin_ws/src/mushr/` if you want to check them out for more details:
  
 - MUX:  `mushr_base/ackermann_cmd_mux`  
 - VESC: `mushr_base/vesc`  
	- `vesc` describes the metapackage  
	- `vesc_main` contains configuration files and launch files for running the vesc  
	- `vesc_ackermann` contains vesc odom info (not depicted in diagram)  
	- `vesc_driver` is the last piece of the diagram connecting the physical vesc with the computer. It also contains the throttle interpolator.  
	- `vesc_msgs` describes the VescState message  
- Teleop: `mushr_base/mushr_base/src/joy_teleop.py`  
- Lidar: `mushr_hardware/ydlidar`  
- Button: `mushr_hardware/push_button_utils`  
- Camera: `mushr_hardware/realsense`  
- Map: `mushr_base/mushr_base/launch/includes/map_server.launch`  

Now you should have a grasp of the basic hardware and software components of MuSHR platform! Please [contact us](/contact) with any questions or feedback about your experience using the MuSHR system and tutorials.
