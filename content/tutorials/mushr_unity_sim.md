---
title: "Using unity to simulate MuSHR"
date: 2021-05-17T17:51:18+05:30
summary: "Using a unity-based physics simulator in place of the default kinematic mushr_sim"
difficulty: "Intermediate"
duration: 10
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: true      # whether Hugo considers this a draft
weight: 3        # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://www.sidharthtalia.com/>Sidharth Talia</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/mushr_unity_sim/thumbnail.gif" width="800" >}} <br>                           
<br>

### Introduction

The default state prediction system [racecar_state.py](https://github.com/prl-mushr/mushr_base/blob/master/mushr_base/src/racecar_state.py) used in mushr_sim works well for low-speed situations where the wheel-slippage and body roll aren't significant. However, once you get to higher speeds (3-4 m/s and above), you want to be able to simulate those things as well so that you know your system is robust to slippage, body roll and so on. For this purpose, we use a unity-based simulator which uses the ["PhysX"](https://github.com/NVIDIAGameWorks/PhysX) physics engine. The system provided by the [mushr_unity_sim](https://github.com/naughtyStark/mushr_unity_sim.git) is a drop in replacement for the default state predictor (but not the entire mushr_sim). It is based on the the [donkey simulator](https://github.com/tawnkramer/sdsandbox) created for/by the DIYRobocar community.


### Goal

The goal of this tutorial is to get the user acquainted to the unity-based simulation system.


### Requirements

Completed the [quickstart](https://mushr.io/tutorials/quickstart/) tutorial.
Make sure you’re using the sandbox map. The quickstart tutorial explains how to change the map.


## Installation:

Below are the instructions for installing the unity simulation system:


### Cloning the repository:

Clone the [mushr_unity_sim](https://github.com/naughtyStark/mushr_unity_sim.git) repository:

{{<highlight bash>}}
$ cd ~/catkin_ws/src
$ git clone https://github.com/naughtyStark/mushr_unity_sim.git
$ cd ~/catkin_ws
$ catkin_make
{{</highlight>}}

The last catkin_make command is necessary as it executes certain setup steps.

Python dependencies: 

{{<highlight bash>}}
$ cd ~/catkin_ws/src/mushr_unity_sim
$ pip install -r requirements.txt
{{</highlight>}}



### Running the base example:
In a new terminal, enter:

{{<highlight bash>}}
$ roslaunch mushr_unity_sim unity_multi.launch
{{</highlight>}}

This should start the simulator (in headless mode) and the scripts necessary for connecting the simulator to ros and rviz. Wait for a few seconds for all the cars to be "plopped" into the simulation. I say plopped because sometimes you may notice the cars moving a little initially without inputs. This happens because the simulator drops the cars from a small height during initialization. The dev thought it would be fun to plop the cars to show they aren't being simulated just based on the input you give to them. You should be able to click on the tkinter tabs (tk tabs) and use WASD to drive the cars around, something like this:

<br>
{{< figure src="/tutorials/mushr_unity_sim/showboat.gif" width="800" >}} <br>                           
<br>

Note that collisions between agents are simulated. The rendering in rviz is a bit choppy when you add more cars but the simulation is still real-time. If you face choppy rendering like above and want to see more realtime rendering like in the thumbnail of this tutorial, you'll have to reduce the number of rendered agents (just unselect the RobotModel on the left pane of rviz). The choppiness is an rviz issue and the cars that are not rendered in rviz are still simulated. 

Note that when driving with 'WASD' keys, the maximum speed is limited to 2 m/s. This is because the keyboard_teleop node translates pressing 'W' to '2 m/s' forward speed.


### Using this simulator for your own purposes:
<br>

#### Running just the simulator:

In a new tab execute this command:

{{<highlight bash>}}
$ ./catkin_mushr/src/mushr_unity_sim/unity/mushr_unity_sim.x86_64 -batchmode
{{</highlight>}}

This command starts the simulator executable kept inside mushr_sim_unity/unity folder in `batchmode`, which runs it in headless mode. The benefit of running in headless mode is that you can get better fps as it no longer needs the GPU. This is the case for people who don't have Nvidia drivers enabled on their system (like me). If you do (god bless), you can start the simulator without the `batchmode` tag if you want to see the simulator's rendering of the scene.

{{< figure src="/tutorials/mushr_unity_sim/render.png" width="800" >}}

The scenery in the simulation isn't much to look at, so you're not missing out on anything that interesting. Ignore the red-sphere around the car, it's simply a marker that the developer didn't bother to remove lest he break the simulator. It does contain a randomly generated road, so if you want you can drive it around with WASD keys on it. If you want to do deep-learning or computer vison stuff, refer to this other tutorial [here](https://mushr.io/tutorials/deep_learning/).


#### Rviz config:

The rviz configuration is saved in `mushr_unity_sim/rviz/mushr_unity_sim.rviz`. You can launch it with the following command:
<br>
{{<highlight bash>}}
$ rviz -d ~/catkin_ws/src/mushr_unity_sim/rviz/mushr_unity_sim.rviz
{{</highlight>}}
<br>


#### Initial pose setting:

The poses of these cars are set by the pose_init.py file. The car's initial pose is set the same way as done for the default mushr_sim; by publishing a message on the topic: `/car_name/initialpose` of type: [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) where `car_name` corresponds to the name of the car. 


#### Including mushr_unity_sim's simulator in your launch file

To launch the simulator from the launch file, add these lines near the top of your launch file:
{{<highlight xml>}}
<node pkg="mushr_unity_sim" type="run_sim.sh" name="simrun_script"/> <!-- this script launches the simulator -->
{{</highlight>}}

You can refer to [unity_multi.launch](https://github.com/naughtyStark/mushr_unity_sim/blob/main/launch/unity_multi.launch#L1) if you're still not sure where to place this line. 

The usual way of invoking the mushr_sim's default state predictor is to include the following lines in your launch file:
<br>
{{<highlight xml>}}
    <group ns="$(arg car_name)">
        <include file="$(find mushr_sim)/launch/single_car.launch">
            <arg name="car_name" value="$(arg car_name)"/>
            <arg name="racecar_version" value="racecar-uw-nano"/>
            <arg name="racecar_color" value="" />
        </include>
    </group>
{{</highlight>}}
<br>
The mushr's [multi_teleop.launch](https://github.com/prl-mushr/mushr_sim/blob/master/launch/multi_teleop.launch#L16) is an example of such a launch file.

In order to invoke the ros-wrapper for the unity simulator, you need to replace the above lines with these:
<br>
{{<highlight xml>}}
    <group ns="$(arg car_name)">
        <include file="$(find mushr_unity_sim)/launch/unity_single.launch"> <!-- this is the only line that needs to be changed -->
            <arg name="car_name" value="$(arg car_name)"/> <!-- name of the car. Same conventions as for the default method -->
            <arg name="racecar_version" value="racecar-uw-nano"/> <!-- simulator only supports the MuSHR car! --> 
            <arg name="racecar_color" value="" /> <!-- car can have whatever color scheme you like -->
        </include>
    </group>
{{</highlight>}}
<br>

With this done, you should be able to run everything else just the way you did with the standard stack. This means that publishing the control inputs to `/car_name/mux/ackermann_cmd/input/navigation` will drive and steer the car, and the car's simulated pose is published to the `/car_name/car_pose` topic, as in the standard stack. In addition to this, the node also publishes an odometry topic `/car_name/car_odom` in case you prefer to get the pose along with the twist. 


#### Setting the friction coefficient:

The friction coefficient can be modified/set by changing it's value in the `mushr_unity_sim.yaml` kept in the "config" folder
{{<highlight yaml>}}
friction_coefficient: 1.0
{{</highlight>}}

The default value is 1.0. For your information, the behavior shown in the thumbnail gif can be achieved by setting the friction coefficient to around 0.7. Lowering the friction value and driving while blasting the [Tokyo Drift theme track](https://www.youtube.com/watch?v=pS5d77DQHOI) is recommended for optimal fun or research purposes.


## Troubleshooting
* **Only 2 or 3 of the 4 cars show up in rviz when running the example launch file:** This can happen if you're launching the simulator on your system for the first time after boot. It happens because the simulator takes some time to load on the first attempt and the pose-init file has already published the initial pose for that car. Keep launching again until it goes away (it will usually go away on the second launch).

