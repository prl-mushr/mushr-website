---
title: "Using SLAM for Map Building"
date: 2021-03-20T17:15:01-07:00
summary: Create custom maps with your car
difficulty: Intermediate
duration: 0
featured: true  # whether this is listed at / (must also be top 6 by weight)
active: true    # whether this is listed at /tutorials/
draft: false      # whether Hugo considers this a draft
weight: 100
---

## By: Madison Doerr and Markus Schiffer

### Introduction
The MuSHR car uses a map of its environment to localize itself. We can create 
custom 2D maps for our cars by using [gmapping](http://wiki.ros.org/gmapping) and our car's laser scanner to 
survey the area to be mapped.

### Goal
Create a custom map of an area by driving the car to survey the surroundings.

### Requirements
  - Complete the [quickstart](/tutorials/quickstart) tutorial
  - Complete the [first steps](/tutorials/first_steps) tutorial
  - A desktop/laptop that can ssh into the car

### Ssh into the car
Boot up the car and connect the controller. 

Ssh into the car on your computer, using the following command while connected to the ROBOT_AP network.
```bash
$ ssh robot@10.42.0.1 -X
```
Or, if you set up Wifi connect in the [first steps](/tutorials/first_steps) tutorial, use the following command while connected to the same wifi network as your robot. Replace the IP listed below with the IP of your robot.
```bash
$ ssh robot@172.16.77.37 -X
```

### Install gmapping
In a terminal sshed into the car using the above steps, make sure gmapping is installed.

If you installed ros-kinetic in the quickstart tutorial:
```bash
$ sudo apt install ros-kinetic-gmapping
```
If you installed ros-melodic in the quickstart tutorial:
```bash
$ sudo apt install ros-melodic-gmapping
```

### Start teleop and gmapping
Once you have sshed into the car, start teleop to start the laser scanner and control the car.
```bash
$ roslaunch mushr_base teleop.launch
```
Verify that the controller is working and you can move the car. If not, try reconnecting the controller and/or restarting teleop.

In a separate terminal sshed into the robot, using the above process, start slam. Slam will take the data gathered from the laser 
scanner started by teleop and transform that into a 2D map. 
```bash
$ roslaunch mushr_base slam.launch
```

### Start rviz to visualize the map building
In a terminal on your computer (not ssh-ed into the robot), set `ROS_IP` to your IP and `ROS_MASTER_URI` to the IP of the car.

Set `ROS_IP`.
```bash
$ export ROS_IP=YOUR-IP
```

Set the `ROS_MASTER_URI` to the IP of the car.
```bash
$ export ROS_MASTER_URI=http://ROBOT_IP:11311
```

Now, in the same terminal, start rviz.
```bash
$ rviz
```
A screen should appear of the robot and the map it is creating. If you don't see the map, make sure the `/car/map` topic is added to the left sidebar. The map will dynamically update and refine as the laser scanner gathers new data.

{{< figure src="/tutorials/mapping/start_map.png" width="400" >}}
</br>

### Survey the surroundings
Now that we have started mapping, we need to survey the area! Use the controller to drive the car very slowly around the area you want to survey, pausing periodically to allow the laser scanner to gather data. You should be able to see the map in rviz updating as you drive.

### Save the map
When you're satisfied with the map in rviz, you can save it to your computer using the following steps.

In a local terminal, ensure `ROS_IP` is set to your IP and `ROS_MASTER_URI` is set to the IP of the car. Then, in that terminal, run map_saver using the following command, replacing `map_name` with the desired name of the map.
```bash
$ rosrun map_server map_saver -f map_name map:=/car/map
```
This should create a .pgm file with the map and a .yaml file with the map metadata. 

### Using the map
To load your new map into the simulator, move both files to `~/catkin_ws/src/mushr_sim/maps/`, replacing map_name with the name of your map.
```bash
$ mv map_name.pgm ~/catkin_ws/src/mushr_sim/maps/
$ mv map_name.yaml ~/catkin_ws/src/mushr_sim/maps/
```
Then, edit `~/catkin_ws/src/mushr_sim/launch/map_server.launch` to set the map to the name of the .yaml file.

```bash
$ nano ~/catkin_ws/src/mushr_sim/launch/map_server.launch
```

It should look like the following, replacing map_name with the name of your map.

{{< highlight python "linenos=table" >}}
<launch>
    <arg name="map" default="$(find mushr_base)/maps/map_name.yaml" />
    <node pkg="map_server" name="map_server" type="map_server" args="$(arg map)" />
</launch>
{{< / highlight >}}

Now, launch the sim to use your new map!
```bash
$ roslaunch mushr_sim teleop.launch
```
In a new terminal, run rviz.
```bash
$ rviz
```
Your new map and the car should appear in the sim! If you don't see the map, make sure you are subscribed to the `/map` topic in the left sidebar. The map might not be in the center of the grid, and you may have to zoom out to see it.

### Touching up the map
The map may have some stray pixels or jagged lines. To touch this up, we recommend using [gimp](https://www.gimp.org/) to edit the .pgm file.

When doing this, use solid black (hex: `#000000`) pixels for edges, solid white (hex: `#FFFFFF`) areas for areas valid for the car, and solid gray areas (hex: `#CDCDCD`) for invalid space for the car.

{{< figure src="/tutorials/mapping/edited_map.png" width="400" >}}


