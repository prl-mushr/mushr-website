---
title: "Using SLAM for Map Building"
date: 2021-03-20T17:15:01-07:00
summary: Create custom maps with your car
difficulty: Intermediate
duration: 0
featured: true  # whether this is listed at / (must also be top 6 by weight)
active: true    # whether this is listed at /tutorials/
draft: false      # whether Hugo considers this a draft
weight: 3
---

## By: [Madison Doerr](https://mcdoerr.github.io/) and [Markus Schiffer](www.linkedin.com/in/markusschiffer)

### Introduction
The MuSHR car uses a map of its environment to localize itself. We can create 
custom 2D maps for our cars by using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and our car's laser scanner to 
survey the area to be mapped.

{{< figure src="/tutorials/mapping/map-demo.png" width="500" >}}

### Goal
Create a custom map of an area by driving the car to survey the surroundings.

### Requirements
  - Complete the [quickstart](/tutorials/humble_quickstart) tutorial
  - Complete the [first steps](/tutorials/first_steps) tutorial
  - A desktop/laptop that can ssh into the car

### Ssh into the car
Boot up the car and connect the controller. 

Ssh into the car on your computer, using the following command while connected to the ROBOT_AP network.
```bash
$ ssh robot@10.42.0.1 -X
```
Or, if you set up Wifi connect in the [first steps](/tutorials/humble_first_steps) tutorial, use the following command while connected to the same wifi network as your robot. Replace the IP listed below with the IP of your robot.
```bash
$ ssh robot@172.16.77.37 -X
```

### Install slam_toolbox
In a terminal sshed into the car using the above steps, make sure `slam_toolbox` is installed.
```bash
$ sudo apt install ros-humble-slam-toolbox
```

### Start teleop and SLAM
Once you have sshed into the car, start teleop (on the car) to start the laser scanner and control the car.
```bash
$ ros2 launch mushr_base teleop.launch.py
```
Verify that the controller is working and you can move the car. If not, try reconnecting the controller and/or restarting teleop.

In a separate terminal sshed into the robot, start SLAM (also on the robot). SLAM will take the data gathered from the laser scanner started by teleop and transform that into a 2D map. The `--ros-args` below point slam_toolbox at the car's scan topic and TF frames.
```bash
$ ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    -p base_frame:=car/base_link \
    -p odom_frame:=car/odom \
    -p map_frame:=map \
    -p scan_topic:=/car/scan
```

### Start rviz2 to visualize the map building
The car and your computer discover each other over DDS when they share the same `ROS_DOMAIN_ID` on the same network. In a terminal on your computer (not ssh-ed into the robot), set the same `ROS_DOMAIN_ID` as the car and start rviz2.
```bash
$ export ROS_DOMAIN_ID=<same id as the car>
$ rviz2
```
Most likely, you will not see the map your car is creating. In order to see it, make sure the `/map` topic is added to the left sidebar. If it is not, you will need to add it manually. Next, set the Fixed Frame value in the Displays > Global Options menu in rviz2 to `map`. Now, the car should be postioned in the map (see screenshot below). The map will dynamically update and refine as the laser scanner gathers new data.

{{< figure src="/tutorials/mapping/start_map_large.png" width="400" >}}
</br>

### Survey the surroundings
Now that we have started mapping, we need to survey the area! Use the controller to drive the car very slowly around the area you want to survey, pausing periodically to allow the laser scanner to gather data. You should be able to see the map in rviz2 updating as you drive. Since the map frame does not exist in rviz2, your car will not appear to move in its environment.

### Save the map
When you're satisfied with the map in rviz2, you can save it to your computer using the following steps.

In a local terminal (with the same `ROS_DOMAIN_ID` as the car), run the map saver using the following command, replacing `map_name` with the desired name of the map.
```bash
$ export ROS_DOMAIN_ID=<same id as the car>
$ ros2 run nav2_map_server map_saver_cli -f map_name --ros-args -r map:=/map
```
This should create a .pgm file with the map and a .yaml file with the map metadata. 

### Using the map
#### On the sim
To load your new map into the simulator, move both files to `~/colcon_ws/src/mushr/mushr_sim/maps/`, replacing map_name with the name of your map.
```bash
$ mv map_name.pgm ~/colcon_ws/src/mushr/mushr_sim/maps/
$ mv map_name.yaml ~/colcon_ws/src/mushr/mushr_sim/maps/
```
Now, launch the sim with your new map by passing its `.yaml` to the `map` argument:
```bash
$ ros2 launch mushr_sim teleop.launch.py map:=/root/colcon_ws/src/mushr/mushr_sim/maps/map_name.yaml
```
In a new terminal, run rviz2.
```bash
$ rviz2
```
Your new map and the car should appear in the sim! If you don't see the map, make sure you are subscribed to the `/map` topic in the left sidebar. The map might not be in the center of the grid, and you may have to zoom out to see it.
#### On the car
To get the map from your computer to the robot, we need to use `scp`, replacing `map_name` with the name of your map and `ROBOT_IP` with the IP of your robot. From your computer, run the following from where you saved your maps:
```bash
$ scp map_name.pgm robot@ROBOT_IP:~/colcon_ws/src/mushr/mushr_sim/maps/
$ scp map_name.yaml robot@ROBOT_IP:~/colcon_ws/src/mushr/mushr_sim/maps/
```
Now, start teleop on the robot, then launch the map server with your map's `.yaml`:
```bash
$ ros2 launch mushr_base teleop.launch.py
$ ros2 launch mushr_sim map_server.launch.py map:=/root/colcon_ws/src/mushr/mushr_sim/maps/map_name.yaml
```
And on your computer (with the same `ROS_DOMAIN_ID` as the car), start rviz2 to see the map!
```bash
$ export ROS_DOMAIN_ID=<same id as the car>
$ rviz2
```
The map should appear, and you can move your robot into the map using `Set 2D Pos Estimate`.

### Touching up the map
The map may have some stray pixels or jagged lines. To touch this up, we recommend using [gimp](https://www.gimp.org/) to edit the .pgm file.

When doing this, use solid black (hex: `#000000`) pixels for edges, solid white (hex: `#FEFEFE`) areas for areas valid for the car, and solid gray areas (hex: `#CDCDCD`) for invalid space for the car.

{{< figure src="/tutorials/mapping/edited_map_large.png" width="400" >}}


