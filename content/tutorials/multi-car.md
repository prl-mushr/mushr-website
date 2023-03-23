---
title: "Simulating Multiple Cars"
date: 2021-05-13T18:12:58-07:00
summary: "Running the Multi Car Simulator"
difficulty: "Beginner"
duration: 15
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: false      # whether Hugo considers this a draft
weight: 3        # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://heatblast016.github.io/>Abhinav Diddee</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/multi_car/cars.png" width="800" >}} <br>                           
<br>

### Introduction
This tutorial will get you started with simulating multiple cars in the MuSHR simulation

### Goal
To launch the simulator with multiple cars, and to explain how to configure it for different use cases

### Requirements
* **requirement 1:** Successful completion of the [quickstart](/tutorials/quickstart) tutorial
* **requirement 2:** Some familiarity with basic ROS concepts
Note: Due to an issue with roslaunch in ROS Kinetic, the instructions for Kinetic users will differ from the instructions for Melodic users, since Kinetic users have to use an outdated version of the Multi Car launchfile
## Running the Simulation
To run the sim with multiple cars run:

Melodic:

```bash
roslaunch mushr_sim multi_car.launch
```

Kinetic:

```bash
roslaunch mushr_sim multi_teleop.launch
```

Two gray teleop windows should appear by default, one for each car.

Next, open rviz with the command:

```bash
rviz -d ~/catkin_ws/src/mushr/mushr_utils/rviz/multi_car.rviz
```
The -d flag tells rviz to use the proper configuration file, so you don't need to manually set up the visualization environment every time.

{{< figure src="/tutorials/multi_car/open.png" >}}

That's all there is to it!
## Adding cars
To add cars, follow the following instructions based on your version of ROS:

Kinetic:

To launch more cars, simply add more blocks in multi_teleop.launch (located in the mushr_sim/launch) by following the template.

Melodic onwards:

To launch more cars, adjust the num_cars argument passed into multi_car.launch by using the following command, where you can replace 3 with however many cars you want. 

```bash
roslaunch mushr_sim multi_car.launch num_cars:=3
```
## Visualizing new cars 
To visualize new cars, first add their robotmodel by clicking the "add" button in the bottom left corner of the 'displays' section in rviz, then scrolling down in the "by display type section", selecting RobotModel, and clicking 'OK'.

There should be a new addition to the 'displays' section with the name 'RobotModel' and an error status. Expand it by clicking on the horizontal triangle at the very far left.

{{< figure src="/tutorials/multi_car/vis1.png" >}}

Then, double click on the field where it says robot_description, and change it to "/prefix/robot_description" — for example, you may be visualizing a third car, so you'd change it to "/car2/robot_description". Then, click on the blank field next to TF Prefix, and set that to the same '/prefix' from earlier (in our example this would be '/car2')  

The RobotModel section should now appear as so:

{{< figure src="/tutorials/multi_car/vis2.png" >}}

The status should now be "Ok"

To visualize the car's laserscan, click the 'add' button again, but this time switch to the "By topic" tab in the pop-up window. Then, scroll down to the car you're currently adding (in our case this car would be named car2), and select the field labeled "LaserScan" under "/scan"

{{< figure src="/tutorials/multi_car/vis3.png" >}}

Then, click "OK"

To add the visualization of the car's name above it, follow the same steps as to add the laserscan, but instead of selecting the "LaserScan" under the "/scan" topic, select the "Marker" (which should be displayed under the "/visualization_marker" topic)

{{< figure src="/tutorials/multi_car/vis4.png" >}}

With that, you've successfully visualized a new car!

{{< figure src="/tutorials/multi_car/vis5.png" >}}

Repeat this step for every new car you instantiate
