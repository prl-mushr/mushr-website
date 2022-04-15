---
title: "Quickstart with Foxglove"
date: 2022-04-13T13:18:04-07:00
summary: "Get the MuShr Sim working on your computer!"
difficulty: "Beginner"
duration: 30
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: true      # whether Hugo considers this a draft
weight: 2     # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://mushr.io/>Madison Doerr</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/first_steps/firststep.jpg" width="800" >}}                         
</br>

### Introduction
Learn how to simulate the MuSHR Car with Foxglove visualizations.

### Requirements
* **A MacOS or Linux machine**

## Installing Docker

First, install [docker](https://docs.docker.com/get-docker/) and [docker compose](https://docs.docker.com/compose/install/) for your machine. The Docker version should be 20+, and Docker-compose version should be 1.29+.

If on Linux, follow the [post install](https://docs.docker.com/engine/install/linux-postinstall/) steps to make sure you can run docker without root.

## Installing MuSHR Docker Container
If you have pre-existing work in a `catkins_ws` workspace, move this work to another folder or stash it somewhere safe. 

Now, clone the MuSHR repo:
{{< highlight bash >}} $ git clone https://github.com/prl-mushr/mushr.git {{< / highlight >}}

Navigate into the repo and switch to the noetic branch (TODO noetic will be main soon) 
{{< highlight bash >}} $ cd mushr && git checkout noetic && cd mushr_utils/install {{< / highlight >}}

Run the installation script. It will prompt you with three questions. For running on the simulator the answers should be no, no, no. 
{{< highlight bash >}} $ ./mushr_install.bash {{< / highlight >}} 

It will also ask if you are ok with adding "xhost +" to your .bashrc (Linux) or .zshrc (MacOS). This is for running GUI apps from within docker and is not explicitly necessary, although recommended.

Close the terminal and open a new one. Then run
{{< highlight bash >}} $ mushr_noetic {{< / highlight >}} 

This should start up the docker container. If the prefix switches to `root`, the installation was successful. 

In the same terminal, build the stack 
{{< highlight bash >}} $ source .bashrc && cd catkin_ws && catkin build {{< / highlight >}}

## Downloading Foxglove Studio
Since we can run the MuSHR stack now, we can use Foxglove Studio to visualize our robot and map. 

First, download [Foxglove Studio](https://foxglove.dev/download). Foxglove is 
still in development and has many features that are added frequently, so make sure to download the most
recent version, or update it if you already have it downloaded.

[TODO: Merge PR into noetic branch for foxglove layout. For now the layout file specified below can be found at [this branch](https://github.com/prl-mushr/mushr/blob/foxglove-viz/mushr_utils/foxglove/foxglove_layout.json). Download it into the location below.]

Open Foxglove Studio, and click the "Layouts" button on the left panel (second from top) and then click
`Import Layout` button pictured below.
{{< figure src="/tutorials/noetic_quickstart/foxglove_layout.png" width="400" >}} 

Import the preset layout at:
{{< highlight bash >}} mushr/mushr_utils/foxglove/foxglove_layout.json {{< / highlight >}}    

Three panels should appear after selecting this layout, as pictured below. The left panel is for the datasource, the central panel is for the map, and the right panel is for teleop driving.

{{< figure src="/tutorials/noetic_quickstart/layout_example.png" width="700" >}}                      

The layout can be edited with the `Add Panel` button on the left sidebar if desired.

## Connecting to Data With Foxglove Studio
Now that Foxglove is set up, we can connect the visualization to our docker container ROS setup. Start the docker container:
{{< highlight bash >}} $ mushr_noetic {{< / highlight >}}
Once the `root` prompt appears, source the `.bashrc`. Note that this does not occur automatically in the docker container.

{{< highlight bash >}} $ source .bashrc {{< / highlight >}}

In the same terminal, start up the simulator with the command:

{{< highlight bash >}} $ roslaunch mushr_sim teleop.launch {{< / highlight >}}

After starting up, the simulator should print out a line similar to 

{{< highlight bash >}} Rosbridge WebSocket server started at ws://0.0.0.0:9090 {{< / highlight >}}

In foxglove, click the top button in the sidebar, labeled `Data Source`. Then select the Plus button in the left panel. This should open up an interface to connect to data. 

{{< figure src="/tutorials/noetic_quickstart/data_panel.png" width="400" >}}

Click the `Open Connection` button. Select `Rosbridge (ROS 1 & ROS 2)` as shown below.

{{< figure src="/tutorials/noetic_quickstart/rosbridge.png" width="700" >}}

Fill out the WebSocket URL with the url and port that the simulator output before, as shown in the image. The URL may differ from pictured. Then, click `Open` in the bottom right corner. The left sidebar should show that a connection has been made. The map should update and the car pose arrow should appear in the map panel in the center. The car can be driven using the right teleop panel.

{{< figure src="/tutorials/noetic_quickstart/example.png" width="900" >}}

TODO: Get robot model to appear & update image

### Changing the map
The map can be changed by editing the following file:
{{< highlight bash >}} src/mushr_sim/launch/teleop.launch {{< / highlight >}}

In this file, edit the line
{{< highlight bash >}} <arg name="map" default="$(find mushr_sim)/maps/sandbox.yaml"/>{{< / highlight >}}
to specify a different map. MuSHR provides some basic maps, but if you want to map your own space, try the [SLAM Tutorial for MuSHR](/tutorials/mapping).

## Troubleshooting
### Incompatible Network Binding
Navigate to the install directory:
{{< highlight bash >}}mushr/mushr_utils/install/{{< / highlight >}}
Open the `docker-compose-cpu.yml` file in this folder and remove the line:

{{< highlight bash >}}network: host{{< / highlight >}}
### Wlan0 Device Not Found
Edit the `.bashrc` file in the docker container and manually set the `ROS_IP` to the IP of your computer.
The IP of your computer can be found in several ways. One way is to run the following:
{{< highlight bash >}}
$ ifconfig
{{< / highlight >}}
After manually setting this value, make to source the `.bashrc`.
{{< highlight bash >}}
$ source ~/.bashrc
{{< / highlight >}}

