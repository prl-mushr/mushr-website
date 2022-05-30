---
title: "Quickstart with Foxglove"
date: 2022-04-13T13:18:04-07:00
summary: "Get the MuSHR Sim working on your computer!"
difficulty: "Beginner"
duration: 30
featured: false  # whether this is listed at / (must also be top 6 by weight).
active: true     # whether this is listed at /tutorials/
draft: false      # whether Hugo considers this a draft
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

First, install [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/) for your machine. The Docker version should be 20+, and Docker Compose version should be 1.29+. Ensure that Docker is running.

If on Linux, follow the [post install](https://docs.docker.com/engine/install/linux-postinstall/) steps to make sure you can run Docker without root privileges.

## Installing MuSHR Docker Container

Make `catkin_ws` where all the code will be.
```bash
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
```

Clone the MuSHR repository (TODO: `noetic` will be main soon)

```bash
$ git clone --branch noetic https://github.com/prl-mushr/mushr.git
```

Run the installation script. It will prompt you with two questions. For running the MuSHR simulator, the answers should be no, no. This will install a series of necessary packages, and create a script `mushr_noetic` in `/usr/local/bin` which initializes a docker container with all of the mushr configs installed. Note it attaches the `catkin_ws` volume so you can edit code outside or inside the docker container. Other files made inside the docker container will not persist unless you commit (see FAQ).

```bash
$ ./mushr/mushr_utils/install/mushr_install.bash
```

Close the terminal and open a new one. Then run
```bash
$ mushr_noetic
```

This should start up the Docker container. (The first time running this command will take some time to download the Docker image.) If the prefix switches to `root`, the installation was successful. If not, please check the "Troubleshooting" section at the end of the tutorial.

In the same terminal (within the Docker container), build the MuSHR software stack.

```bash
$ source .bashrc && cd catkin_ws && catkin build
```

## Downloading Foxglove Studio

Since we can run the MuSHR stack now, we can use Foxglove Studio to visualize our robot and map.

First, download [Foxglove Studio](https://foxglove.dev/download). Foxglove is
still in development and has many features that are added frequently, so make sure to download the most
recent version, or update it if you already have it downloaded.
We recommend at least version TODO.

Open Foxglove Studio, and click the "Layouts" button on the left panel (second from top) and then click
`Import layout` button pictured below.
{{< figure src="/tutorials/noetic_quickstart/foxglove_layout.png" width="400" >}}

Import the preset layout from:
```bash
mushr/mushr_utils/foxglove/foxglove_layout.json
```
Now, download the custom panel extension for MuSHR for Foxglove. Navigate to the Extensions tab (bottom icon on the left panel) and install the `MushrTeleop` extension from the Marketplace.
{{< figure src="/tutorials/noetic_quickstart/mushr_extension.png" width="400" >}}

Now, four panels should appear after selecting this layout, as pictured below. The left panel is for the data source, the central panel is for the map, the top right panel is for teleop driving, and the bottom right panel is for selecting types of robot poses to publish (for use in autonomous naviagation).

{{< figure src="/tutorials/noetic_quickstart/custom_layout.png" width="700" >}}

The layout can be edited with the `Add panel` button on the left sidebar if desired.

## Connecting to Data With Foxglove Studio
Now that Foxglove is set up, we can connect the visualization to our Docker container ROS setup. Start the Docker container:

```bash
$ mushr_noetic
```

Once the `root` prompt appears, source `~/.bashrc`. Note that this does not occur automatically the first time in the Docker container.

```bash
$ source ~/.bashrc
```

In the same terminal, start up the simulator with the command:

```bash
$ roslaunch mushr_sim teleop.launch
```

After starting up, the simulator should print out a line similar to

```bash
Rosbridge WebSocket server started at ws://0.0.0.0:9090
```

In Foxglove, click the top button in the sidebar, labeled `Data Source`. Then select the Plus button in the left panel. This should open up an interface to connect to data.

{{< figure src="/tutorials/noetic_quickstart/data_panel.png" width="400" >}}

Click the `Open Connection` button. Select `Rosbridge (ROS 1 & ROS 2)` as shown below.

{{< figure src="/tutorials/noetic_quickstart/rosbridge.png" width="700" >}}

Fill out the WebSocket URL with the url and port that the simulator output before, as shown in the image. The URL may differ from pictured. Then, click `Open` in the bottom right corner. The left sidebar should show that a connection has been made. The map should update and the car pose arrow should appear in the map panel in the center. The car can be driven using the right teleop panel.

{{< figure src="/tutorials/noetic_quickstart/working_example.png" width="900" >}}

### Changing the map

The map can be changed by editing the following file:
```bash
src/mushr_sim/launch/teleop.launch
```

In this file, edit the line:
```xml
<arg name="map" default="$(find mushr_sim)/maps/sandbox.yaml"/>
```
to specify a different map. MuSHR provides some basic maps, but if you want to map your own space, try the [SLAM Tutorial for MuSHR](/tutorials/mapping).

(TODO: provide instructions for overriding map from the command-line.)

### Setting the car pose
The car pose can be changed by selecting the `Set Pose` button in the bottom right, and then **long pressing (1-2 seconds)** on the `click to publish` button in the right toolbar to select the `Publish Pose` option. Then, click on the map where you would like to publish and click again to set the orientation from the original click.

<div style="width:800px; height:450px"  class="video-container">
    <iframe style="width: 100%; height: 100%; border: none" width="750" src="/tutorials/noetic_quickstart/mushr_reposition.mp4" allowfullscreen></iframe>
</div>

## Troubleshooting

### Docker: Error while fetching server API version

Ensure that Docker is running. In your terminal, `docker ps -a` should not cause an error.

### TCP Registry Lookup Error
Sometimes the following error is recieved:
```bash
ERROR: Get https://registry-1.docker.io/v2/: dial tcp: lookup registry-1.docker.io on [::1]:53: read udp [::1]:42546->[::1]:53: read: connection refused
```

This error seems to stem from a [nameserver issue](https://github.com/docker/cli/issues/2618). Add the following to `/etc/resolv.conf`
```bash
nameserver 8.8.8.8
nameserver 8.8.4.4
```

## FAQ

### I installed some packages in my docker container, how can I "save" them for next time I run `mushr_noetic`
You can save this using the `docker commit` command. First find your container ID

```bash
$ docker ps
```
It should be the only one or the one with `mushr/mushr:XXX` under the image name (XXX different on different systems). Now commit the CONTAINER ID to the IMAGE name. For example:

```bash
$ docker commit de878d464895 mushr/mushr:x86_64
```

