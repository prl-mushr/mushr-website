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

Note: If you already have a `catkin_ws` with changes that you'd like to keep, see the FAQ section at the end of this tutorial.

Clone the MuSHR repository and navigate to the installation scripts directory. (TODO: `noetic` will be main soon)

```bash
$ git clone --branch noetic https://github.com/prl-mushr/mushr.git
$ cd mushr/mushr_utils/install
```

Run the installation script. It will prompt you with three questions. For running the MuSHR simulator, the answers should be no, no, no. At the end, it will also ask if you are ok with adding `xhost +` to your .bashrc (Linux) or .zshrc (MacOS). This is for running GUI apps from within Docker and is not explicitly necessary, although recommended.

```bash
$ ./mushr_install.bash
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

Three panels should appear after selecting this layout, as pictured below. The left panel is for the data source, the central panel is for the map, and the right panel is for teleop driving.

{{< figure src="/tutorials/noetic_quickstart/layout_example.png" width="700" >}}

The layout can be edited with the `Add panel` button on the left sidebar if desired.

## Connecting to Data With Foxglove Studio
Now that Foxglove is set up, we can connect the visualization to our Docker container ROS setup. Start the Docker container:

```bash
$ mushr_noetic
```

Once the `root` prompt appears, source `~/.bashrc`. Note that this does not occur automatically in the Docker container.

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

In Foxglove, click the top button in the sidebar, labeled `Data source`. Then select the Plus button in the left panel. This should open up an interface to connect to data.

{{< figure src="/tutorials/noetic_quickstart/data_panel.png" width="400" >}}

Click the `Open Connection` button. Select `Rosbridge (ROS 1 & ROS 2)` as shown below.

{{< figure src="/tutorials/noetic_quickstart/rosbridge.png" width="700" >}}

Fill out the WebSocket URL with the url and port that the simulator output before, as shown in the image. The URL may differ from pictured. Then, click `Open` in the bottom right corner. The left sidebar should show that a connection has been made. The map should update and the car pose arrow should appear in the map panel in the center. The car can be driven using the right teleop panel.

{{< figure src="/tutorials/noetic_quickstart/example.png" width="900" >}}

TODO: Get robot model to appear & update image

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

## Troubleshooting

### Docker: Error while fetching server API version

Ensure that Docker is running. In your terminal, `docker ps -a` should not cause an error.

### Docker: network mode is incompatible with port bindings

Navigate to the install directory `mushr/mushr_utils/install`. Edit the `docker-compose-cpu.yml` file in this folder to remove the line: `network_mode: "host"`.

### wlan0: Device not found

Edit the `.bashrc` file in the Docker container and manually set the `ROS_IP` to the IP of your Docker container.
One way to find this is to run the following command:

```bash
$ ifconfig
```

After manually setting this value, make to source the `.bashrc` in the Docker container.
```bash
$ source ~/.bashrc
```

## FAQ

### How can I keep another MuSHR-related `catkin_ws` workspace without overwriting it?

These instructions were intended for a first-time setup of MuSHR. To get around that,
set the following paths in your `.bashrc` or `.zshrc` manually to the paths of your existing `catkin_ws` workspace.

```bash
ROS_PACKAGE_PATH, LD_LIBRARY_PATH, ROSLISP_PACKAGE_DIRECTORIES, PKG_CONFIG_PATH, CMAKE_PREFIX_PATH
```

Then, when you want to run the Docker container with the new MuSHR setup, comment these lines out and re-source your `.bashrc` (Linux) or `.zshrc` (MacOS).


