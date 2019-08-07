---
title: "Quickstart"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
duration: 30 
difficulty: Beginner 
summary: Run the MuSHR platform on your machine! 
weight: 1
---

{{< figure src="/tutorials/quickstart/quickstart_header.gif" width="800" >}}
<br>

### Introduction
This tutorial will get you started with MuSHR in simulation!

### Goal 
To get the simulator running on your machine so that you can begin hacking immediately!

### Requirements
A linux machine (OSX and Windows support coming soon!). If you don't run linux natively then get a linux VM: [OSX](https://www.instructables.com/id/How-to-Create-an-Ubuntu-Virtual-Machine-with-Virtu/), [Windows](https://itsfoss.com/install-linux-in-virtualbox/).

## Setup
First we need to make sure you have a few dependencies installed. To make it easier, we have created a docker container with the MuSHR stack and sim running inside it. Here is what you need:

- [docker](https://docs.docker.com/v17.12/install/)
- [docker-compose](https://docs.docker.com/compose/install/)
- git
- A github account. You can signup for one [here](https://github.com/join?source=header-home)
- An NVIDIA graphics card

{{< highlight bash >}}
$ sudo apt install git-all
{{< / highlight >}}

Once you have these, you're good to go!

## Install Sim
Now that we have the dependencies, lets get started! Note, we are assuming you have set up Docker to not need sudo with every call. You can set that up by following [these](https://docs.docker.com/install/linux/linux-postinstall/) steps. Now, open a terminal (CTRL + ALT + T on Linux) check if Docker is running:

### Test docker
{{< highlight bash >}}
$ docker run hello-world
{{< / highlight >}}

If you get the error:
```bash
docker: Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?.
See 'docker run --help'.
```

then run:

{{< highlight bash >}}
$ systemctl start docker
{{< / highlight >}}

### Create a workspace

If you do not already have a ROS workspace, you will need to create one.

**Note: This assumes you are using the workspace at `~/catkin_ws`. We highly recommend you stick to this workspace. If you chose not to, adjust the commands accordingly.**

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src && catkin_init_workspace
$ cd .. && catkin_make
```

### Get the simulator

Let's clone the sim repo:

{{< highlight bash >}}
$ cd ~/catkin_ws/src
$ git clone https://github.com/prl-mushr/mushr_sim
$ cd mushr_sim/docker/
{{< / highlight >}}

In the this directory, there is a file `setup.sh`. This will create a `Dockerfile` and `.env` file needed to run the simulator. All you have to do is run:

```bash
$ ./setup.bash
```

And your `docker` files should be configured. At the [bottom](#setup-sh) will be a section describing what was done. This will hopefully be helpful to those who 

## Run the Sim
To start the sim run:

{{< highlight bash >}}
$ docker-compose up -d
{{< / highlight >}}

You should see a small gray window pop up and rviz with the car model (see below). We've just created a container that you can see if you run: 

{{< highlight bash >}}
$ docker ps
{{< / highlight >}}

{{< figure src="/tutorials/quickstart/rviz_docker.png" caption="The rviz window that should pop up after running `docker-compose`." width="800">}}

Give the car an initial position by clicking `2D Pose Estimate` in rviz and clicking and dragging in the main window. Now you can click on the small gray window and use the WASD keys to drive the car around!

**You've just got your start in robotics!**

## Going Further
Driving the car around is fun, but what if you want to program it? In order to do that you must enter the container and write code like you would on a normal ROS linux system. The remaining tutorials that use the simulator assume you are in the container. To enter the container while everything else is running, run:

{{< highlight bash >}}
$ docker exec -it CONTAINER-ID bash
{{< / highlight >}}

You can get the `CONTAINER-ID` from:

{{< highlight bash >}}
$ docker ps
{{< / highlight >}}

This gets you a bash shell inside the container. You will find all the sim code in `~/catkin_ws/src/mushr_sim`. The user developer will not have root privileges, so you cannot install software. To enter the container as root run:

{{< highlight bash >}}
$ docker exec -it -u 0 CONTAINER-ID bash
{{< / highlight >}}

By being root you can install additional software that you may want.

You can also separate launching the container from launching the sim. To do that edit line 16 in `docker-compose.yml` to `entrypoint: bash`. Then run: 

{{< highlight bash >}}
$ docker-compose build
$ docker-compose up -d
{{< / highlight >}}

Then enter the container

{{< highlight bash >}}
$ docker exec -it CONTAINER-ID bash
{{< / highlight >}}

And source your workspace.

{{< highlight bash >}}
$ . ~/.bashrc
{{< / highlight >}}

 You can start the sim with:

{{< highlight bash >}}
$ roslaunch mushr_sim teleop.launch
{{< / highlight >}}

And start rviz with:

{{< highlight bash >}}
$ rviz
{{< / highlight >}}

To learn about programming the car continue to the [Intro to ROS Tutorial](/tutorials/intro-to-ros)

## EXTRA: Setup.sh

### Dockerfile
In order to get `docker` and the `rviz` GUI to work, there are two configuration changes we need to make. `setup.sh` tries to do these steps automatically. But if your system is not as the script expects, you may have to manually do these steps.

`setup.sh` creates a few files, so you should manually create them:
```bash
$ cp Dockerfile.template Dockerfile
$ cp .env.template .env
```

First let's change the uid/gid of your Docker user to match the current user. This is required for GUI apps (like `rviz`) to connect. Check your UID/GID with the following command:

{{< highlight text >}}
$ id
uid=1003(...) gid=1003(...) groups=1003(...),...
{{< / highlight >}}

In this case, the `uid` and `gid` are 1003. Now that you know these values use your favorite text editor to change line 4 in Dockerfile. You will be changing `{UID}` and `{GID}` to be your `uid` and `gid`. We will use gedit here:

{{< highlight bash >}}
$ gedit Dockerfile
{{< / highlight >}}

### NVIDIA Driver
The last configuration we need to do is make sure your NVIDIA driver matches the one the simulator is looking for. This is required because OpenGL is needed for `rviz`. To see which NVIDIA driver your computer has run:

{{< highlight bash >}}
$ ls /usr/lib/ | grep nvidia
libnvidia-gtk2.so.418.39
libnvidia-gtk3.so.418.39
libvdpau_nvidia.so
nvidia
nvidia-384
nvidia-384-prime
{{< / highlight >}}

The default is `nvidia-390`. But you'll notice above, the version is 384 (`nvidia-384`). Open `.env` and change `NVIDIA={NVIDIA_DRIVER}` to `NVIDIA=384`, the number you found. If you have multiple, use the most recent version.

{{< highlight bash >}}
$ gedit .env
{{< / highlight >}}

That's it for setting up! We're done the hard part :)
