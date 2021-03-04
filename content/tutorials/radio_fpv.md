---
title: "Teleoperating over Radio"
date: 2018-11-28T15:14:54+10:00
featured: false
draft: false
active: true
duration: 45 
difficulty: Beginner 
summary: Use the Bitcraze CrazyRadio dongle to teleoperate the car over radio.
weight: 3
---

<h2> By: Akkshaj Singh</h2>

<br>
{{< figure src="/tutorials/radio_fpv/crazyradio.jpg">}}
<br>

### Introduction
This tutorial will help you integrate a radio-based system for teleoperation.

### Goal 
To allow you to drive the car over radio.

### Requirements
  - Complete the [hardware](/hardware/build_instructions) setup with your car
  - Complete the [quickstart](/tutorials/quickstart) tutorial. (Required for rviz)
  - Complete the [first_steps](/tutorials/first_steps/) tutorial.
  - 2 [CrazyRadio USB transceivers](https://www.bitcraze.io/products/crazyradio-pa/).
  - [FPV Camera Hardware](https://www.getfpv.com/fpv/cameras.html) (camera, transmitter, receiver)
  - [FPV Camera 3D Printed Mount](https://github.com/prl-mushr/mushr-radio-controller/blob/main/FPV%20Camera%20Mount.stl)
  - A desktop/laptop computer that can ssh into the car.
  - An SSH-capable text editor, like Vim or Visual Studio Code (requires the SSH plugin)


## Setting up CrazyRadio

Plug one CrazyRadio USB transceiver into the jetson on the racecar, and one into your computer. Download the code from [this GitHub repository](https://github.com/prl-mushr/mushr-radio-controller) onto both the jetson and the computer. Connect the dualshock controller to your teleoperating laptop over Bluetooth or USB.

{{< highlight bash >}} $ sudo python tx.py {{< / highlight >}}
{{< highlight bash >}} $ sudo python rx.py {{< / highlight >}}

Run `tx.py` on the transmitting laptop, and run `rx.py` on the Jetson. Verify that joystick input appears on your screen and is being sent to the car with the acknowledgements.

## Teleoperation

{{< highlight bash >}}
$ ssh <user>@<car ip> 
{{< / highlight >}}

{{< highlight bash >}}
$ roslaunch racecar teleop.launch 
{{< / highlight >}}

Launch teleop as per normal on the jetson by SSH-ing into the car. (ensure the python scripts from the previous step are still running on both the PC & the racecar). The controller should now be able to drive the car. This system should have a much higher range when compared to the traditional Bluetooth teleoperation system, and will continue to work outside WiFi or Bluetooth range, provided that line-of-sight is maintained. This may be particularly advantageous in dynamic outdoors environments.

## Pairing this system with an FPV camera

{{< figure src="/tutorials/radio_fpv/fpv_camera.jpg">}}

Additionally, we recommend integrating a [Radio FPV camera](https://www.getfpv.com/fpv/cameras.html), like those used for drone racing. These cameras have high quality, range and low latency to allow you to monitor and tele-operate MuSHR at range. Depending on the specific camera model you choose, you will likely need a transmitter and antenna to attach with it. You can power it over USB from the Jetson, and connect a [5.8 GHz radio FPV receiver](https://www.amazon.com/s?k=fpv+receiver) to your computer to view the feed similar to an external webcam. This system is highly customizable with a wide range of compatible hardware, and you can pick what meets your needs and budget accordingly.

A STL 3D printing file that allows you to attach most 19mmx19mmx20mm FPV cameras with M2 screws is included in the `mushr-radio-controller` repository, and is mounted in an identical fashion to the push button at the front of the car.

## Going Further - Using non-standard controllers

Currently, the GitHub Repository supports DualShock 4 and XInput based controllers. To use other input devices, adjust the device capabilities accordingly on `rx.py` using [evdev](https://python-evdev.readthedocs.io/en/latest/).
