---
title: "Teleoperating over Radio"
date: 2018-11-28T15:14:54+10:00
featured: false
draft: false
active: true
duration: 20 
difficulty: Beginner 
summary: Use the Bitcraze CrazyRadio dongle to teleoperate the car over radio.
weight: 2
---

<h2> By: Akkshaj Singh</h2>

### Introduction
This tutorial will help you integrate a radio-based system for teleoperation.

### Goal 
To allow you to drive the car over radio.

### Requirements
  - Complete the [hardware](/hardware/build_instructions) setup with your car
  - Complete the [quickstart](/tutorials/quickstart) tutorial. (Required for rviz)
  - Complete the [first_steps](/tutorials/first_steps/) tutorial.
  - 1 MuSHR racecar.
  - 2 [CrazyRadio USB transceivers](https://www.bitcraze.io/products/crazyradio-pa/).
  - FPV Camera Hardware (camera, transmitter, antenna, receiver, capture card, heatsink) - optional
  - FPV 3D Printed Mount - optional
  - A desktop/laptop computer that can ssh into the car.
  - An SSH-capable text editor, like Vim or Visual Studio Code (requires the SSH plugin)


## Setting up CrazyRadio

<br>
{{< figure src="/tutorials/radio_fpv/crazyradio.jpg">}}
<br>

Plug one CrazyRadio USB transceiver into the jetson on the racecar, and one into your computer. Download the code from [this GitHub repository](https://github.com/prl-mushr/mushr-radio-controller) onto both the jetson and the computer. Connect the dualshock controller to your teleoperating laptop over Bluetooth or USB.

Run `tx.py` on the transmitting laptop, and run `rx.py` on the Jetson. Verify that joystick input appears on your screen and is being sent to the car with the acknowledgements.

## Teleoperation

Launch teleop as per normal on the jetson by SSH-ing into the car. (ensure the python scripts from the previous step are still running on both the PC & the racecar). The controller should now be able to drive the car. This system should have a much higher range when compared to the traditional Bluetooth teleoperation system, and will continue to work outside WiFi or Bluetooth range, provided that line-of-sight is maintained. This may be particularly advantageous in dynamic outdoors environments.

## Using non-standard controllers

Currently, the GitHub Repository supports DualShock 4 and XInput based controllers. To use other input devices, adjust the device capabilities accordingly on `rx.py` using [evdev](https://python-evdev.readthedocs.io/en/latest/).

## Pairing this system with an FPV camera

{{< figure src="/tutorials/radio_fpv/fpv_camera.jpg">}}

If you would like to view a live FPV camera feed, we recommend integrating a Radio FPV camera, like those used on drone racing. You can power it with the Jetson, and connect a radio receiver to your computer to view the feed. An included STL 3D printing file that allows you to attach a FPV camera is included in the `mushr-radio-controller` repository, and is mounted in an identical fashion to the push button at the front of the car.