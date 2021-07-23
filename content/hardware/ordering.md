---
title: "Choosing and Ordering a MuSHR v4 Car"
date: 2018-11-28T15:14:54+10:00
intro_image: imgs/single-car.jpg
image: "/services/default.png"
featured: true
draft: false
active: true
summary: Pick and Order your own MuSHR Racecar!
weight: 1
---

This section explains the MuSHR v4 tier system, allowing you to order and build a version of the MuSHR car that best meets your needs.

___
## Table of Contents

1. [System Overview](#system-overview)
2. [Add-on Modules](#add-on-modules)
3. [Recommended Tier System](#recommended-tier-system)
4. [Custom MuSHR tiers](#custom-mushr-tiers)
5. [3D Printing](#3d-printing-files)

___
## System Overview

This section describes the high level system architecture of the MuSHR racecar and your available options to build the car's hardware. Check out the [build tutorial](/hardware/build_instructions) to get an idea of what building the car is like!

**Processor**

The MuSHR Racecar can be built with a choice of two main processors, the Nvidia Jetson Nano or the Nvidia Jetson Xavier NX. The latter is better for high-performance applications, while the former is better if you're working on a budget. You can view a comparison of the two [here.](https://www.seeedstudio.com/blog/2020/06/04/nvidia-jetson-nano-and-jetson-xavier-nx-comparison-specifications-benchmarking-container-demos-and-custom-model-inference/)

{{< figure src="/hardware/ordering/xavier.png" caption="The Xavier NX processor inside a MuSHR racecar." width="800">}}

**Sensing**

MuSHR supports the Intel RealSense D435/D435i and the D455 cameras for depth perception. The car also has a scanning YDLIDAR X4, and other optional add-on sensors listed below. View a comparison of the different Intel RealSense cameras [here.](https://www.intelrealsense.com/compare-depth-cameras/)

    {{< figure src="/hardware/build_instructions/14_06_car_complete.jpg" caption="A complete MuSHR racecar with a full sensor suite." width="800">}}

**Chassis**

The base chassis of the car is a Redcat Racing Rally Car with two motor options, modified extensively with 3D printed parts. The Pro chassis is more expensive, but follows a simpler build process, not requiring motor replacement, saving hours of build time.

___
## Add-on Modules 

You can also pick add-on modules to supplement your car and augment its capabilities to perform certain functions.

**Push Button**

A button mounted to the front of the car, allowing you to detect if the front bumper is in contact with an object or surface.

    {{< figure src="/hardware/build_instructions/08_01_position_button.png" caption="Push Button" width="800">}}

**T265 Tracking Camera**

The MuSHR Racecar can have a second camera mounted to it, the Intel RealSense T265. This camera is particularly useful for tracking.

    {{< figure src="/hardware/build_instructions/13_05_guide_usb_double.JPG" caption="T265 Tracking Camera" width="800">}}

**Radio FPV**

If you need to tele-operate outdoors or over a longer distance outside standard Bluetooth/WiFi range, you can follow the steps in [this tutorial](/tutorials/radio_fpv) to add a radio-based teleoperation system to your racecar.

{{< figure src="/tutorials/radio_fpv/fpv_camera.jpg" caption="Radio FPV Camera" width="800">}}


**Pushing Bumper**

You can mount a 3D-Printed bumper to the front of the car to perform object manipulation tasks with the car.

{{< figure src="/hardware/ordering/pushing_bumper.jpg" caption="MuSHR racecar with front-mounted pushing bumper." width="800">}}

___
## Recommended Tier System

As a result of the various options available for MuSHR, we have simplified the MuSHR Racecar into three different tiers, illustrated in the table below. These tiers are a recommended guideline to follow, and it is still possible to build a MuSHR racecar with any combination of processor, camera, and chassis.

|Tier|MuSHR Lite|MuSHR|MuSHR Pro|
|---|---|---|---|
|Processor|Nano|Nano|Xavier NX|
|Camera|None|D435|D455|
|Chassis|Base|Base|Pro|
|Cost(in US$)|750|1000|1600|

<br></br>
The MuSHR Lite is ideal for users who are working on a budget, like hobbyists, and the MuSHR Pro is ideal for researchers and academic institutions, who need higher performance and are willing to splurge. The MuSHR tier meets in the middle, with full sensing capabilities but lower performance than the Pro option. If you feel like you don't fit into any of these three categories, don't worry! We have you covered in the next section!

View the BoM for each tier [here.](https://docs.google.com/spreadsheets/d/1VV5fGFJIEthfwishO181WcWzMAkPAFYT/edit#gid=598654221)
___
## Custom MuSHR tiers

It is possible to combine components between multiple tiers, those listed above are just our recommended guidelines. If you are a more advanced user with exact specifications in mind, MuSHR v4's modularity is designed to help support that. Download a more complex BoM that allows you to pick and choose [here.](https://docs.google.com/spreadsheets/d/1Nad3odSet0OVKBTEJDXue9xp8m6JX1xA/edit?usp=drive_web&ouid=117747822978217793476&rtpof=true)

For example, teaching a large class that needs to put together a large number of cars inexpensively and quickly? You can still build a MuSHR Lite with a Pro chassis! Want to use the Xavier NX but you don't need a camera? That's possible too!

___
## 3D Printing Files

Print the parts to build the MuSHR Racecar [here](https://github.com/prl-mushr/mushr_cad/tree/master/v3/stl) If you would like to modify the logo on the sides of the robot, follow [these instructions](/hardware/logo_modification) to generate new .stl files. The same printed parts are required for any supported MuSHR configuration.

One of each STL is required to be printed for the racecar except for the `racecar_cover_image.stl` which needs to be printed twice. We also recommend that `racecar_cover_text.stl` , `racecar_cover_number.stl` and `racecar_cover_image.stl` be printed in a different color to the main racecar body.

`racecar_front_cover_center_t265_plate.stl` is only required if the T265 tracking camera is being used.

___
## Build your Car

To summarise, here's what you should do to get started on building the racecar.

1. Select a Bill-of-Materials that meets your needs.
2. Print the 3D-printed parts for the upper portion of the racecar.
3. Follow the [build instructions](/hardware/build_instructions) to build your MuSHR racecar!
