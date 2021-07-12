---
title: "Upgrading MuSHR v3 to v4"
date: 2018-11-28T15:14:54+10:00
intro_image: imgs/single-car.jpg
featured: false
draft: false
active: true
summary: Replace your NVIDIA Jetson Nano with the new Xavier NX, and your D435i with a D455.
weight: 3
---

While MuSHR v4 adds no new major structural changes, this page provides instructions to replace your MuSHR racecar's NVIDIA Jetson Nano with the more powerful Xavier NX processor, as well as upgrade your Intel RealSense camera.
## Prerequisites

1. NVIDIA Xavier NX
2. 4x 20mm M2.5 pan head screws
3. 36x M2.5 spacers
4. 4x M2.5 Hex Nuts
5. SD card flashed with [NVIDIA Jetson Xavier NX stock image](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit#write)
6. Internet connection for Xavier NX (either WiFi or ethernet)
7. HDMI cable, mouse, keyboard

If upgrading the camera:

1. [New Front Cover](https://github.com/prl-mushr/mushr_cad/blob/master/v3/stl/racecar_front_cover_whole.stl)
2. 2x M4 screws (14mm if using T265 and 8mm if only D455)
3. USB right-angled vertical connector, like [this](https://www.amazon.com/Cellularize-Adapter-10Gbps-Extension-Nintendo/dp/B08CDGHQWF/)

## Removing the Jetson Nano

1. Unscrew the 4 M2.5 flat-head screws that hold the back section of the MuSHR racecar in place. 

2. Unplug all USB cables from the Nano.

3. Unscrew the four screws that hold the Jetson Nano in place, and carefully remove it from the racecar chassis. Ensure the nuts placed inside the Back Cover Top do not fall out.

## Updating the Power System

1. Cut the previously connected battery connector from the wire attached to the buck converter. Ensure the converter is powered off.

2. Take two battery connectors, and connect the black wire of one of the connectors to the red wire of the other connector, with the other red and black wire connected to the length of wire attached to the buck converter. This will allow you to connect two batteries in series in order to provide the higher required voltage for the Xavier NX. See steps 2-13 of the [VESC Preparation section of the build tutorial](/hardware/build_tutorial#vesc-preparation) for a refresher on how to connect the wires if required. The image below shows a completed Xavier NX battery connector.

{{< figure src="/hardware/upgrading/xavier_connector.png" caption="Xavier NX battery connector" width="800">}}

3. Change the output voltage setting on the buck converter from 5V to 14V.

## Installing the Xavier NX

1. Remove the plastic carrier board by removing the four black screws on the corners of the developer kit. 

2. Gently bend the the plastic carrier board such that the Wi-Fi antennae pop out of their enclosure. 

3. Insert the SD Card flashed with the stock image.

4. Take 4 20mm M2.5 screws, and place three spacers on each of the screws. Place the screws through the mounting holes on the Xavier NX and put six spacers below the Xavier NX board. Attach a hex nut to each screw, so it lightly holds the spacers in place against the board, as seen below. 

    {{< figure src="/hardware/upgrading/xavier_top.jpg" caption="Xavier NX (as seen from above)" width="800">}}

    {{< figure src="/hardware/upgrading/xavier_side.jpg" caption="Xavier NX (as seen closely from the side)" width="800">}}

5. Screw the Xavier NX into the Back Cover, in an identical manner to the Nano. 

## Software Setup

Follow the steps in the [Software Setup](/hardware/build_instructions#software-setup) section of the build instructions to set-up the Xavier NX with the MuSHR stack.

## Upgrading the RealSense Camera

If you want to upgrade your D435i to a D455, you will need to re-print the [racecar front cover](https://github.com/prl-mushr/mushr_cad/blob/master/v3/stl/racecar_front_cover_whole.stl). 

1. Remove your old front cover by unscrewing the 5 flat-head screws holding it in place.

2. Mount the D455 to the new front cover with two M4 screws and connect it with a USB-C cable and right-angle connector.

3. Mount the new front cover with 5 flat-head M2.5 screws, the same way the old front cover was mounted.