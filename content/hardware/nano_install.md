---
title: "Jetson Nano Driver Installation"
date: 2018-11-28T15:14:54+10:00
intro_image: imgs/single-car.jpg
image: "/services/default.png"
featured: false
draft: false
active: false
summary: Use a script to setup your SD card
weight: 2
---

This page provides instructions for installing libraries necessary for operating the MuSHR racecar. Before proceeding with these instructions, we first suggest trying our [Jetson Nano image](/hardware/build_instructions#sd-card-setup) that has everything described on this page already installed. However, if your Jetson Nano fails to boot when using our image, you can flash NVIDIA's stock Jetson Nano image to your SD card, and then follow these instructions to install the necessary libraries.

## Prerequisites

The following are required before continuing with installing the necessary libraries.

1. SD card flashed with [NVIDIA stock image](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write)
2. Internet connection for Jetson Nano (either WiFi or ethernet)
3. HDMI cable, mouse, keyboard

## Running Install Script

The following steps will setup the required libraries on the Jetson Nano.

1. Power on and boot the Jetson Nano. 
2. Follow the on-screen prompts to complete setup of the Jetson Nano operating system. If you would like your setup to be consistent with our tutorials, we suggest using the following values: <br/>
**Username: robot**<br/>
**Name: robot**<br/>
**Computer Name: goose**<br/>
3. Download the install script:
    {{< highlight bash >}}
    wget https://raw.githubusercontent.com/prl-mushr/mushr/master/mushr_utils/install_scripts/mushr_install.sh
    {{< / highlight >}} <br/>
4. Make the install script executable:
    {{< highlight bash >}}
    chmod +x mushr_install.sh
    {{< / highlight >}} <br/>
5. Execute the script, following the prompts and entering the user password as needed:
    {{< highlight bash >}}
    ./mushr_install.sh
    {{< / highlight >}} <br/>
6. Plug in any realsense cameras and open the realsense viewer:
    {{< highlight bash >}}
    realsense-viewer
    {{< / highlight >}} <br/>
If the upper right hand corner of the GUI recommends a firmware update, download and install the updated firmware.

7. Open the file **~/catkin_ws/src/mushr/mushr_hardware/mushr_hardware/launch/sensors.launch**. Comment out the lines that launch **rs_camera.launch**, and add the following code as shown in Fig. 1.

    {{< highlight bash >}}
    <node pkg="timed_roslaunch" type="timed_roslaunch.sh"
          args="10 realsense2_camera rs_camera.launch camera:=/$(arg car_name)/camera tf_prefix:=/$(arg car_name)/camera"
          name="rs_camera_timed_roslaunch" output="screen" />
    {{< / highlight >}} <br/>
	{{< figure src="/hardware/nano_install/rs_camera_delayed_launch.png" caption="Fig. 1" width="800">}}

8. Setup an access point on the robot. To do this, first click on the Wi-Fi symbol in the upper right of the Ubuntu GUI and choose **Edit Connections**. Click the plus sign to create a new connection. On the **Choose a Connection Type** dialog, choose **Wi-Fi** from the drop down menu, and then click **Create...**. Enter the name **Robot AP** into the **Connection Name** field. Under the **Wi-Fi** tab, also enter **Robot AP** into the **SSID** field. Under the **Mode** field, choose **Hotspot**. Under the **BSSID** field, choose the last option (there may only be one option). Under the **Wi-Fi Security** tab, choose the **WPA and WPA2 Personal** option in the **Security** field. Then enter a password into the **Password** field. Then click **Save**. This setup is shown below in Fig. 2 and 3.

    {{< figure src="/hardware/nano_install/robot_ap_setup.png" caption="Fig. 2" width="800">}}  

    {{< figure src="/hardware/nano_install/robot_ap_pass.png" caption="Fig. 3" width="800">}}  

9. All of the necessary libraries should now be installed. Now continue finishing software setup [here](/hardware/build_instructions#software-setup).


