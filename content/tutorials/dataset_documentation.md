---
title: "Dataset Documentation"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
difficulty: Beginner
duration: 30
summary: Familiarizing and learning to use the dataset for your own projects
weight: 2
---
## Introduction

This dataset contains more than 30 minutes(~110GB) of synchronized data from a variety of sensor data including images taken from 3 [MuSHR](mushr.io) race cars driving around in a closed lab environment. The purpose of this dataset is to provide a well organized dataset that facilitates multi-purpose Machine Learning/Deep Learning research in fields in robotics, computer vision, and self-driving autonomous vehicles under a low-cost/low-stake environment.

### Goal 
This documentation will help you familiarize with the organization and content of the datasat as well as show you how to get started on using this dataset for your own projects.


## Data Format 

The format of the dataset is one synchronized .bag file that contains sensor information from all three cars clearly mapped to a ground truth label through their timestamp. To get a quick overview of the dataset go to terminal and enter

```bash
$ rosbag info MOCAP_Dataset.bag
```

### Data Organization 

The data in the .bag file is organized like any other .bag file using its ROS topics. Here we don't need to care that much about the organization, all we need to know is that data is organized by topics and each topic is a type of data. For example laser scan data would be in "sensor_msgs/LaserScan"

### Naming Convention

The names of the ROS topics are self-explanatory with the topic name representing what type of data it contains. However since there are data for each of the three cars, the topics will be additionally labelled at the start with the name of the car it belongs(e.g. "/car24/..", /car25/...",  or "/car26/...")

For example pose messages for car 24 
```bash
$ rostopic echo /car24/PoseStamped
```

## Replaying the Dataset and Visualizing topics 

With .bag files you can replay them again 
```bash
$ rosbag play MOCAP_Dataset.bag
```
You can also open rviz to visualize various topics such as RGBD images, car poses, laser scans, etc.
```bash
$ rosrun rviz rivz
```
Add the topics you would like to visualize. 

## Using the Dataset

Not all of the data in side the dataset will be of use to you. The data contained encompasses most of the essential on-car sensors for each of the race cars including, Laser Scanner(YDLiDAR), RGB-D images, car pose, odometry, IMU, VESC state, etc. Depending on what the use for the dataset is, different information in the dataset will be useful. For example, a pose estimation project based on RGB-D images would only need the PoseStamped messages and camera messages for each race car. Similarly a pose estimation project based on laser scans would not need the large RGB-D images. 

### Extracting from the large original dataset to produce your own dataset 

To take the important information, you need to go through the original dataset and create a new dataset in the form or another bag file, csv file, etc. 

Here we have 2 pieces of simple code to extract desired topics into a new bag file

The first one is when you are extracting from a few topics from the original bag file. Here we are extracting PoseStamped messages and camera/color/image_throtted messages from all three cars and creating a new dataset containing only these messages.
```
from rosbag import Bag 
import rospy  
    
bagOut = 'Name_of_your_new_bag'
# Desired topics
topicOne = '/PoseStamped'
topicTwo = '/camera/color/image_throttled'
    
with Bag(bagOut, 'w') as bagNew:
    for topic, msg, t in Bag('MOCAP_Dataset.bag'):
        if (topic == '/car24'+topicOne) or (topic == '/car25'+topicOne) or (topic == '/car26'+topicOne):
            bagNew.write(topic, msg, t)
        if (topic == '/car24'+topicTwo) or (topic == '/car25'+topicTwo) or (topic == '/car26'+topicTwo):
            bagNew.write(topic, msg, t)
```               

The second example below is when you want most information from the original dataset and just don't want some specific information such as `/camera/color/image_throttled` you can use this to filter out unwanted data. Here we don't want the camera/color/image_throttled messages in our new dataset so we filter it out.
```
from rosbag import Bag 
import rospy  
    
bagOut = 'Name_of_your_new_bag'
# Topics you want to filter Out
topicOne = '/camera/color/image_throttled'
    
with Bag(bagOut, 'w') as bagNew:
    for topic, msg, t in Bag('MOCAP_Dataset.bag'):
        if (topic != '/car24'+topicOne) or (topic != '/car25'+topicOne) or (topic != '/car26'+topicOne):
            bagNew.write(topic, msg, t)
```           

Now you can extract any kind of data you want to create your own dataset!

## Data Remarks

### States of the race cars 

It is important to keep in mind that the cars are not always moving, sometimes they get stuck, run into each other, or just simply stops for a rest. 

- In Motion
    - Mostly Random Motion 
- Stationary 
    - Stuck/ran into a wall
    - Collided with another car(s)
    - Stopped itself

You can get the timestamp of these states if you want for your purposes in a few ways:

1. Plotting the x, y positions over the timestamp of the desired car and seeing when x, y is stable.  
2. Playing the dataset and using rviz to visualize the camera topics or PoseStamped topics will allow you to determine whether a car ran into another car, got stuck, or simply stopped itself. 

