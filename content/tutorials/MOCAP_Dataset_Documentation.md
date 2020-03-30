
##  Data Collection & Instrument Description 

This dataset is collected using the [MuSHR](mushr.io) race cars made by the Personal Robotics Lab in the Computer Science Department at the University of Washington. The dataset collected and processed by researchers and students in the University of Washington. 

## Introduction

This dataset contains more than 30 minutes(~110GB) of synchronized data from a variety of sensor data including images taken from 3 [MuSHR](mushr.io) race cars driving around in a closed lab environment. 

### Goal 
This documentation will help you familiarize with the organization and content of the datasat as well as show you how to get started on using this dataset for your own projects.

### Purpose of Dataset 

This purpose of this dataset is to provide a well organized dataset that facilitates multi-purpose Machine Learning/Deep Learning research in fields in robotics, computer vision, and self-driving autonomous vehicles under a low-cost/low-stake environment.

## Data Format 

The format of the dataset is one synchronized .bag file that contains sensor information from all three cars clearly mapped to a ground truth label through their timestamp.


### Data Organization 

The data in the .bag file is organized by data from each car using ROS topics. ROS topics contains ROS messages with can be of different message types depending on the nature of the data being collected.

>E.g. sensor_msgs/LaserScan, geometry_msgs/PoseStamped, etc.

### Naming Convention

The ROS topics are named mostly what what data they contain with additional label at the beginning to specify the car that it belongs to. Since there are three cars, the names of the topics for each car always starts with "/car24/..", /car25/...",  or "/car26/..." respectively. 

For example pose messages for car 24 would can be echoed through

    rostopic echo /car24/PoseStamped

### List of Topics 

View the list of topics  

    rosbag info MOCAP_Dataset.bag

### Replaying the Dataset and Visualizing topics 

Replay the bag again 

    rosbag play MOCAP_Dataset.bag

Open rviz to visualize the RGBD images, car poses, laser scans, etc.

    rosrun rviz rivz

Add the topics you would like to visualize. 

### Data 

Not all of the data in side the dataset will be of use to you. The data contained encompasses most of the essential on-car sensors for each of the race cars including, Laser Scanner(YDLiDAR), RGB-D images, car pose, odometry, IMU, VESC state, etc. The full list can be viewed again with 

    rosbag info MOCAP_Dataset.bag


## Using the Dataset

Depending on what the use for the dataset is, different information in the dataset will be useful. For example, a pose estimation project based on RGB-D images would only need the PoseStamped messages and camera messages for each race car. Similarly a pose estimation project based on laser scans would not need the large RGB-D images. 

### Extracting from the large original dataset to produce your own dataset 

To take the important information and filter out useless data, you need to go through the original dataset and create a new dataset in the form or another bag file, csv file, etc. 

Here we have 2 pieces of simple code to extract desired topics into a new bag file(pose and some image info in these examples). 

The first one is for if you only want a few topics from the original bag file. 

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
				
If you want most information from the original dataset and just don't want some specific information such as `/camera/color/image_throttled` you can use this.

	from rosbag import Bag 
	import rospy  
	
	bagOut = 'Name_of_your_new_bag'
	# Topics you want to filter Out
	topicOne = '/camera/color/image_throttled'
	
	with Bag(bagOut, 'w') as bagNew:
		for topic, msg, t in Bag('MOCAP_Dataset.bag'):
			if (topic != '/car24'+topicOne) or (topic != '/car25'+topicOne) or (topic != '/car26'+topicOne):
				bagNew.write(topic, msg, t)
			

Now you can extract any kind of data you want to create your own dataset!

## Data Remarks

### States of the race cars 

- In Motion
	- Mostly Random Motion 
- Stationary 
	- Stuck/ran into a wall
	- Collided with another car(s)
	- Stopped itself

The cars are not always moving, sometimes they get stuck, run into each other, or just simply stops for a rest. You can get the timestamp of these states in a few ways:
1. Plotting the x, y positions over the timestamp of the desired car and seeing when x, y is stable.  
2. Playing the dataset and using rviz to visualize the camera topics or PoseStamped topics will allow you to determine whether a car ran into another car, got stuck, or simply stopped itself. 









