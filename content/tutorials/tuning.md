---
title: "Tuning Your Car"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
duration: 60
difficulty: Beginner
summary: Run the MuSHR platform on your machine!
weight: 1
---

### Introduction
This guide will provide you with the steps needed to tune the parameters of the VESC. Tuning is a necessary part of your workflow with the car, as it ensures the Odometry from corresponds to accurate ....

What are some applications this is helpful for:
- Controllers: when our controller commands the car to drive a certain speed with a specific steering angle, we want that to correspond as much as possible to the real world.
- Mapping: When we map out our enivornment, the mapping algoritm will use the odometery (where the car has been [get intuitive explanation]) to build the map.

Comment about how this tuning won't be perfect, the dynamics of the car are very complicated, and we are using a simplified model. Our goal will be to design algorithms that can deal with these uncertainties and imperfections in tuning.

### Goal 
Tune the car's VESC parameters

### Requirements

- A computer that can `ssh` into your car.
- A tape measure.

## Setup

Find a relatively open space to run your car. We'll be driving it straight for ~6ft (3m) and turning in a semi-circle of diameter ~3ft (1.5m).

Boot up the car and `ssh` into it.

For every value, give a ballpark region for the value to be in, so they know if they are on the right track.

Tune the car at the speed you plan to drive it at. This will allow you to get more acurate gains at that speed (as the gains may not scale linearly with speed (although we make this assumption)).

## Steering Angle Offset

Give an introduction on what this parameter is.

### How to Tune
Point to the specific line in the vesc.yaml
Create three GIFS from vidoes of the car running, one where it's veering left (low offset), one where it's going perfectly straight ("just right" offset), and one where it's veering right (high offset)

psuedocode for process:
```
while not tuned:
    run teleop
    drive car in a straight line a few times (sometimes it doesn't go perfectly straight after turning, so as long as it goes straight most of the time).
    adjust steering_angle_offset in vesc.yaml (go higher if it veers too much left, lower if it veers too much right)
    stop teleop
```

**Note: Usually this value is between 0.45 and 0.55**

## Speed to ERPM Gain

Introduction/what is this parameter for

### How to Tune

psuedocode:
```
extend tape measure to around 7-8 ft on the floor.
open a terminal on the car and run the command
rostopic echo /vesc/odom/ (this will echo all the odometry information from the car.)
  you won't see anything yet (as we haven't started teleop)

while not tuned:
    place car at the base of the tape measure with the back wheelbase lined up with 0 (refer to picture).
    start teleop (note the position (pose/pose/position) is all zeros. The odometry postion starts at (0, 0, 0) when the car starts, and advances based on driving commands.)
    drive the car forward about 6 ft.
    record the distance traveled. (if your tape measure is in inches, convert to meters)
    compare to output of the rostopic echo command. If the reported distance traveled is larger than the actual, decrease the gain, if the reported distance is smaller, increase the gain.
    stop teleop
```

A good stopping criteria is when the reported distance is within 2-3 inches (0.05-0.076m) of the actual distance.

**Note: This value can vary, but it should be on the order of thousands (2000-5000)**

## Steering Angle Gain

Introduction to the parameter

Get a picture of the kinematic car model, and show how when the steering angle is at 0.34, the turning radius of the car will be a function of the speed. From this, the diameter of the turn can be computed, we'll want to make sure our car actually travels this diameter

### How to Tune

psuedocode:
```
extend tape measure to around 3-4 ft on the floor.

while not tuned:
    place car perpendicular (see picture) to the tape measure with the back wheelbase lined up with the tape measure (refer to picture).
    start teleop
    command the max steering angle to whichever side the tape measure is on, drive the car forward until the back wheelbase intersects with the tape measure.
    record the distance traveled. (if your tape measure is in inches, convert to meters)
    compare to output of the rostopic echo command. If the actual distance is larger than it should be, decrease the gain. If the actual distance is larger than it should beincrease the gain.
    stop teleop
```
