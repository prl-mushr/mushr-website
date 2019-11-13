---
title: "Tuning VESC Parameters"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: false
draft: false
active: true
duration: 60
difficulty: Beginner
summary: Run the MuSHR platform on your machine!
weight: 4
---

### Introduction
If the VESC maps input velocities (m/s) and steering angles (rads) to Electrical RPMs and servo positions. These conversions are not necessarily the same for every car, as there are many physical factors that contribute to differences.  This guide will provide you with the steps needed to tune the parameters of the VESC.

If incorrectly tuned, the commands applied by your controller may not correspond to what's being executed on the physical hardware, making it difficult to debug issues with the controller.

**Note:** The conversion from velocity to ERPM and steering angle to servo position is modeled as a linear function of the inputs, so the values may not be acurate for large ranges of speeds. Try to tune around the speed you expect to drive at (and consider retuing if your speed regime changes drasically).

### Goal
Set the VESC gains (multiplier/strength of an input) and offsets (defualt value when no control is applied).

### Requirements

- A computer that can `ssh` into your car.
- A tape measure.

## Setup

Find a relatively open space to run your car. We'll be driving it straight for ~9ft (3m) and turning in a semi-circle of diameter ~3ft (1m).

Plug the batteries into the car, connect to the car's network and `ssh` into the car.

The file will be located at `~/catkin_ws/src/mushr_base/vesc/vesc_main/config/racecar-uw-nano/vesc.yaml`. If you have a different version of the car, you'll choose that one instead. If you don't know which car you have, you likely have the `racecar-uw-nano` version.

The file will look something like this:

```yaml
# erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
#-4614
speed_to_erpm_gain: -2000
speed_to_erpm_offset: 0.0

[...omited for brevity]

# servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
steering_angle_to_servo_gain: 1.2135
steering_angle_to_servo_offset: 0.55

[...omited for brevity]

vesc_driver:
  port: /dev/vesc
  [...omited for brevity]
```

## Steering Angle Offset

This offset sets the default servo position when the car is driving straight.

### How to Tune
The number you will be changing is associated with `steering_angle_to_servo_offset`.

While the car doesn't drive straight, do the following procedure:

1. Start teleop (`roslaunch mushr_base teleop.launch`)
-  Drive car in a straight line a few times. It's never going to be perfectly straight, so as long as it goes straight most of the time, it'll be fine. You can always come back to retune if it's not sufficiently precise.
-  Adjust `steering_angle_offset` in `vesc.yaml`. Increase the offset if the car veers too much left, decrease if it veers too much right.
-  Stop teleop (`Ctrl-C` in the window you started `teleop.launch` in) and go back to Step 1.

**Note: Usually this value is between 0.4 and 0.6**

<figure>
  <img src="/tutorials/tuning/steering_angle_offset/left.gif" style="width: 32%;" />
  <img src="/tutorials/tuning/steering_angle_offset/straight.gif" style="width: 32%;" />
  <img src="/tutorials/tuning/steering_angle_offset/right.gif" style="width: 32%;" />

  <figcaption>
	<p>The car during the tuning process. In all tests, the car is commanded to drive straight forward. In the left image, the car's steering angle offset is too high. In the center image, the offset is good. In the right image the offset is too low. <b>Note: the car will never drive perfectly straight. It just has to be good enough. In the future your controller will be able to make small corrections!</b></p>
  </figcaption>
</figure>


## Speed to ERPM Gain

This gain converts velocity to ERPM.

### How to Tune

<span style="color:red">Point to the specific line in the vesc.yaml</span>

Before starting to tune:
1. Extend your tape measure to around 9-10 ft on the floor. It's okay if it's slightly less than this.
{{<figure src="/tutorials/tuning/erpm_gain/start.jpg" width="300px">}}

Now, while the car does not drive the reported distance (by `rostopic echo` command):

1. Place car at the base of the tape measure with the back wheelbase (indicated with a white line) lined up with 0.
{{<figure src="/tutorials/tuning/erpm_gain/base_with_line.jpg" width="300px">}}
-  Start teleop. Note the position (pose/pose/position) is all zeros. The odometry postion starts at (0, 0, 0), and when the car starts, and advances based on driving commands.
-  Open a terminal on the car and run the command: `rostopic echo /vesc/odom/pose/pose/position/x`. This will echo all the odometry information -- how far the car has driven in the `x` direction since teleop started. The value should be `0.0` at the start, as the car hasn't moved yet.
-  Drive the car forward about 7-8 ft. The car will drive slightly further as it decelerates and stops. Make sure you only drive forward, not altering the servo position otherwise you'll have both `x` and `y` directional changes (which makes it only slightly harder to check distance traveled).
-  Record the distance traveled. If your tape measure is in inches, convert to meters.
{{<figure src="/tutorials/tuning/erpm_gain/end-with-line.jpg" width="400px">}}
-  Compare to output of the `rostopic echo` command's `x` value. If the reported distance traveled is larger than the actual, decrease the gain. If the reported distance is smaller, increase the gain. At the begining increasing or decreasing by 500 should allow you to quickly hone in on the value.
-  Stop teleop. Go back to step 1 if the values are not sufficiently close (within 2-3 cm).


**Note: This value can vary, but it should be on the order of thousands (2000-5000)**

A good stopping criteria is when the reported distance is within 2-3 inches (0.05-0.076m) of the actual distance. Sometimes the servo will get stuck and veer off course. You can restart teleop and move the car back and try again if this happens. Remember we are looking to get "good" gains, not perfect gains.

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

## Conclusion
With these values, your car should conform to your applied commands much more accurately.
