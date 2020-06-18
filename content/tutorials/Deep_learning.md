---
title: "Applying deep learning to the MuSHR project"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
difficulty: Moderate
duration: 60
summary: Using Imitation learning and Reinforcement learning for driving the MuSHR car.
weight: 2
---

## Introduction

Deep learning is part of a broader family of machine learning methods based on artificial neural networks with representation learning. Learning can be supervised, semi-supervised or unsupervised. In this tutorial, we'll see how to get started with applying imitation learning as well as reinforcement learning to the MuSHR car through the use of a modified simulator developed by the DIYRobcars community, named "Donkey Simulator". 

### Goal
This tutorial will help you get familiar with the framework developed for creating, training and testing the neural network models in the modified simulator.

### Prerequisites
In order to successfully complete this tutorial you will need: 

1. Familiarity with python
2. Pytorch installed on your machine (preferably with gpu). Refer to this ['link'](https://pytorch.org/get-started/locally/) for install instructions
3. Keras?
3. Git installed. Refer to this ['link'](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) for installation instructions
4. A system with at least 8 gigabytes of RAM.


### Notes
It is advised to use a GPU enabled machine for running the models, as the inference time for the models can be as high if running on CPU, which may lead to control related issues when actually using the model to drive the car.

## Setting up the environment

1. You can download the precompiled binary for the simulator from this ['link'](www.google.com) or build from ['source'](www.google.com). We suggest using the precompiled binary if you're unfamiliar with the unity engine. You can name the folder what you want, but for this tutorial we'll call it DonkeySimWin.

2. open the terminal(Linux) or command line interface (Windows) and navigate to the folder where the simulator files are kept (not that DonkeySimWin is just the name of the folder in which the simulator files are kept):
```bash
cd PATH\TO\DonkeySimWin
```
then execute these commands:
```bash
git clone https://github.com/tawnkramer/gym-donkeycar
pip install -e gym-donkeycar
git clone https://github.com/naughtyStark/MUSHR-correspondence.git
```
unpack the contents of the mushr directory into the DonkeySimWin directory

## Running the examples:

1. The reinforcement learning example is already provided by the gym-donkey car package. The aformentioned package provides an open-AI-gym interface for training and inference. You can run the example for training the RL agent as follows:

First start by creating a catkin package for our code:
```bash
python gym-donkeycar/examples/reinforcement_learning/ddqn.py --sim <path to simulator>
```
You can feed the path to the simulator, or you can start the simulator manually by running the donkeysim.exe application before executing the command above

You can run the model in test mode by running:
```bash
python gym-donkeycar/examples/reinforcement_learning/ddqn.py --test --sim <path to simulator>
```

2. Imitation learning: Imitation learning involves 4 steps; data collection, post processing, training, and testing. For collecting training data as well as for running the models, the same file "run_sim.py" needs to be run. 

a) For collecting data, first, start the simulator, then, using the command line (in the same directory), execute:
```bash
python run_sim.py --dataset_name=your_preferred_name_for_the_dataset
```
Options available for this file are:

**dataset_name**: A suffix that will be added to the standard dataset name. for example: MUSHR_320x240_test.npy, where "test" is the suffix

**model**: type of model: image to steering, image to bezier or image to image. This does not apply when collecting data

**test**: whether we're testing the model or not. It is False by default

**manual_control**: how the car is driven manually. The default is to use the mouse for steering and throttle, and use keyboard keys to record/
abort and change driving mode (auto or manual).

**env_name**: name of the environment that you want to be loaded in. The list can be seen by typing -h after the above command

Recording can be started/paused by pressing the key 'K', and stopped (aborts the run_sim.py program) by pressing 'O'
For changing modes: autonomous steering, if test=True and a valid model is selected, can be turned on by pressing 'A'. Switching to manual can be done by pressing 'M'.


b) The next step is to run post processing on the collected data: post processing involves, at minimum, shuffling of the data, and creation of the labels for each image.
```bash
python post_processing_mushr.py --dataset_name=the_name_you_picked_earlier --model=model_type 
```
Options available for this file are:
dataset_name: the output file will have the name MUSHR_320x240_dataset_name.npy.
model: type of model to create the dataset for (steering, bezier, or image-image).


c)For training:

For training an image to image network, run:
```bash
python pytorch_img_to_steering_train.py --dataset_name=the_name_you_picked_earlier
```
the model will be saved with a preset name. Do not change the model name for this example. The file model_runner.py looks for a particular name for each type of file. You can change this in the future if you want (but the same change has to be reflected in the python script used for training as well).

```bash
python pytorch_img_to_steering_test.py --dataset_name=the_name_you_picked_earlier
```

For training/testing image to bezier or image to image type models, you can run:
For image to bezier:
```bash
python pytorch_img_to_bezier_train.py --dataset_name=the_name_you_picked_earlier
python pytorch_img_to_bezier_test.py --dataset_name=the_name_you_picked_earlier
```

For image to image:
```bash
python pytorch_img_to_img_train.py --dataset_name=the_name_you_picked_earlier
python pytorch_img_to_img_test.py --dataset_name=the_name_you_picked_earlier
```
d) driving the 
to use a model for driving the car (assuming that the model exists), run:
```bash
python run_sim.py --test=True --model=model_type --env_name=MUSHR_benchmark
```
The environment "MUSHR_benchmark" is a simpler, fixed environment that you can use to compare the performance of different networks.
the model type refers to the type of network being used, image to image, image to steering or image to bezier.  

## Details regarding the implementation (what would be a better name for this section?):
The simulator provides 3 camera images for each time step. One image is from the forward facing camera (RGB), the other 2 are from cameras looking 15 degrees off from the center

## Wrap up

## Challenges
