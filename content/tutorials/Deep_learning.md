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
![alt-text](https://github.com/naughtyStark/mushr-website/blob/master/mushr_DL.gif)
## Introduction

Deep learning is part of a broader family of machine learning methods based on artificial neural networks with representation learning. Learning can be supervised, semi-supervised or unsupervised. In this tutorial, we'll see how to get started with applying imitation learning as well as reinforcement learning to the MuSHR car through the use of a modified simulator developed by the DIYRobcars community, named [Donkey Simulator](https://github.com/tawnkramer/sdsandbox). 

### Goal
This tutorial will help you get familiar with the framework developed for creating, training and testing the neural network models in the modified driving simulator.

### Prerequisites
In order to successfully complete this tutorial you will need: 

1. Familiarity with python
2. Preferably using python version >= 3.6 (the dependency [screeninfo](https://pypi.org/project/screeninfo/) doesn't work well with python<3.6), pip version >= 20, setuptools >=44
3. Git installed. Refer to this ['link'](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) for installation instructions
4. A system with 8 Gigabytes of RAM, preferably with the GPU enabled.


### Notes
It is advised to use a GPU enabled machine for running the models, as the inference time for the models can be high when running on CPU, which may lead to control related issues when actually using the model to drive the car.

## Setting up the environment

1. Download the precompiled binary for the simulator from these links: 
[Windows](https://drive.google.com/file/d/1gKCxjyaRV37veT3759DrYHIGdrn9ized/view?usp=sharing),
[Linux](https://drive.google.com/file/d/1P_hUH7W4liz8REW2fqjG_yG3CzXqsIma/view?usp=sharing),
[Mac OS](https://drive.google.com/file/d/1h6TSImqIEQeK4dWEFIH32td5hNgLk_up/view?usp=sharing).

2. Open the terminal(Linux) or command line interface (Windows) and navigate to the folder where the simulator files are kept.
```bash
$ cd PATH\TO\DonkeySim
```
then execute these commands:
```bash
$ git clone https://github.com/prl-mushr/gym-donkeycar.git
$ pip3 install gym-donkeycar
$ git clone https://github.com/prl-mushr/MUSHR-DL.git
```
dependencies:
the dependencies can be installed by running 
```bash
$ cd MUSHR-DL
$ pip3 install -r requirements.txt
```
For installing pytorch, follow the instructions on this [link](https://pytorch.org/get-started/locally/) (use pip3 instead of pip if you have python 2.x and 3.x on the same system). 

## Running the examples:

### Running the simulator:
Start the simulator by running the donkey_sim.exe (or donkey_sim.x86_64 for linux). For linux systems, you'll have to provide permission for the exe to run:
```bash
$ sudo chmod +x donkey_sim.x86_64
```

### Reinforcement Learning:
The reinforcement learning example is already provided by the gym-donkey car package. The aformentioned package provides an open-AI-gym interface for training and inference. You can run the example for training the RL agent as follows:
```bash
$ python gym-donkeycar/examples/reinforcement_learning/ddqn.py --sim <path to simulator>
```
You may provide the path to the simulator or start the simulator manually and omit the --sim argument.

You can run the model in test mode by running:
```bash
$ python gym-donkeycar/examples/reinforcement_learning/ddqn.py --test --sim <path to simulator>
```

### Imitation Learning:
Imitation learning: Imitation learning involves 4 steps; data collection, post processing, training, and testing. For collecting training data as well as for running the models, the same file "run_sim.py" needs to be run. 

1. Collecting data:
For collecting data, first, start the simulator, then, using the command line (in the same directory), execute run_sim.py.
Options available for this file are:

    **dataset_name**: A suffix that will be added to the standard dataset name. for example: MUSHR_320x240_test.npy, where "test" is the suffix

    **model**: type of model: image to steering, image to bezier or image to image. This does not apply when collecting data

    **test**: whether we're testing the model or not. It is False by default

    **manual_control**: how the car is driven manually. The default is to use the mouse for steering and throttle, and use keyboard keys to record/
abort and change driving mode (auto or manual).

    **env_name**: name of the environment that you want to be loaded in. The list can be seen by typing -h after the above command

    Recording can be started/paused by pressing the key 'K', and stopped (aborts the run_sim.py program) by pressing 'O'
For changing modes: autonomous steering, if test=True and a valid model is selected, can be turned on by pressing 'A'. Switching to manual can be done by pressing 'M'.

    command:
```bash
$ python run_sim.py --dataset_name=test
```

### Note:
if you're running this tutorial on python version <3.6, you may get an error related to the screeninfo package. In that case, open the file MUSHR-DL/key_check.py and remove all the lines related to screeninfo and set screen_width and screen_height to the pixel width and height of your monitor

2. Post processing:
The next step is to run post processing on the collected data: post processing involves, at minimum, shuffling of the data, and creation of the labels for each image. 
    
    Options available for this file are:

    **dataset_name**: the output file will have the name MUSHR_320x240_dataset_name.npy.

    **model**: type of model to create the dataset for (steering, bezier, or image-image).

    command:
```bash
$ python post_processing_mushr.py --dataset_name=test --model=model_type 
```

3. Training:

**Image to steering angle:**
```bash
$ python pytorch_img_to_steering_train.py --dataset_name=test
```
the model will be saved with a preset name. The same name is used in testing as well as the model running script, and so for the sake of the tutorial the name should be left as is.

```bash
$ python pytorch_img_to_steering_test.py --dataset_name=test
```

**Image to bezier:**
```bash
$ python pytorch_img_to_bezier_train.py --dataset_name=test
$ python pytorch_img_to_bezier_test.py --dataset_name=test
```
**Image to image:**
```bash
$ python pytorch_img_to_img_train.py --dataset_name=test
$ python pytorch_img_to_img_test.py --dataset_name=test
```

4. Driving the car using the trained model:
To use a model for driving the car (assuming that the model exists), run:
```bash
$ python run_sim.py --test=True --model=model_type --env_name=MUSHR_benchmark
```
The environment "MUSHR_benchmark" is a simpler, fixed environment that you can use to compare the performance of different networks.
the model type refers to the type of network being used, image to image, image to steering or image to bezier.  

## Details regarding the implementation (what would be a better name for this section?):
The simulator provides 3 camera images for each time step. One image is from the forward facing camera (RGB), the other 2 are from cameras looking 15 degrees off from the center

## Wrap up

## Challenges
