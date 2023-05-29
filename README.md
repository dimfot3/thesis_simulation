# Thesis_Simulation

## Overview
This repository is part of my Thesis [[1]](#1) at Aristotle University of Thessaloniki's Department of Electrical and Computer Engineering. My Thesis focused on digital indoor reconstruction and people detection utilizing LiDAR technology. We created some simulations in this repository using Gazebo Simulator, which is extensively used in the scientific community for robotic applications to test the perfomance of our applications under different realist circumstances.

This repository contains two indoor spaces: one simulating a bedroom and another simulating a workspace. We utilized various 3D CAD objects from open Blender [[2]](#2) collections to enhance the realism of the environments. Additionally, we created lifelike human entities that move randomly within the spaces. To accomplish this, we employed the default appearance of Gazebo human entities or designed our own using MakeHuman [[3]](#3). In addition to external appearance, we incorporated realistic movement of the human skeleton, such as walking or falling, by utilizing Adobe Mixamo [[4]](#4) to merge models with behaviors and then extracting a Gazebo-compatible model using Blender.

To simplify the procedure, we created a single configuration file that enables for a smooth simulation start and setup. Furthermore, we have created custom plugins that allow the Gazebo simulation to be integrated with the Robotic Operating System (ROS2), allowing for real-time applications across many language frameworks. More information on how to start the simulation will be supplied in the following sections.

## Software Requirements
For the implementation of simulations we used the latest (2022-2023) versions of Gazebo Simulation and ROS2. The project were created and tested using linux Ubuntu 22.04 (Jammy Jellyfish), however it is higly possible that will work in other platfomrs with small modifications.
Required Software:
- Ubuntu 22.04 (Jammy Jellyfish)
- Gazebo Garden [[5]](#5)
- Robotic Operation System (ROS2) Humble Hawksbill [[6]](#6)
- python 3.10.6
- Python packages:
  - matplotlib==3.5.1
  - numpy==1.21.5
  - PyYAML==6.0
  - scipy==1.8.0

## Seting up procedure
Here are the main steps to try this repository:

1. Download the indoor spaces:
   - Use the following link to access the two environment spaces (bedroom and workspace): Bedroom [[7]](#7) and Workspace [[8]](#8)
   - Inside the link, you will find several sdf and xacro files that define specific objects within each space.

2. Build the ROS2 packages:
   - Go to the root folder of the repository.
   - Run the following instructions in your terminal:
     ```
     cd ros_packages && colcon build
     ```

3. Set up the configuration file:
   - The configuration file is the Yaml ``config.yaml`` file. Here is an example:
    ```
    ---
    simulation_env: "bedroom"
    rviz2: true
    static_lidar: true		# keep this true
    human_1: 
     velocity: 1.0
     pace: 1.0
     traj: "test_motion.bin"
     anim_t: 2
     anim_f: 'falling.dae'
    lidar_1:
     x: 0.0
     y: 0.0
     z: 1.9
     rx: 0
     ry: 0
     rz: 0
     channels: 64
     rate: 20
     ```
    To explain more in first line we set the name of environment we wish to be simulated. We provide the ``bedroom`` and the ``workspaceA``. In second line we set if     we want an rviz visualization where we can see the LiDAR and human entitties. The ``static_lidar`` is for debugging purposes, it should be kept ``true``. In         order to define a human in the simulation create a set ``human_k``. Make sure that name follows this notation and the humans are ordered from 1 to n. You can set     explicitely the velocity of human, its pace (how fast his legs are moving) and the trajectory file. You can see more about trajectory below. You can also set a       animation file ``anim_f`` to perform except from walking, like falling. The time which the animation will performned is setted in  ``anim_t`` and if we dont want     any animation we can set time -1.
    In same manner we can set n LiDARs. In ``x,y,z,rx,ry,rz`` we set its position and orientation in space. We can also explicitely change the number of channels and     the rotation rate of the LiDARs.

## Starting Simulation
The simulation is started from the ROS2 launcher python file ``simulation.launch.py``. In order to run the simulation run in the root of this repository <br>
```ros2 launch simulation.launch.py``` <br>
The main procedure of this file:
- reading of cofig yaml
- creation of the humans and LiDARs (XACRO [[9]](#9) to SDF)
- starting Rviz (if true)
- starting a republisher of LiDARs' information from Gazebo topics to ROS2 topics
- starting the simulation

## Links and Citations
<a id="1">[1]</a> Thesis report: https://drive.google.com/file/d/1bU3LGlbmP9Ni8-itYjfeBEJv9t3pE1vR/view?usp=sharing <br>
<a id="2">[2]</a> Blender software: https://www.blender.org/ <br>
<a id="3">[3]</a> MakeHuman software: http://www.makehumancommunity.org/content/downloads.html <br>
<a id="4">[4]</a> Mixamo: https://www.mixamo.com/#/ <br>
<a id="5">[5]</a> Gazebo Garden: https://gazebosim.org/home <br>
<a id="6">[6]</a> ROS2 humble: https://docs.ros.org/en/humble/index.html <br>
<a id="7">[7]</a> Bedroom SDF: https://drive.google.com/file/d/1tx_Km4OHJxzaoKkMi6DDNt0QHsrwEFrI/view?usp=sharing <br>
<a id="8">[8]</a> Workspace SDF: https://drive.google.com/file/d/1xzYyGJfddlzOgr16DMkZ6A1u_8xhAAf1/view?usp=sharing <br>
<a id="9">[9]</a> xacro: http://wiki.ros.org/xacro <br>

