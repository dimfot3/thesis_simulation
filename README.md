# Thesis_Simulation

## Overview
This repository is part of my Thesis [[1]](#1) at Aristotle University of Thessaloniki's Department of Electrical and Computer Engineering. My Thesis focused on digital indoor reconstruction and human detection utilizing LiDAR technology. We created some simulations in this repository using Gazebo Simulator, which is extensively used in the scientific community for robotic applications to test the perfomance of our applications under different realist circumstances.

This repository contains two indoor spaces: one simulating a bedroom and another simulating a workspace. We utilized various 3D CAD objects from open Blender [[2]](#2) collections to enhance the realism of the environments. Additionally, we created lifelike human entities that move randomly within the spaces. To accomplish this, we employed the default appearance of Gazebo human entities or designed our own using MakeHuman [[3]](#3). In addition to external appearance, we incorporated realistic movement of the human skeleton, such as walking or falling, by utilizing Adobe Mixamo [[4]](#4) to merge models with behaviors and then extracting a Gazebo-compatible model using Blender.

To simplify the procedure, we created a single configuration file that enables for a smooth simulation start and setup. Furthermore, we have created custom plugins that allow the Gazebo simulation to be integrated with the Robotic Operating System (ROS2), allowing for real-time applications across many language frameworks. More information on how to start the simulation will be supplied in the following sections.

  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="2d_schem/human_fall_ex1-1.gif">
    <img alt="Small simulated bedroom with human falling" alt="drawing" width="49%" height="250">
  </picture>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="2d_schem/multihuman.png">
  <img alt="Big simulated workspace with multiple humans walking" alt="drawing" width="49%" height="250">
</picture>


## Software Requirements
For the implementation of simulations we used the latest (2022-2023) versions of Gazebo Simulation and ROS2. The project were created and tested using linux Ubuntu 22.04 (Jammy Jellyfish), however it is higly possible that will work in other platfomrs with small modifications.
Required Software:
- Ubuntu 22.04 (Jammy Jellyfish)
- Gazebo Garden [[5]](#5)
- Robotic Operation System (ROS2) Humble Hawksbill [[6]](#6) with XACRO and Rviz2
- python 3.10.6
- Python packages:
  - matplotlib==3.5.1
  - numpy==1.21.5
  - PyYAML==6.0
  - scipy==1.8.0
-  Fast Library for Approximate Nearest Neighbors (FLANN) [[10]](#10)

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
   To elaborate more clearly, the first step involves determining the type of environment we wish to simulate, which, in this case, could be the ``bedroom`` or the ``workspaceA``. The second step involves deciding whether or not we want an rviz visualization, which would allow us to view LiDAR and human entities. In order to incorporate a human character into the simulation, we should create a set termed ``human_k``. It is important to ensure that the name follows this particular notation and that the humans are numerically ordered from 1 to n. There's an option to specify the speed of the human character, their pace (which refers to the speed of leg movements), and the trajectory file. Further details about trajectories can be found below. Additionally, you can assign an animation file labeled ``anim_f`` to the human character, which allows for actions beyond walking, such as falling. The duration of the animation can be defined in ``anim_t``. If you don't want any animation, you can simply set the time to -1. In a similar vein, you can incorporate multiple LiDARs into the simulation. The ``x, y, z, rx, ry, rz`` values allow you to define the LiDAR's position and orientation in space. If needed, you can also adjust the number of channels and the operational rate of the LiDARs explicitly.

## Starting Simulation
The simulation is started from the ROS2 launcher python file ``simulation.launch.py``. In order to run the simulation run in the root of this repository <br>
```ros2 launch simulation.launch.py``` <br>
The main procedure of this file:
- reading of cofig yaml
- creation of the humans and LiDARs (XACRO [[9]](#9) to SDF)
- starting Rviz (if true)
- starting a republisher of LiDARs' information from Gazebo topics to ROS2 topics
- starting the simulation

## Human Walking Trajectories and Animations
To create custom trajectories, you can utilize the ``HumanMotionCreator.py`` script. This script allows you to generate smooth and realistic trajectories within a specific area. It offers functions for plotting and saving the trajectory in a binary format.To use the script, make sure to save it in the "human_models" folder. If you want to use a custom trajectory, specify the file name of the custom binary file in the ``traj`` parameter of the human dictionary in config.yaml. Additionally, a custom plugin has been developed that publishes the human's position in the ROS2 transform topic ``/tf``. This plugin is responsible for moving the human along the custom trajectory specified in ``traj`` and performing the animation ``anim_f`` within the predefined time ``anim_t``.

In this repository the default animation is walking (``walk2.dae``) and happens until ``anim_t`` if ``anim_t>=0`` else happens until we close the simulation. Apart from that there are animations where human is standing (``standing.dae``), showing down (``showing.dae``), standing with two hands up (``handsup2.dae``) and falling down (``falling.dae``). They can be found in ``human_models`` folder and used in humans dictionary parameter ``anim_f`` to be performed at ``anim_t``.

## Docker (Experimental)
In order the simulation to be deployed on various Linux systems, a practical approach would be to use a Docker image built on ROS2. Docker allows to package simulation with all its dependencies, thereby ensuring it can run uniformly across different Linux distributions. Before you begin, you'll need to install Docker on your system. Docker is a platform that enables you to automate the deployment, scaling, and management of applications within containers. In addition, you'll also need to install the NVIDIA Container Toolkit. This toolkit enables the NVIDIA graphics processing unit (GPU) to be accessible in a Docker container, allowing it to leverage the GPU's capabilities for graphics and display. This is particularly useful when running simulations that require significant graphical processing power. <br>

Then in Ubuntu run in root folder of simulation: <br>
In order to build an image (This takes time): ```sudo docker  build -t test .``` <br>
In order to start a container: ```sudo docker run -it  -e DISPLAY=:1  -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all  test```  <br>
After that you will have access in the cmd to the root of linux based system that have all the depedencies installed. 

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
<a id="10">[10]</a> FLANN https://github.com/mariusmuja/flann.git

