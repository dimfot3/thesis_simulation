# Thesis_Simulation

## Overview
This repository is part of my Thesis at Aristotle University of Thessaloniki's Department of Electrical and Computer Engineering. My Thesis focused on digital indoor reconstruction and people detection utilizing LiDAR technology. We created some simulations in this repository using Gazebo Simulator, which is extensively used in the scientific community for robotic applications to test the perfomance of our applications under different realist circumstances.

This repository contains two indoor spaces: one simulating a bedroom and another simulating a workspace. We utilized various 3D CAD objects from open Blender collections to enhance the realism of the environments. Additionally, we created lifelike human entities that move randomly within the spaces. To accomplish this, we employed the default appearance of Gazebo human entities or designed our own using MakeHuman. In addition to external appearance, we incorporated realistic movement of the human skeleton, such as walking or falling, by utilizing Adobe Mixamo to merge models with behaviors and then extracting a Gazebo-compatible model using Blender.

To simplify the procedure, we created a single configuration file that enables for a smooth simulation start and setup. Furthermore, we have created custom plugins that allow the Gazebo simulation to be integrated with the Robotic Operating System (ROS), allowing for real-time applications across many language frameworks. More information on how to start the simulation will be supplied in the following sections.


