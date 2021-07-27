Active Graph SLAM Project
============

Code used for the paper "Fast Uncertainty Quantification for Active Graph SLAM", which extends our previous work accepted for presentation in IROS 2021 "Fast Autonomous Robotic Exploration Using the Underlying Graph Structure".

Tested by jplaced for Ubuntu 20.04, ROS Noetic.


Installation:
------------
Required dependencies:
  * Python3
  * ROS Noetic
  * Gazebo
  * OpenCV
  * Eigen3
  * g2o
  * python3-catkin-tools
  * Python3 modules: numpy, networkx, matplotlib, scikit-learn,

Dependencies contained in 3d party folder working in ROS Noetic:
  * kobuki_plugins
  * open_karto

Build:
------------
1. Download repo
2. >cd active_graph_slam
3. >catkin b
4. Edit path in graph_d_exploration/param/mapping_karto_g2o.yaml and graph_d_exploration/scripts/constants.py

Run simulation:
------------

1. First source workspace:

>source active_graph_slam/devel/setup.bash

2. Launch Gazebo simulator, RViZ and SLAM algorithm:

>roslaunch graph_d_exploration single_willow.launch

3. Launch Active Decision Maker:

>roslaunch graph_d_exploration graph_dopt.launch
