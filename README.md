Active Graph SLAM Project
============

Code used for the paper "Fast Uncertainty Quantification for Active Graph SLAM", which extends our previous work accepted for presentation in IROS 2021 "Fast Autonomous Robotic Exploration Using the Underlying Graph Structure". 

Tested by jplaced for Ubuntu 20.04, ROS Noetic.

Citation
------------
  * Placed, J. A., & Castellanos, J. A. (2021). Fast Uncertainty Quantification for Active Graph SLAM. arXiv preprint arXiv:2110.01289.
  * Placed, J. A., & Castellanos, J. A. (2021). Fast Autonomous Robotic Exploration Using the Underlying Graph Structure. In 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 6672-6679). IEEE.
  
Dependencies:
------------

  * Python3 (numpy, networkx, matplotlib, scikit-learn)
  * ROS Noetic & Gazebo
  * OpenCV: https://opencv.org/
  * Eigen3: https://eigen.tuxfamily.org/dox/GettingStarted.html
  * g2o: Downloand and install from https://github.com/RainerKuemmerle/g2o.
  * python3 catkin tools (sudo apt-get install python3-catkin-tools)
  * SuiteSparse (sudo apt-get install libsuitesparse-dev)

Contained in 3d party folder working in ROS Noetic:
  * kobuki_plugins
  * open_karto

Build:
------------
1. Clone repository
2. Build:

>cd active_graph_slam

>catkin b

4. Edit path where graphs are saved/read, in graph_d_exploration/param/mapping_karto_g2o.yaml and graph_d_exploration/scripts/constants.py

Run simulation:
------------

1. First source workspace:

>source active_graph_slam/devel/setup.bash

2. Launch Gazebo simulator, RViZ and SLAM algorithm:

>roslaunch graph_d_exploration single_willow.launch

3. Launch Active Decision Maker:

>roslaunch graph_d_exploration graph_dopt.launch
