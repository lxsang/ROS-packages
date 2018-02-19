
This repository contains ROS packages that i'im currently working on for mono and multi-robot autonomous expoloration of unknown environment. These packages are built and tested on ROS Kinetic.

# Packages

1. **adaptive_local_planner**: is a *move_base* plugin that use potential field technique as local planner. It allows the robot to navigate flexibly in narrow space. This package depends on the **move_base** package
2. **frontier_allocation**: Detects the frontiers between known and unknown areas and allocate a frontier as the next goal to the navigation stack. This package is used as goal allocator for our autonomous exploration stack
3. **gazebo_sim**: contains launch and 3D models for the simulation of different SLAMs on autonomous exploration, it depends on the **gmapping**, **karto_slam**, and **google cartograper** packages
4. **multi-master_bridge**: a TCP-base protocol for multi-master system. The idea is that each robot runs its own master, and these masters are able to automatically discover and communicate with each other for data exchange base on this protocol
5. **multi_merge**: greedy and probabilistic merging algorithm implementations for multi-master system using multi_master_bridge protocol. This is used to merge the local map produced by two or more robots into a global map
6. **open_karto, slam_karto,** and **sparse_bundle_adjustment**: a fork of the opensource version of **Karto SLAM**
7. **pharos_probabilistic_merging**: probabilistic merging algorithm for the PhaROS platform
