This package contains launch files and 3D models for the "Evaluation of Out-of-the-box ROS 2D SLAMs for Autonomous Exploration of Unknown Indoor Environments" using **gazebo** as simulation evironment.

# lauch files:
 - *launch/simulation_cartographer.launch* is the launch file for Cartographer
 - *launch/simulation_gmapping.launch* is the launch file for Gmapping
 - *launch/simulation_karto.launch* is the launch file for KartoSLAM
 
 usage:
  ```sh
    roslaunch gazebo_sim  simulation_<SLAM>.launch  world:=<model>
  ```
  
 # Model
 - cross
 - loop
 - maze
 - zigzag
 - dia: the model of our lab building
 
 
