### Adaptive local planner
This packages contains two local planners plugins for **move_base**:
- adaptive_local_planner.cpp/h : uses torob move algorithm as local planner (maintain by Guillaume Lozenguez, adapted by Xuan Sang LE)
- pf_local_planner.cpp/h : using potential field as local planner

## Usage
These planners are used as plugins for **move_base**, for detail usage, please take a look at launch files in the **launch** folder

## Simulation

To perform a simulation of the local planner, perform
```sh
  roslaunch adaptive_local_planner turtlebot_in_stage.launch
```
