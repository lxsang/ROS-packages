#!/bin/bash
source /opt/ros/kinetic/setup.bash

PORT=$1
command=$2
GPORT=`expr $PORT + 1`
MY_IP="10.1.10.84"
echo ">"
echo "> Configuration of ROS on $MY_IP:$1"

export ROS_HOSTNAME=$MY_IP
export ROS_MASTER_URI=http://$MY_IP:$PORT
source /opt/ros/kinetic/setup.bash
export PATH="/home/mrsang/bin:$PATH"


# ROS configuration
echo "Setting ros_ws"
source /home/mrsang/myws/rosws/devel/setup.sh

echo "Setting cartographer_ws"
# for cartographer
# /home/mrsang/cartographer_ws/install_isolated/setup.sh

#roscore -p $PORT &
# GAZEBO_MASTER_URI=http://$MY_IP:$GPORT
echo "Gazebo port $GPORT"
ROS_MASTER_URI=http://$MY_IP:$PORT GAZEBO_MASTER_URI=http://$MY_IP:11313 roslaunch multi_robot_sim $command