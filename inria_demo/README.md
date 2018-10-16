# Demo use guide
This document provides a detail setup guide for the demonstration.

Table of content:
1.  [Hardware requirement](#hw)
2.  [Raspberry Pi setup](#rpi)
3.  [Desktop computer setup](#desk)
4.  [Mapping the environment](#mapping)
5.  [Navigation](#nav)


<a name="hw"></a>
## Hardware Requirement
1. **A raspberry Pi V3b+**: Used to run the ROS master and control the robot
2. **A turtlebot base with its charging dock**
3. **A laser sensor** (e.g.  URG-04LX) for mapping and navigation
4. **A desktop computer** (optional): for remote accessing and control 
5. **A tablet** (optional): Used as HCI device to interact with user for robot controlling
6. **XBox 360 controller and adapter** for manual control and mapping

**System mounting**:

![https://github.com/CARMinesDouai/MultiRobotExplorationPackages/raw/master/inria_demo/screenshot.png](https://github.com/CARMinesDouai/MultiRobotExplorationPackages/raw/master/inria_demo/screenshot.png)

<a name="rpi"></a>
## Setting up the raspberry PI
 An Ubuntu 16.04 with ROS stack and all require packages pre-install is  available  at: [http://car.imt-lille-douai.fr/static/piros.img](http://car.imt-lille-douai.fr/static/piros.img). It is easy to write this image to a SD (at least 8GB) card using the ``dd`` command :
 ```sh
 dd bs=4M if=piros.img of=/dev/mmcblk0 conv=fsync
 # replace /dev/mmcblk0 with the correct device name of the SD card 
 ```
 The default username and password is: **ubuntu**
 
Once booting up, one should setup the network connection for the Raspberry PI. First, the PI need to automatically connect to a wifi network, modify the ```/etc/wpa_supplicant/wpa_supplicant.conf``` file to something like this:
```sh
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
ssid="wifi-name"
psk="wifi-passwork"
proto=RSN
key_mgmt=WPA-PSK
pairwise=CCMP
auth_alg=OPEN
}
```

Next, the PI should have a static IP address for remote accessing, modify the ```/etc/network/interfaces``` as follow
```sh
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

# The loopback network interface
auto wlan0

auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp

allow-hotplug wlan0
iface wlan0 inet static
address 10.1.160.200
netmask 255.255.0.0
broadcast 10.1.255.255
gateway 10.1.1.1
wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
```

Note: change the IP addresses to your network configuration.

Last, we need to add all the machines that we want to communicate with from the PI using ROS to ```/etc/hosts```, for example
```sh
# Change the following IP address to the Raspberry PI IP address, the hostname is ubiquityrobot
10.1.160.200 	ubiquityrobot
# For each machine that  communicates with the PI via ROS, add its hostname and IP here,
# For example my laptop's hostname is nono with the IP: 10.1.160.205
# To enable the ROS connection from nono to the rapberry PI, i need to add the following line
10.1.160.205	nono
```

<a name="desk"></a>
 ## Setting up the desktop computer
 
 The desktop computer is used for remote access to the Raspberry PI, for manual mapping process and for running the remote control software written in PhaROS. Therefore, this computer need to be ROS compliant. To install ROS kinetic on the machine, please refer to [ this tutorial ](http://wiki.ros.org/kinetic/Installation/Ubuntu), we recommend the ```ros-kinetic-desktop-full``` installation. Once the ROS installation is done, some extra packages are required to be installed using the following shell script
 
 ```sh
 #! /bin/bash
 # initialize rosdep if you didn't do it
sudo rosdep init
rosdep update
# source the environment to support ROS, for convenient, we put the
# command to ~/.bashrc so that,  the environment will be sourced automatically
# each time a new terminal session is opened
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# create new workspace at the home folder
mkdir ~/rosws
cd ~/rosws
# create source directory
mkdir src
# create an empty workspace
catkin_make
# before cloning our packages, we need install some dependencies:
# the default ros navigation stack which is used in
# our exploration packages 
# and suitesparse library needed by Karto SLAM
sudo apt-get install ros-kinetic-navigation libsuitesparse-dev
# install the turtlebot dependencies
sudo apt-get install ros-kinetic-kobuki ros-kinetic-urg-node  ros-kinetic-kobuki-core ros-kinetic-turtlebot*
# turtlebot support for gazebo
sudo apt-get install ros-kinetic-kobuki-gazebo ros-kinetic-kobuki-gazebo-plugins 
# now install g2o needed for kartoSLAM
sudo apt-get install git
git clone https://github.com/RainerKuemmerle/g2o
cd g2o
mkdir build
cd build
cmake ../
make
sudo make install
cd ../../
# rm -r g2o
# now clone all the packages in my github repository
git clone https://github.com/lxsang/ROS-packages
# rename it to src
rm -r src
mv ROS-packages src
# ignore some packages for now
touch src/distributed_slam_karto/CATKIN_IGNORE
touch src/pharos_probabilistic_merging/CATKIN_IGNORE
touch src/dslam/CATKIN_IGNORE
touch src/multi_master_bridge/CATKIN_IGNORE
touch src/multi_merge/CATKIN_IGNORE
touch src/frontier_allocation
# generate dependent messages
catkin_make tf_lookup_generate_messages
# now build all of our packages
catkin_make
#source our workspace
echo "source ~/rosws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
 ```
 
 Note: you can put theses commands in a single shell script file, ```install.sh``` for example then simply run:
 ```sh
 chmod u+x install.sh
 ./install.sh
 # now this will take some time, take a coffee
 ```
 
 Once everything is installed, the machine need to be connected to the same **wifi network as the raspberry PI** and **configured with a static IP** (e.g. hostname is **nono** and **IP** is 10.1.160.205 for the above example). 
 
 Once again, to be able to communicate with other machine on the ROS network (e.g. the raspberry PI), all remote machines need to be added to ```/etc/hosts```
 ```sh
 # Change the following line to the current hostname and IP address, for example my machine
 # host name is nono and my IP is 10.1.160.205
 10.1.160.205	nono
# For each machine that we want to communicate via ROS, add its hostname and IP here,
# For example if we want to connect to the PI setup ealier
10.1.160.200	ubiquityrobot
 ```
 
 <a name="mapping"></a>
 ## Mapping the environment
 Mapping the environment using KartoSLAM requires a lot of computational resources, therefore, it can not be done using only the Raspberry PI. That why we need the desktop computer. The idea is to use the Raspberry PI as a ROS master that collects the laser and odometry data, then use the desktop computer to run the SLAM algorithm from distance and create a map base on the sensor data from the PI. To that end, we first need to connect to the raspberry PI from the desktop via ssh and run a minimal mapping stack on it. From a terminal, execute the following command:
 
```sh
# make sure that the desktop and the rapberry Pi are on the same wifi network
ssh ubuntu@ubiquityrobot
# password is ubuntu
# the launch the minimal mapping stack from the rasberry pi
roslaunch inria_demo minimal_mapping.launch
```

From another terminal, run the SLAM algorithm on the desktop using:
```sh
# The ROS master uri must be the IP address of the Raspberry Pi, which actually run a ros master
# replace the following address with the PI's address
ROS_MASTER_URI=http://10.1.160.200:11311 roslaunch inria_demo mapping.launch
# or using the hostname:
#ROS_MASTER_URI=http://ubiquityrobot:11311 roslaunch inria_demo mapping.launch
```
 
 While every thing is running, use the Xbox 360 controller to move the turtlebot around the environment to perform the mapping, [RVIZ](http://wiki.ros.org/rviz) can be used to view the map on the desktop.
 When the mapping process finishes, run the following command on other terminal to save the map
 
 ```sh
 rosrun map_server map_saver
 # this command will generate two file : map.pgm and map.yaml that represents the map of the environment
 ```
 
 The final step is to copy this map to the Raspberry Pi for the navigation, using the scp command:
 ```sh
 scp map.* ubuntu@ubiquityrobot:/home/ubuntu/rosws/src/inria_demo/launch/
 ```
 
 That's all. Now the navigation part.
 
 <a name="nav"></a>
 ## Navigation
