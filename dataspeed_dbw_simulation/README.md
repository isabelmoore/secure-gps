![](img/cover_picture.png)

# Documentation for the Dataspeed Drive-by-Wire Gazebo/ROS Simulator

To install `dataspeed_dbw_simulator`, you will first need to set up your computer to accept software from the Dataspeed package server.

To do this, follow the **Setup apt** and **Setup rosdep** instructions for either ROS 1 or ROS 2 found here: [https://bitbucket.org/DataspeedInc/ros_binaries](https://bitbucket.org/DataspeedInc/ros_binaries).

Then install the actual simulator package:

`$ sudo apt-get install ros-$ROS_DISTRO-dataspeed-dbw-simulator`

By default, ROS Melodic is installed with Gazebo 9.0. When using the drive-by-wire simulator in ROS Melodic, it is recommended to install the latest release of 9.x like this:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt upgrade
```

NOTE: This is not needed for ROS Noetic and Gazebo 11.x, or any ROS 2 version.

PDF documentation for the simulator:

### (ROS 2) Foxy, Galactic, or Humble
- Version 3.0.0 ([simulator_manual_v3_0_0.pdf](https://bitbucket.org/dataspeedinc/dataspeed_dbw_simulation/raw/master/simulator_manual_v3_0_0.pdf)) \[latest for ROS 2\]

### (ROS 1) ROS Melodic / Gazebo 9.x or ROS Noetic / Gazebo 11.x
- Version 2.6.2 ([simulator_manual_v2_6_2.pdf](https://bitbucket.org/dataspeedinc/dataspeed_dbw_simulation/raw/master/simulator_manual_v2_6_2.pdf)) \[latest for ROS 1\]
