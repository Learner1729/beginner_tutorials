# Week_8 ROS begineer tutorials

This repository is created in order to complete week 8 ROS begineer tutorials for ENPM808X: Software Development of Robotics ME Coursework.

## Licence
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Prerequisites 

* [ROS Kinetic](https://wiki.ros.org/ROS/Installation) on Ubuntu 16.04. 

>Note: Follow the installation steps given on ROS Kinetic installation page. 

After installing the full version of ROS kinetic distro we need to setup ROS by following the below steps. This steps are needed to be performed only once:

**1.** Setting up rosdep systemwide: `$ sudo rosdep init` <br/>
**2.** Setting up rosdep in a user account: `$ rosdep update` <br/>
**3.** Setting up enivronment variables <br/>
   Updating the bashrc file: `$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc` <br/>
   Sourcing the bashrc file: `$ source ~/.bashrc` <br/>

## Creating a catkin workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Before any updates run the command `$ source devel/setup.bash` 

## Directory hierarchy
``` 
catkin_ws/
-build/
-devel/
-src/
--<ROS package, this is your repsitory for commit>
---package.xml
---CMakeLists.txt
---include/
---src/
---etc...
```
>Note: "-" indicates levels

## Standard install via command-line

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/Learner1729/beginner_tutorials.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH  -> Checks whether the environment variable includes the directory you are in.
```
## Running the nodes using *rosrun*

**1.** Start ROS core $ roscore <br/>
**2.** Steps to run talker, open a new shell without closing the previous shell. <br/>
   Run below commands
   ```
   $ source devel/setup.bash
   $ echo $ROS_PACKAGE_PATH
   $ rosrun beginner_tutorials talker
   ```
**3.** Follow the same steps for running listener node.

## Running the nodes using *roslaunch*

This week's tutorials deal with ROS *services*, *logging* and *launch* files. Launch files are used when multiple nodes are needed to be started from within the package. 
```
$ roslaunch beginner_tutorials SubPub.launch
```