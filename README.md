# ROS begineer tutorials

*This repository is created in order to complete ROS begineer tutorials for ENPM808X: Software Development of Robotics ME Coursework.*

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
---srv/
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
## Running the nodes using rosrun

**1.** Start ROS core $ roscore <br/>
**2.** Steps to run talker, open a new shell without closing the previous shell. <br/>
   Run below commands
   ```
   $ cd ~/catkin_ws
   $ source devel/setup.bash
   $ echo $ROS_PACKAGE_PATH
   $ rosrun beginner_tutorials talker -> Uses default frequency = 10 Hz
   $ rosrun beginner_tutorials talker _freq:=<user-defined-frequency> -> Defines frequency using private freq parameter
   ```
**3.** Steps to run listener node.
   Open a new shell and run below commands
   ```
   $ cd ~/catkin_ws
   $ source devel/setup.bash
   $ echo $ROS_PACKAGE_PATH
   $ rosrun beginner_tutorials listener
   ```

## Running the nodes using roslaunch

Launch files are used when multiple nodes are needed to be started within the package. To *launch* multiple nodes execute following command:
```
$ cd ~/catkin_ws
$ roslaunch beginner_tutorials SubPub.launch
```
> Note: No need to start *rosmaster* node while using *launch* file. It starts when the file is launched if it is not running.

The command above will launch the nodes with default values. To pass the custom frequency value use the following command:
```
$ cd ~/catkin_ws
$ roslaunch beginner_tutorials SubPub.launch freq:=1
```
## Running ROS Services
Create a ros service named *changeBaseString*. This service is used to modify the talker's default message. 
To run this service open a new shell and type:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosservice call /changeBaseString "Hi......"
```
You can also refer to the image file given in the *results* directory.

## Log messages

The human-readable string messages can be viewed in real-time through the rqt_console GUI application.
To run rqt_console, open new shell and type:
```
$ rosrun rqt_console rqt_console
```
You can also refer to the image file given in the *results* directory.
