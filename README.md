# Metacontrol simulation

## Setup using wstool
```
$ mkdir -p ~/metacontrol_ws/src
$ cd ~/metacontrol_ws
$ wstool init src https://raw.githubusercontent.com/rosin-project/metacontrol_sim/master/metacontrol_sim.rosinstall
$ rosdep install --from-path src -y -i -r
```
## Manually setup

### Install Dependencies
```
$ apt-get update
$ apt-get install ros-melodic-ridgeback-description
$ apt-get install ros-melodic-hector-xacro-tools
$ apt-get install ros-melodic-ridgeback-control
```
### Create workspace
```
$ mkdir -p ~/metacontrol_ws/src
$ cd ~/metacontrol_ws/src
```
### Clone required repositories

#### Clone battery plugin repository

git clone https://github.com/rosin-project/brass_gazebo_battery.git

#### Clone Yumi repository

git clone https://github.com/marioney/yumi.git

#### Clone Metacontrol simulation

git clone https://github.com/rosin-project/metacontrol_sim.git

#### Clone metacontrol navigation

git clone https://github.com/rosin-project/metacontrol_nav.git


## Compile

```
$ cd ~/metacontrol_ws/src
$ catkin_make
$ source devel/setup.bash
```
## Launch simulation and navigation

```
$ roslaunch metacontrol_sim ridgeback_yumi_world.launch
$ roslaunch metacontrol_nav odom_navigation_standard.launch
```
