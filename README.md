# Metacontrol simulation

## Setup using wstool
```
$ mkdir -p ~/metacontrol_ws/src
$ cd ~/metacontrol_ws
$ wstool init src https://raw.githubusercontent.com/rosin-project/metacontrol_sim/master/metacontrol_sim.rosinstall
$ rosdep install --from-paths ./src -y -i -r --skip-keys="abb_rws_interface"
```

## Compile

```
$ cd ~/metacontrol_ws/src
$ catkin_make
$ source devel/setup.bash
```
## Launch simulation

```
$ roslaunch metacontrol_sim ridgeback_yumi_world.launch
```
## Launch navigation
#### Using only odometry
```
$ roslaunch metacontrol_nav odom_navigation_standard.launch
```
#### Using map based navigation
```
$ roslaunch metacontrol_nav amcl_demo_standard.launch use_fake_localization:="true"
```
