# Metacontrol simulation
This repository contains the ROS simulated system used to demonstrate the application of the Metacontrol framework presented in [ADD REFERENCE on arXiv] 
In this simulation, a simple navigation task is given to a bridgeback mobile base controlled with the standard ROS navigation stack. A metacontrol subsystem uses a model of available configurations and associated launch files of the navigation stack to reconfigure at runtime the `move_node` when certain quality criteria (non-functional requirements) related to **safety** and **energy** are not met.

## Setup using wstool
In the following instructions we assume that the ROS workspace is called `metacontrol_ws`, but you can give it any name you want.
```console
mkdir -p ~/metacontrol_ws/src
cd ~/metacontrol_ws
wstool init src https://raw.githubusercontent.com/rosin-project/metacontrol_sim/master/metacontrol_sim.rosinstall
rosdep install --from-paths ./src -y -i -r --skip-keys="abb_rws_interface"
cd ~/metacontrol_ws/src
catkin b
source devel/setup.bash
```

## Run the simulation
Simply run the following launch file:
```
roslaunch metacontrol_sim metacontrol_sim_reasoner.launch
```
You should be able to see an RViz instance where you can follow the navigation of the mobile robot.
In the terminal where you executed the launchfile, you should see a very verbose output of the execution of the nodes that constiture the metacontrol subsystem.

## (Outdated, To Review) Launch navigation
#### Using only odometry
```
$ roslaunch metacontrol_nav odom_navigation_standard.launch
```
#### Using map based navigation
```
$ roslaunch metacontrol_nav amcl_demo_standard.launch use_fake_localization:="true"
```
