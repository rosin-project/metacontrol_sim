#! /usr/bin/env python

import rospy

import actionlib

from metacontrol_msgs.msg import *

import subprocess
import os

import dynamic_reconfigure.client

from yaml import load
# import roslib
# roslib.load_manifest("rosparam")
# import rosparam
import rospkg
from move_base_msgs.msg import *

rospack = rospkg.RosPack()
# param = rosparam.load_file(rospack.get_path('metacontrol_sim')+'/yaml/goal.yaml')
dict = load(file(rospack.get_path(
    'metacontrol_sim')+'/yaml/goal.yaml', 'r'))
nav_goal = MoveBaseGoal()
nav_goal.target_pose.header.frame_id = dict['header']['frame_id']
nav_goal.target_pose.pose.position.x = dict['pose']['position']['x']
nav_goal.target_pose.pose.position.y = dict['pose']['position']['y']
nav_goal.target_pose.pose.orientation.x = dict['pose']['orientation']['x']
nav_goal.target_pose.pose.orientation.y = dict['pose']['orientation']['y']
nav_goal.target_pose.pose.orientation.z = dict['pose']['orientation']['z']
nav_goal.target_pose.pose.orientation.w = dict['pose']['orientation']['w']


def start_node(pkg, node_name, node_exec, ns=''):
    command = "rosrun {0} {1}".format(pkg, node_exec)
    my_env = os.environ.copy()
    my_env["ROS_NAMESPACE"] = ns
    p = subprocess.Popen(command, shell=True, env=my_env)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")


def kill_node(node_name, ns=''):
    command = "rosnode kill {0}".format(node_name)
    my_env = os.environ.copy()
    my_env["ROS_NAMESPACE"] = ns
    p = subprocess.Popen(command, shell=True, env=my_env)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")

def launch_config(pkg, launchfile, arg, ns=''):
    rospy.loginfo("launching new configuration...")

    command = "roslaunch {0} {1} profile:={2}".format(pkg, launchfile, arg)
    my_env = os.environ.copy()
    my_env["ROS_NAMESPACE"] = ns
    p = subprocess.Popen(command, shell=True, env=my_env)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")


class RosgraphManipulatorActionServer (object):

    _result = MvpReconfigurationResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
                self._action_name,
                MvpReconfigurationAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._as.start()
        rospy.loginfo ('RosgraphManipulator Action Server started.')
        self._movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


    def execute_cb(self, goal):
        rospy.loginfo ('Rosgraph Manipulator Action Server received goal %s' % str(goal))
        self._result.result = 1

        if (goal.desired_configuration_name == "low_power"):
            self.executeRequest("safe")
        elif (goal.desired_configuration_name == "standard"):
            self.executeRequest("standard")
        else:
            self._result.result = -1
            self._as.set_aborted(self._result)
            rospy.loginfo ('Unknown configuration request %s' % goal, log_level=rospy.ERROR)
            return

        self._as.set_succeeded(self._result)
        return

    def executeRequest(self, configuration="standard"):
        global nav_goal

        kill_node("/move_base")
        rospy.sleep(2)
        launch_config("metacontrol_nav", "move_base.launch", configuration)
        rospy.loginfo('launching new configuration')

        wait = self._movebase_client.wait_for_server(rospy.Duration(6.0))
        if not wait:
            rospy.logerr("MoveBase action server not available")
            return
        rospy.loginfo("Connected to move_base server and sending Nav Goal")

        print nav_goal
        self._movebase_client.send_goal( nav_goal )


    def executeSafeShutdown(self):
        rospy.loginfo('Safe shutdown')
        # TODO: complete how to safely shutdown the entire system

if __name__ == '__main__':
    rospy.init_node('rosgraph_manipulator_action_server')
    server = RosgraphManipulatorActionServer(rospy.get_name())
    rospy.spin()
