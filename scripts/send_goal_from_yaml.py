#! /usr/bin/env python

import rospy

from actionlib import SimpleActionClient

from yaml import load
# import roslib
# roslib.load_manifest("rosparam")
# import rosparam
from rospkg import RosPack
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction


if __name__ == '__main__':
    rospy.init_node('send_goal_from_yaml_node')
    rospack = RosPack()

    # param = rosparam.load_file(rospack.get_path('metacontrol_sim')+'/yaml/goal.yaml')
    goal_yaml_file_name = rospy.get_param('/current_goal_yaml_file',
     rospack.get_path('metacontrol_sim')+'/yaml/goal.yaml')
    dict = load(file(goal_yaml_file_name, 'r'))
    nav_goal = MoveBaseGoal()
    nav_goal.target_pose.header.frame_id = dict['header']['frame_id']
    nav_goal.target_pose.pose.position.x = dict['pose']['position']['x']
    nav_goal.target_pose.pose.position.y = dict['pose']['position']['y']
    nav_goal.target_pose.pose.orientation.x = dict['pose']['orientation']['x']
    nav_goal.target_pose.pose.orientation.y = dict['pose']['orientation']['y']
    nav_goal.target_pose.pose.orientation.z = dict['pose']['orientation']['z']
    nav_goal.target_pose.pose.orientation.w = dict['pose']['orientation']['w']

    # Create SimpleActionClient
    movebase_client = SimpleActionClient('move_base', MoveBaseAction)
    # Wait for server to come up
    # I put it in a for to make sure it finds the action server
    # it may take a while to load all nodes
    for i in range(1, 5):
        wait = movebase_client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logwarn("Waiting for MoveBase action to become available")
        else:
            rospy.loginfo("Connected to move_base server and sending Nav Goal")
            break
    else:
        rospy.logerr("MoveBase action server not available after 5 attemps")

    print nav_goal
    movebase_client.send_goal( nav_goal )
    rospy.loginfo('[send_goal_from_yaml_node] Safe shutdown')
