#! /usr/bin/env python

import rospy

from yaml import load
# import roslib
# roslib.load_manifest("rosparam")
# import rosparam
from rospkg import RosPack
from controller_manager_msgs.srv import LoadController, LoadControllerRequest

if __name__ == '__main__':
    rospy.init_node('activate_observer_node')
    rospack = RosPack()

    # param = rosparam.load_file(rospack.get_path('metacontrol_sim')+'/yaml/goal.yaml')
    observer_dict = load(file(rospack.get_path(
        'metacontrol_sim')+'/yaml/observers.yaml', 'r'))
    for observer_entry in observer_dict:
        print observer_entry
        observer_req = LoadControllerRequest()
        observer_req.name = observer_dict[observer_entry]['name']
        print observer_req.name 
        # Wait for the observer service to become available.
        rospy.loginfo("Waiting for service...")
        rospy.wait_for_service('/load_observer')
        try:
            # Create a service proxy.
            observer_srv_proxy = rospy.ServiceProxy('/load_observer', LoadController)
            # Call the service here.
            service_response = observer_srv_proxy(observer_req)
            print("The load observer service call is completed!")
        except rospy.ServiceException, e:
            print "load observer Service call failed: %s"%e

    rospy.loginfo('Safe shutdown')
