#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool
#
# MSG: actionlib_msgs/GoalStatus\n\
# GoalID goal_id\n\
# uint8 status\n\
# uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
# uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
# uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
#                             #   and has since completed its execution (Terminal State)\n\
# uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
# uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
#                              #    to some failure (Terminal State)\n\
# uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
#                              #    because the goal was unattainable or invalid (Terminal State)\n\
# uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
#                             #    and has not yet completed execution\n\
# uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
#                             #    but the action server has not yet confirmed that the goal is canceled\n\
# uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
#                             #    and was successfully cancelled (Terminal State)\n\
# uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
#                             #    sent over the wire by an action server\n\
#

class CheckNavError(object):

    def __init__(self):

        rospy.init_node('objetive_error_check_node', anonymous=False)
        publish_frequency = rospy.get_param('~publish_frequency', 5.0)
        # move_base_result_topic_name = rospy.get_param('~result_topic', '/move_base/result')
        obj_error_topic_name = rospy.get_param('~obj_error_topic', '/obj_error')
        #Max allowed time before calling it over
        self.max_time_errors = rospy.get_param('~max_error_msgs', 5)


        # Subscribe to move base result
        # rospy.Subscriber(move_base_result_topic_name, MoveBaseActionResult, self.move_base_result_callback)
        rospy.Subscriber('/rosout', Log, self.rosout_callback)
        # Create timer to change power load perdiodically
        rospy.Timer(rospy.Duration(1.0 / publish_frequency), self.timer_callback)
        # Add publisher to publish power_load value
        self.obj_error_publisher = rospy.Publisher(obj_error_topic_name, Bool, queue_size=1)
        self.obj_error = False
        self.count_error_msgs = 0
        self.move_base_result = MoveBaseActionResult()

    def rosout_callback(self, rosout_data):
        
        if rosout_data.level == Log().ERROR:
            if rosout_data.name == str("/move_base"):
                self.obj_error = True
                print str(rosout_data.msg)



    def move_base_result_callback(self, result_data):
        if result_data.status.status == 4: # Goal aborted
            self.obj_error = True
            print "Objective error received"
        elif result_data.status.status == 5: # Goal rejected
            self.obj_error = True
            print "Objective error received"



    def timer_callback(self, event):
        obj_error_msg = Bool(self.obj_error)
        if(self.obj_error):
            self.count_error_msgs = self.count_error_msgs + 1
            if(self.count_error_msgs > self.max_time_errors):
                self.obj_error = False
                self.count_error_msgs = 0

        self.obj_error_publisher.publish(obj_error_msg)



if __name__ == "__main__":

    rospy.loginfo("Main function")

    try:
        CheckNavError()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
    pass
