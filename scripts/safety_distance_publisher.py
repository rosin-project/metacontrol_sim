#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import isnan, isinf
from threading import Lock

class SafetyDistance(object):

    def __init__(self):

        rospy.init_node('safety_distance_computation_node', anonymous=False)
        publish_frequency = rospy.get_param('~publish_frequency', 5.0)
        laser_scan_topic_name = rospy.get_param('~scan_topic', '/scan_filtered')
        d_obstacle_topic_name = rospy.get_param('~d_obstacle_topic', 'd_obstacle')
        d_inflation_topic_name = rospy.get_param('~d_inflation_topic', 'd_inflation')
        self.inflation_radius = rospy.get_param('/move_base/local_costmap/inflater_layer/inflation_radius', 0.35)
        self.robot_radius = rospy.get_param('/move_base/local_costmap/robot_radius', 0.4)
        
        # Subscribe to laser scanS
        rospy.Subscriber(laser_scan_topic_name, LaserScan, self.laser_scan_callback)
        # Create timer to change power load perdiodically
        rospy.Timer(rospy.Duration(1.0 / publish_frequency), self.timer_callback)
        # Add publisher to publish power_load value
        self.d_obstacle_publisher = rospy.Publisher(d_obstacle_topic_name, Float32, queue_size=1)
        self.d_inflation_publisher = rospy.Publisher(d_inflation_topic_name, Float32, queue_size=1)

        # Initialize variables
        self.d_obstacle = 100
        self.d_inflation = 100
        #self.components = 1.0
        self.lock = Lock()
        
    def laser_scan_callback(self, scan_data):

        # Set acceleration_value if it's larger than previuos value
        #rospy.loginfo("[laser_scan_callback]: Number of Scan data: %s", str(len(scan_data.ranges)))
        
        for range_index in range(len(scan_data.ranges)):            
            if isnan(scan_data.ranges[range_index]):
                continue
            if isinf(scan_data.ranges[range_index]):
                continue
            
            # rospy.loginfo("[laser_scan_callback]: Scan data: %s", str(scan_data.ranges[range_index]))
            if self.d_obstacle > scan_data.ranges[range_index]:
                with self.lock:
                    self.d_obstacle = scan_data.ranges[range_index]
                    # rospy.loginfo("[laser_scan_callback]: New min scan data: %s", str(self.d_obstacle))

    def timer_callback(self, event):

        d_obstacle_msg = Float32(self.d_obstacle)
        self.d_obstacle_publisher.publish(d_obstacle_msg)

        d_inflation_msg = Float32(self.d_inflation)
        self.d_inflation_publisher.publish(d_inflation_msg)

        rospy.loginfo("New Safety distance: %s", self.d_obstacle)

        with self.lock:
            # Re-Initialize variables
            self.d_obstacle = 100
            self.d_inflation = 100
     

if __name__ == "__main__":
    
    rospy.loginfo("Main function")

    try:
        SafetyDistance()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
    pass

