#!/usr/bin/env python

import rospy
from brass_gazebo_battery.srv import SetLoad
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rostopic import ROSTopicHz
from threading import Lock


class BatteryLoadController:

    def __init__(self, init_load=1.0):


        rospy.init_node('battery_load_controller_node', anonymous=False)
        const_linear_vel = rospy.get_param('~const_linear_vel', 1.3)
        const_angular_vel = rospy.get_param('~const_angular_vel', 1.3)
        const_acceleration = rospy.get_param('~const_acceleration', 0.1)
        const_frequency = rospy.get_param('~const_frequency', 0.04)
        additional_consumption = rospy.get_param('~additional_consumption', 0.0)
        odom_topic_name = rospy.get_param('~odom_topic', '/odom')
        imu_topic_name = rospy.get_param('~imu_topic', '/imu/data')
        self.cmd_vel_topic_name = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        controller_frequency = rospy.get_param('~controller_frequency', 5.0)
        self.min_power_load = rospy.get_param('~min_power_load', 0.2)
        self.max_power_load = rospy.get_param('~max_power_load', 5.0)
        controller_frequency = rospy.get_param('~controller_frequency', 5.0)
        rospy.loginfo("Controller frequency: %s", str(controller_frequency))
        # Subscribe to Odometry
        rospy.Subscriber(odom_topic_name, Odometry, self.odometry_callback)
        # Subscribe to Imu
        rospy.Subscriber(imu_topic_name, Imu, self.imu_callback)
        # Create timer to change power load perdiodically
        rospy.Timer(rospy.Duration(1.0 / controller_frequency), self.timer_callback)

        # Create ROSTopicHz to get frequency of controller
        self.ros_topic_hz = ROSTopicHz(window_size=100, filter_expr=None)
        # add Subscriber to /cmd_vel to ROSTopicHz to get frequency of controller
        self.hz_subscriber = rospy.Subscriber(self.cmd_vel_topic_name, rospy.AnyMsg, self.ros_topic_hz.callback_hz, callback_args=self.cmd_vel_topic_name)

        # Initialize variables

        self.counter = 0
        self.const_linear_vel = const_linear_vel
        self.const_angular_vel = const_angular_vel
        self.const_acceleration = const_acceleration
        self.const_frequency = const_frequency
        self.additional_consumption = additional_consumption
        self.power_load = init_load
        self.linear_vel_value = 0
        self.angular_vel_value = 0
        self.frequency_value = 0
        self.acceleration_value = 0
        #self.components = 1.0
        self.lock = Lock()
        # First wait for the service to become available.
        rospy.loginfo("Waiting for service...")
        rospy.wait_for_service('/battery_demo_model/set_power_load')
        rospy.loginfo("battery_load_controller_node Initialization completed")


    def add_consumption_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {additional_consumption}""".format(**config))
        return config

        # Set acceleration_value if it's larger than previuos value
        if imu_data.linear_acceleration.x > self.acceleration_value:
            with self.lock:
                self.acceleration_value = imu_data.linear_acceleration.x

        # print self.acceleration_value
        # print acceleration_y


    def imu_callback(self, imu_data):

        # Set acceleration_value if it's larger than previuos value
        if imu_data.linear_acceleration.x > self.acceleration_value:
            with self.lock:
                self.acceleration_value = imu_data.linear_acceleration.x

        # print self.acceleration_value
        # print acceleration_y

    def odometry_callback(self, odom_data):

        # Set linear_vel_value if it's larger than previuos value
        if odom_data.twist.twist.linear.x > self.linear_vel_value:
            with self.lock:
                self.linear_vel_value = odom_data.twist.twist.linear.x
        # Set angular_vel_value if larger than previuos value
        # if odom_data.twist.twist.angular.z > self.angular_vel_value:
        #     with self.lock:
        #         self.angular_vel_value = odom_data.twist.twist.angular.z
        # print self.linear_vel_value


    def timer_callback(self, event):

        times = self.ros_topic_hz.get_hz(self.cmd_vel_topic_name)
        # times = self.ros_topic_hz.get_times()
        if times is not None:
            self.frequency_value = times[0]
        else:
            #print 'Frequency is ' + str(times)
            self.frequency_value = 0

        # Compute "instantaneous" power load
        # self.angular_vel_value * self.const_angular_vel + \
        self.power_load = self.linear_vel_value * self.const_linear_vel + \
            self.acceleration_value * self.const_acceleration + \
            self.frequency_value * self.const_frequency + \
            self.additional_consumption

        # Set minimum power load
        if self.power_load < self.min_power_load:
            self.power_load = self.min_power_load

        # Set maximun power load
        if self.power_load > self.max_power_load:
            self.power_load = self.max_power_load


        with self.lock:
            self.linear_vel_value = 0
            self.angular_vel_value = 0
            self.acceleration_value = 0

        try:
            # Create a service proxy.
            set_load_service = rospy.ServiceProxy('/battery_demo_model/set_power_load', SetLoad)

            # Call the service here.
            service_response = set_load_service(self.power_load)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # rospy.loginfo("New Power load: %s", self.power_load)


if __name__ == '__main__':

    rospy.loginfo("Main function")

    try:
        BatteryLoadController()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
