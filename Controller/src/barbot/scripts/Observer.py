#!/usr/bin/env python

import rospy
import message_filters
from barbot.msg import Location, Euler
from geometry_msgs.msg import PointStamped


class Observer(object):
    def __init__(self, state_topic="calibrated_state", imu_topic="calibrated_imu", location_topic="calibrated_location"):

        self.state_pub = rospy.Publisher(state_topic, Location, queue_size=1)

        imu_sub = rospy.Subscriber(imu_topic, Euler, self.imu_callback)
        location_sub = rospy.Subscriber(location_topic, PointStamped, self.location_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    
    def location_callback(self, location_data):
        self.x = location_data.point.x
        self.y = location_data.point.y


    def imu_callback(self, imu_data):
        self.theta = imu_data.heading
        msg = Location()

        msg.pose.x = self.x
        msg.pose.y = self.y
        msg.pose.theta = self.theta

        self.state_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("observer")
    controller = Observer()
    rospy.spin()
