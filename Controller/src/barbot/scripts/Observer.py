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

        self.theta = 0.0

    
    def location_callback(self, location_data):
        time = location_data.header.stamp
        x = location_data.point.x
        y = location_data.point.y

        msg = Location()
        msg.header.stamp = time
        msg.pose.x = x
        msg.pose.y = y
        msg.pose.theta = self.theta

        self.state_pub.publish(msg)

    def imu_callback(self, imu_data):
        self.theta = imu_data.heading

if __name__ == '__main__':
    rospy.init_node("observer")
    controller = Observer()
    rospy.spin()
