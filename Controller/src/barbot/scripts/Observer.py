#!/usr/bin/env python

import rospy
import message_filters
from barbot.msg import Location, Euler
from geometry_msgs.msg import PointStamped


class Observer(object):
    def __init__(self, state_topic="calibrated_state", imu_topic="calibrated_imu", location_topic="calibrated_location"):

        self.state_pub = rospy.Publisher(state_topic, Location, queue_size=1)

        imu_sub = message_filters.Subscriber(imu_topic, Euler)
        location_sub = message_filters.Subscriber(location_topic, PointStamped)

        ts = message_filters.TimeSynchronizer([imu_sub, location_sub], 1)
        ts.registerCallback(self.callback)
    
    def callback(self, imu_data, location_data):
        time = location_data.header.stamp
        theta = imu_data.heading # I think???
        x = location_data.point.x
        y = location_data.point.y

        msg = Location()
        msg.header.stamp = time
        msg.pose.x = x
        msg.pose.y = y
        msg.pose.theta = theta

        self.state_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("observer")
    controller = Observer()
    rospy.spin()
