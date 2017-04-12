#!/usr/bin/env python

import rospy
from barbot.msg import State, Waypoint, Thruster

# The scheduler node is responsible for receiving ordering information, and sending over to our controller.
class Scheduler(object):
    def __init__(self, name, waypoint_topic):
        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher(waypoint_topic, Waypoint, queue_size=1)

    def push(self, waypoint):
        self.pub.publish(waypoint)

    # This function should connect to the actual scheduler, and push to robot as required
    def listen(self):
        pass


if __name__ == '__main__':
    scheduler = Scheduler("scheduler", "waypoint_topic")