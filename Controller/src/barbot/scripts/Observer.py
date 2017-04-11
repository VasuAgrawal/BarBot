#!/usr/bin/env python

import rospy
from barbot.msg import State, Waypoint, Thruster

# The oberserver estimates the state of our robot, to help the controller 
class Observer(object):
    def __init__(self, name, state_topic):
        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher(state_topic, State, queue_size=1)
        self.state = State()
        while not rospy.is_shutdown():

            #Calculate state here!!
            self.state.pose.x = 0
            self.state.pose.y = 0
            self.state.pose.theta = 0
            self.state.vel.x = 0
            self.state.vel.y = 0
            self.state.vel.theta = 0

            self.pub.publish(self.state)
            self.rate.sleep()





if __name__ == '__main__':
    observer = Observer("observer", "state_topic")