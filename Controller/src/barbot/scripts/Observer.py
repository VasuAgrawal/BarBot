#!/usr/bin/env python

import rospy
from barbot.msg import State, Thruster, Euler
from geometry_msgs.msg import PointStamped, Vector3

# The oberserver estimates the state of our robot, to help the controller 
class Observer(object):
    def __init__(self, name, state_topic):
        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher(state_topic, State, queue_size=1)
        self.state = State()
        self.state.pose.x = 0
        self.state.pose.y = 0
        self.state.pose.theta = 0
        self.state.vel.x = 0
        self.state.vel.y = 0
        self.state.vel.theta = 0
        while not rospy.is_shutdown():

            #Calculate state here!!
            self.state.header.stamp = rospy.Time.now()

            self.pub.publish(self.state)
            self.rate.sleep()

    def location_listener(self, topic):
        rospy.Subscriber(topic, PointStamped, self.location_callback)

    def location_callback(self, data):
        self.state.pose.x = data.point.x
        self.state.pose.y = data.point.y

    def imu_listener(self, topic):
        rospy.Subscriber(topic+"/Euler", Euler, self.euler_callback)
        rospy.Subscriber(topic+"/LinearAcceleration", Vector3, self.linear_callback)
        rospy.Subscriber(topic+"/Gyroscope", Vector3, self.gyro_callback)

    def euler_callback(self, data):
        pass

    def linear_callback(self, data):
        pass

    def gyro_callback(self, data):
        pass
        


if __name__ == '__main__':
    observer = Observer("observer", "state_topic")
    observer.location_listener("/location_topic")
    observer.imu_listener("/imu_topic")