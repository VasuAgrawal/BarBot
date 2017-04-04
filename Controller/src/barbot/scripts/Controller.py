#!/usr/bin/env python

import rospy
from barbot.msg import State, Waypoint, Thruster
from geometry_msgs.msg import Pose2D
import math

class Controller(object):
    def __init__(self, name, thruster_topic, kp, ki, kd, speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.speed = speed
        self.waypoint = None
        self.state = State()
        self.error = 0.0
        self.theta_threshold = 0.1
        self.turn_speed = 10

        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.thruster_pub = rospy.Publisher(thruster_topic, Thruster, queue_size=1)

    def state_listener(self, topic):
        rospy.Subscriber(topic, State, self.state_callback)

    def state_callback(self, data):
        self.state = data
        if (self.waypoint != None):
            robot_waypoint_x = self.waypoint.pose.x - self.state.pose.x
            robot_waypoint_y = self.waypoint.pose.y - self.state.pose.y
            robot_waypoint_t = math.atan2(robot_waypoint_y, robot_waypoint_x)
            error = ((robot_waypoint_t - self.state.pose.theta) + math.pi) % (2*math.pi) - math.pi

            # angle differs by a lot, turn the robot, this can just be hardcoded because PID fixes slight differences
            if(abs(error) > theta_threshold):
                msg = Thruster()
                msg.left = math.copysign(self.turn_speed, error)
                msg.right =- math.copysign(self.turn_speed, error)
                self.thruster_pub.publish(msg)

            else:
                self.error += error

                # ignore kd for now 
                change = error*self.kp + self.error*self.ki

                msg = Thruster()
                msg.left = self.speed + change
                msg.right = self.speed - change
                self.thruster_pub.publish(msg)



                # Do PID control here to figure out thrusters
        else:
            msg = Thruster()
            msg.left = 0.
            msg.right = 0.
            self.thruster_pub.publish(msg)



    # once a new waypoint comes in, overwrites the old one
    def waypoint_listener(self, topic):
        rospy.Subscriber(topic, Waypoint, self.waypoint_callback)

    def waypoint_callback(self, data):
        self.waypoint = data

if __name__ == '__main__':
    controller = Controller("controller", "thruster", 1., 0., 0., 1.)
    controller.waypoint_listener("waypoint_topic")
    controller.state_listener("state_topic")
    rospy.spin()