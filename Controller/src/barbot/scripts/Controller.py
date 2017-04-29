#!/usr/bin/env python
import rospy
from barbot.msg import Location, Thruster
import math

class Controller(object):
    def __init__(self, kp, ki, kd, speed=0.5, turn_speed=0.25, threshold=0.1, theta_threshold=0.05, 
        thruster_topic="thruster", waypoint_topic="waypoint", state_topic="state"):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.speed = speed
        self.threshold = threshold
        self.theta_threshold = theta_threshold
        self.turn_speed = turn_speed

        self.error = 0.0
        self.last_error = 0.0
        self.waypoint = None
        self.running = False
        self.last_time = rospy.get_time()
        self.rate = rospy.Rate(30)
        self.thruster_pub = rospy.Publisher(thruster_topic, Thruster, queue_size=1)

        rospy.Subscriber(state_topic, Location, self.state_callback)
        rospy.Subscriber(waypoint_topic, Location, self.waypoint_callback)

    def waypoint_callback(self, data):
        self.running = True
        self.waypoint = data.pose

    def state_callback(self, data):
        left = 0.0
        rigth = 0.0
        distance2 = 0.0
        theta_error = 0.0

        state = data.pose
        now = data.header.stamp

        if (self.waypoint != None and self.running):
            err_x = self.waypoint.pose.x - state.x
            err_y = self.waypoint.pose.y - state.y

            distance2 = err_x*err_X + err_y*err_y

            if (distance2 > self.threshold*self.threshold):

                goal_theta = math.atan2(err_y, err_x)
                theta_error = ((goal_theta - state.theta) + math.pi) % (2*math.pi) - math.pi

                if(abs(theta_error) > self.theta_threshold):
                    left = math.copysign(self.turn_speed, theta_error)
                    right = -math.copysign(self.turn_speed, theta_error)

                else:
                    self.error += theta_error

                    now = rospy.get_time()
                    dt = now-self.last_time
                    derr = theta_error-self.last_error
                    dterm = derr / dt
                    change = theta_error*self.kp + self.error*self.ki + dterm*self.kd

                    self.last_time = now
                    self.last_error = theta_error

                    left = self.speed + change
                    right = self.speed - change
            else:
                self.running = False # have arrived
                left = 0.
                right = 0.

        else:
            self.running = False # have arrived
            left = 0.
            right = 0.


        #normalize between 0 and 1
        max_thrust = max(abs(left), abs(right))
        if (max_thrust > 1):
            left = left / max_thrust
            right = right / max_thrust

        msg = Thruster()
        msg.header.stamp = rospy.Time.now()
        msg.left = left
        msg.right = right

        print("distance2 is %f, theta_error is %f, thruster left is %f, thruster right is %f" % (distance2, theta_error, left, right))
        self.thruster_pub.publish(msg)




if __name__ == '__main__':
    rospy.init_node("controller")
    controller = Controller(1., 0., 0)
    rospy.spin()