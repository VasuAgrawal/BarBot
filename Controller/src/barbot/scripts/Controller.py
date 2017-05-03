#!/usr/bin/env python
import rospy
from barbot.msg import Location, Thruster
import Tkinter
import math
from geometry_msgs.msg import PointStamped
import matplotlib
import numpy as np

class Controller(object):
    def __init__(self, kp, ki, kd, speed=0.5, turn_speed=0.25, threshold=0.1,
            theta_threshold=0.15,
        thruster_topic="thruster", waypoint_topic="calibrated_waypoint", state_topic="calibrated_state"):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.speed = speed
        self.threshold = threshold
        self.theta_threshold = theta_threshold
        self.turn_speed = turn_speed

        self.error = 0.0
        self.last_error = 0.0

        self.root = Tkinter.Tk()
        self.left = Tkinter.DoubleVar()
        self.right = Tkinter.DoubleVar()

        self.error_var = Tkinter.DoubleVar()
        self.theta_error_var = Tkinter.DoubleVar()

        self.state_x = Tkinter.DoubleVar()
        self.state_y = Tkinter.DoubleVar()
        self.state_t = Tkinter.DoubleVar()

        self.waypoint_x = Tkinter.DoubleVar()
        self.waypoint_y = Tkinter.DoubleVar()

        self.waypoint = None
        self.running = False
        self.last_time = rospy.get_time()
        self.rate = rospy.Rate(30)
        self.thruster_pub = rospy.Publisher(thruster_topic, Thruster, queue_size=1)

        rospy.Subscriber(state_topic, Location, self.state_callback)
        rospy.Subscriber(waypoint_topic, PointStamped, self.waypoint_callback)

        rospy.loginfo("Controller initialized");

    def waypoint_callback(self, data):
        self.running = True
        self.waypoint = data
        self.waypoint_x.set(data.point.x)
        self.waypoint_y.set(data.point.y)

    def state_callback(self, data):
        left = 0.0
        right = 0.0
        distance2 = 0.0
        theta_error = 0.0

        state = data.pose
        now = data.header.stamp

        if (self.waypoint != None and self.running):
            err_x = self.waypoint.point.x - state.x
            err_y = self.waypoint.point.y - state.y

            distance2 = err_x*err_x + err_y*err_y
            self.error_var.set(math.sqrt(distance2))

            if (distance2 > self.threshold*self.threshold):
                goal_theta = math.atan2(err_y, -err_x)
                # goal_theta = math.atan2(err_x, err_y)
                # theta_error = ((goal_theta - state.theta) + math.pi) % (2*math.pi) - math.pi
                theta_error = (((goal_theta - state.theta + math.pi) + 2*math.pi) %
                                (2*math.pi))
                if theta_error > math.pi:
                    theta_error -= 2*math.pi

                self.theta_error_var.set(theta_error)

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


        self.left.set(left)
        self.right.set(right)

        self.state_x.set(state.x)
        self.state_y.set(state.y)
        self.state_t.set(state.theta)

        print("distance2 is %f, theta_error is %f, thruster left is %f, thruster right is %f" % (distance2, theta_error, left, right))
        self.thruster_pub.publish(msg)

    def ui(self):
        Tkinter.Label(master=self.root, text="PID").grid(row=0, column=0, columnspan=2, padx=50)
        Tkinter.Label(master=self.root, text="Thrusters").grid(row=0, column=2, columnspan=2, padx=50)
        Tkinter.Label(master=self.root, text="Error").grid(row=0, column=4, columnspan=2, padx=50)
        Tkinter.Label(master=self.root, text="Robot State").grid(row=0, column=6, columnspan=2, padx=50)
        Tkinter.Label(master=self.root, text="Waypoint").grid(row=0, column=8, columnspan=2, padx=50)

        Tkinter.Label(master=self.root, text="kp").grid(row=1, column=0)
        Tkinter.Label(master=self.root, text="ki").grid(row=2, column=0)
        Tkinter.Label(master=self.root, text="kd").grid(row=3, column=0)

        Tkinter.Label(master=self.root, text="left:").grid(row=1, column=2)
        Tkinter.Label(master=self.root, text="right:").grid(row=2, column=2)

        Tkinter.Label(master=self.root, text="dsitance:").grid(row=1, column=4)
        Tkinter.Label(master=self.root, text="theta:").grid(row=2, column=4)

        Tkinter.Label(master=self.root, text="x:").grid(row=1, column=6)
        Tkinter.Label(master=self.root, text="y:").grid(row=2, column=6)
        Tkinter.Label(master=self.root, text="theta:").grid(row=3, column=6)

        Tkinter.Label(master=self.root, text="x:").grid(row=1, column=8)
        Tkinter.Label(master=self.root, text="y:").grid(row=2, column=8)

        Tkinter.Label(master=self.root, textvariable=self.left).grid(row=1, column=3)
        Tkinter.Label(master=self.root, textvariable=self.right).grid(row=2, column=3)

        Tkinter.Label(master=self.root, textvariable=self.error_var).grid(row=1, column=5)
        Tkinter.Label(master=self.root, textvariable=self.theta_error_var).grid(row=2, column=5)

        Tkinter.Label(master=self.root, textvariable=self.state_x).grid(row=1, column=7)
        Tkinter.Label(master=self.root, textvariable=self.state_y).grid(row=2, column=7)
        Tkinter.Label(master=self.root, textvariable=self.state_t).grid(row=3, column=7)

        Tkinter.Label(master=self.root, textvariable=self.waypoint_x).grid(row=1, column=9)
        Tkinter.Label(master=self.root, textvariable=self.waypoint_y).grid(row=2, column=9)

        self.kp_entry=Tkinter.Entry(master=self.root, width=10)
        self.kp_entry.grid(row=1, column=1)
        self.kp_entry.insert(Tkinter.END, str(self.kp))

        self.ki_entry=Tkinter.Entry(master=self.root, width=10)
        self.ki_entry.grid(row=2, column=1)
        self.ki_entry.insert(Tkinter.END, str(self.ki))

        self.kd_entry=Tkinter.Entry(master=self.root, width=10)
        self.kd_entry.grid(row=3, column=1)
        self.kd_entry.insert(Tkinter.END, str(self.kd))

        Tkinter.Button(master=self.root, text="Update", command=self.updatePID).grid(row=4, column=0, columnspan=2)
        Tkinter.mainloop()

    def updatePID(self):
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            self.kp = kp
            self.ki = ki
            self.kd = kd

            rospy.loginfo("PID parameters updated");
        except:
            self.kp_entry.delete(0, Tkinter.END)
            self.ki_entry.delete(0, Tkinter.END)
            self.kd_entry.delete(0, Tkinter.END)

            self.kp_entry.insert(Tkinter.END, str(self.kp))
            self.ki_entry.insert(Tkinter.END, str(self.ki))
            self.kd_entry.insert(Tkinter.END, str(self.kd))

            print("Invalid Entries")







if __name__ == '__main__':
    rospy.init_node("controller")
    controller = Controller(1., 0., 0.)
    controller.ui()
    rospy.spin()
