#!/usr/bin/env python
import rospy
from barbot.msg import Location, Thruster
import Tkinter
import math
from geometry_msgs.msg import PointStamped

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

        self.error_var.set(self.error)
        self.left.set(left)
        self.right.set(right)

        print("distance2 is %f, theta_error is %f, thruster left is %f, thruster right is %f" % (distance2, theta_error, left, right))
        self.thruster_pub.publish(msg)

    def ui(self):
        Tkinter.Label(master=self.root, text="kp").grid(row=0, column=0)
        Tkinter.Label(master=self.root, text="ki").grid(row=1, column=0)
        Tkinter.Label(master=self.root, text="kd").grid(row=2, column=0)

        Tkinter.Label(master=self.root, text="left").grid(row=4, column=0)
        Tkinter.Label(master=self.root, text="right").grid(row=5, column=0)
        Tkinter.Label(master=self.root, text="error").grid(row=6, column=0)

        Tkinter.Label(master=self.root, textvariable=self.left).grid(row=4, column=1)
        Tkinter.Label(master=self.root, textvariable=self.right).grid(row=5, column=1)
        Tkinter.Label(master=self.root, textvariable=self.error_var).grid(row=6, column=1)

        self.kp_entry=Tkinter.Entry(master=self.root)
        self.kp_entry.grid(row=0, column=1)
        self.kp_entry.insert(Tkinter.END, str(self.kp))

        self.ki_entry=Tkinter.Entry(master=self.root)
        self.ki_entry.grid(row=1, column=1)
        self.ki_entry.insert(Tkinter.END, str(self.ki))

        self.kd_entry=Tkinter.Entry(master=self.root)
        self.kd_entry.grid(row=2, column=1)
        self.kd_entry.insert(Tkinter.END, str(self.kd))

        Tkinter.Button(master=self.root, text="Apply", command=self.updatePID).grid(row=3, column=1, columnspan=2)
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
