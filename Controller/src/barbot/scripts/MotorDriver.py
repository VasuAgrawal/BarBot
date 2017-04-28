#!/usr/bin/env python
import rospy
from barbot.msg import Thruster

def writeMotor(data):
    rospy.loginfo("Writing %f to left, %f to right", data.left, data.right)

def main():
    rospy.init_node("Motor_controller")
    rospy.Subscriber("thruster", Thruster, writeMotor)
    rospy.spin()

if __name__ == "__main__":
    main()
