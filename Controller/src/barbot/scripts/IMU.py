#!/usr/bin/env python

import rospy
from Adafruit_BNO055 import BNO055
from geometry_msgs.msg import Quaternion, Vector3
from barbot.msg import Euler


# IMU serialize to ROS, will be used in observer
class IMU(object):
    def __init__(self, name, topic_name):
        # initialize the IMU
        self.bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        if not self.bno.begin():
            raise Exception("IMU don't work :(")
        status, self_test, error = self.bno.get_system_status()
        if status == 0x01:
            print("IMU in error mode! Error: %r" % error)

        # ROS stuff
        rospy.init_node(name)
        rate = rospy.Rate(10)

        eulerPub = rospy.Publisher(topic_name+'/Euler', Euler)
        quatPub = rospy.Publisher(topic_name+'/Quaternion', Quaternion)
        magPub = rospy.Publisher(topic_name+'/Magnetometer', Vector3)
        gyroPub = rospy.Publisher(topic_name+'/Gyroscope', Vector3)
        accelPub = rospy.Publisher(topic_name+'/Accelerometer', Vector3)
        linearPub = rospy.Publisher(topic_name+'/LinearAcceleration', Vector3)
        gravityPub = rospy.Publisher(topic_name+'/GravityAcceleration', Vector3)

        while not rospy.is_shutdown():
            eulerPub.publish(self.getEuler())
            quatPub.publish(self.getQuaternion())
            magPub.publish(self.getMagnetometer())
            gyroPub.publish(self.getGyroscope())
            accelPub.publish(self.getAccelerometer())
            linearPub.publish(self.getLinearAcceleration())
            gravityPub.publish(self.getGravityAcceleration())

            rate.sleep()

    def getEuler(self):
        # Euler angles in degrees
        heading, roll, pitch = self.bno.read_euler()
        return Euler(heading, roll, pitch)

    def getQuaternion(self):
        x, y, z, w = self.bno.read_quaterion()
        return Quaternion(x, y, z, w)

    def getMagnetometer(self):
        # in micro-Teslas
        x, y, z = self.bno.read_magnetometer()
        return Vector3(x, y, z)

    def getGyroscope(self):
        # in degrees per second
        x, y, z = self.bno.read_gyroscope()
        return Vector3(x, y, z)

    def getAccelerometer(self):
        # in meters per second squared
        x, y, z = self.bno.read_accelerometer()
        return Vector3(x, y, z)

    def getLinearAcceleration(self):
        # acceleration from movement, not gravity
        # in meters per second squared
        x, y, z = self.bno.read_linear_acceleration()
        return Vector3(x, y, z)

    def getGravityAcceleration(self):
        # acceleration just from gravity
        # in meters per second squared
        x, y, z = self.bno.read_gravity()
        return Vector3(x, y, z)

if __name__ == '__main__':
    imu = IMU("IMU", "/imu_topic")