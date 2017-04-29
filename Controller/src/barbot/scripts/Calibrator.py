#!/usr/bin/env python
from __future__ import print_function

import time
import threading
import collections

import rospy
from barbot.msg import Euler
from geometry_msgs.msg import PointStamped


location_deq = collections.deque(maxlen=10)
imu_deq = collections.deque(maxlen=10)

calibration_points = []
calibrated = False


def handle_location(data):
    location_deq.append(data)

    # If calibrated, convert this reading into a calibrated one and republish.
    if calibrated:
        location_pub.publish(data) # Do something better here


def handle_imu(data):
    imu_deq.append(data)
    
    # If calibrated, convert this reading into a calibrated one and republish.
    if calibrated:
        location_pub.publish(data)


def average(iterable):
    if iterable:
        return sum(iterable) / len(iterable)
    return 0


class CalibrationPoint(object):
    def __init__(self, x, y, z, roll, pitch, heading):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.heading = heading


def handle_user_input():
    instructions = ("Move the robot to the %s corner, with some fixed" +
    " orientation relative to the corner. Let the robot sit in place for 10" +
    " seconds before pressing ENTER.")
    
    locations = ["BOTTOM LEFT", "TOP LEFT", "TOP RIGHT", "BOTTOM RIGHT"]

    rospy.loginfo("Beginning calibration sequence!")
    time.sleep(1) # Let it find a logger node, this takes time?
    rospy.loginfo("Please follow the instructions below to calibrate the robot")

    for location in locations:
        rospy.logwarn(instructions % location)
        raw_input()

        # We know that the user has held the robot in place for the necessary
        # amount of time. We can now take the last few readings from both of the
        # deqs and average them all together to develop some reading for this
        # point.

        location_data = []
        imu_data = []
        DESIRED = 5

        # TODO: More error handling
        for i in range(10):
            if len(location_data) < DESIRED:
                try:
                    location_data.append(location_deq.popleft())
                except IndexError:
                    rospy.logwarn("Error in getting location data")

            # if len(imu_data) < DESIRED:
                # try:
                    # imu_data.append(imu_deq.popleft())
                # except IndexError:
                    # rospy.logwarn("Error in getting imu data")

        avg_x = average([data.point.x for data in location_data])
        avg_y = average([data.point.y for data in location_data])
        avg_z = average([data.point.z for data in location_data])
        avg_heading = average([data.heading for data in imu_data])
        avg_roll = average([data.roll for data in imu_data])
        avg_pitch = average([data.pitch for data in imu_data])
        rospy.loginfo("Calibration parameters at %s:\n"
                "location: [x:    %f, y:     %f, z:       %f]\n"
                "imu:      [roll: %f, pitch: %f, heading: %f]\n",
                location, avg_x, avg_y, avg_z, avg_roll,
                avg_pitch, avg_heading)
        calibration_points.append(CalibrationPoint(avg_x, avg_y, avg_z,
            avg_roll, avg_pitch, avg_heading))


    global calibrated
    calibrated = True

    rospy.loginfo("Ending calibration sequence.")
    rospy.loginfo("Beginning to send data on calibrated channels.")


if __name__ == "__main__":

    threading.Thread(target=handle_user_input).start()
    
    global location_pub
    global imu_pub
    location_pub = rospy.Publisher("calibrated_location", PointStamped, 
            queue_size=1)
    imu_pub = rospy.Publisher("calibrated_imu", Euler, queue_size=1)

    rospy.init_node("Calibrator", log_level=rospy.INFO)
    rospy.Subscriber("raw_location", PointStamped, handle_location)
    rospy.Subscriber("imu_topic/Euler", Euler, handle_imu)
    rospy.spin()
