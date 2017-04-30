#!/usr/bin/env python
from __future__ import print_function

import time
import threading
import collections

import rospy
from barbot.msg import Euler
from geometry_msgs.msg import PointStamped

import numpy as np
import math

location_deq = collections.deque(maxlen=10)
imu_deq = collections.deque(maxlen=10)

calibration_points = []
calibrated = False

pool_plane_origin = np.array([0.0, 0.0, 0.0])
pool_plane_normal = np.array([0.0, 0.0, 0.0])
rmat = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

imu_offset = 0.0

def handle_location(data):
    location_deq.append(data)

    # If calibrated, convert this reading into a calibrated one and republish.
    if calibrated:
        # Project uncalibrated location onto the pool plane
        # http://stackoverflow.com/questions/9605556/how-to-project-a-3d-point-to-a-3d-plane
        raw = np.array([data.point.x, data.point.y, data.point.z])
        v = raw - pool_plane_origin
        dist = np.dot(v, pool_plane_normal)
        projected = raw - dist*pool_plane_normal

        # Rotate data from pool plane to XY plane
        final = rmat.dot(projected)

        # Publish normalized data
        (data.point.x,data.point.y,data.point.z) = (final[0],final[1],final[2])
        location_pub.publish(data)

def handle_imu(data):
    imu_deq.append(data)
    
    # If calibrated, convert this reading into a calibrated one and republish.
    if calibrated:
        data.heading += imu_offset
        imu_pub.publish(data)

def handle_waypoint(data):
    # If calibrated, convert this reading into a calibrated one and republish.
    if calibrated:
        # Project uncalibrated location onto the pool plane
        # http://stackoverflow.com/questions/9605556/how-to-project-a-3d-point-to-a-3d-plane
        raw = np.array([data.point.x, data.point.y, data.point.z])
        v = raw - pool_plane_origin
        dist = np.dot(v, pool_plane_normal)
        projected = raw - dist*pool_plane_normal

        # Rotate data from pool plane to XY plane
        final = rmat.dot(projected)

        # Publish normalized data
        (data.point.x,data.point.y,data.point.z) = (final[0],final[1],final[2])
        waypoint_pub.publish(data)        

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
    instructions = ("Move the robot to the %s corner, pointing towards" +
    " the right wall. Let the robot sit in place for 10" +
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

            if len(imu_data) < DESIRED:
                try:
                    imu_data.append(imu_deq.popleft())
                except IndexError:
                    rospy.logwarn("Error in getting imu data")

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

    # Compute information about the pool plane from calibration data
    # Use bottom left, top left, bottom right
    p0 = calibration_points[0]
    p1 = calibration_points[1]
    p2 = calibration_points[3]

    # Use bottom left calibration point as origin of pool plane
    global pool_plane_origin
    pool_plane_origin = np.array([p0.x, p0.y, p0.z])

    # Compute normal of the pool plane
    v1 = np.array([p1.x-p0.x, p1.y-p0.y, p1.z-p0.z])
    v2 = np.array([p2.x-p0.x, p2.y-p0.y, p2.z-p0.z])
    global pool_plane_normal
    pool_plane_normal = np.cross(v1, v2) / np.linalg.norm(np.cross(v1, v2))

    # Compute rotation matrix of pool plane from Z axis
    # http://stackoverflow.com/questions/9423621/3d-rotations-of-a-plane
    M = pool_plane_normal # Normal vector to pool plane
    N = np.array([0.0, 0.0, 1.0]) # Normal vector to plane I am rotating to (XY)
    costheta = np.dot(M, N) / (np.linalg.norm(M) * np.linalg.norm(N))
    axis = np.cross(M, N) / np.linalg.norm(np.cross(M, N))

    c = costheta
    s = math.sqrt(1-c*c)
    C = 1-c
    (x, y, z) = (axis[0], axis[1], axis[2])

    global rmat
    rmat = np.array([[x*x*C+c,   x*y*C-z*s, x*z*C+y*s],
                    [y*x*C+z*s, y*y*C+c,   y*z*C-x*s],
                    [z*x*C-y*s, z*y*C+x*s, z*z*C+c  ]])

    # Compute rotation between GLS frame and pool frame
    # v2 is from bottom left to bottom right - treat this as x axis
    v2_norm = np.linalg.norm(v2)
    frame_offset = math.acos(np.dot(v2, np.array([1.0, 0.0, 0.0])) / (v2_norm))
    global imu_offset
    imu_offset = frame_offset + calibration_points[0].heading # SIGNS HERE!

    rospy.loginfo("IMU offset: %f\n", imu_offset)

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
    waypoint_pub = rospy.Publisher("calibrated_waypoint", PointStamped,
            queue_size=1)
    imu_pub = rospy.Publisher("calibrated_imu", Euler, queue_size=1)

    rospy.init_node("Calibrator", log_level=rospy.INFO)
    rospy.Subscriber("raw_location", PointStamped, handle_location)
    rospy.Subscriber("raw_waypoint", PointStamped, handle_waypoint)
    rospy.Subscriber("imu_topic/Euler", Euler, handle_imu)
    rospy.spin()
