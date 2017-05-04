#!/usr/bin/env python
from __future__ import print_function

import time
import threading
import collections

import rospy
from barbot.msg import Euler
from geometry_msgs.msg import PointStamped, Vector3

import numpy as np
import math

location_deq = collections.deque(maxlen=10)
imu_deq = collections.deque(maxlen=10)
mag_deq = collections.deque(maxlen=10)
grav_deq = collections.deque(maxlen=10)

calibration_points = []
calibrated = True

gls_origin = None
rmat = None

frame_offset = 0.0
imu_offset = 0

gvec = np.array([0.0, 0.0, -1.0])
grav_data = np.array([0.0, 0.0, -1.0])
rgrav = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

def project(point):
    v = point - pool_plane_origin
    dist = np.dot(v, pool_plane_normal)
    projected = point - dist * pool_plane_normal
    final = rmat.dot(projected)
    final[2] = 0
    return final

def compute_projection(p0, p1, p2):
    # Frame of physical pool
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    v3 = np.array([0.0, 0.0, 1.0])

    # Frame of global localization points
    # X axis, measured from p1 to p0
    w1 = np.array([p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]])
    w1 = w1 / np.linalg.norm(w1)
    w1 = np.array([w1[0][0], w1[1][0], w1[2][0]])

    # Temporary "y axis", measured from p2 to p0
    temp = np.array([p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]])
    temp = temp / np.linalg.norm(temp)
    temp = np.array([temp[0][0], temp[1][0], temp[2][0]])
    
    # Z axis, orthogonal to X axis
    w3 = np.cross(np.transpose(w1), np.transpose(temp))
    
    # Y axis, orthogonal to both X and Z axis
    w2 = np.cross(w3, w1)

    # Matrices representing frames in pool (v) and GLS (w) space
    v = np.transpose(np.vstack((v1, v2, v3)))
    w = np.transpose(np.vstack((w1, w2, w3)))

    # Use SVD to compute rotation matrix from w to v
    B = np.dot(v, np.transpose(w))
    U, S, V = np.linalg.svd(B)
    M = np.diag([1.0, 1.0, np.linalg.det(U)*np.linalg.det(V)])
    rmat = np.dot(np.dot(U, M), V)

    return rmat

def handle_location(data):
    location_deq.append(data)

    # If calibrated, convert this reading into a calibrated one and republish.
    if calibrated:
        point = np.array([[data.point.x], [data.point.y], [data.point.z]])

        # Apply transformation from GLS frame to pool frame
        # First subtract translation offset and apply rotation
        projected = rmat.dot(point - gls_origin)

        # Publish normalized data
        (data.point.x, data.point.y, data.point.z) = (projected[0][0], projected[1][0], projected[2][0])
        location_pub.publish(data)

def handle_grav(data):
    grav_deq.append(data)
    grav_data = np.array([data.x, data.y, data.z])
    norm = np.linalg.norm(grav_data)

    phi = math.asin(data.x / norm)
    cphi = math.cos(phi)
    sphi = math.sin(phi)

    theta = math.atan2(data.y / (-data.x*cphi), data.z / (data.x*cphi))
    ctheta = math.cos(theta)
    stheta = math.sin(theta)

    rxry = np.array ( [
        [cphi, 0, sphi],
        [stheta*sphi, ctheta, -stheta*cphi],
        [-ctheta*sphi, stheta, ctheta*cphi]
        ] )

    global rgrav
    rgrav = np.transpose(rxry)


def handle_imu(data):
    imu_deq.append(data)
    
    # # If calibrated, convert this reading into a calibrated one and republish.
    # if calibrated:
        # heading = data.heading - imu_offset # degrees
        # heading = int(heading + 360) % 360 # convert to [0, 360)
        # print("Subtracting IMU Offset: ", heading, end=" ")

        # # Multiply by pi/180, convert to radians
        # heading = math.radians(heading) # [0, 2*PI)
        # print("Converted to radians: ", heading, end=" ")

        # # Attempt to subtract the frame offset
        # heading -= frame_offset
        # print("Subtracted frame offset: ", heading, end=" ")
        
        # # Set it back to the right range
        # heading += 2 * math.pi
        # heading %= 2 * math.pi # [0, 2*PI)
        # heading -= 1 * math.pi # looks nicer, in [-PI, PI)
        # print("Final heading: ", heading)

        # # heading = data.heading + imu_offset
        # # heading = int(heading) % 360
        # # heading = float(heading) / 180.0 * math.pi
        # # heading += math.pi / 2
        # # heading -= frame_offset
        # # heading = heading % (2*math.pi)
        # # heading = (heading + math.pi) % (2*math.pi) - math.pi

        # data.heading = heading
        # imu_pub.publish(data)

def handle_mag(data):
    mag_deq.append(data)

    if calibrated:

        global rgrav
        data_vec = np.array([data.x, data.y, data.z])
        data_vec_w = rgrav.dot(data_vec)

        heading = math.atan2(data_vec_w[1], data_vec_w[0]) # [-pi, pi)
        # if heading < 0:
            # heading += 2 * math.pi # [0, 2*pi)
        print("heading: %4f" % heading)

        heading -= imu_offset
        # heading = (heading + 2 * math.pi) % (2 * math.pi) # [0, 2*pi)
        print("imu off: %4f, minus imu off: %4f, minus in deg: %d" %
                (imu_offset, heading, math.degrees(heading)))

        # heading -= frame_offset
        # print("frame off: %4f, minus frame off: %4f" % (frame_offset, heading))

        # heading *= -1
        heading += 3 * 2 * math.pi
        heading %= 2 * math.pi
        if heading > math.pi:
            heading -= 2 * math.pi

        print("Normalized heading to [-pi, pi): %f, %f" % (heading,
                math.degrees(heading)))
        print()

        imu_data = Euler()
        imu_data.heading = heading
        imu_pub.publish(imu_data)
        

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
    def __init__(self, x, y, z, roll, pitch, heading, mag_x, mag_y, mag_z):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.heading = heading
        self.mag_x = mag_x
        self.mag_y = mag_y
        self.mag_z = mag_z

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
        mag_data = []
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
            
            if len(mag_data) < DESIRED:
                try:
                    mag_data.append(mag_deq.popleft())
                except IndexError:
                    rospy.logwarn("Error in getting mag data")

        avg_x = average([data.point.x for data in location_data])
        avg_y = average([data.point.y for data in location_data])
        avg_z = average([data.point.z for data in location_data])
        avg_heading = average([data.heading for data in imu_data])
        avg_roll = average([data.roll for data in imu_data])
        avg_pitch = average([data.pitch for data in imu_data])
        avg_mag_x = average([data.x for data in mag_data])
        avg_mag_y = average([data.y for data in mag_data])
        avg_mag_z = average([data.z for data in mag_data])

        rospy.loginfo("Calibration parameters at %s:\n"
                "location: [x:    %f, y:     %f, z:       %f]\n"
                "imu:      [roll: %f, pitch: %f, heading: %f]\n"
                "mag:      [x:    %f, y:     %f, z:       %f]\n",
                location, avg_x, avg_y, avg_z,
                avg_roll, avg_pitch, avg_heading,
                avg_mag_x, avg_mag_y, avg_mag_z)
        calibration_points.append(CalibrationPoint(avg_x, avg_y, avg_z,
            avg_roll, avg_pitch, avg_heading, avg_mag_x, avg_mag_y, avg_mag_z))

    # Compute information about the pool plane from calibration data
    # Use bottom left, top left, bottom right
    p0 = calibration_points[0]
    p1 = calibration_points[1]
    p2 = calibration_points[3]

    # Compute points in GLS frame, in numpy array column vector format
    p0_gls = np.array([[p0.x], [p0.y], [p0.z]])
    p1_gls = np.array([[p1.x], [p1.y], [p1.z]])
    p2_gls = np.array([[p2.x], [p2.y], [p2.z]])

    # Set origin of GLS frame
    global gls_origin
    gls_origin = p0_gls

    # Compute rotation matrix from GLS frame to pool frame
    global rmat
    rmat = compute_projection(p0_gls, p1_gls, p2_gls)

    # Compute rotation between GLS frame and pool frame
    # v2 is from bottom left to bottom right - treat this as x axis
    projected_v2 = project(v2)
    projected_v2_norm = np.linalg.norm(projected_v2)

    global frame_offset
    frame_offset = math.acos(np.dot(projected_v2, np.array([1.0, 0.0, 0.0])) / 
            (projected_v2_norm)) # range [0, pi)

    global imu_offset
    # imu_offset = calibration_points[0].heading # SIGNS HERE!
    # avg_mag_x = average([point.mag_x for point in calibration_points])
    # avg_mag_y = average([point.mag_y for point in calibration_points])
    mag_x = calibration_points[0].mag_x
    mag_y = calibration_points[0].mag_y
    imu_offset = math.atan2(mag_y, mag_x) # range [-pi, pi)
    # if imu_offset < 0:
        # imu_offset += 2 * math.pi # range[0, 2*pi)

    rospy.loginfo("Heading offset: %f\n", imu_offset)
    rospy.loginfo("Frame offset: %f\n", frame_offset)

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
    rospy.Subscriber("imu_topic/Magnetometer", Vector3, handle_mag)
    rospy.Subscriber("imu_topic/GravityAcceleration", Vector3, handle_grav)
    rospy.spin()
