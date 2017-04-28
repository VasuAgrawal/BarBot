#!/usr/bin/env python
import socket
import sys
import rospy
import time
import struct

from geometry_msgs.msg import PointStamped
from barbot.msg import State, Thruster
from positions_pb2 import Point
from positions_pb2 import Locations
from positions_pb2 import ConnectionRequest



HEADER_LEN = 4 # Number of bytes for the header
ENDIANNESS = "little"
BIG_ENDIAN = False

# Protobuf strings are serialized to bytes encoded in UTF8
def make_packet_from_bytes(data_bytes):
    # Just do a length encoding of the data bytes
    try:
        # Unsigned integer
        header = struct.pack("<i", len(data_bytes))
    except OverflowError as e:
        # The data is too long to fit, so we have to do something.
        raise e

    return header + data_bytes


# The scheduler node is responsible for receiving ordering information, and sending over to our controller.
class Scheduler(object):
    def __init__(self, name, beacon, waypoint_topic, location_topic):
        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher(waypoint_topic, PointStamped, queue_size=1)
        self.loc_pub = rospy.Publisher(location_topic, PointStamped, queue_size=1)
        self.beacon = beacon
        self.customer_beacon = 8
        self.calibrated = False
        self.bottom_left = Point()
        self.top_left = Point()
        self.top_right = Point()
        self.bottom_right = Point()
        self.bar = Point()

    def push_waypoint(self, waypoint):
        self.pub.publish(waypoint)

    def push_location(self, point):
        self.loc_pub.publish(point)

    # This function should connect to the actual scheduler, and push to robot as required
    def listen(self, server_address):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print >> sys.stderr, 'connecting to %s port %s' % server_address

        sock.connect(server_address)
        print "connected!"

        request = ConnectionRequest()
        request.type = ConnectionRequest.ROBOT
        data = make_packet_from_bytes(request.SerializeToString())
        sock.send(data)

        while True:
            if(not self.calibrated):
                raw_input("move the robot to bottom left of the pool, then press Enter.")
                done = False
                while( not done ):
                    header = sock.recv(HEADER_LEN)
                    num_to_read = struct.unpack("<i",header)[0]
                    data = sock.recv(num_to_read)

                    loc = Locations()
                    loc.ParseFromString(data)

                    if self.beacon in loc.locations:
                        point = loc.locations[self.beacon]
                        self.bottom_left = point
                        print("Location: %f %f %f" % (point.x, point.y, point.z))
                        done = True
                raw_input("move the robot to top left of the pool, then press Enter")
                done = False
                while( not done ):
                    header = sock.recv(HEADER_LEN)
                    num_to_read = struct.unpack("<i",header)[0]
                    data = sock.recv(num_to_read)

                    loc = Locations()
                    loc.ParseFromString(data)

                    if self.beacon in loc.locations:
                        point = loc.locations[self.beacon]
                        self.top_left = point
                        print("Location: %f %f %f" % (point.x, point.y, point.z))
                        done = True
                raw_input("move the robot to top right of the pool, then press Enter")
                done = False
                while( not done ):
                    header = sock.recv(HEADER_LEN)
                    num_to_read = struct.unpack("<i",header)[0]
                    data = sock.recv(num_to_read)

                    loc = Locations()
                    loc.ParseFromString(data)

                    if self.beacon in loc.locations:
                        point = loc.locations[self.beacon]
                        self.top_right = point
                        print("Location: %f %f %f" % (point.x, point.y, point.z))
                        done = True
                raw_input("move the robot to bottom right of the pool, then press Enter")
                done = False
                while( not done ):
                    header = sock.recv(HEADER_LEN)
                    num_to_read = struct.unpack("<i",header)[0]
                    data = sock.recv(num_to_read)

                    loc = Locations()
                    loc.ParseFromString(data)

                    if self.beacon in loc.locations:
                        point = loc.locations[self.beacon]
                        bottom_right = point
                        print("Location: %f %f %f" % (point.x, point.y, point.z))
                        done = True
                raw_input("move the robot to the bar, then press Enter")
                done = False
                while( not done ):
                    header = sock.recv(HEADER_LEN)
                    num_to_read = struct.unpack("<i",header)[0]
                    data = sock.recv(num_to_read)

                    loc = Locations()
                    loc.ParseFromString(data)

                    if self.beacon in loc.locations:
                        point = loc.locations[self.beacon]
                        bar = point
                        print("Location: %f %f %f" % (point.x, point.y, point.z))
                        done = True

                #Copmute Transformation Here
                self.calibrated = True

            else:
                header = sock.recv(HEADER_LEN)
                num_to_read = struct.unpack("<i",header)[0]
                data = sock.recv(num_to_read)

                loc = Locations()
                loc.ParseFromString(data)

                if self.beacon in loc.locations:
                    point = loc.locations[self.beacon]

                    location = PointStamped()
                    location.point.x = point.x
                    location.point.y = point.y
                    location.point.z = point.z
                    location.header.stamp = rospy.Time.now()

                    self.push_location(location)

                if self.beacon in loc.locations:
                    point = loc.locations[self.customer_beacon]

                    location = PointStamped()
                    location.point.x = point.x
                    location.point.y = point.y
                    location.point.z = point.z
                    location.header.stamp = rospy.Time.now()

                    self.push_waypoint(location)


if __name__ == '__main__':
    scheduler = Scheduler("scheduler", 7, "waypoint_topic", "location_topic")
    server_address = ('localhost', 4242)
    scheduler.listen(server_address)