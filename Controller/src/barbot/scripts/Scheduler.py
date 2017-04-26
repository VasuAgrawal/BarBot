#!/usr/bin/env python
import socket
import sys
import rospy
import time

from barbot.msg import State, Waypoint, Thruster
from positions_pb2 import Point
from positions_pb2 import Locations
from positions_pb2 import ConnectionRequest



HEADER_LEN = 4 # Number of bytes for the header
ENDIANNESS = "little"

# Protobuf strings are serialized to bytes encoded in UTF8
def make_packet_from_bytes(data_bytes):
    # Just do a length encoding of the data bytes
    try:
        # Unsigned integer
        header = len(data_bytes).to_bytes(HEADER_LEN, ENDIANNESS)
    except OverflowError as e:
        # The data is too long to fit, so we have to do something.
        raise e

    return header + data_bytes



# The scheduler node is responsible for receiving ordering information, and sending over to our controller.
class Scheduler(object):
    def __init__(self, name, beacon, waypoint_topic):
        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher(waypoint_topic, Waypoint, queue_size=1)

    def push(self, waypoint):
        self.pub.publish(waypoint)

    # This function should connect to the actual scheduler, and push to robot as required
    def listen(self, server_address):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print >> sys.stderr, 'connecting to %s port %s' % server_address

        sock.connect(server_address)
        print "connected!"

        request = ConnectionRequest()
        request.type = ConnectionRequest.ROBOT
        data = make_packet_from_bytes(request.SerializeToString())
        self._sock.send(data)


        while True:
            header = sock.recv(HEADER_LEN)
            num_to_read = int.from_bytes(header, ENDIANNESS)
            data = sock.recv(num_to_read)

            loc = Locations()
            loc.ParseFromString(data)

            if self.beacon in loc.locations:
                point = loc.locations[self.beacon]

                waypoint = Waypoint()
                waypoint.pose.x = float(point.x)
                waypoint.pose.y = float(point.y)
                waypoint.pose.theta = float(point.z)
                waypoint.header.stamp = rospy.Time.now()

                self.push(waypoint)

if __name__ == '__main__':
    scheduler = Scheduler("scheduler", 1, "waypoint_topic")
    server_address = ('localhost', 10000)
    scheduler.listen(server_address)