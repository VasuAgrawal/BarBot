#!/usr/bin/env python
import socket
import sys
import rospy
import time
import struct

from geometry_msgs.msg import PointStamped, Pose
from barbot.msg import Thruster
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
        header = struct.pack("<i", len(data_bytes))
    except OverflowError as e:
        # The data is too long to fit, so we have to do something.
        raise e

    return header + data_bytes

class Broadcaster(object):
    def __init__(self, dwm_id, addr="localhost", port=4242):
        self._dwm_id = dwm_id
        self._addr = addr
        self._port = port
        self._location_pub = rospy.Publisher("raw_location", 
                PointStamped, queue_size=1)
        self._waypoint_pub = rospy.Publisher("waypoint", Pose, queue_size=1)
        self._sock = None
        rospy.loginfo("Broadcaster initialized\n");


    def _connect(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self._addr, self._port))
            request = ConnectionRequest()
            request.type = ConnectionRequest.ROBOT
            data = make_packet_from_bytes(request.SerializeToString())
            self._sock.send(data)

        except Exception:
            rospy.logwarn("Unable to connect to %s:%d", self._addr, self._port)
            self._sock = None


    def _push_location(self, point):
        self._location_pub.publish(point)


    def update(self):
        while not rospy.is_shutdown():
            if self._sock is None:
                time.sleep(1)
                rospy.loginfo("Attempting to connect to scheduler!")
                self._connect()

            try:
                # This should take some time, so this doesn't just spin.
                header = self._sock.recv(HEADER_LEN)
                num_to_read = struct.unpack("<i", header)[0]
                data = self._sock.recv(num_to_read)

                loc = Locations()
                loc.ParseFromString(data)

                if self._dwm_id in loc.locations:
                    point = loc.locations[self._dwm_id]

                    location = PointStamped()
                    location = PointStamped()
                    location.point.x = point.x
                    location.point.y = point.y
                    location.point.z = point.z
                    location.header.stamp = rospy.Time.now()

                    self._push_location(location)
                else:
                    rospy.logwarn(
                            "Received location update without robot ID: %d",
                            self._dwm_id)

            except Exception:
                rospy.logwarn("Something broke, perhaps connection died?")
                self._sock = None


if __name__ == "__main__":
    broadcaster = Broadcaster(dwm_id=7, addr="128.237.167.97") # Initializes publishers
    rospy.init_node("SchedulerBroadcaster", log_level=rospy.INFO)
    broadcaster.update()
