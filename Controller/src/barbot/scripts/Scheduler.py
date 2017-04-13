#!/usr/bin/env python
import socket
import sys
import rospy
from barbot.msg import State, Waypoint, Thruster

# The scheduler node is responsible for receiving ordering information, and sending over to our controller.
class Scheduler(object):
    def __init__(self, name, waypoint_topic):
        rospy.init_node(name)
        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher(waypoint_topic, Waypoint, queue_size=1)

    def push(self, waypoint):
        self.pub.publish(waypoint)

    # This function should connect to the actual scheduler, and push to robot as required
    def listen(self, server_address):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print >> sys.stderr, 'starting up on %s port %s' % server_address
        sock.bind(server_address)
        sock.listen(1)
        while True:
            # Wait for a connection
            print >>sys.stderr, 'waiting for a connection'
            connection, client_address = sock.accept()
            try:
                print >>sys.stderr, 'connection from', client_address

                # Receive the data in small chunks and retransmit it
                while True:

                    data = connection.recv(64)
                    print >>sys.stderr, 'received "%s"' % data

                    locations = data.split(' ')
                    if(len(locations) != 3):
                        print("message ignored")
                        continue

                    waypoint = Waypoint()
                    waypoint.pose.x = float(locations[0])
                    waypoint.pose.y = float(locations[1])
                    waypoint.pose.theta = float(locations[2])
                    waypoint.header.stamp = rospy.Time.now()

                    self.push(waypoint)
                        
            finally:
                # Clean up the connection
                connection.close()



if __name__ == '__main__':
    scheduler = Scheduler("scheduler", "waypoint_topic")
    server_address = ('localhost', 10000)
    scheduler.listen(server_address)