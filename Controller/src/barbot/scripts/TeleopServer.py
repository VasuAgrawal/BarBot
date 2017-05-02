#!/usr/bin/env python

import time
import os
import logging
import json
import Queue
import sys
import threading

import numpy as np
import tornado
import tornado.gen
import tornado.httpserver
import tornado.locks
import tornado.web
import tornado.websocket

import rospy
from barbot.msg import Thruster
from barbot.msg import Mode

thruster_queue = Queue.Queue()
mode_queue = Queue.Queue()


class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")


class HeartbeatHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        rospy.logdebug("Received heartbeat message")
        # print("Received heartbeat message")

        # Doesn't matter what the message is, we just need a message of some
        # sort to update the alive handler.
        global alive_time
        alive_time = time.time()


class GamepadHandler(tornado.websocket.WebSocketHandler):

    def move_skid_mid(self, axes):
        FORWARD = np.array([1.0, 1.0])
        BACKWARD = np.array([-1.0, -1.0])
        CCW = np.array([-1.0, 1.0])
        CW = np.array([1.0, -1.0])
        
        forward = -min(axes[3], 0)
        backward = max(axes[3], 0)
        ccw = -min(axes[0], 0)
        cw = max(axes[0], 0)
        
        outputs = np.maximum(-1.0, np.minimum(1.0,
                        (FORWARD * forward + BACKWARD * backward + 
                            CCW * ccw + CW * cw)))
        outputs[1] *= -1
        return outputs


    @tornado.gen.coroutine
    def on_message(self, message):
        state = json.loads(message)
        rospy.logdebug("Received message: %s", message)
        # print("Received message: %s", message)

        axes = state['axes']

        outputs = np.array([0, 0])
        outputs = self.move_skid_mid(axes)
        rospy.logdebug("Raw outputs: %s", outputs)

        msg = Thruster()
        msg.left = outputs[0]
        msg.right = outputs[1]
        thruster_queue.put(msg)


        if state['buttons'][0]: # A
            msg = Mode()
            msg.mode = Mode.AUTON
            mode_queue.put(msg)
        elif state['buttons'][1]: # B
            msg = Mode()
            msg.mode = Mode.TELEOP
            mode_queue.put(msg)


    def open(self):
        rospy.loginfo("Opened websocket connection!")
        # print("Opened websocket connection!")


    @tornado.gen.coroutine
    def on_close(self):
        rospy.loginfo("Closed websocket connection :(")
        # print("Closed websocket connection :(")
        
        msg = Thruster()
        msg.left = 0
        msg.right = 0
        thruster_queue.put(msg)
        

class JoystickServer(tornado.httpserver.HTTPServer):
    
    def __init__(self, *args, **kwargs):
        return


    def __new__(cls, *args, **kwargs):
        return super(JoystickServer, cls).__new__(cls, cls.make_app())


    @staticmethod
    def make_app():
        settings = {
            "static_path": os.path.join(os.path.dirname(__file__), "static"),
        }

        return tornado.web.Application([
            (r"/", RootHandler),
            (r"/gamepad", GamepadHandler),
            (r"/heartbeat", HeartbeatHandler),
        ], xheaders=True, debug=True, **settings)


def check_dead():
    if rospy.is_shutdown():
        rospy.logwarn("Shutting down webserver!")
        sys.exit()

def webserver():
    rospy.loginfo("Starting webserver!")
    JoystickServer().listen(8000)
    tornado.ioloop.PeriodicCallback(check_dead, 500).start()
    tornado.ioloop.IOLoop.current().start()


def ros_publisher():

    rospy.logdebug("Starting to wait on data items.")
    while not rospy.is_shutdown():
        try:
            thruster_msg = thruster_queue.get(timeout=.5)
            rospy.loginfo("Ros node received thruster item: %s", thruster_msg)
            thruster_pub.publish(thruster_msg)
        except Queue.Empty:
            rospy.logdebug("No messages in thruster queue to publish.")

        try:
            mode_msg = mode_queue.get_nowait()
            rospy.loginfo("Ros node received mode item: %s", mode_msg)
            mode_pub.publish(mode_msg)
        except Queue.Empty:
            rospy.logdebug("No messages in mode queue to publish.")

    rospy.logwarn("Shutting down ros publisher!")
    sys.exit(0)


if __name__ == "__main__":
    global thruster_pub
    global mode_pub 
    thruster_pub = rospy.Publisher("teleop", Thruster, queue_size=1)
    mode_pub = rospy.Publisher("mode", Mode, queue_size=1)
    rospy.init_node("Teleop_server", log_level=rospy.INFO)

    tornado_thread = threading.Thread(target=webserver).start()
    ros_publisher()
    sys.exit(0)

