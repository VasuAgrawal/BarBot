#!/usr/bin/env python3

import time
import os
import logging
import pprint
import json
import serial

import numpy as np
import tornado
import tornado.web
import tornado.websocket
import tornado.httpserver

# TODO: Search through serial ports, open up the first one.
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception:
    ser = None

def map_value(x, from_lo, from_hi, to_lo, to_hi):
    from_range = from_hi - from_lo
    to_range = to_hi - to_lo
    return (((x - from_lo) / from_range) * to_range) + to_lo
 
class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")

ALIVE = False
alive_time = time.time()

last_write_time = time.time()

def watchdog():
    logging.debug("Watchdog time difference: %f", time.time() - alive_time)
    ALIVE = (time.time() - alive_time) < 1
    if not ALIVE:
        outputs = np.array([0.0, 0.0, 0.0, 0.0])
        outputs = map_value(outputs, -1, 1, 1300, 1700)
        logging.debug("Mapped outputs: %s", outputs)
        out = ','.join(map(lambda x: str(int(x)), outputs)) + '\n'

        # Send a stop message
        if (ser):
            ser.write(out.encode('ascii'))
            logging.info("WATCHDOG, Writing string " + repr(out))
        else:
            logging.info("WATCHDOG, Writing string " + repr(out))


class HeartbeatHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        logging.debug("Received heartbeat message")
        # Doesn't matter what the message is, we just need a message of some
        # sort to update the alive handler.
        global alive_time
        alive_time = time.time()


class GamepadHandler(tornado.websocket.WebSocketHandler):
    # ROBOT MOTORS:
    # ---------------
    # | 1    ^    4 |
    # |      |      |
    # |             |
    # |             |
    # | 2         3 |
    # ---------------
    
       
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
        return outputs


    def on_message(self, message):
        state = json.loads(message)
        logging.debug("Received message: %s", message)

        axes = state['axes']

        outputs = np.array([0, 0, 0, 0])
        outputs = self.move_skid_mid(axes)
        logging.debug("Raw outputs: %s", outputs)

        # Convert outputs from a [-1.0, 1.0] scale to [min, max] linearly
        outputs = map_value(outputs, -1, 1, 1300, 1700)

        logging.debug("Mapped outputs: %s", outputs)
        out = ','.join(map(lambda x: str(int(x)), outputs)) + '\n'
        logging.info("Writing string " + repr(out))

        if (ser):
            if (time.time() - last_write_time > .2):
                ser.write(out.encode('ascii'))
                last_write_time = time.time()
                # logging.info("Writing string " + repr(out))
                # logging.info("Read back " + repr(ser.readline()))


    def open(self):
        logging.info("Opened websocket connection!")

    def on_close(self):
        logging.info("Closed websocket connection :(")
        ser.write("1500 1500\n".encode('ascii'))


class JoystickServer(tornado.httpserver.HTTPServer):
    
    def __init__(self, *args, **kwargs):
        return


    def __new__(cls, *args, **kwargs):
        return super().__new__(cls, cls.make_app())


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


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)
    JoystickServer().listen(8000)
    tornado.ioloop.PeriodicCallback(watchdog, 1000).start() # Call watchdog every 1 sec
    tornado.ioloop.IOLoop.current().start()
