#!/usr/bin/env python3

import time
import os
import logging
import pprint
import json
import serial
import copy

import numpy as np
import tornado
import tornado.web
import tornado.websocket
import tornado.httpserver
from Adafruit_PWM_Servo_Driver import PWM

import threading


# TODO: Search through serial ports, open up the first one.
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception:
    ser = None
# ser = None

ALIVE = False
alive_time = time.time()
last_write_time = time.time()

out_message_lock = threading.Lock()
out_message = "1500 1500\n"

pwm = PWM(0x40)
PWM_FREQ = 50
pwm.setPWMFreq(PWM_FREQ)

def map_value(x, from_lo, from_hi, to_lo, to_hi):
    from_range = from_hi - from_lo
    to_range = to_hi - to_lo
    return (((x - from_lo) / from_range) * to_range) + to_lo


def writer():
    while True:
        start = time.time()
        with out_message_lock:
            logging.debug("Should be writing %s", repr(out_message))
            if (ser):
                logging.info("Finally writing %s", repr(out_message))
                ser.write(out_message.encode('ascii'))
            else:
                logging.warning("Not connected to serial port?")

        time.sleep(max(0, .2 - (time.time() - start)))


def I2C_writer():
    while True:
        start = time.time()
        out = ""

        with out_message_lock:
            out = copy.deepcopy(out_message)

        logging.debug("Should be writing %s", repr(out))
        left, right = map(int, out.strip().split())

        us_per_s = 1000000
        periods_per_s = PWM_FREQ
        us_per_period = us_per_s / periods_per_s
        ticks_per_period = 4096
        ticks_per_us = ticks_per_period / us_per_period
        pwm.setPWM(0, 0, int(left * ticks_per_us))
        pwm.setPWM(3, 0, int(right * ticks_per_us))

        time.sleep(max(0, .2 - (time.time() - start)))
        


class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")


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
        logging.info("Copying string " + repr(out))

        with out_message_lock:
            global out_message
            out_message = out

        # global last_write_time
        # if (ser):
            # if (time.time() - last_write_time > .2):
                # ser.write(out.encode('ascii'))
                # last_write_time = time.time()
                # logging.info("Writing string " + repr(out))
                # logging.info("Read back " + repr(ser.readline()))


    def open(self):
        logging.info("Opened websocket connection!")

    @tornado.gen.coroutine
    def on_close(self):
        logging.info("Closed websocket connection :(")
        # if (ser):
            # ser.write("1500 1500\n".encode('ascii'))
        
        with out_message_lock:
            global message
            out_message = "1500 1500\n"



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
    threading.Thread(target=writer).start()
    logging.getLogger().setLevel(logging.DEBUG)
    JoystickServer().listen(8000)
    # tornado.ioloop.PeriodicCallback(writer, 200).start() # Call watchdog every 1 sec
    tornado.ioloop.IOLoop.current().start()
