#!/usr/bin/env python3

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
    ser = serial.Serial('/dev/ttyACM0', 115200)
except Exception:
    ser = None

def map_value(x, from_lo, from_hi, to_lo, to_hi):
    from_range = from_hi - from_lo
    to_range = to_hi - to_lo
    return (((x - from_lo) / from_range) * to_range) + to_lo
 
class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")

class GamepadHandler(tornado.websocket.WebSocketHandler):
    FORWARD = np.array([1.0, 0.0, 0.0, 1.0])
    BACKWARD = np.array([0.0, 1.0, 1.0, 0.0])
    LEFT = np.array([0.0, 0.0, 1.0, 1.0])
    RIGHT = np.array([1.0, 1.0, 0.0, 0.0])
    CCW = np.array([0.0, 1.0, 0.0, 1.0])
    CW = np.array([1.0, 0.0, 1.0, 0.0])

    def on_message(self, message):
        state = json.loads(message)
        logging.debug("Received message: %s", message)

        axes = state['axes']
        forward = -min(axes[3], 0)
        backward = max(axes[3], 0)
        left = -min(axes[2], 0)
        right = max(axes[2], 0)
        ccw = -min(axes[0], 0)
        cw = max(axes[0], 0)

        outputs = np.minimum(1.0,
            (self.FORWARD * forward + self.BACKWARD * backward + self.LEFT *
                left + self.RIGHT * right + self.CCW * ccw + self.CW * cw))
        logging.debug("Raw outputs: %s", outputs)

        # Map to something that's not full power
        outputs = map_value(outputs, -1, 1, 1300, 1700)
        if any(state['buttons']):
            outputs = np.ones(4) * 1500

        logging.debug("Mapped outputs: %s", outputs)

        out = ','.join(map(lambda x: str(int(x)), outputs)) + '\n'
        logging.debug("Writing string " + repr(out))

        if (ser):
            ser.write(out.encode('ascii'))
            logging.info("Writing string " + repr(out))
            logging.info("Read back " + repr(ser.readline()))
        else:
            logging.info("Writing string " + repr(out))


    def open(self):
        logging.info("Opened websocket connection!")

    def on_close(self):
        logging.info("Closed websocket connection :(")


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
        ], xheaders=True, debug=True, **settings)


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)
    JoystickServer().listen(8000)
    tornado.ioloop.IOLoop.current().start()

    led.stop()
    GPIO.cleanup()
