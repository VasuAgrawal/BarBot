#!/usr/bin/env python3

import os
import logging
import pprint
import json

import tornado
import tornado.web
import tornado.websocket
import tornado.httpserver

try:
    import RPi.GPIO as GPIO
except ImportError:
    import fake_gpio as GPIO


LED_PIN = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
led = GPIO.PWM(LED_PIN, 50)
led.start(0)
 
class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")

class GamepadHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        state = json.loads(message)
        # logging.info("Received message: %s", message)
        led.ChangeDutyCycle(max(state['axes'][3], 0))

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
    logging.getLogger().setLevel(logging.DEBUG)
    JoystickServer().listen(8000)
    tornado.ioloop.IOLoop.current().start()

    led.stop()
    GPIO.cleanup()
