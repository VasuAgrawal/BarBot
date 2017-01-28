#!/usr/bin/env python3

import os
import logging

import tornado
import tornado.web
import tornado.websocket
import tornado.httpserver

class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")

class GamepadHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        logging.info("Received message: %s", message)

    def open(self):
        logging.info("Opened websocket connection!")

    def on_close(self):
        logging.info("Closed websocket connection :(")


class JoystickServer(tornado.httpserver.HTTPServer):

    @staticmethod
    def make_app():
        settings = {
            "static_path": os.path.join(os.path.dirname(__file__), "static"),
        }

        return tornado.web.Application([
            (r"/", RootHandler),
            (r"/gamepad", GamepadHandler),
        ], xheaders=True, debug=True, **settings)


    def __new__(cls, *args, **kwargs):
        return super().__new__(cls, cls.make_app())

    def __init__(self, *args, **kwargs):
        # super().__init__(*args, **kwargs)
        return

if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    JoystickServer().listen(8000)
    tornado.ioloop.IOLoop.current().start()
