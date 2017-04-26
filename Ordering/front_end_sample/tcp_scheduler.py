#!/usr/bin/env python3
import logging

import tornado
import tornado.tcpserver
import tornado.gen
import tornado.ioloop

class Scheduler(tornado.tcpserver.TCPServer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @tornado.gen.coroutine
    def handle_stream(self, stream, address):
        logging.info("Incoming connection request from %s", address)


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    Scheduler().listen(4242)
    tornado.ioloop.IOLoop.current().start()
