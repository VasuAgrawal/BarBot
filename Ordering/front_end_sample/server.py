#!/usr/bin/env python3
import logging
import json

import tornado.ioloop
import tornado.web

class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")


class CustomerHandler(tornado.web.RequestHandler):
    def get(self):
        print("Doing customer handler!")
        # TODO figure out how to not hard code "static" here
        self.render("static/html/customer.html")


class BartenderHandler(tornado.web.RequestHandler):
    def get(self):
        # TODO figure out how to not hard code "static" here
        self.render("static/html/bartender.html")


class ApiDrinkHandler(tornado.web.RequestHandler):
    def get(self):
        print(self.get_argument("id", default=None))
        self.write(json.dumps({"drink1": "beer"}))

    def post(self):
        print(self.get_argument("id"))


class ApiOrderHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({"order1": "some order"}))


class ApiRobotHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({"robot": "robot data!"}))


class ApiCustomerHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({"customer": "customer data"}))

def main():
    logging.info("Starting logging!")
    logging.root.setLevel(logging.DEBUG)

    handlers = [
        #TODO tune these regex
        (r"/?", RootHandler),
        (r"/customer/?", CustomerHandler),
        (r"/bartender/?", BartenderHandler),
        (r"/v0/drink/", ApiDrinkHandler),
        (r"/v0/order/", ApiOrderHandler),
        (r"/v0/robot/", ApiRobotHandler),
        (r"/v0/customer/", ApiCustomerHandler),
    ]
    
    tornado.web.Application(
        handlers,
        static_path="static",
        debug=True,
        autoreload=True,
    ).listen(8888)

    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
