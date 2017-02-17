#!/usr/bin/env python3
import logging
import json

import tornado.ioloop
import tornado.web

class BaseHandler(tornado.web.RequestHandler):
    def get_current_user(self):
        return self.get_secure_cookie("user")

class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")

class LoginHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/login.html")

class AboutHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/about.html")

class CustomerHandler(tornado.web.RequestHandler):
    @tornado.web.authenticated # Example of authentication
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
        (r"/login/?", LoginHandler),
        (r"/about/?", AboutHandler),
    ]

    settings = {
        "static_path": "static",
        "cookie_secret": "TODO some real cookie secret",
        "login_url": "/",
        "debug": True,
        "autoreload": True,
    }
    
    tornado.web.Application(handlers, **settings).listen(8888)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
